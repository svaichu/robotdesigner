"""robosuite backend: any single-arm robosuite environment, with the robot built from our config.

Importing this package imports neither robosuite, mujoco nor torch; ``validate`` and ``make_env`` import them.
``apply_compat`` lets robosuite 1.5.x run on current MuJoCo.
"""

from __future__ import annotations

from typing import Any

from ..base import Capabilities, Problem
from ._compat import CompatReport, apply_compat

__all__ = ["CompatReport", "RobosuiteBackend", "apply_compat"]

ROBOSUITE_SCENE = "env"  # the scene is whatever the robosuite environment builds


class RobosuiteBackend:
    name = "robosuite"

    def capabilities(self) -> Capabilities:
        return Capabilities(
            control_modes=frozenset({"joint_pos_delta"}),
            task_ids=frozenset({"robosuite/*"}),  # the environment name is checked in validate()
            devices=frozenset({"cpu"}),
            robot_kinds=frozenset({"parametric"}),
            integrators=frozenset({"euler"}),  # robosuite's world sets none, so MuJoCo's default applies
        )

    def validate(self, cfg: Any) -> list[Problem]:
        try:
            from robosuite import macros
            from robosuite.models.grippers import ALL_GRIPPERS
        except ImportError:
            return [Problem("sim.backend", "robosuite is not installed; install robotdesigner[robosuite]")]

        from ...config import InertiaScaledGains
        from . import _introspect as intro

        problems: list[Problem] = []
        sim, task = cfg.sim, cfg.task

        if abs(sim.dt - macros.SIMULATION_TIMESTEP) > 1e-12:
            problems.append(
                Problem("sim.dt", f"robosuite fixes the physics timestep at {macros.SIMULATION_TIMESTEP}, got {sim.dt}")
            )
        if sim.scene != ROBOSUITE_SCENE:
            problems.append(
                Problem("sim.scene", f"must be {ROBOSUITE_SCENE!r}: the robosuite environment builds its own scene")
            )
        if not isinstance(cfg.control.gains, InertiaScaledGains):
            problems.append(
                Problem(
                    "control.gains",
                    "robosuite's joint controller scales its gains by the joint inertia; use InertiaScaledGains "
                    "(bandwidth = sqrt(kp), damping_ratio)",
                )
            )
        tool = cfg.robot.tool
        if tool is None:
            problems.append(Problem("robot.tool", "robosuite environments need a gripper; give the arm a ToolSpec"))
        elif tool.gripper not in ALL_GRIPPERS:
            problems.append(
                Problem(
                    "robot.tool.gripper",
                    f"unknown robosuite gripper {tool.gripper!r}; available: {sorted(ALL_GRIPPERS)}",
                )
            )

        name = task.id.split("/", 1)[1]
        cls = intro.env_class(name)
        if cls is None:
            problems.append(
                Problem(
                    "task.id", f"robosuite has no environment {name!r}; single-arm ones: {intro.single_arm_env_names()}"
                )
            )
        elif not intro.is_single_arm_manipulation(cls):
            problems.append(
                Problem(
                    "task.id",
                    f"{name!r} needs more than one arm; single-arm environments: {intro.single_arm_env_names()}",
                )
            )
        else:
            if not callable(getattr(cls, "_check_success", None)):
                problems.append(Problem("task.id", f"{name!r} defines no success check"))
            if name.endswith("Single") or task.params.kwargs.get("single_object_mode") == 1:
                problems.append(
                    Problem(
                        "task.id",
                        f"{name!r} picks a different object every episode, so its observation keys change; "
                        "fix the object instead: PickPlace with single_object_mode=2 and object_type, "
                        "or NutAssemblySquare / NutAssemblyRound",
                    )
                )
            known = set(intro.constructor_params(cls))
            unknown = sorted(set(task.params.kwargs) - known)
            accepted = sorted(known - set(_reserved()))
            if unknown:
                problems.append(
                    Problem(
                        "task.params.env_kwargs",
                        f"{name} does not accept {unknown}; accepted: {accepted}",
                    )
                )
        return problems

    def make_env(self, cfg: Any):
        from .env import RobosuiteEnv

        return RobosuiteEnv(cfg)


def _reserved():
    from ...config.task import RESERVED_ENV_KWARGS

    return RESERVED_ENV_KWARGS
