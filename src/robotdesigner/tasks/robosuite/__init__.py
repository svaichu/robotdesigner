"""robosuite as a source of task assets: arenas, objects and their placement. Imports neither robosuite nor MuJoCo.

``robosuite/<EnvName>`` loads the arena and the objects of the robosuite environment ``<EnvName>`` into the
simulation named by ``SimConfig.simulator``. robosuite does not simulate anything here: its physics settings,
controllers, robots and rewards are not used. See ``params.py`` for what the task config holds.
"""

from __future__ import annotations

from typing import Any

from ...config.errors import Problem
from ...config.robot import Pose
from ._compat import CompatReport, apply_compat
from .params import RESERVED_ENV_KWARGS, LiftObject, ReachObject, RobosuiteParams

__all__ = [
    "PROVIDER",
    "CompatReport",
    "LiftObject",
    "ReachObject",
    "RobosuiteParams",
    "RobosuiteTasks",
    "apply_compat",
    "default_mount",
]

SCENE = "empty"  # the arena brings its own floor, so the simulator adds none


class RobosuiteTasks:
    SIMULATORS = frozenset({"mujoco"})

    def task_names(self) -> tuple[str, ...]:
        return ("<EnvName>",)

    def params_class(self, name: str) -> type | None:
        return RobosuiteParams if name else None

    def validate(self, cfg: Any, simulator: str) -> list[Problem]:
        if simulator not in self.SIMULATORS:
            return [Problem("sim.simulator", f"task {cfg.task.id!r} runs on {sorted(self.SIMULATORS)}, not on {simulator!r}")]
        try:
            apply_compat()
            import robosuite  # noqa: F401
        except ImportError:
            return [Problem("task.params.name", "robosuite is not installed; install robotdesigner[robosuite]")]

        from . import _introspect as intro

        problems: list[Problem] = []
        if cfg.sim.scene != SCENE:
            problems.append(
                Problem("sim.scene", f"must be {SCENE!r} for robosuite tasks: the arena brings its own floor")
            )
        params = cfg.task.params
        name = params.name
        cls = intro.env_class(name)
        if cls is None:
            problems.append(
                Problem("task.params.name", f"robosuite has no environment {name!r}; single-arm ones: {intro.single_arm_env_names()}")
            )
            return problems
        if not intro.is_single_arm_manipulation(cls):
            problems.append(
                Problem(
                    "task.params.name",
                    f"{name!r} needs more than one arm; single-arm environments: {intro.single_arm_env_names()}",
                )
            )
            return problems
        if name.endswith("Single") or params.kwargs.get("single_object_mode") == 1:
            problems.append(
                Problem(
                    "task.params.name",
                    f"{name!r} picks a different object every episode, so its observation keys change; fix the "
                    "object instead: PickPlace with single_object_mode=2 and object_type, or NutAssemblySquare / "
                    "NutAssemblyRound",
                )
            )
        known = set(intro.constructor_params(cls))
        unknown = sorted(set(params.kwargs) - known)
        if unknown:
            accepted = sorted(known - RESERVED_ENV_KWARGS)
            problems.append(
                Problem("task.params.env_kwargs", f"{name} does not accept {unknown}; accepted: {accepted}")
            )
        if not problems:
            from .scene import load_scene

            try:
                names = [o.name for o in load_scene(name, params.kwargs, cfg.sim.seed).objects if not o.visual]
            except Exception as e:  # robosuite's own checks, for example an unsupported option combination
                return [Problem("task", f"robosuite could not build {name!r}: {type(e).__name__}: {e}")]
            if params.objective.object not in names:
                problems.append(
                    Problem(
                        "task.params.objective.object",
                        f"{params.objective.object!r} is not an object of {name!r}; objects in the scene: {names}",
                    )
                )
        return problems

    def build(self, cfg: Any, simulator: str) -> Any:
        from .task import RobosuiteTask

        return RobosuiteTask(cfg)


PROVIDER = RobosuiteTasks()


def default_mount(env_name: str, **env_kwargs: Any) -> Pose:
    """Where robosuite mounts a stock arm in ``env_name`` (world frame, identity orientation).

    Use it as the base of your own robot, for example ``chain(..., base_pose=default_mount("Lift"))``, so the arm
    stands where the robosuite scene expects it. Builds the robosuite environment once, so it needs robosuite.
    """
    from .scene import load_scene

    pos = load_scene(env_name, env_kwargs).default_mount
    return Pose(tuple(float(x) for x in pos), (1.0, 0.0, 0.0, 0.0))
