"""CPU MuJoCo backend: the reference and debugging backend."""

from __future__ import annotations

from typing import Any

from ..base import Capabilities, Problem


class MujocoBackend:
    name = "mujoco"

    def capabilities(self) -> Capabilities:
        return Capabilities(
            control_modes=frozenset({"joint_pos_abs", "joint_pos_delta", "joint_torque", "joint_vel"}),
            task_ids=frozenset({"native/reach"}),
            devices=frozenset({"cpu"}),
            robot_kinds=frozenset({"parametric"}),
            integrators=frozenset({"euler", "implicit", "implicitfast", "rk4"}),
        )

    def validate(self, cfg: Any) -> list[Problem]:
        from .scenes import SCENES

        problems = []
        if cfg.sim.scene not in SCENES:
            problems.append(Problem("sim.scene", f"unknown scene {cfg.sim.scene!r}; available: {sorted(SCENES)}"))
        return problems

    def make_env(self, cfg: Any):
        from .reach import MujocoReachEnv

        return MujocoReachEnv(cfg)
