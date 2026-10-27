"""CPU MuJoCo backend: the reference and debugging backend."""

from __future__ import annotations

from typing import Any

from ..base import Capabilities, Problem


class MujocoBackend:
    name = "mujoco"

    def capabilities(self) -> Capabilities:
        return Capabilities(
            control_modes=frozenset({"joint_pos_abs", "joint_torque"}),
            task_ids=frozenset({"native/reach"}),
            devices=frozenset({"cpu"}),
            robot_kinds=frozenset({"parametric"}),
            integrators=frozenset({"euler", "implicit", "implicitfast", "rk4"}),
        )

    def validate(self, cfg: Any) -> list[Problem]:
        return []

    def make_env(self, cfg: Any):
        raise NotImplementedError("the MuJoCo reach environment arrives with the envs milestone")
