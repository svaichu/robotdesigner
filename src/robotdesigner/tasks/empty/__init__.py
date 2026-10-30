"""``empty``: a placeholder task provider, shipped so that a default environment can run (``empty/idle``).

The robot stands in the simulator's scene and nothing is asked of it. This is the one task the package names itself;
see the decision log of ``docs/spec_robotconfig_env.md`` (the entry "the package ships one placeholder task"). The
namespace is unrelated to the ``empty`` scene preset of the simulator (``sim.scene``).
"""

from __future__ import annotations

from typing import Any

from ...config.errors import Problem
from ...config.robot import Pose
from .params import TASKS, EmptyParams

__all__ = ["PROVIDER", "EmptyParams", "EmptyTasks"]


class EmptyTasks:
    SIMULATORS = frozenset({"mujoco", "warp"})

    def task_names(self) -> tuple[str, ...]:
        return TASKS

    def params_class(self, name: str) -> type | None:
        return EmptyParams if name in TASKS else None

    def default_mount(self, name: str) -> Pose:
        return Pose.identity()

    def validate(self, cfg: Any, simulator: str) -> list[Problem]:
        if simulator not in self.SIMULATORS:
            return [Problem("sim.simulator", f"task {cfg.task.id!r} runs on {sorted(self.SIMULATORS)}, not on {simulator!r}")]
        return []

    def build(self, cfg: Any, simulator: str) -> Any:
        from .task import IdleTask

        return IdleTask()


PROVIDER = EmptyTasks()
