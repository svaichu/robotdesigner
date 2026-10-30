"""A task provider for the tests only: namespace ``toy``, one goal task (``toy/reach``).

robotdesigner ships no task of its own; its providers are the packages under ``robotdesigner.tasks`` (robosuite) and
whatever a project registers. The engine tests (batched env, control modes, resets, rendering overlays, goal regions) need
a task that has no third-party assets and whose state is easy to predict, so this package is one. ``tests/conftest.py``
registers it, lazily, under the namespace ``toy``. It also shows what a provider looks like: ``params.py`` is the config
side (numpy only), ``reach.py`` the engine side (needs torch and MuJoCo), ``PROVIDER`` ties them together.
"""

from __future__ import annotations

import importlib
from typing import Any

from robotdesigner.config.errors import Problem
from robotdesigner.config.robot import Pose

from .params import ReachParams

__all__ = ["PROVIDER", "ReachParams", "ToyTasks"]


class ToyTasks:
    """name -> (params class, ``"module:Class"`` of the engine side, imported on build)."""

    TASKS: dict[str, tuple[type, str]] = {
        "reach": (ReachParams, "tests.toy_provider.reach:ReachTask"),
    }
    SIMULATORS = frozenset({"mujoco", "warp"})

    def task_names(self) -> tuple[str, ...]:
        return tuple(self.TASKS)

    def params_class(self, name: str) -> type | None:
        entry = self.TASKS.get(name)
        return None if entry is None else entry[0]

    def default_mount(self, name: str) -> Pose:
        return Pose.identity()

    def validate(self, cfg: Any, simulator: str) -> list[Problem]:
        if simulator not in self.SIMULATORS:
            return [
                Problem("sim.simulator", f"task {cfg.task.id!r} runs on {sorted(self.SIMULATORS)}, not on {simulator!r}")
            ]
        return []

    def build(self, cfg: Any, simulator: str) -> Any:
        module, _, cls = self.TASKS[cfg.task.params.name][1].partition(":")
        return getattr(importlib.import_module(module), cls)(cfg)


PROVIDER = ToyTasks()
