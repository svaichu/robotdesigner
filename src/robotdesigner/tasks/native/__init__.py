"""Provider of the tasks that need no third-party assets. Importing this package imports no engine."""

from __future__ import annotations

import importlib
from typing import Any

from ...config.errors import Problem
from .params import ReachParams

__all__ = ["PROVIDER", "NativeTasks", "ReachParams"]


class NativeTasks:
    """name -> (params class, ``"module:Class"`` of the engine side, imported on build)."""

    TASKS: dict[str, tuple[type, str]] = {
        "reach": (ReachParams, "robotdesigner.tasks.native.reach:ReachTask"),
    }
    BACKENDS = frozenset({"mujoco"})

    def task_names(self) -> tuple[str, ...]:
        return tuple(self.TASKS)

    def params_class(self, name: str) -> type | None:
        entry = self.TASKS.get(name)
        return None if entry is None else entry[0]

    def validate(self, cfg: Any, backend: str) -> list[Problem]:
        if backend not in self.BACKENDS:
            return [
                Problem("sim.backend", f"task {cfg.task.id!r} runs on {sorted(self.BACKENDS)}, not on {backend!r}")
            ]
        return []

    def build(self, cfg: Any, backend: str) -> Any:
        module, _, cls = self.TASKS[cfg.task.id.split("/", 1)[1]][1].partition(":")
        return getattr(importlib.import_module(module), cls)(cfg)


PROVIDER = NativeTasks()
