"""Parameters of ``empty/idle``. numpy only."""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any, ClassVar, Mapping

from ...config._canon import check_keys
from ...config.errors import ConfigError

TASKS = ("idle",)


@dataclass(frozen=True, slots=True)
class EmptyParams:
    """The placeholder task: the robot in an empty world with nothing to do. ``name`` is ``"idle"``."""

    provider: ClassVar[str] = "empty"
    uses_goal: ClassVar[bool] = False  # no target region, no tolerance

    name: str

    def __post_init__(self):
        if self.name not in TASKS:
            raise ConfigError("name", f"unknown task {self.name!r}; known: {list(TASKS)}")

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "EmptyParams":
        check_keys(d, {f.name for f in fields(cls)}, "")
        return cls(name=d["name"])
