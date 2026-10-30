"""Parameters of the tasks in the ``toy`` namespace (test-only provider). numpy only."""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any, ClassVar, Mapping

from robotdesigner.config._canon import as_bool, as_float, check_keys
from robotdesigner.config.errors import ConfigError


@dataclass(frozen=True, slots=True)
class ReachParams:
    provider: ClassVar[str] = "toy"
    uses_goal: ClassVar[bool] = True  # the task is defined by a target region and a success tolerance

    name: str
    reset_noise: float
    terminate_on_success: bool

    def __post_init__(self):
        noise = as_float(self.reset_noise, "reset_noise")
        if noise < 0.0:
            raise ConfigError("reset_noise", "must be >= 0")
        object.__setattr__(self, "reset_noise", noise)
        as_bool(self.terminate_on_success, "terminate_on_success")

    def to_dict(self) -> dict[str, Any]:
        return {f.name: getattr(self, f.name) for f in fields(self)}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "ReachParams":
        check_keys(d, {f.name for f in fields(cls)}, "")
        return cls(**d)
