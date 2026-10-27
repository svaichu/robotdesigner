"""Task description. Never mentions the number of joints or link lengths."""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any, Mapping

from ._canon import as_bool, as_float, as_int, as_vector, check_keys, within
from .errors import ConfigError


@dataclass(frozen=True, slots=True)
class FixedBox:
    """Targets uniform in an axis-aligned box, in the base frame."""

    low: tuple[float, float, float]
    high: tuple[float, float, float]

    def __post_init__(self):
        low = as_vector(self.low, 3, "low")
        high = as_vector(self.high, 3, "high")
        for i, (a, b) in enumerate(zip(low, high)):
            if a > b:
                raise ConfigError(f"low[{i}]", f"low must be <= high ({a} > {b})")
        object.__setattr__(self, "low", low)
        object.__setattr__(self, "high", high)

    def to_dict(self) -> dict[str, Any]:
        return {"kind": "fixed_box", "low": list(self.low), "high": list(self.high)}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "FixedBox":
        check_keys(d, {"kind", "low", "high"}, "")
        return cls(low=d["low"], high=d["high"])


@dataclass(frozen=True, slots=True)
class OwnReachable:
    """Targets are end-effector positions of uniformly sampled joint configurations."""

    def to_dict(self) -> dict[str, Any]:
        return {"kind": "own_reachable"}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "OwnReachable":
        check_keys(d, {"kind"}, "")
        return cls()


TargetRegion = FixedBox | OwnReachable
_REGIONS = {"fixed_box": FixedBox, "own_reachable": OwnReachable}


def region_from_dict(d: Mapping[str, Any]) -> TargetRegion:
    if not isinstance(d, Mapping) or "kind" not in d:
        raise ConfigError("", "expected a mapping with a 'kind' key")
    cls = _REGIONS.get(d["kind"])
    if cls is None:
        raise ConfigError("kind", f"unknown target region {d['kind']!r}; expected one of {sorted(_REGIONS)}")
    return cls.from_dict(d)


@dataclass(frozen=True, slots=True)
class ReachParams:
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


# Typed parameter class per task id.
TASK_PARAMS: dict[str, type] = {"native/reach": ReachParams}


@dataclass(frozen=True, slots=True)
class TaskConfig:
    id: str
    horizon: int
    success_tolerance: float
    target_region: TargetRegion
    params: Any

    def __post_init__(self):
        params_cls = TASK_PARAMS.get(self.id)
        if params_cls is None:
            raise ConfigError("id", f"unknown task {self.id!r}; known: {sorted(TASK_PARAMS)}")
        horizon = as_int(self.horizon, "horizon")
        if horizon < 1:
            raise ConfigError("horizon", "must be >= 1")
        object.__setattr__(self, "horizon", horizon)
        tol = as_float(self.success_tolerance, "success_tolerance")
        if tol <= 0.0:
            raise ConfigError("success_tolerance", "must be > 0")
        object.__setattr__(self, "success_tolerance", tol)
        if not isinstance(self.target_region, (FixedBox, OwnReachable)):
            raise ConfigError("target_region", "expected FixedBox or OwnReachable")
        if not isinstance(self.params, params_cls):
            raise ConfigError("params", f"task {self.id!r} requires {params_cls.__name__}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "horizon": self.horizon,
            "success_tolerance": self.success_tolerance,
            "target_region": self.target_region.to_dict(),
            "params": self.params.to_dict(),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "TaskConfig":
        check_keys(d, {"id", "horizon", "success_tolerance", "target_region", "params"}, "")
        params_cls = TASK_PARAMS.get(d["id"])
        if params_cls is None:
            raise ConfigError("id", f"unknown task {d['id']!r}; known: {sorted(TASK_PARAMS)}")
        return cls(
            id=d["id"],
            horizon=d["horizon"],
            success_tolerance=d["success_tolerance"],
            target_region=within("target_region", region_from_dict, d["target_region"]),
            params=within("params", params_cls.from_dict, d["params"]),
        )
