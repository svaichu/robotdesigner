"""Task description: an id, an episode length and, for goal-defined tasks, a target region and tolerance.

``TaskConfig.id`` is ``<namespace>/<name>``. The namespace names a task provider (``config.providers``) that owns
the typed ``params``; this module knows no provider and no task. Never mentions the number of joints or link
lengths.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ._canon import as_float, as_int, as_vector, check_keys, within
from .errors import ConfigError
from .providers import known_task_ids, params_class


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
class TaskConfig:
    """``target_region`` and ``success_tolerance`` define the goal of tasks whose ``params`` class sets
    ``uses_goal = True``; they must be ``None`` for tasks that define their own goal.

    The ``params`` class of a task comes from its provider. Besides ``to_dict`` / ``from_dict`` the environment reads
    ``reset_noise`` (rad, uniform noise on the initial joint positions) and ``terminate_on_success`` from it when
    they exist.
    """

    id: str
    horizon: int
    success_tolerance: float | None
    target_region: TargetRegion | None
    params: Any

    def __post_init__(self):
        params_cls = params_class(self.id)
        if params_cls is None:
            raise ConfigError("id", f"unknown task {self.id!r}; known: {known_task_ids()}")
        horizon = as_int(self.horizon, "horizon")
        if horizon < 1:
            raise ConfigError("horizon", "must be >= 1")
        object.__setattr__(self, "horizon", horizon)
        if getattr(params_cls, "uses_goal", False):
            tol = as_float(self.success_tolerance, "success_tolerance")
            if tol <= 0.0:
                raise ConfigError("success_tolerance", "must be > 0")
            object.__setattr__(self, "success_tolerance", tol)
            if not isinstance(self.target_region, (FixedBox, OwnReachable)):
                raise ConfigError("target_region", "expected FixedBox or OwnReachable")
        else:
            if self.success_tolerance is not None:
                raise ConfigError(
                    "success_tolerance", f"must be None for task {self.id!r}, which defines its own success"
                )
            if self.target_region is not None:
                raise ConfigError("target_region", f"must be None for task {self.id!r}, which defines its own goal")
        if not isinstance(self.params, params_cls):
            raise ConfigError("params", f"task {self.id!r} requires {params_cls.__name__}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "horizon": self.horizon,
            "success_tolerance": self.success_tolerance,
            "target_region": None if self.target_region is None else self.target_region.to_dict(),
            "params": self.params.to_dict(),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "TaskConfig":
        check_keys(d, {"id", "horizon", "success_tolerance", "target_region", "params"}, "")
        params_cls = params_class(d["id"])
        if params_cls is None:
            raise ConfigError("id", f"unknown task {d['id']!r}; known: {known_task_ids()}")
        region = d["target_region"]
        return cls(
            id=d["id"],
            horizon=d["horizon"],
            success_tolerance=d["success_tolerance"],
            target_region=None if region is None else within("target_region", region_from_dict, region),
            params=within("params", params_cls.from_dict, d["params"]),
        )
