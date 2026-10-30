"""Task description: the provider's params object, an episode length and, for goal-defined tasks, a target region
and tolerance.

The ``params`` object is the task: its class names the provider (class attribute ``provider``, a namespace of
``config.providers``) and its ``name`` field names the task within it. ``TaskConfig.id`` is the derived
``<provider>/<name>``. This module knows no provider and no task. Never mentions the number of joints or link lengths.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ._canon import as_float, as_int, as_vector, check_keys, within
from .errors import ConfigError
from .robot import Pose
from .providers import get_task_provider, known_task_ids, list_task_providers


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

    ``params`` is an instance of the provider's params class: that class has a class attribute ``provider`` (the
    namespace) and a field ``name`` (the task within the provider). Besides ``to_dict`` / ``from_dict`` the
    environment reads ``reset_noise`` (rad, uniform noise on the initial joint positions) and
    ``terminate_on_success`` from it when they exist.

    ``mount`` is the pose of the robot base in the world: where the scene expects the arm
    (``TaskProvider.default_mount(name)``). The robot itself has no pose, so ``robot_hash`` identifies the embodiment
    alone. Target regions are in the robot's base frame and follow the mount.
    """

    params: Any
    horizon: int
    success_tolerance: float | None
    target_region: TargetRegion | None
    mount: Pose

    def __post_init__(self):
        provider = getattr(self.params, "provider", None)
        name = getattr(self.params, "name", None)
        if not isinstance(provider, str) or not isinstance(name, str):
            raise ConfigError(
                "params",
                f"expected a task provider's params object (a class attribute 'provider' and a field 'name'), "
                f"got {type(self.params).__name__}",
            )
        params_cls = _params_class(provider, name, "params", "params.name")
        if not isinstance(self.params, params_cls):
            raise ConfigError("params", f"task {self.id!r} requires {params_cls.__name__}")
        if not isinstance(self.mount, Pose):
            raise ConfigError("mount", "expected a Pose: the robot base in the world")
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

    @property
    def id(self) -> str:
        """``"<provider>/<name>"``, for messages and logs; derived from ``params``, never stored."""
        return f"{self.params.provider}/{self.params.name}"

    def to_dict(self) -> dict[str, Any]:
        return {
            "provider": self.params.provider,
            "params": self.params.to_dict(),
            "horizon": self.horizon,
            "success_tolerance": self.success_tolerance,
            "target_region": None if self.target_region is None else self.target_region.to_dict(),
            "mount": self.mount.to_dict(),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "TaskConfig":
        check_keys(d, {"provider", "params", "horizon", "success_tolerance", "target_region", "mount"}, "")
        raw = d["params"]
        if not isinstance(raw, Mapping):
            raise ConfigError("params", f"expected a mapping, got {type(raw).__name__}")
        if "name" not in raw:
            raise ConfigError("params.name", "missing")
        params_cls = _params_class(d["provider"], raw["name"], "provider", "params.name")
        region = d["target_region"]
        return cls(
            params=within("params", params_cls.from_dict, raw),
            horizon=d["horizon"],
            success_tolerance=d["success_tolerance"],
            target_region=None if region is None else within("target_region", region_from_dict, region),
            mount=within("mount", Pose.from_dict, d["mount"]),
        )


def _params_class(provider: str, name: str, provider_path: str, name_path: str | None = None) -> type:
    """The params class of task ``name`` of ``provider``; errors say where the bad value was written."""
    name_path = name_path or provider_path
    if not isinstance(provider, str) or provider not in list_task_providers():
        raise ConfigError(provider_path, f"unknown task provider {provider!r}; registered: {list(list_task_providers())}")
    cls = get_task_provider(provider).params_class(name) if isinstance(name, str) else None
    if cls is None:
        raise ConfigError(name_path, f"unknown task '{provider}/{name}'; known: {known_task_ids()}")
    return cls
