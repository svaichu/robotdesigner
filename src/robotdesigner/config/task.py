"""Task description. Never mentions the number of joints or link lengths."""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any, ClassVar, Mapping

from ._canon import as_bool, as_float, as_int, as_vector, canonicalize, check_keys, within
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
    uses_goal: ClassVar[bool] = True  # the task is defined by a target region and a success tolerance

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


# Constructor arguments of a robosuite environment that robotdesigner sets itself, from the robot, control,
# sim and task config. They must not appear in ``RobosuiteParams.env_kwargs``.
RESERVED_ENV_KWARGS = frozenset(
    {
        "robots", "controller_configs", "gripper_types", "base_types", "mount_types", "initial_qpos",
        "initialization_noise", "control_freq", "horizon", "hard_reset", "ignore_done", "seed", "lite_physics",
        "has_renderer", "has_offscreen_renderer", "use_camera_obs", "camera_names", "camera_heights",
        "camera_widths", "camera_depths", "camera_segmentations", "renderer", "renderer_config", "render_camera",
        "render_collision_mesh", "render_visual_mesh", "render_gpu_device_id",
    }
)  # fmt: skip


def _freeze(value: Any, path: str) -> Any:
    """JSON-like value as an immutable structure: lists become tuples, mappings become sorted item tuples."""
    if value is None or isinstance(value, (bool, str, int)):
        return value
    if isinstance(value, float):
        return as_float(value, path)
    if isinstance(value, Mapping):
        items = sorted(value.items(), key=lambda kv: str(kv[0]))
        return ("__map__",) + tuple((str(k), _freeze(v, f"{path}.{k}")) for k, v in items)
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(v, f"{path}[{i}]") for i, v in enumerate(value))
    raise ConfigError(
        path,
        f"unsupported value of type {type(value).__name__}; use bool, int, float, str, None, lists or mappings",
    )


def _thaw(value: Any) -> Any:
    if isinstance(value, tuple) and value and value[0] == "__map__":
        return {k: _thaw(v) for k, v in value[1:]}
    if isinstance(value, tuple):
        return tuple(_thaw(v) for v in value)
    return value


@dataclass(frozen=True, slots=True)
class RobosuiteParams:
    """Parameters of any ``robosuite/<EnvName>`` task.

    ``env_kwargs`` are passed to the robosuite environment constructor (``reward_shaping``, ``table_full_size``,
    ...). Everything not listed there takes robosuite's own default, which the environment records at build time
    (``resolved_env_kwargs``); it is not part of the config hash. The robot, controller, gripper, control frequency,
    horizon, rendering and seed are set from the other config sections and cannot be given here.
    """

    uses_goal: ClassVar[bool] = False  # robosuite defines success and reward itself

    env_kwargs: Any
    reset_noise: float
    terminate_on_success: bool

    def __post_init__(self):
        raw = self.env_kwargs
        if isinstance(raw, tuple) and raw and raw[0] == "__map__":
            raw = _thaw(raw)
        if not isinstance(raw, Mapping):
            raise ConfigError("env_kwargs", f"expected a mapping, got {type(raw).__name__}")
        reserved = sorted(set(raw) & RESERVED_ENV_KWARGS)
        if reserved:
            raise ConfigError(
                "env_kwargs", f"{reserved} are set from the robot, control, sim and task config; remove them"
            )
        for key in raw:
            if not isinstance(key, str) or not key.isidentifier():
                raise ConfigError("env_kwargs", f"keys must be argument names, got {key!r}")
        object.__setattr__(self, "env_kwargs", _freeze(dict(raw), "env_kwargs"))
        noise = as_float(self.reset_noise, "reset_noise")
        if noise < 0.0:
            raise ConfigError("reset_noise", "must be >= 0")
        object.__setattr__(self, "reset_noise", noise)
        as_bool(self.terminate_on_success, "terminate_on_success")

    @property
    def kwargs(self) -> dict[str, Any]:
        """The environment keyword arguments as a plain dict."""
        return _thaw(self.env_kwargs)

    def to_dict(self) -> dict[str, Any]:
        return {
            "env_kwargs": canonicalize(self.kwargs),
            "reset_noise": self.reset_noise,
            "terminate_on_success": self.terminate_on_success,
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "RobosuiteParams":
        check_keys(d, {"env_kwargs", "reset_noise", "terminate_on_success"}, "")
        return cls(**d)


# Typed parameter class per task id; ids with a registered prefix (``robosuite/<EnvName>``) share one class.
TASK_PARAMS: dict[str, type] = {"native/reach": ReachParams}
TASK_PREFIX_PARAMS: dict[str, type] = {"robosuite/": RobosuiteParams}


def params_class(task_id: str) -> type | None:
    """The parameter class of ``task_id``, or None when the id is unknown."""
    if task_id in TASK_PARAMS:
        return TASK_PARAMS[task_id]
    for prefix, cls in TASK_PREFIX_PARAMS.items():
        if task_id.startswith(prefix) and len(task_id) > len(prefix):
            return cls
    return None


def _known_ids() -> list[str]:
    return sorted(TASK_PARAMS) + [f"{p}<name>" for p in sorted(TASK_PREFIX_PARAMS)]


@dataclass(frozen=True, slots=True)
class TaskConfig:
    """``target_region`` and ``success_tolerance`` define the goal of tasks like ``native/reach``; they must be
    ``None`` for tasks that define their own goal (every ``robosuite/<EnvName>``)."""

    id: str
    horizon: int
    success_tolerance: float | None
    target_region: TargetRegion | None
    params: Any

    def __post_init__(self):
        params_cls = params_class(self.id) if isinstance(self.id, str) else None
        if params_cls is None:
            raise ConfigError("id", f"unknown task {self.id!r}; known: {_known_ids()}")
        horizon = as_int(self.horizon, "horizon")
        if horizon < 1:
            raise ConfigError("horizon", "must be >= 1")
        object.__setattr__(self, "horizon", horizon)
        if params_cls.uses_goal:
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
        params_cls = params_class(d["id"]) if isinstance(d["id"], str) else None
        if params_cls is None:
            raise ConfigError("id", f"unknown task {d['id']!r}; known: {_known_ids()}")
        region = d["target_region"]
        return cls(
            id=d["id"],
            horizon=d["horizon"],
            success_tolerance=d["success_tolerance"],
            target_region=None if region is None else within("target_region", region_from_dict, region),
            params=within("params", params_cls.from_dict, d["params"]),
        )
