"""Control interface configuration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ._canon import as_float, check_keys
from .errors import ConfigError

CONTROL_MODES = (
    "joint_pos_abs",
    "joint_pos_delta",
    "joint_torque",
    "joint_vel",
    "ee_pos_delta",
    "ee_pose_delta",
)
_SCALED_MODES = frozenset({"joint_pos_delta", "joint_vel"})


def _gain(x: Any, name: str) -> float | tuple[float, ...]:
    try:
        items = tuple(x)
    except TypeError:
        v = as_float(x, name)
        if v < 0.0:
            raise ConfigError(name, "must be >= 0") from None
        return v
    out = tuple(as_float(v, f"{name}[{i}]") for i, v in enumerate(items))
    if not out:
        raise ConfigError(name, "must not be empty")
    for i, v in enumerate(out):
        if v < 0.0:
            raise ConfigError(f"{name}[{i}]", "must be >= 0")
    return out


@dataclass(frozen=True, slots=True)
class FixedGains:
    """Actuator gains. Scalars apply to every joint; sequences are per joint."""

    kp: float | tuple[float, ...]
    kd: float | tuple[float, ...]

    def __post_init__(self):
        object.__setattr__(self, "kp", _gain(self.kp, "kp"))
        object.__setattr__(self, "kd", _gain(self.kd, "kd"))

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": "fixed",
            "kp": list(self.kp) if isinstance(self.kp, tuple) else self.kp,
            "kd": list(self.kd) if isinstance(self.kd, tuple) else self.kd,
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "FixedGains":
        check_keys(d, {"kind", "kp", "kd"}, "")
        if d["kind"] != "fixed":
            raise ConfigError("kind", f"unsupported gains {d['kind']!r}")
        return cls(kp=d["kp"], kd=d["kd"])


@dataclass(frozen=True, slots=True)
class ControlConfig:
    mode: str
    gains: FixedGains
    action_scale: float | None

    def __post_init__(self):
        if self.mode not in CONTROL_MODES:
            raise ConfigError("mode", f"unknown mode {self.mode!r}; expected one of {list(CONTROL_MODES)}")
        if not isinstance(self.gains, FixedGains):
            raise ConfigError("gains", "expected FixedGains")
        if self.mode in _SCALED_MODES:
            if self.action_scale is None:
                raise ConfigError("action_scale", f"required for mode {self.mode!r}")
            scale = as_float(self.action_scale, "action_scale")
            if scale <= 0.0:
                raise ConfigError("action_scale", "must be > 0")
            object.__setattr__(self, "action_scale", scale)
        elif self.action_scale is not None:
            raise ConfigError("action_scale", f"must be None for mode {self.mode!r}")

    def to_dict(self) -> dict[str, Any]:
        return {"mode": self.mode, "gains": self.gains.to_dict(), "action_scale": self.action_scale}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "ControlConfig":
        from ._canon import within

        check_keys(d, {"mode", "gains", "action_scale"}, "")
        return cls(
            mode=d["mode"],
            gains=within("gains", FixedGains.from_dict, d["gains"]),
            action_scale=d["action_scale"],
        )
