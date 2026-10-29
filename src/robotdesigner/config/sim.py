"""Simulation settings and per-backend options. Nothing here has a default value."""

from __future__ import annotations

import re
from dataclasses import dataclass, fields
from typing import Any, Mapping

from ._canon import as_bool, as_float, as_int, as_str, check_keys
from .errors import ConfigError

INTEGRATORS = ("euler", "implicit", "implicitfast", "rk4")
_DEVICE = re.compile(r"cpu|cuda:\d+")


@dataclass(frozen=True, slots=True)
class SimConfig:
    backend: str
    num_envs: int
    device: str
    dt: float
    integrator: str
    control_freq: float
    seed: int
    scene: str

    def __post_init__(self):
        as_str(self.backend, "backend")
        as_str(self.scene, "scene")
        n = as_int(self.num_envs, "num_envs")
        if n < 1:
            raise ConfigError("num_envs", "must be >= 1")
        object.__setattr__(self, "num_envs", n)
        if not isinstance(self.device, str) or not _DEVICE.fullmatch(self.device):
            raise ConfigError("device", f"must be 'cpu' or 'cuda:N', got {self.device!r}")
        if self.integrator not in INTEGRATORS:
            raise ConfigError("integrator", f"unknown {self.integrator!r}; expected one of {list(INTEGRATORS)}")
        dt = as_float(self.dt, "dt")
        if dt <= 0.0:
            raise ConfigError("dt", "must be > 0")
        object.__setattr__(self, "dt", dt)
        freq = as_float(self.control_freq, "control_freq")
        if freq <= 0.0:
            raise ConfigError("control_freq", "must be > 0")
        object.__setattr__(self, "control_freq", freq)
        substeps = 1.0 / (freq * dt)
        if substeps < 1.0 - 1e-9 or abs(substeps - round(substeps)) > 1e-9:
            raise ConfigError(
                "control_freq",
                f"1 / (control_freq * dt) must be a positive integer, got {substeps:.9g} "
                f"(control_freq={freq}, dt={dt})",
            )
        object.__setattr__(self, "seed", as_int(self.seed, "seed"))

    @property
    def n_substeps(self) -> int:
        return round(1.0 / (self.control_freq * self.dt))

    def to_dict(self) -> dict[str, Any]:
        return {f.name: getattr(self, f.name) for f in fields(self)}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "SimConfig":
        check_keys(d, {f.name for f in fields(cls)}, "")
        return cls(**d)


@dataclass(frozen=True, slots=True)
class MujocoOptions:
    self_collision: bool

    def __post_init__(self):
        as_bool(self.self_collision, "self_collision")

    def to_dict(self) -> dict[str, Any]:
        return {f.name: getattr(self, f.name) for f in fields(self)}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "MujocoOptions":
        check_keys(d, {f.name for f in fields(cls)}, "")
        return cls(**d)


BASE_PLACEMENTS = ("env_default", "explicit")


@dataclass(frozen=True, slots=True)
class RobosuiteOptions:
    """Options of the robosuite backend.

    ``base_placement`` decides where the robot goes. ``"env_default"``: the robot root sits where the robosuite
    environment mounts a stock arm (its ``base_xpos_offset`` rule) and ``robot.base_pose`` is relative to that
    point, for example the shoulder height above it. ``"explicit"``: the robot root sits at the arena origin and
    ``robot.base_pose`` is the pose of the arm base in the arena frame.
    """

    base_placement: str

    def __post_init__(self):
        if self.base_placement not in BASE_PLACEMENTS:
            raise ConfigError(
                "base_placement", f"unknown {self.base_placement!r}; expected one of {list(BASE_PLACEMENTS)}"
            )

    def to_dict(self) -> dict[str, Any]:
        return {f.name: getattr(self, f.name) for f in fields(self)}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "RobosuiteOptions":
        check_keys(d, {f.name for f in fields(cls)}, "")
        return cls(**d)


# One typed options class per backend name. Backends outside this package may register theirs.
BACKEND_OPTIONS: dict[str, type] = {"mujoco": MujocoOptions, "robosuite": RobosuiteOptions}


def register_backend_options(backend: str, cls: type) -> None:
    """Declare the options class of a backend. It needs ``to_dict`` and ``from_dict``."""
    BACKEND_OPTIONS[backend] = cls
