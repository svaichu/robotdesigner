"""Simulation settings, including the options of the chosen simulator. Nothing here has a default value.

``SimConfig.simulator`` names the simulator (a physics engine, for example ``"mujoco"``), never a source of tasks;
tasks come from task providers (``config.providers``). ``SimConfig.options`` holds the settings that exist for that
simulator only, as an instance of the options class registered for its name.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, fields
from typing import Any, Mapping

from ._canon import as_bool, as_float, as_int, as_str, check_keys, within
from .errors import ConfigError

INTEGRATORS = ("euler", "implicit", "implicitfast", "rk4")
_DEVICE = re.compile(r"cpu|cuda:\d+")


@dataclass(frozen=True, slots=True)
class MujocoOptions:
    self_collision: bool

    def __post_init__(self):
        as_bool(self.self_collision, "self_collision")

    @classmethod
    def default(cls) -> "MujocoOptions":
        """The options ``BaseEnvConfig`` uses (an optional hook of every options class)."""
        return cls(self_collision=False)

    def to_dict(self) -> dict[str, Any]:
        return {f.name: getattr(self, f.name) for f in fields(self)}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "MujocoOptions":
        check_keys(d, {f.name for f in fields(cls)}, "")
        return cls(**d)


# One typed options class per simulator name. Simulators outside this package may register theirs.
SIMULATOR_OPTIONS: dict[str, type] = {"mujoco": MujocoOptions}


def register_simulator_options(simulator: str, cls: type) -> None:
    """Declare the options class of a simulator. It needs ``to_dict`` and ``from_dict``, and may have a ``default()``
    classmethod: ``BaseEnvConfig.sim(simulator=...)`` uses it to reset the options when the simulator changes."""
    SIMULATOR_OPTIONS[simulator] = cls


def _options_class(simulator: str) -> type:
    cls = SIMULATOR_OPTIONS.get(simulator)
    if cls is None:
        raise ConfigError(
            "simulator", f"no options registered for simulator {simulator!r}; known: {sorted(SIMULATOR_OPTIONS)}"
        )
    return cls


@dataclass(frozen=True, slots=True)
class SimConfig:
    simulator: str
    num_envs: int
    device: str
    dt: float
    integrator: str
    control_freq: float
    seed: int
    scene: str
    options: Any

    def __post_init__(self):
        as_str(self.simulator, "simulator")
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
        options_cls = _options_class(self.simulator)
        if not isinstance(self.options, options_cls):
            raise ConfigError("options", f"simulator {self.simulator!r} requires {options_cls.__name__}")

    @property
    def n_substeps(self) -> int:
        return round(1.0 / (self.control_freq * self.dt))

    def to_dict(self) -> dict[str, Any]:
        d = {f.name: getattr(self, f.name) for f in fields(self) if f.name != "options"}
        d["options"] = self.options.to_dict()
        return d

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "SimConfig":
        check_keys(d, {f.name for f in fields(cls)}, "")
        options_cls = _options_class(as_str(d["simulator"], "simulator"))
        rest = {k: v for k, v in d.items() if k != "options"}
        return cls(**rest, options=within("options", options_cls.from_dict, d["options"]))
