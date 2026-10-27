"""The complete, declarative description of one environment, with one hash."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ._canon import HASH_DISPLAY_LENGTH, canonical_json, check_keys, sha256_hex, within
from .control import ControlConfig
from .errors import ConfigError
from .robot import ParametricArm
from .sim import BACKEND_OPTIONS, SimConfig
from .task import TaskConfig


@dataclass(frozen=True, slots=True)
class EnvConfig:
    robot: ParametricArm
    control: ControlConfig
    task: TaskConfig
    sim: SimConfig
    backend_options: Any

    def __post_init__(self):
        for name, cls in (
            ("robot", ParametricArm),
            ("control", ControlConfig),
            ("task", TaskConfig),
            ("sim", SimConfig),
        ):
            if not isinstance(getattr(self, name), cls):
                raise ConfigError(name, f"expected {cls.__name__}")

        options_cls = BACKEND_OPTIONS.get(self.sim.backend)
        if options_cls is None:
            raise ConfigError(
                "sim.backend", f"no options registered for backend {self.sim.backend!r}; known: {sorted(BACKEND_OPTIONS)}"
            )
        if not isinstance(self.backend_options, options_cls):
            raise ConfigError(
                "backend_options", f"backend {self.sim.backend!r} requires {options_cls.__name__}"
            )

        n = self.robot.n
        for name in ("kp", "kd"):
            v = getattr(self.control.gains, name, None)
            if isinstance(v, tuple) and len(v) != n:
                raise ConfigError(f"control.gains.{name}", f"expected {n} per-joint values, got {len(v)}")

    # --- resolution and hashing ----------------------------------------------------

    def resolve(self) -> "EnvConfig":
        """A concrete config with region policies resolved into numbers. Idempotent.

        Nothing needs resolving yet; later policies (family-reachable regions) will be turned into
        numbers here, before hashing.
        """
        return self

    def hash_dict(self) -> dict[str, Any]:
        r = self.resolve()
        return {
            "robot_hash": r.robot.full_hash,
            "control": r.control.to_dict(),
            "task": r.task.to_dict(),
            "sim": r.sim.to_dict(),
            "backend_options": r.backend_options.to_dict(),
        }

    @property
    def full_env_hash(self) -> str:
        return sha256_hex(canonical_json(self.hash_dict()))

    @property
    def env_hash(self) -> str:
        return self.full_env_hash[:HASH_DISPLAY_LENGTH]

    # --- serialization -------------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        return {
            "robot": self.robot.to_dict(),
            "control": self.control.to_dict(),
            "task": self.task.to_dict(),
            "sim": self.sim.to_dict(),
            "backend_options": self.backend_options.to_dict(),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "EnvConfig":
        check_keys(d, {"robot", "control", "task", "sim", "backend_options"}, "")
        sim = within("sim", SimConfig.from_dict, d["sim"])
        options_cls = BACKEND_OPTIONS.get(sim.backend)
        if options_cls is None:
            raise ConfigError(
                "sim.backend", f"no options registered for backend {sim.backend!r}; known: {sorted(BACKEND_OPTIONS)}"
            )
        return cls(
            robot=within("robot", ParametricArm.from_dict, d["robot"]),
            control=within("control", ControlConfig.from_dict, d["control"]),
            task=within("task", TaskConfig.from_dict, d["task"]),
            sim=sim,
            backend_options=within("backend_options", options_cls.from_dict, d["backend_options"]),
        )
