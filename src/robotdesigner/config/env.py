"""The complete, declarative description of one environment, with one hash."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .._canon import HASH_DISPLAY_LENGTH, canonical_json, check_keys, sha256_hex, within
from .control import NATIVE_MODES, ControlConfig, NativeGains
from ..errors import ConfigError
from ..robot import ExternalRobot, ParametricArm, robot_from_dict
from .mount import check_mount_fits_robot
from .sim import SimConfig
from .task import TaskConfig


@dataclass(frozen=True, slots=True)
class EnvConfig:
    robot: ParametricArm | ExternalRobot
    control: ControlConfig
    task: TaskConfig
    sim: SimConfig

    def __post_init__(self):
        for name, cls in (
            ("robot", (ParametricArm, ExternalRobot)),
            ("control", ControlConfig),
            ("task", TaskConfig),
            ("sim", SimConfig),
        ):
            if not isinstance(getattr(self, name), cls):
                names = " or ".join(c.__name__ for c in (cls if isinstance(cls, tuple) else (cls,)))
                raise ConfigError(name, f"expected {names}")

        n = self.robot.n
        for name in ("kp", "kd"):
            v = getattr(self.control.gains, name, None)
            if isinstance(v, tuple) and len(v) != n:
                raise ConfigError(f"control.gains.{name}", f"expected {n} per-joint values, got {len(v)}")
        self._check_gains_against_robot()
        check_mount_fits_robot(self.task.mount, self.robot)

    def _check_gains_against_robot(self) -> None:
        gains = self.control.gains
        if isinstance(self.robot, ExternalRobot):
            # FixedGains and InertiaScaledGains replace the model's actuators with ours, so any mode fits; NativeGains
            # drives the model's own actuators, so the mode must fit their kind.
            if isinstance(gains, NativeGains):
                fits = NATIVE_MODES[self.robot.actuator_kind]
                if self.control.mode not in fits:
                    hint = (
                        f"the modes that fit are {list(fits)}" if fits else
                        "no mode fits: restrict robot.joints to joints with one kind of actuator, or use FixedGains"
                    )
                    raise ConfigError(
                        "control.mode",
                        f"mode {self.control.mode!r} does not fit the model's {self.robot.actuator_kind!r} actuators; {hint}",
                    )
        elif isinstance(gains, NativeGains):
            raise ConfigError("control.gains", "NativeGains drives a model's own actuators: it needs an ExternalRobot")

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
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "EnvConfig":
        check_keys(d, {"robot", "control", "task", "sim"}, "")
        return cls(
            robot=within("robot", robot_from_dict, d["robot"]),
            control=within("control", ControlConfig.from_dict, d["control"]),
            task=within("task", TaskConfig.from_dict, d["task"]),
            sim=within("sim", SimConfig.from_dict, d["sim"]),
        )
