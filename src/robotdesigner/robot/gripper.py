"""A parametric two-finger parallel gripper. numpy only; the MuJoCo simulator builds it (``simulators/mujoco/arm_spec.py``)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .._canon import as_float, check_keys, within
from ..errors import ConfigError
from .pose import Pose


@dataclass(frozen=True, slots=True)
class TwoFingerGripper:
    """Two capsule fingers on a palm, mounted on the arm's end effector.

    Gripper frame: ``mount`` relative to the end-effector frame; fingers point along +x and open along ±y. The palm is a
    box ``palm_length`` long (x); the fingers start at its far end. Each finger slides on a prismatic joint from ``0``
    (closed: the finger surfaces touch) to ``stroke`` (open); the two joints are coupled, so the gripper has one degree of
    freedom, the opening ``w`` in ``[0, 2 * stroke]`` (the gap between the finger surfaces). The simulator drives it
    with one position servo (``kp``, ``kd``, ``force_limit`` on the opening) and the env adds one action dimension.
    ``tcp_offset`` of the tool-centre point from the gripper frame is ``(palm_length + finger_length, 0, 0)``.
    """

    finger_length: float
    finger_radius: float
    stroke: float
    finger_mass: float
    palm_length: float
    palm_mass: float
    kp: float
    kd: float
    force_limit: float
    mount: Pose

    _POSITIVE = (
        "finger_length", "finger_radius", "stroke", "finger_mass", "palm_length", "palm_mass", "force_limit",
    )

    def __post_init__(self):
        for name in self._POSITIVE:
            v = as_float(getattr(self, name), name)
            if v <= 0.0:
                raise ConfigError(name, "must be > 0")
            object.__setattr__(self, name, v)
        for name in ("kp", "kd"):
            v = as_float(getattr(self, name), name)
            if v < 0.0:
                raise ConfigError(name, "must be >= 0")
            object.__setattr__(self, name, v)
        if not isinstance(self.mount, Pose):
            raise ConfigError("mount", "expected a Pose")

    @classmethod
    def default(cls, **overrides: Any) -> "TwoFingerGripper":
        """A small gripper (6 cm fingers, 4 cm stroke per finger); keyword arguments replace fields."""
        values: dict[str, Any] = dict(
            finger_length=0.06, finger_radius=0.006, stroke=0.04, finger_mass=0.03, palm_length=0.03,
            palm_mass=0.1, kp=200.0, kd=10.0, force_limit=20.0, mount=Pose.identity(),
        )
        values.update(overrides)
        return cls(**values)

    @property
    def max_opening(self) -> float:
        """The largest gap between the finger surfaces, metres."""
        return 2.0 * self.stroke

    @property
    def tcp_offset(self) -> tuple[float, float, float]:
        return (self.palm_length + self.finger_length, 0.0, 0.0)

    def to_dict(self) -> dict[str, Any]:
        d = {k: getattr(self, k) for k in (*self._POSITIVE, "kp", "kd")}
        d["mount"] = self.mount.to_dict()
        return d

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "TwoFingerGripper":
        check_keys(d, {*cls._POSITIVE, "kp", "kd", "mount"}, "")
        values = {k: d[k] for k in d if k != "mount"}
        return cls(mount=within("mount", Pose.from_dict, d["mount"]), **values)
