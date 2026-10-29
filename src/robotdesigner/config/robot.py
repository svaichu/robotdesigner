"""Robot embodiments as frozen, validated, hashable data. No simulator is imported here."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

from ._canon import (
    HASH_DISPLAY_LENGTH,
    as_float,
    as_vector,
    canonical_json,
    check_keys,
    sha256_hex,
    within,
)
from .errors import ConfigError

SUPPORTED_SCHEMA_VERSIONS = frozenset({1})


def _normalize_quat(q: tuple[float, ...], path: str) -> tuple[float, ...]:
    norm = math.sqrt(sum(c * c for c in q))
    if norm == 0.0:
        raise ConfigError(path, "must be non-zero")
    q = tuple(c / norm for c in q)
    # q and -q are the same rotation: make the first non-zero component positive.
    for c in q:
        if c != 0.0:
            if c < 0.0:
                q = tuple(-v for v in q)
            break
    return q


@dataclass(frozen=True, slots=True)
class Pose:
    """Position (m) and orientation as a quaternion ``(w, x, y, z)``."""

    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]

    def __post_init__(self):
        object.__setattr__(self, "pos", as_vector(self.pos, 3, "pos"))
        quat = as_vector(self.quat, 4, "quat")
        object.__setattr__(self, "quat", _normalize_quat(quat, "quat"))

    @classmethod
    def identity(cls) -> "Pose":
        return cls(pos=(0.0, 0.0, 0.0), quat=(1.0, 0.0, 0.0, 0.0))

    def to_dict(self) -> dict[str, Any]:
        return {"pos": list(self.pos), "quat": list(self.quat)}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "Pose":
        check_keys(d, {"pos", "quat"}, "")
        return cls(pos=d["pos"], quat=d["quat"])


@dataclass(frozen=True, slots=True)
class Capsule:
    """Capsule running from the link origin to ``tip_offset``."""

    radius: float

    def __post_init__(self):
        r = as_float(self.radius, "radius")
        if r <= 0.0:
            raise ConfigError("radius", "must be > 0")
        object.__setattr__(self, "radius", r)

    def to_dict(self) -> dict[str, Any]:
        return {"kind": "capsule", "radius": self.radius}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "Capsule":
        check_keys(d, {"kind", "radius"}, "")
        if d["kind"] != "capsule":
            raise ConfigError("kind", f"unsupported geometry {d['kind']!r}")
        return cls(radius=d["radius"])


@dataclass(frozen=True, slots=True)
class JointSpec:
    """Hinge joint. ``axis`` is in the joint frame; ``range`` is ``None`` (unlimited) or ``(lo, hi)`` in rad."""

    axis: tuple[float, float, float]
    range: tuple[float, float] | None
    damping: float
    armature: float
    frictionloss: float
    effort_limit: float

    def __post_init__(self):
        axis = as_vector(self.axis, 3, "axis")
        norm = math.sqrt(sum(c * c for c in axis))
        if norm == 0.0:
            raise ConfigError("axis", "must be non-zero")
        object.__setattr__(self, "axis", tuple(c / norm for c in axis))

        if self.range is not None:
            lo, hi = as_vector(self.range, 2, "range")
            if not lo < hi:
                raise ConfigError("range", "lo must be < hi")
            object.__setattr__(self, "range", (lo, hi))

        for name in ("damping", "armature", "frictionloss"):
            v = as_float(getattr(self, name), name)
            if v < 0.0:
                raise ConfigError(name, "must be >= 0")
            object.__setattr__(self, name, v)
        effort = as_float(self.effort_limit, "effort_limit")
        if effort <= 0.0:
            raise ConfigError("effort_limit", "must be > 0")
        object.__setattr__(self, "effort_limit", effort)

    def to_dict(self) -> dict[str, Any]:
        return {
            "axis": list(self.axis),
            "range": None if self.range is None else list(self.range),
            "damping": self.damping,
            "armature": self.armature,
            "frictionloss": self.frictionloss,
            "effort_limit": self.effort_limit,
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "JointSpec":
        check_keys(d, {"axis", "range", "damping", "armature", "frictionloss", "effort_limit"}, "")
        return cls(**{k: d[k] for k in d})


@dataclass(frozen=True, slots=True)
class LinkSpec:
    """A rigid link.

    ``joint_frame`` is the pose of the joint frame relative to the previous link's end frame (the
    base frame for the first segment). ``tip_offset`` is the position of the link's end frame in the
    link frame; the end frame has the link frame's orientation. ``inertia`` is ``None`` (derived from
    geometry and mass) or the three principal moments about the centre of mass.
    """

    joint_frame: Pose
    tip_offset: tuple[float, float, float]
    mass: float
    geometry: Capsule
    inertia: tuple[float, float, float] | None

    def __post_init__(self):
        if not isinstance(self.joint_frame, Pose):
            raise ConfigError("joint_frame", "expected a Pose")
        object.__setattr__(self, "tip_offset", as_vector(self.tip_offset, 3, "tip_offset"))
        m = as_float(self.mass, "mass")
        if m <= 0.0:
            raise ConfigError("mass", "must be > 0")
        object.__setattr__(self, "mass", m)
        if not isinstance(self.geometry, Capsule):
            raise ConfigError("geometry", "expected a Capsule")
        if self.inertia is not None:
            inertia = as_vector(self.inertia, 3, "inertia")
            if any(v <= 0.0 for v in inertia):
                raise ConfigError("inertia", "principal moments must be > 0")
            object.__setattr__(self, "inertia", inertia)

    def to_dict(self) -> dict[str, Any]:
        return {
            "joint_frame": self.joint_frame.to_dict(),
            "tip_offset": list(self.tip_offset),
            "mass": self.mass,
            "geometry": self.geometry.to_dict(),
            "inertia": None if self.inertia is None else list(self.inertia),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "LinkSpec":
        check_keys(d, {"joint_frame", "tip_offset", "mass", "geometry", "inertia"}, "")
        return cls(
            joint_frame=within("joint_frame", Pose.from_dict, d["joint_frame"]),
            tip_offset=d["tip_offset"],
            mass=d["mass"],
            geometry=within("geometry", Capsule.from_dict, d["geometry"]),
            inertia=d["inertia"],
        )


@dataclass(frozen=True, slots=True)
class Segment:
    joint: JointSpec
    link: LinkSpec

    def __post_init__(self):
        if not isinstance(self.joint, JointSpec):
            raise ConfigError("joint", "expected a JointSpec")
        if not isinstance(self.link, LinkSpec):
            raise ConfigError("link", "expected a LinkSpec")

    def to_dict(self) -> dict[str, Any]:
        return {"joint": self.joint.to_dict(), "link": self.link.to_dict()}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "Segment":
        check_keys(d, {"joint", "link"}, "")
        return cls(
            joint=within("joint", JointSpec.from_dict, d["joint"]),
            link=within("link", LinkSpec.from_dict, d["link"]),
        )


@dataclass(frozen=True, slots=True)
class ToolSpec:
    """A gripper mounted on the end effector.

    Stored and hashed by every backend. Only the robosuite backend uses it (the gripper and its mount offsets); the
    native MuJoCo backend ignores it.
    """

    gripper: str
    mount_pos: tuple[float, float, float]
    mount_quat: tuple[float, float, float, float]

    def __post_init__(self):
        if not isinstance(self.gripper, str) or not self.gripper:
            raise ConfigError("gripper", "must be a non-empty string")
        object.__setattr__(self, "mount_pos", as_vector(self.mount_pos, 3, "mount_pos"))
        quat = as_vector(self.mount_quat, 4, "mount_quat")
        object.__setattr__(self, "mount_quat", _normalize_quat(quat, "mount_quat"))

    def to_dict(self) -> dict[str, Any]:
        return {
            "gripper": self.gripper,
            "mount_pos": list(self.mount_pos),
            "mount_quat": list(self.mount_quat),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "ToolSpec":
        check_keys(d, {"gripper", "mount_pos", "mount_quat"}, "")
        return cls(**{k: d[k] for k in d})


@dataclass(frozen=True, slots=True)
class ParametricArm:
    """A serial chain of hinge joints and capsule links on a fixed base.

    Every MuJoCo-based backend names the elements ``base``, ``link_{i}`` and ``joint_{i}`` (robosuite adds its
    ``robot0_`` prefix); the end effector is the ``ee`` site natively and the ``right_hand`` body in robosuite.
    ``label`` is metadata and does not enter the hash.
    """

    schema_version: int
    segments: tuple[Segment, ...]
    base_pose: Pose
    home_qpos: tuple[float, ...]
    end_effector: Pose
    tool: ToolSpec | None
    label: str = ""

    def __post_init__(self):
        if isinstance(self.schema_version, bool) or self.schema_version not in SUPPORTED_SCHEMA_VERSIONS:
            raise ConfigError(
                "schema_version",
                f"unsupported {self.schema_version!r}; supported: {sorted(SUPPORTED_SCHEMA_VERSIONS)}",
            )
        segments = tuple(self.segments)
        if len(segments) < 1:
            raise ConfigError("segments", "need at least one segment (n >= 1)")
        for i, s in enumerate(segments):
            if not isinstance(s, Segment):
                raise ConfigError(f"segments[{i}]", "expected a Segment")
        object.__setattr__(self, "segments", segments)

        for name in ("base_pose", "end_effector"):
            if not isinstance(getattr(self, name), Pose):
                raise ConfigError(name, "expected a Pose")
        if self.tool is not None and not isinstance(self.tool, ToolSpec):
            raise ConfigError("tool", "expected a ToolSpec or None")

        home = as_vector(self.home_qpos, len(segments), "home_qpos")
        for i, (q, s) in enumerate(zip(home, segments)):
            rng = s.joint.range
            if rng is not None and not rng[0] <= q <= rng[1]:
                raise ConfigError(f"home_qpos[{i}]", f"{q} is outside the joint range {list(rng)}")
        object.__setattr__(self, "home_qpos", home)

        if not isinstance(self.label, str):
            raise ConfigError("label", "must be a string")

    @property
    def n(self) -> int:
        return len(self.segments)

    # --- serialization and hashing -------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": "parametric_arm",
            "schema_version": self.schema_version,
            "segments": [s.to_dict() for s in self.segments],
            "base_pose": self.base_pose.to_dict(),
            "home_qpos": list(self.home_qpos),
            "end_effector": self.end_effector.to_dict(),
            "tool": None if self.tool is None else self.tool.to_dict(),
            "label": self.label,
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "ParametricArm":
        check_keys(
            d,
            {"kind", "schema_version", "segments", "base_pose", "home_qpos", "end_effector", "tool"},
            "",
            optional={"label"},
        )
        if d["kind"] != "parametric_arm":
            raise ConfigError("kind", f"expected 'parametric_arm', got {d['kind']!r}")
        segs = d["segments"]
        if isinstance(segs, (str, bytes)) or not hasattr(segs, "__iter__"):
            raise ConfigError("segments", "expected a sequence")
        tool = d["tool"]
        return cls(
            schema_version=d["schema_version"],
            segments=tuple(within(f"segments[{i}]", Segment.from_dict, s) for i, s in enumerate(segs)),
            base_pose=within("base_pose", Pose.from_dict, d["base_pose"]),
            home_qpos=d["home_qpos"],
            end_effector=within("end_effector", Pose.from_dict, d["end_effector"]),
            tool=None if tool is None else within("tool", ToolSpec.from_dict, tool),
            label=d.get("label", ""),
        )

    def hash_dict(self) -> dict[str, Any]:
        """The dict that is hashed: everything except the label."""
        d = self.to_dict()
        del d["label"]
        return d

    def canonical_json(self) -> str:
        return canonical_json(self.hash_dict())

    @property
    def full_hash(self) -> str:
        return sha256_hex(self.canonical_json())

    @property
    def config_hash(self) -> str:
        return self.full_hash[:HASH_DISPLAY_LENGTH]
