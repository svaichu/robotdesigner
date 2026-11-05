"""A robot that exists as a model somewhere else: a reference plus the numbers a config needs. No simulator is imported.

An :class:`ExternalRobot` does not hold the model. It names a *robot provider* (the owner of a namespace such as
``menagerie``), the robot and model file within it, and a content id (``source``) the provider vouches for. It carries
the facts the rest of the config layer needs without opening the model: which joints a controller drives, their
ranges and effort limits, the home posture and the end effector. A provider's ``describe`` produces these numbers from
the model; a config can also be written by hand. ``source`` is hashed, so a changed model is a different robot.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .._canon import (
    HASH_DISPLAY_LENGTH,
    as_float,
    as_str,
    as_vector,
    canonical_json,
    check_keys,
    sha256_hex,
    within,
)
from ..errors import ConfigError
from .pose import Pose

ACTUATOR_KINDS = ("position", "torque", "velocity", "mixed")


@dataclass(frozen=True, slots=True)
class ExternalJoint:
    """A joint of the model that a controller drives. ``name`` is as in the model, without any prefix the compile adds;
    ``range`` is ``None`` (unlimited) or ``(lo, hi)`` in rad or m; ``effort_limit`` is N·m or N."""

    name: str
    range: tuple[float, float] | None
    effort_limit: float

    def __post_init__(self):
        as_str(self.name, "name")
        if self.range is not None:
            lo, hi = as_vector(self.range, 2, "range")
            if not lo < hi:
                raise ConfigError("range", "lo must be < hi")
            object.__setattr__(self, "range", (lo, hi))
        effort = as_float(self.effort_limit, "effort_limit")
        if effort <= 0.0:
            raise ConfigError("effort_limit", "must be > 0")
        object.__setattr__(self, "effort_limit", effort)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "range": None if self.range is None else list(self.range),
            "effort_limit": self.effort_limit,
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "ExternalJoint":
        check_keys(d, {"name", "range", "effort_limit"}, "")
        return cls(name=d["name"], range=d["range"], effort_limit=d["effort_limit"])


@dataclass(frozen=True, slots=True)
class EndEffectorRef:
    """The end effector of the model: an existing ``site``, or a ``body`` plus an ``offset`` in the body's frame.
    Exactly one of ``site`` and ``body`` is given; ``offset`` is the identity when ``site`` is given."""

    site: str | None
    body: str | None
    offset: Pose

    def __post_init__(self):
        if (self.site is None) == (self.body is None):
            raise ConfigError("", "give exactly one of site and body")
        for name in ("site", "body"):
            if getattr(self, name) is not None:
                as_str(getattr(self, name), name)
        if not isinstance(self.offset, Pose):
            raise ConfigError("offset", "expected a Pose")
        if self.site is not None and self.offset != Pose.identity():
            raise ConfigError("offset", "must be the identity when a site is given: the site already is a frame")

    def to_dict(self) -> dict[str, Any]:
        return {"site": self.site, "body": self.body, "offset": self.offset.to_dict()}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "EndEffectorRef":
        check_keys(d, {"site", "body", "offset"}, "")
        return cls(site=d["site"], body=d["body"], offset=within("offset", Pose.from_dict, d["offset"]))


@dataclass(frozen=True, slots=True)
class ExternalRobot:
    """A reference to a robot model of a provider, with the numbers the config layer needs.

    ``provider`` is a registered robot provider (see ``config.robot_providers``); ``name`` the robot within it and
    ``entry`` the model file within the robot. ``source`` is a content id the provider vouches for (for MuJoCo
    Menagerie: the git tree id of the model directory). ``joints`` are the joints a ``ControlConfig`` drives, in
    order: ``n = len(joints)``. ``home_qpos`` has one value per joint, inside its range. ``home_root`` is the root
    body at home in the model's world frame for a model with a free joint, else ``None``. ``keyframe`` names the model
    keyframe that sets the state of the joints that are not driven (finger joints). ``end_effector`` is ``None`` for a
    robot without one. ``actuator_kind`` describes the model's own actuators on ``joints``: ``position``, ``torque``,
    ``velocity`` or ``mixed``. ``label`` is metadata and does not enter the hash.
    """

    provider: str
    name: str
    entry: str
    source: str
    joints: tuple[ExternalJoint, ...]
    home_qpos: tuple[float, ...]
    home_root: Pose | None
    keyframe: str | None
    end_effector: EndEffectorRef | None
    actuator_kind: str
    label: str = ""

    def __post_init__(self):
        for name in ("provider", "name", "entry", "source"):
            as_str(getattr(self, name), name)
        # Imported here: the registry is in the config package, which imports this one.
        from ..config.robot_providers import list_robot_providers

        if self.provider not in list_robot_providers():
            raise ConfigError(
                "provider", f"unknown robot provider {self.provider!r}; registered: {list(list_robot_providers())}"
            )

        joints = self.joints
        if isinstance(joints, (str, bytes)) or not hasattr(joints, "__iter__"):
            raise ConfigError("joints", "expected a sequence of ExternalJoint")
        joints = tuple(joints)
        if not joints:
            raise ConfigError("joints", "need at least one joint")
        for i, j in enumerate(joints):
            if not isinstance(j, ExternalJoint):
                raise ConfigError(f"joints[{i}]", "expected an ExternalJoint")
        names = [j.name for j in joints]
        for i, name in enumerate(names):
            if name in names[:i]:
                raise ConfigError(f"joints[{i}].name", f"duplicate joint name {name!r}")
        object.__setattr__(self, "joints", joints)

        home = as_vector(self.home_qpos, len(joints), "home_qpos")
        for i, (q, j) in enumerate(zip(home, joints)):
            if j.range is not None and not j.range[0] <= q <= j.range[1]:
                raise ConfigError(f"home_qpos[{i}]", f"{q} is outside the joint range {list(j.range)}")
        object.__setattr__(self, "home_qpos", home)

        if self.home_root is not None and not isinstance(self.home_root, Pose):
            raise ConfigError("home_root", "expected a Pose or None")
        if self.keyframe is not None:
            as_str(self.keyframe, "keyframe")
        if self.end_effector is not None and not isinstance(self.end_effector, EndEffectorRef):
            raise ConfigError("end_effector", "expected an EndEffectorRef or None")
        if self.actuator_kind not in ACTUATOR_KINDS:
            raise ConfigError(
                "actuator_kind", f"unknown kind {self.actuator_kind!r}; expected one of {list(ACTUATOR_KINDS)}"
            )
        if not isinstance(self.label, str):
            raise ConfigError("label", "must be a string")

    @property
    def n(self) -> int:
        return len(self.joints)

    # --- serialization and hashing -------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": "external_robot",
            "provider": self.provider,
            "name": self.name,
            "entry": self.entry,
            "source": self.source,
            "joints": [j.to_dict() for j in self.joints],
            "home_qpos": list(self.home_qpos),
            "home_root": None if self.home_root is None else self.home_root.to_dict(),
            "keyframe": self.keyframe,
            "end_effector": None if self.end_effector is None else self.end_effector.to_dict(),
            "actuator_kind": self.actuator_kind,
            "label": self.label,
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "ExternalRobot":
        check_keys(
            d,
            {
                "kind", "provider", "name", "entry", "source", "joints", "home_qpos", "home_root", "keyframe",
                "end_effector", "actuator_kind",
            },
            "",
            optional={"label"},
        )
        if d["kind"] != "external_robot":
            raise ConfigError("kind", f"expected 'external_robot', got {d['kind']!r}")
        joints = d["joints"]
        if isinstance(joints, (str, bytes)) or not hasattr(joints, "__iter__"):
            raise ConfigError("joints", "expected a sequence")
        root, ee = d["home_root"], d["end_effector"]
        return cls(
            provider=d["provider"],
            name=d["name"],
            entry=d["entry"],
            source=d["source"],
            joints=tuple(within(f"joints[{i}]", ExternalJoint.from_dict, j) for i, j in enumerate(joints)),
            home_qpos=d["home_qpos"],
            home_root=None if root is None else within("home_root", Pose.from_dict, root),
            keyframe=d["keyframe"],
            end_effector=None if ee is None else within("end_effector", EndEffectorRef.from_dict, ee),
            actuator_kind=d["actuator_kind"],
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
