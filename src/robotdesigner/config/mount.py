"""Where the robot base stands in a scene: a mount relative to the scene's mount frame. Numpy only.

The scene owns a *mount frame* (``TaskProvider.mount_frame(params)``, a world pose). A task adds only what differs
from it:

- :class:`Mount` for a fixed-base arm: ``offset``, a :class:`Pose` in the mount frame (identity: snapped exactly);
- :class:`MobileMount` for a robot with a floating base: ``spawn`` (x, y[, z]) in the mount frame and ``heading``
  (yaw, rad, about the frame's +z; 0 faces the frame's +x). ``z`` is added to the robot's own home base height.

``world_mount`` is the only function that composes them: ``T_world_base = T_frame @ T_mount``.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from .._canon import as_float, as_vector, check_keys, within
from ..errors import ConfigError
from ..robot import ExternalRobot, Pose

__all__ = ["MobileMount", "Mount", "is_mobile", "mount_from_dict", "mount_in", "mount_pose", "scene_frame", "snapped", "world_mount"]

_MIGRATION = (
    "mounts are relative to the scene's mount frame now: write Mount(offset=Pose(...)) for a fixed-base arm "
    "(Mount() snaps it onto the frame) or MobileMount(spawn=(x, y), heading=...) for a mobile robot"
)


@dataclass(frozen=True, slots=True)
class Mount:
    """A fixed-base robot: ``offset`` is the base relative to the scene's mount frame (identity: snapped exactly)."""

    offset: Pose = Pose.identity()

    def __post_init__(self):
        if not isinstance(self.offset, Pose):
            raise ConfigError("offset", "expected a Pose: the robot base in the scene's mount frame")

    def to_dict(self) -> dict[str, Any]:
        return {"kind": "mount", "offset": self.offset.to_dict()}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> Mount:
        check_keys(d, {"offset"}, "", optional={"kind"})
        return cls(offset=within("offset", Pose.from_dict, d["offset"]))


@dataclass(frozen=True, slots=True)
class MobileMount:
    """A robot with a floating base: it spawns at ``spawn`` (m, in the mount frame; ``z`` is added to the robot's own
    home base height, default 0) facing ``heading`` (rad, yaw about the frame's +z, 0 = the frame's +x)."""

    spawn: tuple[float, ...] = (0.0, 0.0, 0.0)
    heading: float = 0.0

    def __post_init__(self):
        spawn = self.spawn
        if isinstance(spawn, (str, bytes)) or not hasattr(spawn, "__len__") or len(spawn) not in (2, 3):
            raise ConfigError("spawn", "expected (x, y) or (x, y, z) in metres")
        spawn = as_vector(spawn, len(spawn), "spawn")
        object.__setattr__(self, "spawn", spawn if len(spawn) == 3 else (*spawn, 0.0))
        object.__setattr__(self, "heading", as_float(self.heading, "heading"))

    def to_dict(self) -> dict[str, Any]:
        return {"kind": "mobile_mount", "spawn": list(self.spawn), "heading": self.heading}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> MobileMount:
        check_keys(d, {"spawn", "heading"}, "", optional={"kind"})
        return cls(spawn=d["spawn"], heading=d["heading"])


def mount_from_dict(d: Any) -> Mount | MobileMount:
    """Inverse of ``to_dict``. The old absolute form (``pos`` / ``quat``) is an error that shows the replacement."""
    if isinstance(d, Pose) or (isinstance(d, Mapping) and ({"pos", "quat"} & set(d))):
        raise ConfigError("", f"an absolute pose is no longer a mount: {_MIGRATION}")
    if not isinstance(d, Mapping):
        raise ConfigError("", f"expected a mapping, got {type(d).__name__}")
    kind = d.get("kind", "mount" if "offset" in d else None)
    if kind == "mount":
        return Mount.from_dict(d)
    if kind == "mobile_mount":
        return MobileMount.from_dict(d)
    raise ConfigError("kind", f"unknown mount kind {kind!r}; expected 'mount' or 'mobile_mount'")


def check_mount(mount: Any) -> None:
    """Raise a ``ConfigError`` unless ``mount`` is a :class:`Mount` or a :class:`MobileMount`."""
    if isinstance(mount, Pose):
        raise ConfigError("mount", f"got a Pose: {_MIGRATION}")
    if not isinstance(mount, (Mount, MobileMount)):
        raise ConfigError("mount", f"expected Mount or MobileMount, got {type(mount).__name__}")


def is_mobile(robot: Any) -> bool:
    """Whether ``robot`` has a floating base (a model with a free joint, ``home_root`` set). A planar base built from
    slide joints is part of the model's own joints and is not detected."""
    return isinstance(robot, ExternalRobot) and robot.home_root is not None


def snapped(robot: Any) -> Mount | MobileMount:
    """The mount that snaps ``robot`` onto the scene's mount frame: ``Mount()`` for a fixed-base robot,
    ``MobileMount()`` (spawn at the frame, heading 0) for a floating-base one."""
    return MobileMount() if is_mobile(robot) else Mount()


def check_mount_fits_robot(mount: Mount | MobileMount, robot: Any) -> None:
    """A fixed-base robot takes a :class:`Mount`, a floating-base one a :class:`MobileMount`."""
    if is_mobile(robot) and isinstance(mount, Mount):
        raise ConfigError(
            "task.mount", f"robot {robot.name!r} has a floating base: use MobileMount(spawn=(x, y), heading=...)"
        )
    if not is_mobile(robot) and isinstance(mount, MobileMount):
        raise ConfigError("task.mount", "MobileMount is for a robot with a floating base; a fixed-base robot takes Mount(offset=...)")


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _qrot(q, v):
    w, x, y, z = q
    t = (2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0]))
    return (
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    )


def compose(a: Pose, b: Pose) -> Pose:
    """``a @ b``: the pose ``b`` expressed in the frame ``a``, as a world pose."""
    p = _qrot(a.quat, b.pos)
    return Pose(tuple(a.pos[i] + p[i] for i in range(3)), _qmul(a.quat, b.quat))


def mount_pose(mount: Mount | MobileMount) -> Pose:
    """The mount as a pose in the scene's mount frame."""
    if isinstance(mount, Mount):
        return mount.offset
    h = 0.5 * mount.heading
    return Pose(mount.spawn, (math.cos(h), 0.0, 0.0, math.sin(h)))


def scene_frame(cfg: Any) -> Pose:
    """The scene's mount frame, as the task provider names it (a world pose). ``cfg`` is an ``EnvConfig`` or a ``TaskConfig``."""
    from .providers import get_task_provider

    task = getattr(cfg, "task", cfg)
    return get_task_provider(task.params.provider).mount_frame(task.params)


def mount_in(frame: Pose, mount: Mount | MobileMount) -> Pose:
    """The robot base in the world, given the mount frame (the one the engine read from the model, or the provider's)."""
    return compose(frame, mount_pose(mount))


def world_mount(cfg: Any) -> Pose:
    """The robot base in the world: the scene's mount frame composed with the mount. ``cfg`` is an ``EnvConfig`` or a
    ``TaskConfig``. This is the config-side pose (what ``viz`` and the kinematics oracle use); the simulator reads the
    frame from the model's marker and checks that it agrees (``simulators.mujoco.mount``)."""
    return mount_in(scene_frame(cfg), getattr(cfg, "task", cfg).mount)
