"""Scene presets. Robot elements and scene elements are kept separate."""

from __future__ import annotations

from typing import Callable

import mujoco


def _empty(spec: mujoco.MjSpec) -> None:
    pass


def _plain_floor(spec: mujoco.MjSpec) -> None:
    # Scene geoms collide with robot geoms (contype 1) but not with each other.
    spec.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[5.0, 5.0, 0.1],
        contype=0,
        conaffinity=1,
    )
    spec.worldbody.add_light(name="light", pos=[0.0, 0.0, 3.0], dir=[0.0, 0.0, -1.0])


SCENES: dict[str, Callable[[mujoco.MjSpec], None]] = {
    "empty": _empty,
    "plain_floor": _plain_floor,
}


def apply_scene(spec: mujoco.MjSpec, name: str) -> None:
    if name not in SCENES:
        raise ValueError(f"unknown scene {name!r}; available: {sorted(SCENES)}")
    SCENES[name](spec)
