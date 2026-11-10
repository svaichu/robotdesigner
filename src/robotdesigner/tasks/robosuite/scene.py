"""Load the assets of a robosuite environment: its arena, its objects and the rule that places them.

The robosuite environment is built once, with a stock robot and without rendering, only to read those assets; it is
closed before this function returns and its simulation is never stepped. Needs robosuite and MuJoCo.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Any

import numpy as np

from ...robot import Pose
from ._compat import apply_compat
from ._introspect import constructor_params, env_class

PREFIX = "rs_"  # names of everything attached from the scene start with this, in the compiled model
MOUNT_SITE = "mount"  # the marker for the scene's mount frame; ``rs_mount`` in the compiled model

# Top-level sections that are simulator settings: ``SimConfig`` and the simulator options decide them, not robosuite.
_DROP_SECTIONS = ("option", "size", "visual")
_IK_TARGET_SUFFIX = "_eef_target"  # mocap bodies robosuite's IK controllers move; nothing here uses them


@dataclass(frozen=True)
class ObjectInfo:
    name: str
    root_body: str
    joints: tuple[str, ...]
    visual: bool  # a marker without collision geoms (for example a target bin); not observed


@dataclass(frozen=True)
class SceneAssets:
    xml: str  # arena and objects as one MJCF document, without simulator settings
    objects: tuple[ObjectInfo, ...]
    sampler: Any  # robosuite's object placement sampler, or None
    mount: Pose  # where robosuite puts the stock robot's root body in this environment (world frame): the scene's mount frame


def clean_xml(xml: str) -> str:
    """Remove simulator settings and IK mocap bodies from a robosuite MJCF document."""
    root = ET.fromstring(xml)
    for tag in _DROP_SECTIONS:
        for el in root.findall(tag):
            root.remove(el)
    world = root.find("worldbody")
    if world is not None:
        for body in list(world.findall("body")):
            if body.get("mocap") == "true" and (body.get("name") or "").endswith(_IK_TARGET_SUFFIX):
                world.remove(body)
    return ET.tostring(root, encoding="unicode")


def matrix_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """A rotation matrix as a unit quaternion ``(w, x, y, z)``, ``w >= 0``."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    t = np.trace(R)
    if t > 0.0:
        s = 2.0 * np.sqrt(t + 1.0)
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0)
        q = np.zeros(4)
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q = q / np.linalg.norm(q)
    return tuple(float(x) for x in (q if q[0] >= 0.0 else -q))


def add_mount_marker(xml: str, pose: Pose) -> str:
    """Add the mount marker, a site at ``pose`` in the world body of an MJCF document (static: it is under no moving body)."""
    root = ET.fromstring(xml)
    world = root.find("worldbody")
    if world is None:
        world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "site", {
        "name": MOUNT_SITE,
        "pos": " ".join(repr(float(x)) for x in pose.pos),
        "quat": " ".join(repr(float(x)) for x in pose.quat),
        "size": "0.01 0.01 0.01",
        "rgba": "0 0 0 0",
    })
    return ET.tostring(root, encoding="unicode")


def load_scene(env_name: str, env_kwargs: dict[str, Any], seed: int = 0) -> SceneAssets:
    """Build the robosuite environment ``env_name`` once and keep its assets.

    robosuite draws some of them at construction, for example the size of the cube in ``Lift``. They are drawn from
    ``seed``, so the same config gives the same scene.
    """
    apply_compat()
    import robosuite
    from robosuite.models.tasks import ManipulationTask

    seed = int(seed) % 2**32
    cls = env_class(env_name)
    extra = {"seed": seed} if cls is not None and "seed" in constructor_params(cls) else {}  # robosuite >= 1.5.2
    state = np.random.get_state()
    np.random.seed(seed)  # robosuite 1.5.1 draws from the global stream while it builds the environment
    try:
        env = robosuite.make(
            env_name,
            robots="Panda",
            has_renderer=False,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            **extra,
            **env_kwargs,
        )
    finally:
        np.random.set_state(state)
    try:
        model = env.model
        objects = list(getattr(model, "mujoco_objects", None) or [])
        xml = ManipulationTask(mujoco_arena=model.mujoco_arena, mujoco_robots=[], mujoco_objects=objects).get_xml()
        infos = tuple(
            ObjectInfo(o.name, o.root_body, tuple(o.joints or ()), getattr(o, "obj_type", "all") == "visual")
            for o in objects
        )
        robot = env.robots[0]
        mount = Pose(
            tuple(float(x) for x in np.asarray(robot.base_pos, dtype=float)),
            matrix_to_quat(np.asarray(robot.base_ori, dtype=float)),
        )
        return SceneAssets(
            xml=add_mount_marker(clean_xml(xml), mount),
            objects=infos,
            sampler=getattr(env, "placement_initializer", None),
            mount=mount,
        )
    finally:
        env.close()
