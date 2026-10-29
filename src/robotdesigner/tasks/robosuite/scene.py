"""Load the assets of a robosuite environment: its arena, its objects and the rule that places them.

The robosuite environment is built once, with a stock robot and without rendering, only to read those assets; it is
closed before this function returns and its simulation is never stepped. Needs robosuite and MuJoCo.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Any

import numpy as np

from ._compat import apply_compat
from ._introspect import constructor_params, env_class

PREFIX = "rs_"  # names of everything attached from the scene start with this, in the compiled model

# Top-level sections that are simulator settings: ``SimConfig`` and the backend options decide them, not robosuite.
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
    default_mount: np.ndarray  # where robosuite mounts a stock arm in this environment, world frame


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
        return SceneAssets(
            xml=clean_xml(xml),
            objects=infos,
            sampler=getattr(env, "placement_initializer", None),
            default_mount=np.asarray(env.robots[0].base_pos, dtype=float).copy(),
        )
    finally:
        env.close()
