"""The robot base in the world, read from the compiled scene.

The scene owns the mount frame. A task that has one adds a *marker*, a site in a static place, in ``attach(spec)`` and names
it with ``TaskImpl.mount_site()``. The compile reads the marker's world pose from a compile of the scene alone (static: the
pose does not depend on how the marker is nested or prefixed) and creates the ``base`` body at ``marker @ mount``. The pose
the provider's ``mount_frame(params)`` gave (used by ``viz`` and the kinematics oracle) must agree with the marker: when it
does not, the compile raises and names both poses. A task without a marker (``mount_site()`` returns ``None``: the simulator
presets, ``empty/idle``) has the provider's frame, which is the world origin for them.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...config import EnvConfig, Pose, scene_frame
from ...config.mount import mount_in
from ...errors import ConfigError
from .scenes import apply_scene

__all__ = ["POSE_ATOL", "marker_pose", "resolve_world_mount"]

POSE_ATOL = 1e-6  # m for the position, and for each component of the quaternion (sign aside)


def marker_pose(cfg: EnvConfig, scene_hook, site: str) -> Pose:
    """The world pose of the site ``site`` in the scene of ``cfg`` without the robot."""
    spec = mujoco.MjSpec()
    spec.compiler.degree = False  # all angles here are radians
    apply_scene(spec, cfg.sim.scene)
    if scene_hook is not None:
        scene_hook(spec)
    model = spec.compile()
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
    if sid < 0:
        raise ConfigError("task", f"the task names the mount marker {site!r} but its scene has no site of that name")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, data.site_xmat[sid])
    return Pose(tuple(float(x) for x in data.site_xpos[sid]), tuple(float(x) for x in quat))


def _agree(a: Pose, b: Pose) -> bool:
    qa, qb = np.asarray(a.quat), np.asarray(b.quat)
    return bool(
        np.allclose(a.pos, b.pos, atol=POSE_ATOL)
        and (np.allclose(qa, qb, atol=POSE_ATOL) or np.allclose(qa, -qb, atol=POSE_ATOL))
    )


def resolve_world_mount(cfg: EnvConfig, scene_hook=None, mount_site: str | None = None) -> Pose:
    """``frame @ mount``, where ``frame`` is the marker's world pose when the task has a marker (checked against the
    provider's ``mount_frame``) and the provider's frame otherwise."""
    frame = scene_frame(cfg)
    if mount_site is not None:
        found = marker_pose(cfg, scene_hook, mount_site)
        if not _agree(found, frame):
            raise ConfigError(
                "task",
                f"the scene's mount marker {mount_site!r} is at {found.to_dict()} but the provider's mount_frame says "
                f"{frame.to_dict()}: viz and the kinematics oracle use the second, the simulator the first",
            )
        frame = found
    return mount_in(frame, cfg.task.mount)
