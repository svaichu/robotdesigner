"""Camera framing from configs alone: numpy only, no engine import until ``CameraSpec.to_mjv``."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..config import FixedBox
from ..config import kinematics as kin

# (azimuth, elevation) in degrees, world frame. Azimuth 0 looks along +x; the camera sits opposite to its view
# direction, so "front" (180) looks at the robot from +x towards -x.
VIEWS: dict[str, tuple[float, float]] = {
    "iso": (135.0, -25.0),
    "front": (180.0, -15.0),
    "side": (90.0, -10.0),
    "top": (90.0, -89.0),
}


@dataclass(frozen=True)
class CameraSpec:
    """A free camera: look-at point, distance, azimuth and elevation (degrees), as in MuJoCo."""

    lookat: tuple[float, float, float]
    distance: float
    azimuth: float
    elevation: float

    def to_mjv(self):
        import mujoco

        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = self.lookat
        cam.distance = self.distance
        cam.azimuth = self.azimuth
        cam.elevation = self.elevation
        return cam


def _parts(cfg):
    """``cfg`` is an ``EnvConfig`` (robot and target region) or a bare ``ParametricArm``."""
    robot = getattr(cfg, "robot", cfg)
    region = getattr(getattr(cfg, "task", None), "target_region", None)
    return robot, region


def workspace_points(cfg, n_samples: int = 512, seed: int = 0) -> np.ndarray:
    """World-frame points the camera should cover: base, floor under the base, end-effector samples over the
    joint ranges, the home pose, and the corners of a ``FixedBox`` target region. Uses its own random
    generator, so it never touches an environment's random stream."""
    robot, region = _parts(cfg)
    rng = np.random.default_rng(seed)
    T = kin.pose_to_matrix(robot.base_pose)
    base = T[:3, 3]
    pts = [base[None], np.array([[base[0], base[1], 0.0]])]
    pts.append(kin.forward_ee_batch(robot, kin.sample_joint_positions(robot, n_samples, rng)))
    pts.append(kin.forward_ee_batch(robot, np.asarray(robot.home_qpos, dtype=float)[None]))
    if isinstance(region, FixedBox):
        lo, hi = np.asarray(region.low, dtype=float), np.asarray(region.high, dtype=float)
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        pts.append(corners @ T[:3, :3].T + T[:3, 3])
    return np.concatenate(pts, axis=0)


def auto_camera(
    *cfgs,
    view: str = "iso",
    fovy: float = 45.0,
    aspect: float = 4.0 / 3.0,
    margin: float = 1.15,
    n_samples: int = 512,
) -> CameraSpec:
    """A camera that sees the whole workspace of every config in ``cfgs``.

    Giving several configs returns ONE camera that covers all of them, so clips of different embodiments are
    drawn at the same scale. ``fovy`` (degrees) and ``aspect`` (width / height) are those of the render.
    """
    if not cfgs:
        raise ValueError("auto_camera needs at least one config")
    if view not in VIEWS:
        raise ValueError(f"unknown view {view!r}; available: {sorted(VIEWS)}")
    pts = np.concatenate([workspace_points(c, n_samples) for c in cfgs], axis=0)
    center = (pts.min(axis=0) + pts.max(axis=0)) / 2.0
    radius = max(float(np.linalg.norm(pts - center, axis=1).max()), 0.1)
    # The tighter of the vertical and horizontal half angles decides how far back the camera must sit.
    half = math.atan(math.tan(math.radians(fovy) / 2.0) * min(1.0, aspect))
    az, el = VIEWS[view]
    return CameraSpec(tuple(float(x) for x in center), margin * radius / math.sin(half), az, el)
