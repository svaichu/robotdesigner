"""Pure-numpy forward kinematics of a ``ParametricArm``: the engine-independent oracle.

Convention: ``T_0 = base_pose``; for each segment ``F_i = T_i . joint_frame_i``,
``L_i = F_i . Rot(axis_i, q_i)``, ``tip_i = L_i . Pose(tip_offset_i, identity)``,
``T_{i+1} = tip_i``; ``ee = T_n . end_effector``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .robot import ParametricArm, Pose


@dataclass(frozen=True)
class FK:
    link_frames: np.ndarray  # (n, 4, 4) world poses of the link frames (after the joint rotation)
    tip_frames: np.ndarray  # (n, 4, 4) world poses of each link's end frame
    ee: np.ndarray  # (4, 4) end-effector pose


def quat_to_matrix(q) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_to_matrix(pose: Pose) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(pose.quat)
    T[:3, 3] = pose.pos
    return T


def axis_angle_matrices(axis, angles: np.ndarray) -> np.ndarray:
    """Rotations about a unit ``axis`` by each of ``angles`` (B,) as (B, 4, 4) transforms."""
    ax = np.asarray(axis, dtype=float)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    s = np.sin(angles)[:, None, None]
    c = np.cos(angles)[:, None, None]
    T = np.zeros((len(angles), 4, 4))
    T[:, :3, :3] = np.eye(3) + s * K + (1 - c) * (K @ K)
    T[:, 3, 3] = 1.0
    return T


def _forward_batch(arm: ParametricArm, q: np.ndarray):
    q = np.asarray(q, dtype=float)
    if q.ndim != 2 or q.shape[1] != arm.n:
        raise ValueError(f"q must have shape (B, {arm.n}), got {q.shape}")
    B = q.shape[0]
    T = np.broadcast_to(pose_to_matrix(arm.base_pose), (B, 4, 4))
    links, tips = [], []
    for i, seg in enumerate(arm.segments):
        F = T @ pose_to_matrix(seg.link.joint_frame)
        L = F @ axis_angle_matrices(seg.joint.axis, q[:, i])
        tip_local = np.eye(4)
        tip_local[:3, 3] = seg.link.tip_offset
        T = L @ tip_local
        links.append(L)
        tips.append(T)
    ee = T @ pose_to_matrix(arm.end_effector)
    return np.stack(links, axis=1), np.stack(tips, axis=1), ee


def forward(arm: ParametricArm, q) -> FK:
    """Forward kinematics for one joint configuration ``q`` of shape ``(n,)``."""
    q = np.asarray(q, dtype=float)
    if q.shape != (arm.n,):
        raise ValueError(f"q must have shape ({arm.n},), got {q.shape}")
    links, tips, ee = _forward_batch(arm, q[None])
    return FK(link_frames=links[0], tip_frames=tips[0], ee=ee[0])


def forward_ee_batch(arm: ParametricArm, q_batch) -> np.ndarray:
    """End-effector positions in the world frame (``base_pose`` included), ``(B, n) -> (B, 3)``."""
    return _forward_batch(arm, q_batch)[2][:, :3, 3]


def sample_joint_positions(arm: ParametricArm, num: int, rng: np.random.Generator) -> np.ndarray:
    """``(num, n)`` samples, uniform in each joint range and in ``[-pi, pi]`` for unlimited joints."""
    lo = np.empty(arm.n)
    hi = np.empty(arm.n)
    for i, seg in enumerate(arm.segments):
        lo[i], hi[i] = (-np.pi, np.pi) if seg.joint.range is None else seg.joint.range
    return rng.uniform(lo, hi, size=(num, arm.n))
