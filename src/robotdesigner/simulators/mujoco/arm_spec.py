"""The kinematic chain of a ``ParametricArm`` as MuJoCo spec bodies, shared by every MuJoCo-based simulator."""

from __future__ import annotations

import mujoco
import numpy as np

from ...robot import ParametricArm, TwoFingerGripper

_LIMITED = mujoco.mjtLimited.mjLIMITED_TRUE
_UNLIMITED = mujoco.mjtLimited.mjLIMITED_FALSE

# Collision filtering: robot geoms have contype 1; scene geoms have conaffinity 1 (see scenes.py).
# Giving robot geoms conaffinity 1 as well turns on self-collision.
ROBOT_CONTYPE = 1


def add_arm(parent, arm: ParametricArm, *, conaffinity: int):
    """Add ``link_i`` bodies, ``joint_i`` hinges and ``geom_i`` capsules under ``parent``.

    ``parent`` is the base body: the base pose (the task's mount) is the caller's business. Angles in the spec are radians
    (the caller must set ``spec.compiler.degree = False``). Returns ``(last_body, tip)`` where ``tip`` is the last
    link's ``tip_offset`` in its own frame, from which the caller places the end effector.
    """
    prev_tip = np.zeros(3)
    for i, seg in enumerate(arm.segments):
        j, link = seg.joint, seg.link
        # Pose(previous tip_offset, identity) composed with joint_frame: the rotation is joint_frame's.
        body = parent.add_body(
            name=f"link_{i}",
            pos=list(prev_tip + np.asarray(link.joint_frame.pos)),
            quat=list(link.joint_frame.quat),
        )
        lo_hi = j.range
        body.add_joint(
            name=f"joint_{i}",
            type=mujoco.mjtJoint.mjJNT_HINGE,
            axis=list(j.axis),
            limited=_UNLIMITED if lo_hi is None else _LIMITED,
            range=[0.0, 0.0] if lo_hi is None else list(lo_hi),
            damping=j.damping,
            armature=j.armature,
            frictionloss=j.frictionloss,
        )
        geom_kwargs = dict(
            name=f"geom_{i}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[0.0, 0.0, 0.0, *link.tip_offset],
            size=[link.geometry.radius, 0.0, 0.0],
            contype=ROBOT_CONTYPE,
            conaffinity=conaffinity,
        )
        if link.inertia is None:
            # Mass and inertia derived from the capsule at the configured mass.
            body.add_geom(mass=link.mass, **geom_kwargs)
        else:
            # Explicit principal moments about the centre of mass (capsule midpoint), axes of the link frame.
            body.add_geom(mass=0.0, **geom_kwargs)
            body.explicitinertial = True
            body.mass = link.mass
            body.ipos = list(np.asarray(link.tip_offset) / 2.0)
            body.iquat = [1.0, 0.0, 0.0, 0.0]
            body.inertia = list(link.inertia)
        prev_tip = np.asarray(link.tip_offset, dtype=float)
        parent = body
    return parent, prev_tip


def _quat_mul(a, b) -> list[float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def _rotate(quat, v) -> np.ndarray:
    w, x, y, z = quat
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    return R @ np.asarray(v, dtype=float)


def add_gripper(parent, arm: ParametricArm, tip: np.ndarray, *, conaffinity: int) -> None:
    """Add the arm's ``TwoFingerGripper`` under the last link ``parent`` (``tip`` is that link's tip, as returned by
    :func:`add_arm`).

    Names: body ``gripper``, bodies ``finger_0`` / ``finger_1`` with slide joints ``finger_joint_0`` / ``finger_joint_1``
    (coupled by an equality constraint), tendon ``gripper_width`` (the sum of both joints), actuator ``act_gripper`` and
    site ``tcp``. The gripper frame is ``end_effector`` composed with ``gripper.mount``.
    """
    g: TwoFingerGripper = arm.gripper
    ee = arm.end_effector
    pos = tip + np.asarray(ee.pos) + _rotate(ee.quat, g.mount.pos)
    base = parent.add_body(name="gripper", pos=list(pos), quat=_quat_mul(ee.quat, g.mount.quat))
    r = g.finger_radius
    # Palm: a box from the frame to ``palm_length`` along +x, wide enough to carry both fingers fully open.
    base.add_geom(
        name="palm",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[g.palm_length / 2.0, 0.0, 0.0],
        size=[g.palm_length / 2.0, g.stroke + 2.0 * r, r],
        mass=g.palm_mass,
        contype=ROBOT_CONTYPE,
        conaffinity=conaffinity,
    )
    for i, side in enumerate((1.0, -1.0)):
        finger = base.add_body(name=f"finger_{i}", pos=[g.palm_length, side * r, 0.0])
        finger.add_joint(
            name=f"finger_joint_{i}",
            type=mujoco.mjtJoint.mjJNT_SLIDE,
            axis=[0.0, side, 0.0],
            limited=_LIMITED,
            range=[0.0, g.stroke],
            damping=0.0,
        )
        finger.add_geom(
            name=f"finger_geom_{i}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[0.0, 0.0, 0.0, g.finger_length, 0.0, 0.0],
            size=[r, 0.0, 0.0],
            mass=g.finger_mass,
            contype=ROBOT_CONTYPE,
            # Fingers always collide with the scene and with each other's object; the palm and fingers never collide
            # (siblings in the same chain are filtered by MuJoCo's parent-child rule only for direct parents).
            conaffinity=conaffinity,
            friction=[1.5, 0.005, 0.0001],
        )
    base.add_site(name="tcp", pos=list(g.tcp_offset))
