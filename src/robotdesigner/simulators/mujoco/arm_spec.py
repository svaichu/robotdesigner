"""The kinematic chain of a ``ParametricArm`` as MuJoCo spec bodies, shared by every MuJoCo-based simulator."""

from __future__ import annotations

import mujoco
import numpy as np

from ...config import ParametricArm

_LIMITED = mujoco.mjtLimited.mjLIMITED_TRUE
_UNLIMITED = mujoco.mjtLimited.mjLIMITED_FALSE

# Collision filtering: robot geoms have contype 1; scene geoms have conaffinity 1 (see scenes.py).
# Giving robot geoms conaffinity 1 as well turns on self-collision.
ROBOT_CONTYPE = 1


def add_arm(parent, arm: ParametricArm, *, conaffinity: int):
    """Add ``link_i`` bodies, ``joint_i`` hinges and ``geom_i`` capsules under ``parent``.

    ``parent`` is the base body: ``arm.base_pose`` is the caller's business. Angles in the spec are radians
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
