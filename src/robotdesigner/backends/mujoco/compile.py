"""Compile a ``ParametricArm`` into a MuJoCo model and a backend-neutral ``RobotInterface``."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from ...config import EnvConfig, Pose
from .scenes import apply_scene

_INTEGRATORS = {
    "euler": mujoco.mjtIntegrator.mjINT_EULER,
    "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
    "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    "rk4": mujoco.mjtIntegrator.mjINT_RK4,
}
_LIMITED = mujoco.mjtLimited.mjLIMITED_TRUE
_UNLIMITED = mujoco.mjtLimited.mjLIMITED_FALSE

# Collision filtering: robot geoms have contype 1; scene geoms have conaffinity 1 (see scenes.py).
# Giving robot geoms conaffinity 1 as well turns on self-collision.
ROBOT_CONTYPE = 1


@dataclass(frozen=True)
class RobotInterface:
    """What tasks and wrappers need to know about the compiled robot. Nothing engine-specific by name."""

    n: int
    joint_names: tuple[str, ...]
    qpos_adr: np.ndarray  # (n,) int
    qvel_adr: np.ndarray  # (n,) int
    actuator_ids: np.ndarray  # (n,) int
    ee_site_id: int
    joint_range: np.ndarray  # (n, 2); -inf / +inf where unlimited
    effort_limit: np.ndarray  # (n,)
    home_qpos: np.ndarray  # (n,)
    base_pose: Pose


@dataclass(frozen=True)
class CompiledRobot:
    model: mujoco.MjModel
    spec: mujoco.MjSpec
    interface: RobotInterface


def _per_joint(value, n: int) -> list[float]:
    if isinstance(value, tuple):
        return list(value)
    return [float(value)] * n


def compile_robot(cfg: EnvConfig) -> CompiledRobot:
    arm = cfg.robot
    n = arm.n
    control = cfg.control
    if control.mode not in ("joint_pos_abs", "joint_torque"):
        raise NotImplementedError(f"control mode {control.mode!r} is not compiled yet")

    spec = mujoco.MjSpec()
    spec.option.timestep = cfg.sim.dt
    spec.option.integrator = _INTEGRATORS[cfg.sim.integrator]
    apply_scene(spec, cfg.sim.scene)

    self_collision = bool(getattr(cfg.backend_options, "self_collision", False))
    conaffinity = 1 if self_collision else 0

    base = spec.worldbody.add_body(
        name="base", pos=list(arm.base_pose.pos), quat=list(arm.base_pose.quat)
    )

    parent = base
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

    # Pose(tip_offset, identity) composed with end_effector.
    parent.add_site(
        name="ee",
        pos=list(prev_tip + np.asarray(arm.end_effector.pos)),
        quat=list(arm.end_effector.quat),
    )

    kp = _per_joint(control.gains.kp, n)
    kd = _per_joint(control.gains.kd, n)
    for i, seg in enumerate(arm.segments):
        effort = seg.joint.effort_limit
        act = spec.add_actuator(
            name=f"act_{i}", target=f"joint_{i}", trntype=mujoco.mjtTrn.mjTRN_JOINT
        )
        act.forcelimited = _LIMITED
        act.forcerange = [-effort, effort]
        act.ctrllimited = _LIMITED
        if control.mode == "joint_pos_abs":
            act.gainprm[0] = kp[i]
            act.biasprm[:3] = [0.0, -kp[i], -kd[i]]
            act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            rng = seg.joint.range
            act.ctrlrange = [-np.pi, np.pi] if rng is None else list(rng)
        else:  # joint_torque
            act.ctrlrange = [-effort, effort]

    model = spec.compile()

    def jid(i):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}")

    joint_ids = [jid(i) for i in range(n)]
    joint_range = np.empty((n, 2))
    for i, seg in enumerate(arm.segments):
        joint_range[i] = (-np.inf, np.inf) if seg.joint.range is None else seg.joint.range
    interface = RobotInterface(
        n=n,
        joint_names=tuple(f"joint_{i}" for i in range(n)),
        qpos_adr=model.jnt_qposadr[joint_ids].astype(int),
        qvel_adr=model.jnt_dofadr[joint_ids].astype(int),
        actuator_ids=np.array(
            [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{i}") for i in range(n)], dtype=int
        ),
        ee_site_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee"),
        joint_range=joint_range,
        effort_limit=np.array([s.joint.effort_limit for s in arm.segments]),
        home_qpos=np.array(arm.home_qpos),
        base_pose=arm.base_pose,
    )
    return CompiledRobot(model=model, spec=spec, interface=interface)
