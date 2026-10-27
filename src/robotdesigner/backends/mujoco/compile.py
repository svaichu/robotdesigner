"""Compile a ``ParametricArm`` into a MuJoCo model and a backend-neutral ``RobotInterface``."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from ...config import EnvConfig, FixedGains, InertiaScaledGains, Pose
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
    kp: np.ndarray | None  # (n,) effective position gains; None for torque control
    kd: np.ndarray | None  # (n,) effective damping / velocity gains; None for torque control


@dataclass(frozen=True)
class CompiledRobot:
    model: mujoco.MjModel
    spec: mujoco.MjSpec
    interface: RobotInterface


def _per_joint(value, n: int) -> list[float]:
    if isinstance(value, tuple):
        return list(value)
    return [float(value)] * n


SUPPORTED_MODES = ("joint_pos_abs", "joint_pos_delta", "joint_torque", "joint_vel")


def inertia_scaled_gains(model: mujoco.MjModel, iface_home: np.ndarray, qpos_adr, qvel_adr, joint_damping, gains):
    """``kp_i = w^2 M_ii``, ``kd_i = max(2 zeta sqrt(kp_i M_ii) - damping_i, 0)`` with ``M`` at home."""
    data = mujoco.MjData(model)
    data.qpos[qpos_adr] = iface_home
    mujoco.mj_forward(model, data)
    M = np.zeros((model.nv, model.nv))
    try:  # newer MuJoCo: mj_fullM(model, data, dst)
        mujoco.mj_fullM(model, data, M)
    except TypeError:  # older MuJoCo: mj_fullM(model, dst, qM)
        mujoco.mj_fullM(model, M, data.qM)
    m_diag = M[qvel_adr, qvel_adr]
    kp = gains.bandwidth**2 * m_diag
    kd = np.maximum(2.0 * gains.damping_ratio * np.sqrt(kp * m_diag) - joint_damping, 0.0)
    return kp, kd


def _build_spec(cfg: EnvConfig, kp: list[float], kd: list[float]) -> mujoco.MjSpec:
    arm = cfg.robot
    n = arm.n
    control = cfg.control

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

    for i, seg in enumerate(arm.segments):
        effort = seg.joint.effort_limit
        act = spec.add_actuator(
            name=f"act_{i}", target=f"joint_{i}", trntype=mujoco.mjtTrn.mjTRN_JOINT
        )
        act.forcelimited = _LIMITED
        act.forcerange = [-effort, effort]
        act.ctrllimited = _LIMITED
        rng = seg.joint.range
        if control.mode in ("joint_pos_abs", "joint_pos_delta"):
            act.gainprm[0] = kp[i]
            act.biasprm[:3] = [0.0, -kp[i], -kd[i]]
            act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            if rng is not None:
                act.ctrlrange = list(rng)
            elif control.mode == "joint_pos_abs":
                act.ctrlrange = [-np.pi, np.pi]
            else:  # a delta target on an unlimited joint is not bounded
                act.ctrllimited = _UNLIMITED
        elif control.mode == "joint_vel":
            # Velocity servo: force = kd * (ctrl - qvel).
            act.gainprm[0] = kd[i]
            act.biasprm[:3] = [0.0, 0.0, -kd[i]]
            act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            act.ctrllimited = _UNLIMITED
        else:  # joint_torque
            act.ctrlrange = [-effort, effort]
    return spec


def compile_robot(cfg: EnvConfig) -> CompiledRobot:
    arm = cfg.robot
    n = arm.n
    control = cfg.control
    if control.mode not in SUPPORTED_MODES:
        raise NotImplementedError(f"control mode {control.mode!r} is not compiled")

    gains = control.gains
    if isinstance(gains, FixedGains):
        kp, kd = _per_joint(gains.kp, n), _per_joint(gains.kd, n)
    else:
        # Two passes: compile with placeholder gains, read M at home, then build with the final gains.
        assert isinstance(gains, InertiaScaledGains)
        first = _build_spec(cfg, [0.0] * n, [0.0] * n).compile()
        ids = [mujoco.mj_name2id(first, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}") for i in range(n)]
        kp_arr, kd_arr = inertia_scaled_gains(
            first,
            np.array(arm.home_qpos),
            first.jnt_qposadr[ids],
            first.jnt_dofadr[ids],
            np.array([s.joint.damping for s in arm.segments]),
            gains,
        )
        kp, kd = list(kp_arr), list(kd_arr)

    spec = _build_spec(cfg, kp, kd)
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
        kp=None if control.mode == "joint_torque" else np.array(kp),
        kd=None if control.mode == "joint_torque" else np.array(kd),
    )
    return CompiledRobot(model=model, spec=spec, interface=interface)
