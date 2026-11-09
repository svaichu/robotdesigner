"""Compile a ``ParametricArm`` into a MuJoCo model and a simulator-neutral ``RobotInterface``."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from ...config import EnvConfig, FixedGains, InertiaScaledGains, Pose
from .arm_spec import add_arm, add_gripper
from .mount import resolve_world_mount
from .scenes import apply_scene

_INTEGRATORS = {
    "euler": mujoco.mjtIntegrator.mjINT_EULER,
    "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
    "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    "rk4": mujoco.mjtIntegrator.mjINT_RK4,
}
_LIMITED = mujoco.mjtLimited.mjLIMITED_TRUE
_UNLIMITED = mujoco.mjtLimited.mjLIMITED_FALSE


@dataclass(frozen=True)
class RobotInterface:
    """What tasks and wrappers need to know about the compiled robot. Nothing engine-specific by name."""

    n: int
    joint_names: tuple[str, ...]
    qpos_adr: np.ndarray  # (n,) int
    qvel_adr: np.ndarray  # (n,) int
    actuator_ids: np.ndarray  # (n,) int
    ee_site_id: int | None  # None for an external robot without an end effector
    joint_range: np.ndarray  # (n, 2); -inf / +inf where unlimited
    effort_limit: np.ndarray  # (n,)
    home_qpos: np.ndarray  # (n,)
    mount: Pose  # the robot base in the world: the scene's mount marker (or frame) composed with the task's mount
    kp: np.ndarray | None  # (n,) effective position gains; None for torque control
    kd: np.ndarray | None  # (n,) effective damping / velocity gains; None for torque control
    # Gripper (``arm.gripper``); ``None`` / 0.0 when the arm has none. The actuator is the last one (``act_gripper``).
    gripper_actuator_id: int | None = None
    gripper_qpos_adr: np.ndarray | None = None  # (2,) int, the two finger joints
    gripper_max_opening: float = 0.0  # metres, the control range of the actuator is [0, gripper_max_opening]
    tcp_site_id: int | None = None
    # External robots (``ExternalRobot``); ``None`` for a ``ParametricArm``. The Warp simulator ignores them.
    ctrl_range: np.ndarray | None = None  # (n, 2) ctrlrange of the driven actuators; -inf / +inf where unlimited
    root_qpos_adr: int | None = None  # first of the 7 qpos of a free root joint
    home_root: Pose | None = None  # the root body at home in the world: mount composed with ``ExternalRobot.home_root``
    reset_qpos: np.ndarray | None = None  # (nq,) the state of the model at reset: the keyframe, mapped onto this model
    held_ctrl: np.ndarray | None = None  # (nu,) ctrl of the actuators that are not driven (the keyframe's), 0 where driven

    @property
    def has_gripper(self) -> bool:
        return self.gripper_actuator_id is not None

    @property
    def action_dim(self) -> int:
        """Joint commands plus one gripper command when there is a gripper."""
        return self.n + (1 if self.has_gripper else 0)


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


def _build_spec(cfg: EnvConfig, kp: list[float], kd: list[float], scene_hook=None, mount: Pose | None = None) -> mujoco.MjSpec:
    arm = cfg.robot
    control = cfg.control

    spec = mujoco.MjSpec()
    spec.compiler.degree = False  # MjSpec reads angles as degrees by default; all angles here are radians
    spec.option.timestep = cfg.sim.dt
    spec.option.integrator = _INTEGRATORS[cfg.sim.integrator]
    apply_scene(spec, cfg.sim.scene)
    if scene_hook is not None:  # scene elements the task adds (arena, objects), before the robot
        scene_hook(spec)

    self_collision = bool(getattr(cfg.sim.options, "self_collision", False))
    conaffinity = 1 if self_collision else 0

    base = spec.worldbody.add_body(
        name="base", pos=list(mount.pos), quat=list(mount.quat)
    )

    parent, prev_tip = add_arm(base, arm, conaffinity=conaffinity)
    if arm.gripper is not None:
        add_gripper(parent, arm, prev_tip, conaffinity=conaffinity)

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

    g = arm.gripper
    if g is not None:
        # One degree of freedom: the two finger slides are tied together and driven through their sum (the opening).
        eq = spec.add_equality(name="fingers_coupled")
        eq.type = mujoco.mjtEq.mjEQ_JOINT
        eq.name1, eq.name2 = "finger_joint_1", "finger_joint_0"
        eq.data[:5] = [0.0, 1.0, 0.0, 0.0, 0.0]
        tendon = spec.add_tendon(name="gripper_width")
        tendon.wrap_joint("finger_joint_0", 1.0)
        tendon.wrap_joint("finger_joint_1", 1.0)
        act = spec.add_actuator(name="act_gripper", target="gripper_width", trntype=mujoco.mjtTrn.mjTRN_TENDON)
        act.gainprm[0] = g.kp
        act.biasprm[:3] = [0.0, -g.kp, -g.kd]
        act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        act.ctrllimited = _LIMITED
        act.ctrlrange = [0.0, g.max_opening]
        act.forcelimited = _LIMITED
        act.forcerange = [-g.force_limit, g.force_limit]
    return spec


def compile_robot(cfg: EnvConfig, scene_hook=None, mount_site: str | None = None) -> CompiledRobot:
    """Compile ``cfg.robot`` into a model. ``scene_hook(spec)``, if given, adds the task's scene elements to the spec;
    ``mount_site`` names the marker it adds for the scene's mount frame (``TaskImpl.mount_site()``), where the base is snapped."""
    if type(cfg.robot).__name__ == "ExternalRobot":
        from .external import compile_external

        return compile_external(cfg, scene_hook, mount_site)
    arm = cfg.robot
    n = arm.n
    control = cfg.control
    if control.mode not in SUPPORTED_MODES:
        raise NotImplementedError(f"control mode {control.mode!r} is not compiled")

    mount = resolve_world_mount(cfg, scene_hook, mount_site)
    gains = control.gains
    if isinstance(gains, FixedGains):
        kp, kd = _per_joint(gains.kp, n), _per_joint(gains.kd, n)
    else:
        # Two passes: compile with placeholder gains, read M at home, then build with the final gains.
        assert isinstance(gains, InertiaScaledGains)
        first = _build_spec(cfg, [0.0] * n, [0.0] * n, scene_hook, mount).compile()
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

    spec = _build_spec(cfg, kp, kd, scene_hook, mount)
    model = spec.compile()

    def jid(i):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}")

    joint_ids = [jid(i) for i in range(n)]
    joint_range = np.empty((n, 2))
    for i, seg in enumerate(arm.segments):
        joint_range[i] = (-np.inf, np.inf) if seg.joint.range is None else seg.joint.range
    gripper_fields = {}
    if arm.gripper is not None:
        fids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"finger_joint_{i}") for i in (0, 1)]
        gripper_fields = dict(
            gripper_actuator_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_gripper"),
            gripper_qpos_adr=model.jnt_qposadr[fids].astype(int),
            gripper_max_opening=arm.gripper.max_opening,
            tcp_site_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp"),
        )
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
        mount=mount,
        kp=None if control.mode == "joint_torque" else np.array(kp),
        kd=None if control.mode == "joint_torque" else np.array(kd),
        **gripper_fields,
    )
    return CompiledRobot(model=model, spec=spec, interface=interface)
