"""Compile an ``ExternalRobot``: the provider's model is attached to the scene and driven by its own actuators.

The placement follows ``docs/simulators.md``: the compile adds an empty, welded body ``base`` at the mount (so that
``SimView`` frames work as for a ``ParametricArm``) and attaches the provider's ``MjSpec`` into a frame of the world body
at the same pose, with the prefix ``ext_``. The model's world frame is therefore the base frame. A free root joint stays
a direct child of the world, which MuJoCo requires.
"""

from __future__ import annotations

import warnings

import mujoco
import numpy as np

from ...config import EnvConfig, ExternalRobot, Pose, get_robot_provider
from ...config.kinematics import quat_to_matrix
from ...errors import ConfigError, Problem
from .compile import _INTEGRATORS, CompiledRobot, RobotInterface
from .scenes import apply_scene

PREFIX = "ext_"
_KINDS = ("position", "velocity", "torque")
_OBJ = mujoco.mjtObj
_HINGE, _SLIDE = int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)
_FREE, _BALL = int(mujoco.mjtJoint.mjJNT_FREE), int(mujoco.mjtJoint.mjJNT_BALL)
_JOINT_QPOS = {_FREE: 7, _BALL: 4}


def compose(a: Pose, b: Pose) -> Pose:
    """``a`` followed by ``b``: the pose of a frame given in the frame ``a`` as a pose in the frame ``a`` is in."""
    pos = quat_to_matrix(a.quat) @ np.asarray(b.pos) + np.asarray(a.pos)
    aw, ax, ay, az = a.quat
    bw, bx, by, bz = b.quat
    quat = (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )
    return Pose(tuple(float(x) for x in pos), tuple(float(x) for x in quat))


def _model_options(child: mujoco.MjModel, scene: mujoco.MjSpec) -> None:
    """The one place where the contact and solver options of the model (``cone``, ``impratio``, solver, noslip) would be
    carried over to the scene. For now the scene's win, which is what ``MjSpec.attach`` does by itself."""


def _id(model, kind, name) -> int:
    return mujoco.mj_name2id(model, kind, name)


def actuator_kind(model: mujoco.MjModel, a: int) -> str:
    """``position``, ``velocity`` or ``torque`` from the gain / bias of actuator ``a``; ``mixed`` if it is none of them."""
    if int(model.actuator_biastype[a]) == int(mujoco.mjtBias.mjBIAS_NONE):
        return "torque"
    if int(model.actuator_biastype[a]) == int(mujoco.mjtBias.mjBIAS_AFFINE):
        b = model.actuator_biasprm[a]
        if b[1] < 0.0 and b[2] <= 0.0:
            return "position"
        if b[1] == 0.0 and b[2] < 0.0:
            return "velocity"
    return "mixed"


class Facts:
    """What the model of an external robot looks like, read from a standalone compile of the provider's spec."""

    def __init__(self, robot: ExternalRobot, model: mujoco.MjModel):
        self.model = model
        self.joint_ids: list[int | None] = []
        self.actuator_ids: list[int | None] = []
        self.problems: list[Problem] = []
        m = model
        if m.nu == 0:
            self.problems.append(Problem("robot", f"the model of {robot.provider}/{robot.name} has no actuators"))
        for i, j in enumerate(robot.joints):
            where = f"robot.joints[{i}]"
            jid = _id(m, _OBJ.mjOBJ_JOINT, j.name)
            self.joint_ids.append(None if jid < 0 else jid)
            self.actuator_ids.append(None)
            if jid < 0:
                self.problems.append(Problem(f"{where}.name", f"the model has no joint {j.name!r}"))
                continue
            if int(m.jnt_type[jid]) not in (_HINGE, _SLIDE):
                self.problems.append(Problem(f"{where}.name", f"joint {j.name!r} is not a hinge or a slide"))
                continue
            acts = [
                a for a in range(m.nu) if int(m.actuator_trntype[a]) == int(mujoco.mjtTrn.mjTRN_JOINT) and int(m.actuator_trnid[a, 0]) == jid
            ]
            if len(acts) != 1:
                self.problems.append(Problem(where, f"joint {j.name!r} has {len(acts)} actuators with a joint transmission, expected 1"))
                continue
            self.actuator_ids[-1] = acts[0]
        driven = [a for a in self.actuator_ids if a is not None]
        kinds = {actuator_kind(m, a) for a in driven}
        found = next(iter(kinds)) if len(kinds) == 1 else "mixed" if kinds else None
        if found is not None and found != robot.actuator_kind:
            self.problems.append(
                Problem("robot.actuator_kind", f"the driven actuators of the model are {found!r}, the config says {robot.actuator_kind!r}")
            )
        self.driven = driven
        self.key = None
        if robot.keyframe is not None:
            key = _id(m, _OBJ.mjOBJ_KEY, robot.keyframe)
            if key < 0:
                self.problems.append(Problem("robot.keyframe", f"the model has no keyframe {robot.keyframe!r}"))
            else:
                self.key = key
        free = [j for j in range(m.njnt) if int(m.jnt_type[j]) == _FREE]
        self.free = free[0] if len(free) == 1 else None
        if len(free) > 1:
            self.problems.append(Problem("robot", f"the model has {len(free)} free joints, expected at most 1 (the root)"))
        elif free and robot.home_root is None:
            self.problems.append(Problem("robot.home_root", "the model has a free root joint: home_root is required"))
        elif not free and robot.home_root is not None:
            self.problems.append(Problem("robot.home_root", "the model has no free root joint: home_root must be None"))
        ee = robot.end_effector
        if ee is not None:
            if ee.site is not None and _id(m, _OBJ.mjOBJ_SITE, ee.site) < 0:
                self.problems.append(Problem("robot.end_effector.site", f"the model has no site {ee.site!r}"))
            if ee.body is not None and _id(m, _OBJ.mjOBJ_BODY, ee.body) < 0:
                self.problems.append(Problem("robot.end_effector.body", f"the model has no body {ee.body!r}"))


def _open(robot: ExternalRobot):
    """``(spec, facts)`` of the provider's model; ``ConfigError`` from the provider (a stale ``source``) is passed on."""
    provider = get_robot_provider(robot.provider)
    try:
        model = provider.spec(robot).compile()
    except ConfigError:
        raise
    except Exception as e:  # a model that does not open or compile
        raise ConfigError("robot", f"the model of {robot.provider}/{robot.name} could not be opened: {type(e).__name__}: {e}") from e
    return Facts(robot, model)


def problems_of(cfg: EnvConfig) -> list[Problem]:
    """Every reason the model cannot be compiled and driven as configured; nothing is built."""
    robot = cfg.robot
    provider = get_robot_provider(robot.provider)
    problems = list(provider.validate(robot, "mujoco"))
    try:
        facts = _open(robot)
    except ConfigError as e:
        found = Problem(e.path or "robot", e.reason)
        return problems if found in problems else problems + [found]  # the provider's own check may have said it already
    problems += facts.problems
    own = facts.model.opt.timestep
    if cfg.sim.dt > 2.0 * own:
        warnings.warn(
            f"sim.dt = {cfg.sim.dt} is more than twice the timestep of the model of {robot.provider}/{robot.name} ({own}); "
            "its own actuators and contacts may be unstable at that step",
            stacklevel=2,
        )
    return problems


def compile_external(cfg: EnvConfig, scene_hook=None) -> CompiledRobot:
    robot: ExternalRobot = cfg.robot
    control = cfg.control
    facts = _open(robot)
    if facts.problems:
        raise ConfigError(facts.problems[0].path, "; ".join(f"{p.path}: {p.message}" for p in facts.problems))
    provider = get_robot_provider(robot.provider)

    mount = cfg.task.mount
    spec = mujoco.MjSpec()
    spec.compiler.degree = False  # all angles here are radians
    spec.option.timestep = cfg.sim.dt
    spec.option.integrator = _INTEGRATORS[cfg.sim.integrator]
    apply_scene(spec, cfg.sim.scene)
    if scene_hook is not None:
        scene_hook(spec)
    spec.worldbody.add_body(name="base", pos=list(mount.pos), quat=list(mount.quat))
    frame = spec.worldbody.add_frame(pos=list(mount.pos), quat=list(mount.quat))
    child = provider.spec(robot)
    _model_options(facts.model, spec)
    spec.attach(child, prefix=PREFIX, frame=frame)

    ee = robot.end_effector
    if ee is not None and ee.body is not None:
        body = spec.body(PREFIX + ee.body)
        body.add_site(name="ee", pos=list(ee.offset.pos), quat=list(ee.offset.quat))
    model = spec.compile()
    c = facts.model

    # The attached model's joints and actuators come last, in their own order: ids shift by what the scene has before them.
    joff, aoff = model.njnt - c.njnt, model.nu - c.nu
    n = robot.n
    joint_ids = [joff + jc for jc in facts.joint_ids]
    for j, jid in zip(robot.joints, joint_ids):
        if _id(model, _OBJ.mjOBJ_JOINT, PREFIX + j.name) != jid:
            raise ConfigError("robot.joints", f"joint {j.name!r} did not keep its place when the model was attached")
    act_ids = np.array([aoff + a for a in facts.actuator_ids], dtype=int)

    # The state at reset: the keyframe mapped onto this model, joint by joint, and the ctrl of the actuators not driven.
    reset_qpos = held = None
    if facts.key is not None:
        reset_qpos = model.qpos0.copy()
        for jc in range(c.njnt):
            jp = joff + jc
            size = _JOINT_QPOS.get(int(c.jnt_type[jc]), 1)
            reset_qpos[model.jnt_qposadr[jp] : model.jnt_qposadr[jp] + size] = c.key_qpos[facts.key][c.jnt_qposadr[jc] : c.jnt_qposadr[jc] + size]
    held = np.zeros(model.nu)
    if facts.key is not None:
        held[aoff:] = c.key_ctrl[facts.key]
    held[act_ids] = 0.0

    root_adr = home_root = None
    if facts.free is not None:
        root_adr = int(model.jnt_qposadr[joff + facts.free])
        home_root = compose(mount, robot.home_root)

    # The range [-1, 1] maps onto: the actuator's ctrlrange; a position servo without one takes the joint's range.
    ctrl_range = np.where(
        model.actuator_ctrllimited[act_ids, None].astype(bool),
        model.actuator_ctrlrange[act_ids],
        np.array([-np.inf, np.inf]),
    )
    if robot.actuator_kind == "position":
        unlimited = ~model.actuator_ctrllimited[act_ids].astype(bool)
        ctrl_range[unlimited] = np.array([(-np.inf, np.inf) if j.range is None else j.range for j in robot.joints])[unlimited]
    servo = control.mode in ("joint_pos_abs", "joint_pos_delta")
    kp = model.actuator_gainprm[act_ids, 0].copy() if servo else None
    kd = -model.actuator_biasprm[act_ids, 2].copy() if servo else None

    ee_site = None
    if ee is not None:
        ee_site = _id(model, _OBJ.mjOBJ_SITE, "ee" if ee.body is not None else PREFIX + ee.site)
    joint_range = np.array([(-np.inf, np.inf) if j.range is None else j.range for j in robot.joints], dtype=float)
    interface = RobotInterface(
        n=n,
        joint_names=tuple(PREFIX + j.name for j in robot.joints),
        qpos_adr=model.jnt_qposadr[joint_ids].astype(int),
        qvel_adr=model.jnt_dofadr[joint_ids].astype(int),
        actuator_ids=act_ids,
        ee_site_id=ee_site,
        joint_range=joint_range,
        effort_limit=np.array([j.effort_limit for j in robot.joints]),
        home_qpos=np.array(robot.home_qpos),
        mount=mount,
        kp=kp,
        kd=kd,
        ctrl_range=ctrl_range,
        root_qpos_adr=root_adr,
        home_root=home_root,
        reset_qpos=reset_qpos,
        held_ctrl=held,
    )
    return CompiledRobot(model=model, spec=spec, interface=interface)


class MujocoKinematics:
    """The host-side kinematics oracle of a MuJoCo-based robot provider (``RobotProvider.kinematics``): the model is
    compiled alone, its world frame is the robot's base frame, and ``mj_kinematics`` gives the end effector."""

    def __init__(self, robot: ExternalRobot, spec: mujoco.MjSpec):
        if robot.end_effector is None:
            raise ConfigError("robot.end_effector", "the robot has no end effector: it has no kinematics to ask for")
        self.robot = robot
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        m = self.model
        self._adr = np.array([m.jnt_qposadr[_id(m, _OBJ.mjOBJ_JOINT, j.name)] for j in robot.joints])
        ee = robot.end_effector
        self._site = _id(m, _OBJ.mjOBJ_SITE, ee.site) if ee.site is not None else None
        self._body = _id(m, _OBJ.mjOBJ_BODY, ee.body) if ee.body is not None else None
        self._offset = np.asarray(ee.offset.pos)
        self._rest = m.qpos0.copy()
        if robot.keyframe is not None:
            self._rest = m.key_qpos[_id(m, _OBJ.mjOBJ_KEY, robot.keyframe)].copy()
        for j in range(m.njnt):
            if int(m.jnt_type[j]) == _FREE and robot.home_root is not None:
                a = m.jnt_qposadr[j]
                self._rest[a : a + 7] = [*robot.home_root.pos, *robot.home_root.quat]
        self._lo = np.array([-np.pi if j.range is None else j.range[0] for j in robot.joints])
        self._hi = np.array([np.pi if j.range is None else j.range[1] for j in robot.joints])

    def forward_ee_batch(self, q, base: Pose | None = None) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        if q.ndim != 2 or q.shape[1] != self.robot.n:
            raise ValueError(f"q must have shape (B, {self.robot.n}), got {q.shape}")
        out = np.empty((len(q), 3))
        for i, qi in enumerate(q):
            self.data.qpos[:] = self._rest
            self.data.qpos[self._adr] = qi
            mujoco.mj_kinematics(self.model, self.data)
            if self._site is not None:
                out[i] = self.data.site_xpos[self._site]
            else:
                out[i] = self.data.xpos[self._body] + self.data.xmat[self._body].reshape(3, 3) @ self._offset
        if base is not None:
            out = out @ quat_to_matrix(base.quat).T + np.asarray(base.pos)
        return out

    def sample_joint_positions(self, num: int, rng: np.random.Generator) -> np.ndarray:
        return rng.uniform(self._lo, self._hi, size=(num, self.robot.n))
