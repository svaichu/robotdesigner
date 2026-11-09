"""The CPU MuJoCo simulator running an ``ExternalRobot`` of the test-only robot provider (no download)."""

import warnings
from dataclasses import replace

import numpy as np
import pytest

torch = pytest.importorskip("torch")
mujoco = pytest.importorskip("mujoco")
pytest.importorskip("torchrl")

from torchrl.envs.utils import check_env_specs

from robotdesigner.config import (
    BaseEnvConfig,
    ConfigError,
    ControlConfig,
    EnvConfig,
    ExternalJoint,
    ExternalRobot,
    FixedBox,
    MujocoOptions,
    NativeGains,
    OwnReachable,
    Pose,
    SimConfig,
    TaskConfig,
)
from robotdesigner.config import MobileMount, Mount
from robotdesigner.config import kinematics as kin
from robotdesigner.envs import make_env
from robotdesigner.simulators.mujoco.compile import compile_robot
from robotdesigner.simulators.mujoco.external import compose
from robotdesigner.viz.camera import workspace_points
from tests.toy_provider import ReachParams
from tests.toy_robot_provider import PROVIDER

ROTATED = Pose((0.4, -0.3, 0.2), (float(np.cos(0.4)), 0.0, 0.0, float(np.sin(0.4))))
OTHER = Pose((-0.5, 0.6, 0.0), (float(np.cos(-0.9)), 0.0, 0.0, float(np.sin(-0.9))))


def to_mount(robot, pose):
    """The mount that places ``robot`` at ``pose`` (yaw only for a floating base) in a scene whose frame is the origin."""
    pose = pose or Pose.identity()
    if getattr(robot, "home_root", None) is None:
        return Mount(pose)
    return MobileMount(pose.pos, 2.0 * float(np.arctan2(pose.quat[3], pose.quat[0])))


def reach_cfg(robot, mount=None, mode="joint_pos_abs", num_envs=3, noise=0.0, region=None, dt=0.002, scale=None, seed=0):
    return EnvConfig(
        robot=robot,
        control=ControlConfig(mode, NativeGains(), scale),
        task=TaskConfig(
            ReachParams("reach", noise, False), 20, 0.02, region or FixedBox((0.2, -0.3, 0.0), (0.6, 0.3, 0.4)), mount=to_mount(robot, mount),
        ),
        sim=SimConfig("mujoco", num_envs, "cpu", dt, "euler", 50, seed, "plain_floor", options=MujocoOptions(False)),
    )


def idle_cfg(robot, mount, num_envs=4):
    return (
        BaseEnvConfig().robot(robot).task(mount=to_mount(robot, mount)).sim(num_envs=num_envs, scene="plain_floor").resolve()
    )


def err_of(build):
    with pytest.raises(ConfigError) as e:
        build()
    return str(e.value)


@pytest.fixture(scope="module")
def arm3():
    return PROVIDER.describe("arm3")


@pytest.fixture(scope="module")
def floater():
    return PROVIDER.describe("floater")


@pytest.fixture(scope="module")
def armfinger():
    return PROVIDER.describe("armfinger", driven=("j1", "j2"))


def world_ee(env):
    return np.stack([d.site_xpos[env.interface.ee_site_id] for d in env._sim.data])


# --- a fixed arm at a rotated mount --------------------------------------------------------------


def test_world_ee_is_the_mount_times_the_base_frame_ee(arm3):
    from robotdesigner.config.kinematics import pose_to_matrix

    base_ee = []
    for mount in (Pose.identity(), ROTATED, OTHER):
        env = make_env(reach_cfg(arm3, mount, num_envs=2))
        env.reset()
        sim = env._sim
        T = pose_to_matrix(mount)
        expected = kin.forward_ee_batch(arm3, np.array([arm3.home_qpos]), None)[0]
        assert np.allclose(sim.ee_pos_base()[0], expected, atol=1e-9)
        assert np.allclose(world_ee(env)[0], T[:3, :3] @ expected + T[:3, 3], atol=1e-9)
        assert np.allclose(kin.forward_ee_batch(arm3, np.array([arm3.home_qpos]), mount)[0], world_ee(env)[0], atol=1e-9)
        base_ee.append(sim.ee_pos_base()[0])
    assert np.allclose(base_ee[0], base_ee[1], atol=1e-9) and np.allclose(base_ee[0], base_ee[2], atol=1e-9)


def test_interface_of_a_fixed_arm(arm3):
    env = make_env(reach_cfg(arm3, ROTATED))
    i = env.interface
    assert i.n == 3 and i.action_dim == 3 and i.joint_names == ("ext_j1", "ext_j2", "ext_j3")
    assert np.allclose(i.kp, 100.0) and np.allclose(i.kd, 0.0)
    assert np.allclose(i.ctrl_range, [[-2, 2], [-1.5, 1.5], [-1.5, 1.5]]) and i.root_qpos_adr is None
    assert i.mount == ROTATED and np.allclose(i.home_qpos, arm3.home_qpos)
    names = [mujoco.mj_id2name(env._compiled.model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(a)) for a in i.actuator_ids]
    assert names == ["ext_a1", "ext_a2", "ext_a3"]


def test_check_env_specs(arm3, floater):
    check_env_specs(make_env(reach_cfg(arm3, ROTATED)))
    check_env_specs(make_env(idle_cfg(floater, ROTATED)))


# --- a floating base at three poses -----------------------------------------------------------------


@pytest.mark.parametrize("mount", [Pose.identity(), ROTATED, OTHER])
def test_free_root_starts_at_mount_times_home_root(floater, mount):
    env = make_env(idle_cfg(floater, mount, num_envs=4))
    env.reset()
    want = compose(mount, floater.home_root)
    a = env.interface.root_qpos_adr
    for d in env._sim.data:
        assert np.allclose(d.qpos[a : a + 3], want.pos, atol=1e-9)
        assert np.allclose(d.qpos[a + 3 : a + 7], want.quat, atol=1e-9)
        assert np.allclose(d.qpos[env.interface.qpos_adr], floater.home_qpos, atol=1e-9)
    assert env.interface.home_root == want and env.interface.ee_site_id is None


def test_masked_reset_returns_only_the_masked_roots(floater):
    env = make_env(idle_cfg(floater, ROTATED, num_envs=4))
    td = env.reset()
    for _ in range(30):
        td["action"] = torch.ones(4, 2)
        td = env.step(td)["next"]
    a = env.interface.root_qpos_adr
    want = np.asarray([*env.interface.home_root.pos, *env.interface.home_root.quat])
    roots = np.stack([d.qpos[a : a + 7] for d in env._sim.data])
    assert not np.allclose(roots, want, atol=1e-3)  # the robot moved
    before = roots.copy()
    td["_reset"] = torch.tensor([[True], [False], [True], [False]])
    env.reset(td)
    after = np.stack([d.qpos[a : a + 7] for d in env._sim.data])
    assert np.allclose(after[[0, 2]], want, atol=1e-9)
    assert np.array_equal(after[[1, 3]], before[[1, 3]])


# --- actuators ----------------------------------------------------------------------------------------


def test_actuators_that_are_not_driven_are_held(armfinger):
    env = make_env(reach_cfg(armfinger, num_envs=2))
    assert env.action_spec.shape == (2, 2) and env.interface.n == 2 and env.interface.action_dim == 2
    td = env.reset()
    model = env._compiled.model
    finger = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "ext_af")
    assert env.interface.held_ctrl[finger] == pytest.approx(0.03) and finger not in env.interface.actuator_ids
    for _ in range(50):
        td["action"] = env.action_spec.rand()
        td = env.step(td)["next"]
        assert all(d.ctrl[finger] == pytest.approx(0.03) for d in env._sim.data)
    check_env_specs(env)


def test_reset_noise_changes_only_driven_joints(armfinger):
    env = make_env(reach_cfg(armfinger, num_envs=4, noise=0.3, seed=3))
    env.reset()
    model = env._compiled.model
    finger_adr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "ext_finger")]
    driven = np.stack([d.qpos[env.interface.qpos_adr] for d in env._sim.data])
    assert np.ptp(driven[:, 0]) > 1e-3
    assert all(d.qpos[finger_adr] == pytest.approx(0.02) for d in env._sim.data)  # the keyframe value, no noise


def test_native_position_mapping_reaches_the_ctrlrange_not_the_joint_range(armfinger):
    # j1: joint range +-2 but servo ctrlrange +-3
    env = make_env(reach_cfg(armfinger, num_envs=1))
    td = env.reset()
    for sign, (lo, hi) in ((1.0, (None, 3.0)), (-1.0, (-3.0, None))):
        td["action"] = torch.tensor([[sign, sign]])
        env.step(td)
        j1 = env.interface.actuator_ids[0]
        assert env._sim.data[0].ctrl[j1] == pytest.approx(hi if sign > 0 else lo)
    assert env.interface.joint_range[0].tolist() == [-2.0, 2.0]


def test_native_torque_mapping_reaches_the_ctrlrange(floater):
    env = make_env(idle_cfg(floater, Pose.identity(), num_envs=1))
    assert env.cfg.control.mode == "joint_torque" and env.interface.kp is None and env.interface.kd is None
    td = env.reset()
    td["action"] = torch.tensor([[1.0, -1.0]])
    env.step(td)
    assert np.allclose(env._sim.data[0].ctrl[env.interface.actuator_ids], [1.0, -1.0])


def test_delta_mode_accumulates_a_clipped_target(arm3):
    env = make_env(reach_cfg(arm3, mode="joint_pos_delta", scale=0.5, num_envs=1))
    td = env.reset()
    for _ in range(10):
        td["action"] = torch.ones(1, 3)
        td = env.step(td)["next"]
    assert np.allclose(env.q_target[0], [2.0, 1.5, 1.5])


# --- determinism and tasks ------------------------------------------------------------------------------


def test_same_seed_same_rollout(arm3):
    outs = []
    for _ in range(2):
        env = make_env(reach_cfg(arm3, ROTATED, num_envs=2, noise=0.1, seed=7))
        env.set_seed(5)
        g = torch.Generator().manual_seed(1)
        td = env.reset()
        for _ in range(8):
            td["action"] = torch.rand(2, 3, generator=g) * 2 - 1
            td = env.step(td)["next"]
        outs.append(td["proprio", "qpos"].clone())
    assert torch.equal(outs[0], outs[1])


def test_own_reachable_targets_use_the_provider_oracle(arm3):
    env = make_env(reach_cfg(arm3, ROTATED, num_envs=64, region=OwnReachable()))
    td = env.reset()
    reach = torch.linalg.norm(td["task", "target_pos"] - torch.tensor([0.0, 0.0, 0.1]), dim=-1)
    assert float(reach.max()) <= 0.9 + 1e-6 and float(reach.min()) > 0.0


def test_the_camera_frames_an_external_robot(arm3):
    pts = workspace_points(reach_cfg(arm3, ROTATED))
    assert pts.ndim == 2 and pts.shape[1] == 3 and np.isfinite(pts).all()


def test_a_task_that_reads_the_end_effector_rejects_a_robot_without_one(floater):
    msg = err_of(lambda: make_env(reach_cfg(floater, mode="joint_torque", region=FixedBox((0, 0, 0), (1, 1, 1)))))
    assert "robot.end_effector" in msg


def test_the_end_effector_can_be_a_body_with_an_offset():
    robot = PROVIDER.describe("armfinger", driven=("j1", "j2"))
    assert robot.end_effector.body == "hand"
    env = make_env(reach_cfg(robot, ROTATED, num_envs=1))
    env.reset()
    expected = kin.forward_ee_batch(robot, np.array([robot.home_qpos]), ROTATED)[0]
    assert np.allclose(world_ee(env)[0], expected, atol=1e-9)


# --- problems ---------------------------------------------------------------------------------------------


def test_zero_actuator_model_is_a_problem():
    robot = ExternalRobot(
        provider="toyrobot", name="passive", entry="passive.xml", source=__import__("tests.toy_robot_provider", fromlist=["x"]).source_of(
            __import__("tests.toy_robot_provider", fromlist=["x"]).PASSIVE
        ),
        joints=(ExternalJoint("j1", (-2.0, 2.0), 10.0),), home_qpos=(0.0,), home_root=None, keyframe=None,
        end_effector=None, actuator_kind="position",
    )
    msg = err_of(lambda: make_env(idle_cfg(robot, Pose.identity())))
    assert "has no actuators" in msg and "robot.joints[0]" in msg


def test_wrong_actuator_kind_stale_source_unknown_joint_keyframe_and_entry(arm3):
    cfg = lambda robot, **kw: reach_cfg(robot, **kw)
    msg = err_of(lambda: make_env(cfg(replace(arm3, actuator_kind="torque"), mode="joint_torque")))
    assert "robot.actuator_kind" in msg and "'position'" in msg
    assert "robot.source: the model changed since this config was written" in err_of(lambda: make_env(cfg(replace(arm3, source="stale"))))
    bad_joint = replace(arm3, joints=(ExternalJoint("nope", (-2.0, 2.0), 40.0), *arm3.joints[1:]))
    assert "robot.joints[0].name: the model has no joint 'nope'" in err_of(lambda: make_env(cfg(bad_joint)))
    assert "robot.keyframe: the model has no keyframe 'away'" in err_of(lambda: make_env(cfg(replace(arm3, keyframe="away"))))
    assert "robot.entry" in err_of(lambda: make_env(cfg(replace(arm3, entry="other.xml"))))


def test_a_free_root_needs_home_root_and_a_fixed_base_must_not_have_one(arm3, floater):
    assert "robot.home_root" in err_of(lambda: make_env(idle_cfg(replace(floater, home_root=None), Pose.identity())))
    assert "robot.home_root" in err_of(lambda: make_env(reach_cfg(replace(arm3, home_root=Pose.identity()))))


def test_a_large_dt_is_reported_and_the_scene_keeps_dt_and_integrator(arm3):
    with pytest.warns(UserWarning, match="more than twice the timestep"):
        env = make_env(reach_cfg(arm3, dt=0.02))
    assert env._compiled.model.opt.timestep == 0.02
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        env = make_env(reach_cfg(arm3, dt=0.002))
    assert env._compiled.model.opt.timestep == 0.002 and env._compiled.model.opt.integrator == mujoco.mjtIntegrator.mjINT_EULER


def test_capabilities_list_the_external_kind():
    from robotdesigner.simulators import get_simulator

    assert get_simulator("mujoco").capabilities().robot_kinds == frozenset({"parametric", "external"})


def test_parametric_compile_still_works_next_to_an_external_one(arm3):
    from tests.test_mujoco_env import make_cfg

    assert compile_robot(make_cfg()).interface.ctrl_range is None


# --- our own actuators on an external robot (FixedGains, InertiaScaledGains) -------------------------------------------

from robotdesigner.config import FixedGains, InertiaScaledGains


def own_cfg(robot, gains, mode="joint_pos_abs", scale=None, num_envs=2, mount=None):
    cfg = reach_cfg(robot, mount, num_envs=num_envs)  # native gains in a mode that fits; the control is replaced below
    return replace(cfg, control=ControlConfig(mode, gains, scale))


def actuator_names(env):
    model = env._compiled.model
    return [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(a)) for a in env.interface.actuator_ids]


@pytest.mark.parametrize("mode,scale", [("joint_pos_abs", None), ("joint_pos_delta", 0.3), ("joint_vel", 1.0), ("joint_torque", None)])
def test_the_four_modes_run_on_an_external_arm_with_our_gains(arm3, mode, scale):
    env = make_env(own_cfg(arm3, FixedGains(80.0, 8.0), mode, scale))
    check_env_specs(env)
    assert actuator_names(env) == ["act_0", "act_1", "act_2"] and env.interface.n == 3
    td = env.reset()
    for _ in range(10):
        td["action"] = env.action_spec.rand()
        td = env.step(td)["next"]
    assert torch.isfinite(td["proprio", "qpos"]).all()
    if mode == "joint_torque":
        assert env.interface.kp is None
    else:
        assert np.allclose(env.interface.kp, 80.0) and np.allclose(env.interface.kd, 8.0)


def test_per_joint_gains_and_the_model_keeps_its_damping_armature_and_keyframe(arm3):
    env = make_env(own_cfg(arm3, FixedGains((10.0, 20.0, 30.0), (1.0, 2.0, 3.0))))
    assert np.allclose(env.interface.kp, [10, 20, 30]) and np.allclose(env.interface.kd, [1, 2, 3])
    model = env._compiled.model
    assert model.dof_damping[env.interface.qvel_adr[0]] == 0.5 and model.dof_armature[env.interface.qvel_adr[0]] == 0.01
    assert model.nu == 3 and np.allclose(model.actuator_forcerange[env.interface.actuator_ids], [[-40, 40], [-30, 30], [-20, 20]])
    env.reset()
    assert np.allclose(env._sim.data[0].qpos[env.interface.qpos_adr], arm3.home_qpos)


def test_actuators_that_are_not_driven_survive_and_are_held(armfinger):
    env = make_env(own_cfg(armfinger, FixedGains(60.0, 6.0), num_envs=2))
    model = env._compiled.model
    finger = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "ext_af")
    assert finger >= 0 and model.nu == 3 and env.interface.action_dim == 2
    assert env.interface.held_ctrl[finger] == pytest.approx(0.03) and finger not in env.interface.actuator_ids
    td = env.reset()
    for _ in range(50):
        td["action"] = env.action_spec.rand()
        td = env.step(td)["next"]
        assert all(d.ctrl[finger] == pytest.approx(0.03) for d in env._sim.data)
    assert actuator_names(env) == ["act_0", "act_1"]


def test_our_gains_equal_the_models_own_when_the_numbers_are_equal(arm3):
    native = make_env(reach_cfg(arm3, num_envs=1))
    own = make_env(own_cfg(arm3, FixedGains(100.0, 0.0), num_envs=1))
    rng = np.random.default_rng(0)
    acts = [torch.tensor(rng.uniform(-0.05, 0.05, (1, 3)), dtype=torch.float32) for _ in range(60)]
    traces = []
    for env in (native, own):
        td = env.reset()
        q = []
        for a in acts:
            td["action"] = a
            td = env.step(td)["next"]
            q.append(env._sim.data[0].qpos[env.interface.qpos_adr].copy())
        traces.append(np.array(q))
    assert np.abs(traces[0] - traces[1]).max() < 1e-9


def test_inertia_scaled_gains_use_the_mass_matrix_and_the_damping_of_the_model(arm3):
    w, zeta = 20.0, 1.0
    env = make_env(own_cfg(arm3, InertiaScaledGains(w, zeta)))
    model, iface = env._compiled.model, env.interface
    data = mujoco.MjData(model)
    data.qpos[iface.qpos_adr] = arm3.home_qpos
    mujoco.mj_forward(model, data)
    M = np.zeros((model.nv, model.nv))
    try:  # newer MuJoCo: mj_fullM(model, data, dst); older: mj_fullM(model, dst, qM)
        mujoco.mj_fullM(model, data, M)
    except TypeError:
        mujoco.mj_fullM(model, M, data.qM)
    m_diag = M[iface.qvel_adr, iface.qvel_adr]
    assert np.allclose(iface.kp, w**2 * m_diag, rtol=1e-9)
    damping = model.dof_damping[iface.qvel_adr]
    assert damping[0] == 0.5 and np.allclose(iface.kd, np.maximum(2 * zeta * np.sqrt(iface.kp * m_diag) - damping, 0.0))
    assert np.allclose(model.actuator_gainprm[iface.actuator_ids, 0], iface.kp)


def test_joint_torque_on_a_position_servo_arm_sags_and_is_clipped_by_the_effort_limit(arm3):
    native = make_env(reach_cfg(arm3, num_envs=1))
    torque = make_env(own_cfg(arm3, FixedGains(80.0, 8.0), "joint_torque", num_envs=1))
    for env in (native, torque):
        td = env.reset()
        for _ in range(100):
            td["action"] = torch.zeros(1, 3)
            td = env.step(td)["next"]
    q_native = native._sim.data[0].qpos[native.interface.qpos_adr]
    q_torque = torque._sim.data[0].qpos[torque.interface.qpos_adr]
    assert abs(q_native[1] - 0.0) < 0.2 and abs(q_torque[1] - arm3.home_qpos[1]) > 0.1  # one holds position, the other sags
    td = torque.reset()
    td["action"] = torch.ones(1, 3)
    torque.step(td)
    d = torque._sim.data[0]
    assert np.allclose(d.ctrl[torque.interface.actuator_ids], [40, 30, 20])  # the ends of the ctrlrange are the effort limits
    assert (np.abs(d.actuator_force[torque.interface.actuator_ids]) <= np.array([40, 30, 20]) + 1e-9).all()


def test_our_gains_on_a_floating_robot(floater):
    cfg = replace(idle_cfg(floater, ROTATED), control=ControlConfig("joint_torque", FixedGains(10.0, 1.0), None))
    env = make_env(cfg)
    td = env.reset()
    a = env.interface.root_qpos_adr
    want = compose(ROTATED, floater.home_root)
    assert np.allclose(env._sim.data[0].qpos[a : a + 3], want.pos)
    td["action"] = torch.ones(4, 2)
    env.step(td)
    assert actuator_names(env) == ["act_0", "act_1"] and env._compiled.model.nu == 2


def test_the_stale_source_check_still_applies_with_our_own_gains(arm3):
    msg = err_of(lambda: make_env(own_cfg(replace(arm3, source="stale"), FixedGains(60.0, 6.0))))
    assert "robot.source: the model changed since this config was written" in msg


def test_a_model_without_actuators_can_be_driven_with_our_gains():
    from tests.toy_robot_provider import PASSIVE, source_of

    robot = ExternalRobot(
        provider="toyrobot", name="passive", entry="passive.xml", source=source_of(PASSIVE),
        joints=(ExternalJoint("j1", (-2.0, 2.0), 10.0),), home_qpos=(0.0,), home_root=None, keyframe=None,
        end_effector=None, actuator_kind="position",
    )
    assert "has no actuators" in err_of(lambda: make_env(idle_cfg(robot, Pose.identity())))  # native: refused
    cfg = BaseEnvConfig().robot(robot).control(gains=FixedGains(60.0, 6.0)).sim(num_envs=2, scene="plain_floor").resolve()
    env = make_env(cfg)
    td = env.reset()
    td["action"] = torch.ones(2, 1)
    td = env.step(td)["next"]
    assert torch.isfinite(td["proprio", "qpos"]).all() and env._compiled.model.nu == 1


# --- rendering (needs an OpenGL context; MUJOCO_GL=egl on a headless machine) ---------------------------------------

from tests.test_render import needs_gl


@pytest.mark.render
@needs_gl
def test_an_external_robot_renders_a_frame_that_is_not_blank(arm3, floater):
    for robot, env in ((arm3, make_env(reach_cfg(arm3, ROTATED, num_envs=1))), (floater, make_env(idle_cfg(floater, ROTATED, num_envs=1)))):
        env.reset()
        frame = env.render(0, size=(96, 128))
        assert frame.shape == (96, 128, 3) and frame.dtype == np.uint8
        assert np.ptp(frame.reshape(-1, 3), axis=0).max() > 20, f"{robot.name}: the frame is blank"
        env.close()


def test_the_camera_frames_a_robot_without_an_end_effector(floater):
    pts = workspace_points(idle_cfg(floater, ROTATED))
    centre = compose(ROTATED, floater.home_root).pos
    assert np.isfinite(pts).all() and np.abs(pts - centre).max() < 1.5 and (np.abs(pts - centre).max(axis=1) <= 0.6 + 1e-9).sum() >= 8
