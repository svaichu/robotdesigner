"""Existing tasks on Menagerie robots, spawn poses and the contact-option policy (marker ``menagerie``; robosuite rows also ``robosuite``).

No task provider was changed for any of this.
"""

import importlib.util
import json
import warnings
from pathlib import Path

import numpy as np
import pytest

mm = pytest.importorskip("mujoco_menagerie")
mujoco = pytest.importorskip("mujoco")
torch = pytest.importorskip("torch")
pytest.importorskip("torchrl")

from torchrl.envs.utils import check_env_specs

from robotdesigner.config import BaseEnvConfig, ConfigError, MobileMount, Mount, Pose, world_mount
from robotdesigner.config.kinematics import pose_to_matrix
from robotdesigner.envs import make_env
from robotdesigner.robot.providers.menagerie import PROVIDER
from robotdesigner.simulators.mujoco.external import compose

pytestmark = pytest.mark.menagerie

ROOT = Path(__file__).resolve().parents[1]
OK = sorted(n for n, s in json.loads((ROOT / "tests" / "menagerie_expected.json").read_text()).items() if s == "ok")
YAW90 = Pose((0.4, -0.3, 0.0), (float(np.cos(np.pi / 4)), 0.0, 0.0, float(np.sin(np.pi / 4))))
SPAWN90 = MobileMount((0.4, -0.3), np.pi / 2)  # the same pose, as a floating base takes it


def describe(name, **kw):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return PROVIDER.describe(name, **kw)


def err_of(build):
    with pytest.raises(ConfigError) as e:
        build()
    return str(e.value)


# --- rule 1: empty/idle on every robot that runs ------------------------------------------------------------------


@pytest.mark.parametrize("name", OK)
def test_empty_idle_on_a_fixed_base_robot_in_the_empty_scene(name):
    robot = describe(name)
    if robot.home_root is not None:
        pytest.skip("a floating robot has no floor in the empty scene and falls; it runs on plain_floor in test_menagerie_provider.py")
    env = BaseEnvConfig().robot(robot).sim(num_envs=2, scene="empty").build()
    td = env.reset()
    for _ in range(5):
        td["action"] = env.action_spec.rand()
        td = env.step(td)["next"]
    assert torch.isfinite(td["proprio", "qpos"]).all()
    env.close()


# --- rule 2: robosuite/Lift with a library robot -------------------------------------------------------------------------


def lift_cfg(robot, num_envs=2, seed=0):
    from robotdesigner.tasks.robosuite import LiftObject, RobosuiteParams

    params = RobosuiteParams("Lift", {}, LiftObject("cube", 0.84), 0.0, False)
    return (
        BaseEnvConfig().robot(robot).task(params, mount=Mount()).sim(num_envs=num_envs, scene="empty", seed=seed).resolve()
    )


@pytest.mark.robosuite
@pytest.mark.parametrize("name", ["franka_emika_panda", "universal_robots_ur5e"])
def test_lift_builds_steps_and_places_the_cube_as_for_a_parametric_arm(name):
    pytest.importorskip("robosuite")
    from tests.test_robosuite_task import make_cfg

    robot = describe(name)
    check_env_specs(make_env(lift_cfg(robot, num_envs=3, seed=7)))  # a fresh env after it: the check draws from the task's stream
    env = make_env(lift_cfg(robot, num_envs=3, seed=7))
    td = env.reset()
    assert set(td["task"].keys()) == {"ee_pos", "cube_pos", "cube_quat"} and env.action_spec.shape == (3, robot.n)
    arm = make_env(make_cfg(num_envs=3, seed=7, base=env.cfg.task.mount))  # a ParametricArm at the same offset, same seed
    assert torch.equal(arm.reset()["task", "cube_pos"], td["task", "cube_pos"])  # the task draws do not depend on the robot
    ee = torch.from_numpy(env._sim.ee_pos_base()).float()
    assert torch.allclose(td["task", "ee_pos"], ee, atol=1e-5)  # the end effector is the robot's
    for _ in range(10):
        td["action"] = torch.zeros(3, robot.n)  # the fingers are held; Lift cannot be solved, it must only run
        td = env.step(td)["next"]
    assert torch.isfinite(td["task", "cube_pos"]).all()
    env.close()
    arm.close()


@pytest.mark.robosuite
def test_a_robot_without_an_end_effector_is_refused_by_a_task_that_reads_it():
    pytest.importorskip("robosuite")
    robot = describe("unitree_go2")
    assert robot.end_effector is None
    msg = err_of(lambda: make_env(lift_cfg(robot)))
    assert "robot.end_effector" in msg and "robosuite/Lift" in msg


# --- rule 3: the mount is the spawn pose ------------------------------------------------------------------------------------


def settle(env, steps, action=0.0):
    td = env.reset()
    zs = []
    d = env._sim.data[0]
    a = env.interface.root_qpos_adr
    for _ in range(steps):
        td["action"] = torch.full((env._sim.num_envs, env.interface.n), action)
        td = env.step(td)["next"]
        zs.append(float(d.qpos[a + 2]))
    return np.array(zs)


def standing(name, mount, **control):
    robot = describe(name)
    cfg = BaseEnvConfig().robot(robot).task(mount=mount).sim(num_envs=1, scene="plain_floor")
    if control:
        cfg = cfg.control(**control)
    return robot, make_env(cfg.resolve())


def test_a_fixed_arm_at_a_yawed_and_shifted_mount():
    from robotdesigner.config import kinematics as kin

    robot = describe("universal_robots_ur5e")
    env = make_env(BaseEnvConfig().robot(robot).task(mount=Mount(YAW90)).sim(num_envs=1, scene="plain_floor").resolve())
    env.reset()
    base_ee = kin.forward_ee_batch(robot, np.array([robot.home_qpos]), None)[0]
    T = pose_to_matrix(YAW90)
    assert np.allclose(env._sim.ee_pos_base()[0], base_ee, atol=1e-6)
    assert np.allclose(env._sim.data[0].site_xpos[env.interface.ee_site_id], T[:3, :3] @ base_ee + T[:3, 3], atol=1e-6)
    env.close()


@pytest.mark.parametrize("name", ["unitree_go1", "unitree_go2"])
def test_legged_robots_stand_where_the_keyframe_puts_them_at_the_default_mount(name):
    robot, env = standing(name, MobileMount())
    env.reset()
    a = env.interface.root_qpos_adr
    assert np.allclose(env._sim.data[0].qpos[a : a + 3], robot.home_root.pos, atol=1e-9)
    env.close()


def test_go1_holds_its_pose_at_the_default_mount():
    robot, env = standing("unitree_go1", MobileMount(), mode="joint_pos_delta", action_scale=0.05)
    zs = settle(env, 100)
    assert abs(zs[-1] - robot.home_root.pos[2]) < 0.01
    env.close()


@pytest.mark.parametrize("name", ["unitree_go1", "unitree_go2"])
def test_legged_robots_at_a_rotated_and_shifted_mount(name):
    robot, env = standing(name, SPAWN90)
    env.reset()
    want = compose(YAW90, robot.home_root)
    a = env.interface.root_qpos_adr
    d = env._sim.data[0]
    assert np.allclose(d.qpos[a : a + 3], want.pos, atol=1e-9) and np.allclose(d.qpos[a + 3 : a + 7], want.quat, atol=1e-9)
    # the base frame is the mount: in it the robot starts at the model's own home root
    in_base = (d.qpos[a : a + 3] - np.asarray(YAW90.pos)) @ pose_to_matrix(YAW90)[:3, :3]
    assert np.allclose(in_base, robot.home_root.pos, atol=1e-9)
    env.close()


@pytest.mark.parametrize("name", ["unitree_go1", "unitree_go2"])
def test_legged_robots_dropped_from_0_3_m_fall_and_land(name):
    mount = MobileMount((0.0, 0.0, 0.3))  # z is added to the home height
    robot, env = standing(name, mount)
    zs = settle(env, 100)
    assert env.interface.home_root.pos[2] == pytest.approx(robot.home_root.pos[2] + 0.3)
    assert zs[0] > robot.home_root.pos[2] + 0.2  # it starts above the floor, not in it
    assert zs.min() > 0.02 and np.isfinite(zs).all()  # and lands: the trunk never goes through the floor
    assert zs[-1] < robot.home_root.pos[2] + 0.05  # it came down
    env.close()


def test_tidybot_base_moves_along_the_mounts_x_axis():
    robot = describe("stanford_tidybot", effort_limit=1000.0)
    assert [j.name for j in robot.joints][:3] == ["joint_x", "joint_y", "joint_th"]
    env = make_env(
        BaseEnvConfig().robot(robot).task(mount=Mount(YAW90)).control(mode="joint_pos_delta", action_scale=0.01)
        .sim(num_envs=1, scene="plain_floor").resolve()
    )
    td = env.reset()
    model, data = env._compiled.model, env._sim.data[0]
    body = int(model.jnt_bodyid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "ext_joint_x")])
    start = data.xpos[body].copy()
    for _ in range(40):
        action = torch.zeros(1, robot.n)
        action[0, 0] = 1.0  # the base x slide
        td["action"] = action
        td = env.step(td)["next"]
    moved = data.xpos[body] - start
    direction = pose_to_matrix(YAW90)[:3, :3] @ np.array([1.0, 0.0, 0.0])
    assert np.linalg.norm(moved) > 0.05 and np.allclose(moved / np.linalg.norm(moved), direction, atol=0.05)
    env.close()


# --- rule 4: the option policy ------------------------------------------------------------------------------------------------


def _script():
    spec = importlib.util.spec_from_file_location("menagerie_options", ROOT / "benchmarks" / "menagerie_options.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_default_policy_keeps_the_scenes_options_and_the_legged_robots_stand():
    from robotdesigner.simulators.mujoco import external

    assert external.CARRY_MODEL_OPTIONS is False  # the decision of docs/simulators.md
    script = _script()
    for name, drift, tilt in (("unitree_go1", 0.01, 1.0), ("unitree_a1", 0.03, 1.0)):
        r = script.stand(name, carry=False, dt_factor=1, seconds=2.0)
        assert abs(r["drift"]) < drift and r["tilt"] < tilt and r["penetration"] < 0.03, (name, r)
        assert r["cone"] == "mjCONE_PYRAMIDAL" and r["impratio"] == 1.0  # the scene's, not the model's (elliptic, 100)


def test_carrying_the_options_is_possible_and_changes_the_solver_settings():
    script = _script()
    r = script.stand("unitree_go1", carry=True, dt_factor=1, seconds=0.5)
    assert r["cone"] == "mjCONE_ELLIPTIC" and r["impratio"] == 100.0
    assert script.external.CARRY_MODEL_OPTIONS is True
    script.external.CARRY_MODEL_OPTIONS = False
