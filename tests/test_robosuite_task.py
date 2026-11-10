"""robosuite as a source of task assets on the MuJoCo simulation. Needs the robosuite extra: pytest -m robosuite."""

import numpy as np
import pytest

pytestmark = pytest.mark.robosuite
pytest.importorskip("robosuite")
torch = pytest.importorskip("torch")
mujoco = pytest.importorskip("mujoco")
pytest.importorskip("torchrl")

from torchrl.envs.utils import check_env_specs, step_mdp  # noqa: E402

from robotdesigner import make_env  # noqa: E402
from robotdesigner.config import world_mount  # noqa: E402
from robotdesigner.config import (  # noqa: E402
    ConfigError, ControlConfig, EnvConfig, FixedGains, MujocoOptions, Mount, Pose, SimConfig, TaskConfig, chain,
)
from robotdesigner.tasks.robosuite import LiftObject, ReachObject, RobosuiteParams, PROVIDER  # noqa: E402

COMMON = dict(radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0)
HIGH = Mount(Pose((0.0, 0.0, 0.288), (1.0, 0.0, 0.0, 0.0)))  # 0.288 m above the stock mount: the arm does not touch the scene


def make_cfg(name="Lift", objective=None, kwargs=None, num_envs=2, seed=0, scene="empty", base=HIGH, n=3, axes="zyy",
             mode="joint_pos_abs", max_episode_steps=50, noise=0.0):
    arm = chain(n, axes, total_length=0.9, total_mass=3.0, range=(-2.5, 2.5), **COMMON)
    objective = objective or LiftObject("cube", 0.84)
    return EnvConfig(
        robot=arm,
        control=ControlConfig(mode, FixedGains(60.0, 6.0), None),
        task=TaskConfig(RobosuiteParams(name, kwargs or {}, objective, noise, False), max_episode_steps, None, None, mount=base),
        sim=SimConfig("mujoco", num_envs, "cpu", 0.002, "euler", 50, seed, scene, options=MujocoOptions(False)),
    )


def body_world(env, name, i=0):
    _, body, _ = env.task._by_name[name]
    return env._sim.data[i].xpos[body].copy()


def test_env_specs_and_observation_layout():
    env = make_env(make_cfg(num_envs=3))
    check_env_specs(env)
    td = env.reset()
    assert set(td["task"].keys()) == {"ee_pos", "cube_pos", "cube_quat"}
    assert td["task", "cube_pos"].shape == (3, 3) and td["task", "cube_quat"].shape == (3, 4)
    assert env.task.object_names == ("cube",) and env.task.fixtures == ()
    env.close()


def test_cube_is_placed_by_robosuite_and_rests_on_the_table():
    env = make_env(make_cfg(num_envs=4, seed=1))
    td = env.reset()
    zero = torch.zeros(4, 3)
    for _ in range(30):  # 0.6 s
        td["action"] = zero
        td = step_mdp(env.step(td))
    for i in range(4):
        x, y, z = body_world(env, "cube", i)
        assert abs(x) <= 0.031 and abs(y) <= 0.031  # robosuite's Lift placement range around the table centre
        assert z == pytest.approx(0.8 + 0.0214, abs=0.003)  # table top 0.8 m, half the cube edge
    env.close()


def test_the_scene_itself_is_seeded():
    # robosuite draws the cube size when it builds the environment; the same config must give the same cube
    def rest_height(seed):
        env = make_env(make_cfg(num_envs=1, seed=seed))
        td = env.reset()
        for _ in range(30):
            td["action"] = torch.zeros(1, 3)
            td = step_mdp(env.step(td))
        z = float(body_world(env, "cube")[2])
        env.close()
        return z

    assert rest_height(2) == rest_height(2)


def test_placement_is_seeded_and_envs_differ():
    def first_reset(seed):
        env = make_env(make_cfg(num_envs=3, seed=seed))
        out = env.reset()["task", "cube_pos"].clone()
        env.close()
        return out

    a, b, c = first_reset(3), first_reset(3), first_reset(4)
    assert torch.equal(a, b) and not torch.equal(a, c)
    assert not torch.equal(a[0], a[1])
    env = make_env(make_cfg(num_envs=2, seed=0))
    env.set_seed(9)
    x = env.reset()["task", "cube_pos"].clone()
    env.set_seed(9)
    assert torch.equal(env.reset()["task", "cube_pos"], x)
    env.close()


def test_the_same_scene_is_placed_for_every_embodiment():
    # the placement comes from the task's own stream, so it does not depend on the robot's number of joints
    cubes = {}
    for n, axes in ((2, "zy"), (3, "zyy"), (5, "zyyyy")):
        env = make_env(make_cfg(num_envs=3, seed=2, n=n, axes=axes, noise=0.1))
        cubes[n] = [env.reset()["task", "cube_pos"].clone() for _ in range(2)]
        env.close()
    for n in (3, 5):
        assert all(torch.equal(a, b) for a, b in zip(cubes[2], cubes[n]))
    assert not torch.equal(cubes[2][0], cubes[2][1])


def test_masked_reset_only_replaces_the_masked_envs():
    env = make_env(make_cfg(num_envs=2))
    td = env.reset()
    td["action"] = torch.zeros(2, 3)
    out = env.step(td)
    before = out["next", "task", "cube_pos"].clone()
    td = step_mdp(out)
    td["_reset"] = torch.tensor([[True], [False]])
    after = env.reset(td)["task", "cube_pos"]
    assert torch.equal(after[1], before[1]) and not torch.equal(after[0], before[0])
    env.close()


def test_the_arm_collides_with_the_table():
    # base 1.2 m up, left of the cube, joint 1 commanded to 1.5 rad (the arm points almost straight down). In free
    # space the tip would end at z = 1.2 - 0.9 * sin(1.5) = 0.30; here the arm comes down on the table and rests on it.
    cfg = make_cfg(num_envs=1, base=Pose((-0.2, 0.0, 1.2), (1.0, 0.0, 0.0, 0.0)))
    env = make_env(cfg)
    td = env.reset()
    for _ in range(100):
        td["action"] = torch.tensor([[0.0, 1.5 / 2.5, 0.0]])  # joint 1 about y: +x turns towards -z
        td = step_mdp(env.step(td))
    sim = env._sim
    data, model = sim.data[0], sim.model
    pairs = {frozenset((model.geom(c.geom1).name, model.geom(c.geom2).name)) for c in data.contact[: data.ncon]}
    touching = [p for p in pairs if "rs_table_collision" in p and any(n.startswith("geom_") for n in p)]
    assert touching, pairs  # a robot link is in contact with the table
    assert data.site_xpos[env._iface.ee_site_id][2] > 0.5  # blocked: the free-space pose is at z = 0.30
    env.close()


def test_objectives_decide_success_and_reward_is_distance():
    def success_at_reset(objective):
        env = make_env(make_cfg(objective=objective, num_envs=2))
        td = env.reset()
        out = (td["success"].clone(), td["task", "ee_pos"].clone(), td["task", "cube_pos"].clone())
        td["action"] = torch.zeros(2, 3)
        reward = env.step(td)["next", "reward"]
        env.close()
        return out, reward

    (s, ee, cube), reward = success_at_reset(LiftObject("cube", 0.0))
    assert s.all() and reward.shape == (2, 1)
    assert torch.allclose(reward.squeeze(-1), -(ee - cube).norm(dim=-1), atol=0.05)  # distance, one step later
    assert not success_at_reset(LiftObject("cube", 1.0))[0][0].any()
    assert success_at_reset(ReachObject("cube", 5.0))[0][0].all()
    assert not success_at_reset(ReachObject("cube", 1e-3))[0][0].any()


def test_validate_collects_every_problem():
    cfg = make_cfg(scene="plain_floor", kwargs={"no_such_argument": 1}, objective=ReachObject("sphere", 0.1))
    with pytest.raises(ConfigError) as e:
        make_env(cfg)
    msg = str(e.value)
    assert "sim.scene" in msg and "no_such_argument" in msg and "2 problems" in msg
    with pytest.raises(ConfigError, match="objects in the scene: \\['cube'\\]"):
        make_env(make_cfg(objective=ReachObject("sphere", 0.1)))
    with pytest.raises(ConfigError, match="robosuite has no environment 'NoSuchEnv'"):
        make_env(make_cfg(name="NoSuchEnv"))
    with pytest.raises(ConfigError, match="more than one arm"):
        make_env(make_cfg(name="TwoArmLift"))
    with pytest.raises(ConfigError, match="different object every episode"):
        make_env(make_cfg(name="PickPlaceSingle"))


@pytest.mark.parametrize(
    "name,kwargs,objects",
    [
        ("Stack", {}, {"cubeA", "cubeB"}),
        ("NutAssemblySquare", {}, {"SquareNut", "RoundNut"}),
        ("PickPlace", {"single_object_mode": 2, "object_type": "can"}, {"Milk", "Bread", "Cereal", "Can"}),
        ("Door", {}, {"Door"}),
    ],
)
def test_other_robosuite_scenes(name, kwargs, objects):
    first = sorted(objects)[0]
    env = make_env(make_cfg(name=name, kwargs=kwargs, objective=ReachObject(first, 0.1), num_envs=2))
    check_env_specs(env)
    td = env.reset()
    assert set(env.task.object_names) == objects  # visual-only markers are not observed
    assert {k.rsplit("_", 1)[0] for k in td["task"].keys() if k != "ee_pos"} == objects
    for _ in range(5):
        td["action"] = env.action_spec.rand()
        td = step_mdp(env.step(td))
    assert all(torch.isfinite(v).all() for v in td["task"].values())
    env.close()


def test_fixtures_are_placed_once_at_bind_time():
    env = make_env(make_cfg(name="Door", objective=ReachObject("Door", 0.1)))
    assert env.task.fixtures == ("Door",)
    env.reset()
    world = body_world(env, "Door")
    assert np.linalg.norm(world) > 0.1  # moved off the model origin by robosuite's placement
    assert np.allclose(world, body_world(env, "Door", 1))  # one placement shared by every env
    env.reset()
    assert np.allclose(world, body_world(env, "Door"))  # and it does not change between episodes
    env.close()


def lift_params(**kw):
    return RobosuiteParams("Lift", kw, LiftObject("cube", 0.84), 0.0, False)


def test_mount_frame_is_where_robosuite_puts_a_stock_arm():
    frame = PROVIDER.mount_frame(lift_params())
    assert frame.pos == pytest.approx((-0.56, 0.0, 0.912)) and frame.quat == (1.0, 0.0, 0.0, 0.0)
    cfg = make_cfg(base=Mount())  # snapped onto the frame
    assert world_mount(cfg) == frame


def test_mount_frame_follows_the_scene_kwargs():
    base = PROVIDER.mount_frame(lift_params())
    wide = PROVIDER.mount_frame(lift_params(table_full_size=(1.0, 0.8, 0.05)))
    assert wide.pos[0] == pytest.approx(base.pos[0] - 0.1)  # half the change in table length
    a = make_cfg(kwargs={"table_full_size": (1.0, 0.8, 0.05)}, base=Mount())
    b = make_cfg(base=Mount())
    assert a.env_hash != b.env_hash and world_mount(a).pos[0] == pytest.approx(world_mount(b).pos[0] - 0.1)


def test_the_frame_is_the_same_for_two_seeds_and_is_read_once_per_scene(monkeypatch):
    import robotdesigner.tasks.robosuite as rs
    import robotdesigner.tasks.robosuite.scene as scene

    calls = []

    def counting(*args, **kw):
        calls.append(args)
        return scene.load_scene(*args, **kw)

    a = world_mount(make_cfg(seed=0, base=Mount()))
    b = world_mount(make_cfg(seed=123, base=Mount()))
    assert a == b
    rs._FRAMES.clear()
    monkeypatch.setattr(scene, "load_scene", counting)
    for _ in range(3):
        PROVIDER.mount_frame(lift_params())
    assert len(calls) == 1


def stock_base_pose(env_name):
    """What robosuite itself says: the root of a stock Panda in ``env_name``."""
    import robosuite

    from robotdesigner.tasks.robosuite._compat import apply_compat
    from robotdesigner.tasks.robosuite.scene import matrix_to_quat

    apply_compat()
    env = robosuite.make(env_name, robots="Panda", has_renderer=False, has_offscreen_renderer=False, use_camera_obs=False)
    try:
        robot = env.robots[0]
        return np.asarray(robot.base_pos, dtype=float), matrix_to_quat(np.asarray(robot.base_ori, dtype=float))
    finally:
        env.close()


def single_arm_names():
    from robotdesigner.tasks.robosuite import _introspect as intro

    return [n for n in intro.single_arm_env_names() if not n.endswith("Single")]


@pytest.mark.parametrize("name", ["PickPlace", "Lift", "Door", "Stack"])
def test_the_frame_is_the_pose_of_a_stock_arm_built_by_robosuite_itself(name):
    objective = ReachObject("cube", 0.05)
    frame = PROVIDER.mount_frame(RobosuiteParams(name, {}, objective, 0.0, False))
    pos, quat = stock_base_pose(name)
    assert np.allclose(frame.pos, pos, atol=1e-9)
    assert np.allclose(frame.quat, quat, atol=1e-9) or np.allclose(frame.quat, -np.asarray(quat), atol=1e-9)


def test_the_frame_matches_every_environment_that_builds():
    objective = ReachObject("cube", 0.05)
    checked = []
    for name in single_arm_names():
        try:
            frame = PROVIDER.mount_frame(RobosuiteParams(name, {}, objective, 0.0, False))
            pos, quat = stock_base_pose(name)
        except Exception:  # an environment robosuite cannot build here: not a property of the frame
            continue
        assert np.allclose(frame.pos, pos, atol=1e-9), name
        assert np.isclose(abs(np.dot(frame.quat, quat)), 1.0, atol=1e-9), name
        checked.append(name)
    assert "Lift" in checked and len(checked) >= 8


def test_the_compiled_scene_has_rs_mount_at_the_frame_and_the_base_is_on_it():
    cfg = make_cfg(base=Mount(), num_envs=1)
    env = make_env(cfg)
    env.reset()
    model, data = env._compiled.model, env._sim.data[0]
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "rs_mount")
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    assert site >= 0 and env.task.mount_site() == "rs_mount"
    assert np.allclose(data.site_xpos[site], (-0.56, 0.0, 0.912), atol=1e-9)
    assert np.allclose(data.xpos[base], data.site_xpos[site], atol=1e-9)
    assert np.allclose(data.xmat[base], data.site_xmat[site], atol=1e-9)
    assert env.world_mount.pos == pytest.approx((-0.56, 0.0, 0.912))
    env.close()


def test_base_frame_observations_match_a_hand_placed_base_at_the_same_pose():
    snapped = make_env(make_cfg(base=Mount(), num_envs=2, seed=3))
    placed = make_env(make_cfg(base=Mount(Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))), num_envs=2, seed=3))
    a, b = snapped.reset(), placed.reset()
    assert torch.allclose(a["task", "ee_pos"], b["task", "ee_pos"], atol=1e-6)
    assert torch.allclose(a["task", "cube_pos"], b["task", "cube_pos"], atol=1e-6)
    snapped.close()
    placed.close()


def test_control_modes_and_time_step_are_ours_not_robosuite_s():
    # robosuite fixes dt = 0.002 and joint_pos_delta; here the sim config decides
    import dataclasses

    cfg = make_cfg(mode="joint_torque", num_envs=1)
    cfg = dataclasses.replace(cfg, sim=dataclasses.replace(cfg.sim, dt=0.005, control_freq=40.0))
    env = make_env(cfg)
    assert env._sim.model.opt.timestep == pytest.approx(0.005)
    td = env.reset()
    td["action"] = torch.zeros(1, 3)
    env.step(td)
    env.close()


def test_action_space_is_the_joint_count_for_a_robosuite_task():
    # robot.tool is ignored by the MuJoCo simulator, so no gripper dimensions are added to the n joint actions
    for n, axes in ((2, "zy"), (4, "zyyy")):
        env = make_env(make_cfg(n=n, axes=axes, num_envs=2))
        assert env.action_space == torch.Size([n])
        assert env.batch_size + env.action_space == env.action_spec.shape
        assert env.observation_space["task"]["ee_pos"] == torch.Size([3])
        assert env.observation_space["task"]["cube_pos"] == torch.Size([3])
        env.close()
