"""robosuite backend. Needs the robosuite extra; run with ``pytest -m robosuite``."""

import dataclasses

import numpy as np
import pytest

pytestmark = pytest.mark.robosuite
mujoco = pytest.importorskip("mujoco")
pytest.importorskip("robosuite")
torch = pytest.importorskip("torch")
pytest.importorskip("torchrl")

from torchrl.envs.utils import check_env_specs  # noqa: E402

from robotdesigner.config import (  # noqa: E402
    ConfigError,
    ControlConfig,
    EnvConfig,
    FixedGains,
    InertiaScaledGains,
    Pose,
    RobosuiteOptions,
    RobosuiteParams,
    SimConfig,
    TaskConfig,
    ToolSpec,
    chain,
)
from robotdesigner.config import kinematics as kin  # noqa: E402
from robotdesigner.envs import make_env  # noqa: E402

HOME = (0.0, -1.062284, 1.908116, 0.724964)  # from our kinematics: gripper axis down, hand over the table centre


def make_arm(gripper="PandaGripper", base=(0.0, 0.0, 1.1)):
    return chain(
        4,
        "zyyy",
        link_length=[0.10, 0.40, 0.40, 0.15],
        total_mass=8.0,
        radius=0.04,
        damping=1.0,
        armature=0.05,
        frictionloss=0.0,
        effort_limit=80.0,
        range=(-2.9, 2.9),
        home_qpos=HOME,
        base_pose=Pose(base, (1.0, 0.0, 0.0, 0.0)),
        end_effector=Pose((0.0, 0.0, 0.0), (0.70710678, 0.0, 0.70710678, 0.0)),
        tool=ToolSpec(gripper, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
    )


def make_cfg(task="Lift", kwargs=None, num_envs=2, arm=None, noise=0.0, seed=3, placement="env_default"):
    return EnvConfig(
        robot=arm or make_arm(),
        control=ControlConfig("joint_pos_delta", InertiaScaledGains(7.0, 1.0), 0.05),
        task=TaskConfig(f"robosuite/{task}", 60, None, None, RobosuiteParams(kwargs or {}, noise, False)),
        sim=SimConfig("robosuite", num_envs, "cpu", 0.002, "euler", 20.0, seed, "env"),
        backend_options=RobosuiteOptions(placement),
    )


def hand_pose(env, i=0):
    e = env.robosuite_envs[i]
    m, d = e.sim.model._model, e.sim.data._data
    bid = lambda name: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)  # noqa: E731
    return (
        d.xpos[bid("robot0_base")].copy(),
        d.xpos[bid("robot0_right_hand")].copy(),
        d.xmat[bid("robot0_right_hand")].reshape(3, 3).copy(),
    )


# action dimension = joints + the gripper's degrees of freedom (the wiping tool has none)
@pytest.mark.parametrize(
    "task, gripper, kwargs, action_dim",
    [
        ("Lift", "PandaGripper", {}, 5),
        ("Stack", "PandaGripper", {}, 5),
        ("Door", "PandaGripper", {}, 5),
        ("Wipe", "WipingGripper", {}, 4),
        ("PickPlace", "PandaGripper", {"single_object_mode": 2, "object_type": "can"}, 5),
    ],
)
def test_any_single_arm_env_runs_with_our_robot(task, gripper, kwargs, action_dim):
    env = make_env(make_cfg(task, kwargs, arm=make_arm(gripper)))
    try:
        check_env_specs(env)
        td = env.reset()
        for _ in range(3):
            td["action"] = env.action_spec.rand()
            td = env.step(td)["next"]
        assert td["reward"].shape == (2, 1) and torch.isfinite(td["proprio", "qpos"]).all()
        assert env.action_spec.shape == (2, action_dim)
        assert env.env_name == task and env.robot_hash == env.cfg.robot.config_hash
        assert env.resolved_env_kwargs  # the library defaults the run used are recorded, not hidden
    finally:
        env.close()


def test_robot_in_robosuite_is_our_robot():
    env = make_env(make_cfg(num_envs=1))
    try:
        td = env.reset()
        root, hand, rot = hand_pose(env)
        fk = kin.forward(env.cfg.robot, td["proprio", "qpos"][0].double().numpy()).ee
        np.testing.assert_allclose(root + fk[:3, 3], hand, atol=1e-6)  # env_default: the stock table mount at x = -0.56
        np.testing.assert_allclose(fk[:3, :3], rot, atol=1e-6)
        assert abs(root[0] + 0.56) < 1e-9  # the stock mount point of a table task
        assert rot[2, 2] < -0.99  # at home the gripper axis points down
        e = env.robosuite_envs[0]
        m, d = e.sim.model._model, e.sim.data._data
        rng = np.random.default_rng(0)
        for q in kin.sample_joint_positions(
            env.cfg.robot, 20, rng
        ):  # also away from home, set directly in the simulation
            for i in range(4):
                d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"robot0_joint_{i}")]] = q[i]
            mujoco.mj_kinematics(m, d)
            root, hand, rot = hand_pose(env)
            fk = kin.forward(env.cfg.robot, q).ee
            np.testing.assert_allclose(root + fk[:3, 3], hand, atol=1e-9)
            np.testing.assert_allclose(fk[:3, :3], rot, atol=1e-9)
    finally:
        env.close()


def test_explicit_placement_puts_the_arm_where_base_pose_says():
    stock = make_env(make_cfg(num_envs=1))
    explicit = make_env(make_cfg(num_envs=1, arm=make_arm(base=(-0.56, 0.0, 1.1)), placement="explicit"))
    try:
        stock.reset(), explicit.reset()
        root, hand_a, _ = hand_pose(stock)
        root_b, hand_b, _ = hand_pose(explicit)
        assert np.abs(root_b).max() < 1e-12 and abs(root[0] + 0.56) < 1e-9
        np.testing.assert_allclose(hand_a, hand_b, atol=1e-9)
    finally:
        stock.close(), explicit.close()


def test_control_holds_and_tracks_the_accumulated_target():
    env = make_env(make_cfg(num_envs=2))
    try:
        zero = torch.zeros(tuple(env.action_spec.shape))
        zero[:, -1] = -1.0  # gripper
        td = env.reset()
        q0 = td["proprio", "qpos"].clone()
        for _ in range(30):
            td["action"] = zero
            td = env.step(td)["next"]
        assert (td["proprio", "qpos"] - q0).abs().max() < 1e-3  # gravity is compensated

        td = env.reset()
        q0 = td["proprio", "qpos"].clone()
        push = zero.clone()
        push[:, 0] = 1.0  # +0.05 rad of target per step on the first joint, for 10 steps
        for k in range(60):
            td["action"] = push if k < 10 else zero
            td = env.step(td)["next"]
        np.testing.assert_allclose(env.q_target[:, 0] - q0[:, 0].numpy(), 0.5, atol=1e-5)
        assert (td["proprio", "qpos"][:, 0] - q0[:, 0]).sub(0.5).abs().max() < 0.02
    finally:
        env.close()


def test_seeds_masked_reset_and_injected_state():
    def cubes(seed, masks):
        env = make_env(make_cfg(num_envs=2, seed=seed, noise=0.02))
        try:
            td = env.reset()
            out = [td["task", "cube_pos"].clone()]
            for mask in masks:
                td["_reset"] = torch.tensor(mask).reshape(2, 1)
                td = env.reset(td)
                out.append(td["task", "cube_pos"].clone())
            return out
        finally:
            env.close()

    a, b, other = cubes(5, [[True, True]]), cubes(5, [[True, True]]), cubes(6, [[True, True]])
    assert all(torch.equal(x, y) for x, y in zip(a, b))  # same seed, same episodes
    assert not torch.equal(a[0], other[0]) and not torch.equal(a[0][0], a[0][1])
    kept = cubes(5, [[False, True]])
    assert torch.equal(kept[1][1], a[1][1])  # sub-env 1's stream does not depend on what happens to sub-env 0
    assert torch.equal(kept[1][0], kept[0][0])  # a masked-out sub-env keeps its state

    env = make_env(make_cfg(num_envs=2))
    try:
        td = env.reset()
        td["qpos"] = torch.tensor([[0.1, -1.0, 1.8, 0.7]] * 2)
        td["qvel"] = torch.tensor([[0.0] * 4, [0.2, 0.0, 0.0, 0.0]])
        out = env.reset(td)
        torch.testing.assert_close(out["proprio", "qpos"], td["qpos"], atol=1e-6, rtol=0)
        assert abs(out["proprio", "qvel"][1, 0].item() - 0.2) < 1e-6
    finally:
        env.close()


def test_validation_lists_every_problem():
    good = make_cfg(num_envs=1)
    bad = dataclasses.replace(
        good,
        task=dataclasses.replace(good.task, id="robosuite/TwoArmLift"),
        sim=dataclasses.replace(good.sim, dt=0.001, scene="plain_floor"),
        control=ControlConfig("joint_pos_delta", FixedGains(50.0, 5.0), 0.05),
        robot=dataclasses.replace(good.robot, tool=None),
    )
    with pytest.raises(ConfigError) as e:
        make_env(bad)
    text = str(e.value)
    for path in ("sim.dt", "sim.scene", "control.gains", "robot.tool", "task.id"):
        assert path in text
    with pytest.raises(ConfigError, match="does not accept .*reward_shapin"):
        make_env(make_cfg(kwargs={"reward_shapin": True}))
    with pytest.raises(ConfigError, match="no environment 'Nope'"):
        make_env(make_cfg("Nope"))
    with pytest.raises(ConfigError, match="different object every episode"):
        make_env(make_cfg("PickPlaceSingle", num_envs=1))
    with pytest.raises(ConfigError, match="WipingGripper"):  # robosuite's own constraint, reported as a config error
        make_env(make_cfg("Wipe", num_envs=1))
