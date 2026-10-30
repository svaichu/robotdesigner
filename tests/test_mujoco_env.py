from dataclasses import replace

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("mujoco")
pytest.importorskip("torchrl")

from torchrl.envs.utils import check_env_specs  # noqa: E402

from robotdesigner.config import (  # noqa: E402
    Pose,
    ControlConfig, EnvConfig, FixedBox, FixedGains, MujocoOptions, OwnReachable, SimConfig, TaskConfig, chain,
)
from tests.toy_provider import ReachParams
from robotdesigner.envs import make_env  # noqa: E402

COMMON = dict(radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0)


def make_cfg(n=2, num_envs=3, seed=0, horizon=20, region=None, mode="joint_pos_abs", noise=0.0, terminate=False):
    arm = chain(n, "zy" * (n // 2), total_length=0.9, total_mass=2.0, range=(-2.0, 2.0), **COMMON)
    return EnvConfig(
        robot=arm,
        control=ControlConfig(mode, FixedGains(60.0, 6.0), None),
        task=TaskConfig(ReachParams("reach", noise, terminate), horizon, 0.02,
                        region or FixedBox((0.2, -0.3, 0.0), (0.6, 0.3, 0.4)), mount=Pose.identity()),
        sim=SimConfig("mujoco", num_envs, "cpu", 0.002, "euler", 50, seed, "plain_floor", options=MujocoOptions(False)),
    )


@pytest.mark.parametrize("n", [2, 4])
def test_check_env_specs(n):
    env = make_env(make_cfg(n=n, num_envs=3))
    assert env.batch_size == torch.Size([3]) and env.device == torch.device("cpu")
    assert len(env.robot_hash) == 12 and len(env.env_hash) == 12
    check_env_specs(env)


def test_reset_step_shapes_and_rollout():
    env = make_env(make_cfg(n=2, num_envs=3))
    td = env.reset()
    assert td["task", "ee_pos"].shape == (3, 3) and td["proprio", "qpos"].shape == (3, 2)
    td["action"] = env.action_spec.rand()
    td = env.step(td)
    nxt = td["next"]
    assert nxt["reward"].shape == (3, 1) and nxt["episode_step"].tolist() == [[1]] * 3
    assert nxt["task", "ee_pos"].dtype == torch.float32
    traj = env.rollout(5)
    assert traj.shape == (3, 5)


def test_masked_reset():
    env = make_env(make_cfg(num_envs=3, noise=0.0))
    td = env.reset()
    for _ in range(4):
        td["action"] = torch.ones(3, 2)
        td = env.step(td)["next"]
    before = td.clone()
    td["_reset"] = torch.tensor([[True], [False], [False]])
    out = env.reset(td)
    assert torch.allclose(out["proprio", "qpos"][0], torch.zeros(2), atol=1e-6)
    assert out["episode_step"][0].item() == 0
    assert torch.equal(out["proprio", "qpos"][1:], before["proprio", "qpos"][1:])
    assert torch.equal(out["episode_step"][1:], before["episode_step"][1:])
    assert torch.equal(out["task", "target_pos"][1:], before["task", "target_pos"][1:])


def test_determinism():
    runs = []
    for _ in range(2):
        env = make_env(make_cfg(num_envs=2, noise=0.1, seed=5))
        env.set_seed(11)
        g = torch.Generator().manual_seed(3)
        td = env.reset()
        target = td["task", "target_pos"].clone()
        obs = []
        for _ in range(6):
            td["action"] = torch.rand(2, 2, generator=g) * 2 - 1
            td = env.step(td)["next"]
            obs.append(td["proprio", "qpos"].clone())
        runs.append((target, torch.stack(obs)))
    assert torch.equal(runs[0][0], runs[1][0]) and torch.equal(runs[0][1], runs[1][1])
    other = make_env(make_cfg(num_envs=2, noise=0.1, seed=5))
    other.set_seed(12)
    assert not torch.equal(other.reset()["task", "target_pos"], runs[0][0])


def test_task_draws_do_not_depend_on_the_robot():
    # the task has its own random stream: with the same seed, two embodiments get the same targets, reset after reset,
    # although their reset noise has a different shape
    targets = {}
    for n in (2, 4):
        env = make_env(make_cfg(n=n, num_envs=3, noise=0.1, seed=7))
        draws = [env.reset()["task", "target_pos"].clone() for _ in range(3)]
        env.set_seed(21)
        draws.append(env.reset()["task", "target_pos"].clone())
        targets[n] = torch.stack(draws)
        env.close()
    assert torch.equal(targets[2], targets[4])
    assert not torch.equal(targets[2][0], targets[2][1])  # and the stream does advance


def test_injected_state():
    env = make_env(make_cfg(num_envs=3))
    td = env.reset()
    td["target"] = torch.tensor([[0.3, 0.1, 0.2]] * 3)
    td["qpos"] = torch.tensor([[0.5, -0.5]] * 3)
    td["qvel"] = torch.tensor([[0.1, 0.2]] * 3)
    out = env.reset(td)
    assert torch.allclose(out["task", "target_pos"], td["target"])
    assert torch.allclose(out["proprio", "qpos"], td["qpos"], atol=1e-6)
    assert torch.allclose(out["proprio", "qvel"], td["qvel"], atol=1e-6)


def test_reward_success_truncation():
    env = make_env(make_cfg(num_envs=2, horizon=3, terminate=True))
    td = env.reset()
    ee = td["task", "ee_pos"]
    # target at the current ee position: success immediately
    td["target"] = ee.clone()
    td = env.reset(td)
    td["action"] = torch.zeros(2, 2)
    nxt = env.step(td)["next"]
    assert nxt["reward"].abs().max() < 0.05
    td = nxt
    td["action"] = torch.zeros(2, 2)
    for _ in range(2):
        nxt = env.step(td)["next"]
        td = nxt
        td["action"] = torch.zeros(2, 2)
    assert nxt["truncated"].all() and nxt["done"].all()


def test_own_reachable_targets_are_reachable():
    env = make_env(make_cfg(n=2, num_envs=64, region=OwnReachable()))
    t = env.reset()["task", "target_pos"].numpy()
    assert np.linalg.norm(t, axis=1).max() <= 0.9 + 1e-5  # within total arm length of the base


def test_action_mapping_modes():
    pos = make_env(make_cfg(num_envs=1))
    assert np.allclose(pos._scale, 2.0) and np.allclose(pos._offset, 0.0)
    tq = make_env(make_cfg(num_envs=1, mode="joint_torque"))
    assert np.allclose(tq._scale, 50.0)
    check_env_specs(tq)


def test_make_env_rejects_unsupported():
    cfg = make_cfg()
    bad = replace(cfg, control=ControlConfig("ee_pos_delta", FixedGains(1.0, 1.0), None),
                  sim=replace(cfg.sim, device="cuda:0"))
    from robotdesigner.config import ConfigError

    with pytest.raises(ConfigError) as e:
        make_env(bad)
    assert "control.mode" in str(e.value) and "sim.device" in str(e.value)
