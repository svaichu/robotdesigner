import numpy as np
import pytest

from robotdesigner.mujoco.composer import build_arm
from robotdesigner.envs.abstract_mujoco_env import make_env


@pytest.fixture
def env_3dof():
    spec = build_arm(ndof=3, control="pos")
    env = make_env(spec, render_mode=None)
    yield env
    env.close()


@pytest.fixture
def env_5dof():
    spec = build_arm(ndof=5, control="torque")
    env = make_env(spec, render_mode=None)
    yield env
    env.close()


def test_make_env_creates_env(env_3dof):
    from robotdesigner.envs.abstract_mujoco_env import AbstractMujocoEnv
    assert isinstance(env_3dof, AbstractMujocoEnv)


@pytest.mark.parametrize("ndof", [1, 3, 5])
def test_action_space_shape(ndof):
    spec = build_arm(ndof=ndof)
    env = make_env(spec, render_mode=None)
    assert env.action_space.shape == (ndof,)
    env.close()


@pytest.mark.parametrize("ndof", [1, 3, 5])
def test_observation_space_shape(ndof):
    spec = build_arm(ndof=ndof)
    env = make_env(spec, render_mode=None)
    # obs = [ee_pos(3), target_pos(3), qpos(ndof), qvel(ndof), torque(ndof)]
    assert env.observation_space.shape == (6 + ndof * 3,)
    env.close()


def test_reset_returns_correct_obs_shape(env_3dof):
    obs, info = env_3dof.reset()
    assert obs.shape == (6 + 3 * 3,)
    assert isinstance(info, dict)


def test_step_returns_correct_tuple(env_3dof):
    env_3dof.reset()
    action = env_3dof.action_space.sample()
    obs, reward, terminated, truncated, info = env_3dof.step(action)
    assert obs.shape == (6 + 3 * 3,)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert isinstance(info, dict)


def test_reward_is_negative_distance(env_3dof):
    env_3dof.reset()
    action = np.zeros(3, dtype=np.float32)
    obs, reward, _, _, _ = env_3dof.step(action)
    ee_pos = obs[:3]
    target_pos = obs[3:6]
    expected = -float(np.linalg.norm(ee_pos - target_pos))
    assert abs(reward - expected) < 1e-5


def test_set_target(env_3dof):
    target = np.array([0.5, 0.1, 0.3], dtype=np.float32)
    env_3dof.set_target(target)
    obs, _ = env_3dof.reset()
    np.testing.assert_allclose(obs[3:6], target, atol=1e-6)


def test_random_rollout(env_5dof):
    obs, _ = env_5dof.reset()
    for _ in range(10):
        action = env_5dof.action_space.sample()
        obs, reward, terminated, truncated, _ = env_5dof.step(action)
        assert obs.shape == (6 + 5 * 3,)
        assert np.isfinite(reward)
        if terminated or truncated:
            break
