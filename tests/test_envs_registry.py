import pytest

from robotdesigner.envs import list_mjx_environments, get_mjx_default_config, load_mjx_environment
from robotdesigner.mujoco.composer import build_arm


def test_list_mjx_environments_contains_default():
    envs = list_mjx_environments()
    assert isinstance(envs, tuple)
    assert "AbstractArmMjx" in envs


def test_get_mjx_default_config():
    config = get_mjx_default_config("AbstractArmMjx")
    assert hasattr(config, "ctrl_dt")
    assert hasattr(config, "sim_dt")
    assert hasattr(config, "episode_length")


def test_get_mjx_default_config_unknown_env():
    with pytest.raises(ValueError, match="not found"):
        get_mjx_default_config("NonExistentEnv")


jax = pytest.importorskip("jax")


def test_load_mjx_environment_properties():
    spec = build_arm(ndof=3, control="pos")
    env = load_mjx_environment("AbstractArmMjx", spec=spec)
    assert env.action_size == 3
    assert env.observation_size == 6 + 3 * 3


def test_mjx_reset_obs_shape():
    spec = build_arm(ndof=3, control="pos")
    env = load_mjx_environment("AbstractArmMjx", spec=spec)
    rng = jax.random.PRNGKey(0)
    state = env.reset(rng)
    assert state.obs.shape == (6 + 3 * 3,)


def test_mjx_step_reward_finite():
    spec = build_arm(ndof=2, control="torque")
    env = load_mjx_environment("AbstractArmMjx", spec=spec)
    rng = jax.random.PRNGKey(42)
    state = env.reset(rng)
    action = jax.numpy.zeros(env.action_size)
    state = env.step(state, action)
    assert jax.numpy.isfinite(state.reward)
