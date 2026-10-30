import numpy as np
import pytest

from robotdesigner.config import ConfigError, Pose, TaskConfig, get_task_provider, list_task_providers
from robotdesigner.tasks.empty import EmptyParams


def test_discovery_and_config_side():
    assert "empty" in list_task_providers()
    provider = get_task_provider("empty")
    assert provider.task_names() == ("idle",) and provider.params_class("idle") is EmptyParams
    assert provider.params_class("busy") is None
    assert provider.default_mount("idle") == Pose.identity() and provider.default_mount("idle").quat == (1.0, 0.0, 0.0, 0.0)
    task = TaskConfig(EmptyParams("idle"), 10, None, None, Pose.identity())
    assert task.id == "empty/idle" and task.to_dict()["provider"] == "empty"
    assert TaskConfig.from_dict(task.to_dict()) == task
    with pytest.raises(ConfigError, match="^name"):
        EmptyParams("busy")
    with pytest.raises(ConfigError, match="success_tolerance"):
        TaskConfig(EmptyParams("idle"), 10, 0.02, None, Pose.identity())


torch = pytest.importorskip("torch")
pytest.importorskip("mujoco")
pytest.importorskip("torchrl")

from torchrl.envs.utils import check_env_specs  # noqa: E402

from robotdesigner import make_env  # noqa: E402
from robotdesigner.config import ControlConfig, EnvConfig, FixedGains, MujocoOptions, SimConfig, chain  # noqa: E402


def make_cfg(horizon=7, num_envs=3, simulator="mujoco"):
    arm = chain(1, "z", link_length=0.5, mass=1.0, radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0,
                effort_limit=50.0, range=(-2.0, 2.0))
    return EnvConfig(
        robot=arm,
        control=ControlConfig("joint_pos_abs", FixedGains(60.0, 6.0), None),
        task=TaskConfig(EmptyParams("idle"), horizon, None, None, Pose.identity()),
        sim=SimConfig(simulator, num_envs, "cpu", 0.002, "euler", 50, 0, "plain_floor", options=MujocoOptions(False)),
    )


def test_runs_a_one_link_arm_and_passes_the_spec_check():
    env = make_env(make_cfg())
    check_env_specs(env)
    td = env.reset()
    assert list(td["task"].keys()) == [] and td["proprio", "qpos"].shape == (3, 1)


def test_reward_zero_never_terminated_truncated_at_horizon():
    env = make_env(make_cfg(horizon=7))
    traj = env.rollout(7, break_when_any_done=False)
    assert torch.all(traj["next", "reward"] == 0)
    assert not traj["next", "terminated"].any() and not traj["next", "success"].any()
    assert traj["next", "truncated"][:, :6].sum() == 0 and traj["next", "truncated"][:, 6].all()


def test_draws_nothing_from_the_task_stream():
    env = make_env(make_cfg())
    state = env._task_gen.get_state().clone()
    env.reset()
    env.rollout(3, break_when_any_done=False)
    assert torch.equal(env._task_gen.get_state(), state)
