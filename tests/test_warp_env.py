"""WarpEnv. Runs on Warp's CPU device here and on CUDA where it exists (marker ``gpu``)."""

from dataclasses import replace

import numpy as np
import pytest

torch = pytest.importorskip("torch")
mujoco = pytest.importorskip("mujoco")
pytest.importorskip("torchrl")
pytest.importorskip("mujoco_warp")

from torchrl.envs.utils import check_env_specs, step_mdp  # noqa: E402

from robotdesigner.config import BaseEnvConfig, Chain, ConfigError, ControlConfig, FixedGains, SimConfig, WarpOptions  # noqa: E402
from robotdesigner.envs import make_env  # noqa: E402
from robotdesigner.simulators.base import task_seed  # noqa: E402
from robotdesigner.simulators.warp.env import WarpEnv  # noqa: E402
from tests.test_mujoco_env import make_cfg  # noqa: E402
from tests.test_render import needs_gl  # noqa: E402

DEVICES = ["cpu", pytest.param("cuda:0", marks=[pytest.mark.gpu, pytest.mark.skipif(
    not torch.cuda.is_available(), reason="needs CUDA")])]
OPTIONS = WarpOptions(njmax=64, nconmax=16, cuda_graph=False)


def warp_env(cfg, device, options=OPTIONS):
    """Build a WarpEnv directly: make_env rejects the cpu device, which only the tests use."""
    from robotdesigner.config.providers import get_task_provider

    s = cfg.sim
    sim = SimConfig("warp", s.num_envs, device, s.dt, s.integrator, s.control_freq, s.seed, s.scene, options)
    cfg = replace(cfg, sim=sim)
    return WarpEnv(cfg, get_task_provider(cfg.task.params.provider).build(cfg, "warp"))


def with_mode(cfg, mode):
    scale = {"joint_pos_abs": None, "joint_pos_delta": 0.05, "joint_vel": 2.0, "joint_torque": None}[mode]
    return replace(cfg, control=ControlConfig(mode, FixedGains(60.0, 6.0), scale))


@pytest.mark.parametrize("device", DEVICES)
def test_check_env_specs_and_devices(device):
    env = warp_env(make_cfg(n=2, num_envs=3), device)
    check_env_specs(env)
    td = env.rollout(5)
    assert td.batch_size == torch.Size([3, 5]) and td.device == torch.device(device)
    assert env.action_space == torch.Size([2]) and env.device == torch.device(device)
    env.close()


@pytest.mark.parametrize("device", DEVICES)
def test_empty_idle_task(device):
    cfg = BaseEnvConfig().robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0)).sim(num_envs=4).resolve()
    env = warp_env(cfg, device)
    check_env_specs(env)
    env.close()


@pytest.mark.parametrize("device", DEVICES)
@pytest.mark.parametrize("mode", ["joint_pos_abs", "joint_pos_delta", "joint_vel", "joint_torque"])
def test_every_control_mode_steps(device, mode):
    env = warp_env(with_mode(make_cfg(n=2, num_envs=3), mode), device)
    td = env.reset()
    td["action"] = torch.full((3, 2), 0.5, device=env.device)
    nxt = env.step(td)["next"]
    assert torch.isfinite(nxt["proprio", "qpos"]).all() and (nxt["proprio", "qpos"] != td["proprio", "qpos"]).any()
    if mode == "joint_pos_delta":
        assert env.q_target.shape == (3, 2) and env.q_target.dtype == np.float64
    env.close()


@pytest.mark.parametrize("device", DEVICES)
def test_masked_reset(device):
    env = warp_env(make_cfg(num_envs=4, noise=0.2), device)
    td = env.reset()
    for _ in range(5):
        td["action"] = torch.ones(4, 2, device=env.device)
        td = step_mdp(env.step(td))
    before = td.clone()
    td["_reset"] = torch.tensor([[True], [False], [True], [False]], device=env.device)
    out = env.reset(td)
    keep = torch.tensor([False, True, False, True], device=env.device)
    assert torch.equal(out["proprio", "qpos"][keep], before["proprio", "qpos"][keep])
    assert torch.equal(out["episode_step"][keep], before["episode_step"][keep])
    assert (out["episode_step"][~keep] == 0).all() and not out["done"].any()
    env.close()


@pytest.mark.parametrize("device", DEVICES)
def test_reset_agrees_with_the_cpu_simulator(device):
    cfg = make_cfg(n=4, num_envs=8, noise=0.1)
    cpu, warp = make_env(cfg), warp_env(cfg, device)
    a, b = cpu.reset(), warp.reset()
    assert torch.equal(a["task", "target_pos"], b["task", "target_pos"].cpu())
    assert torch.allclose(a["proprio", "qpos"], b["proprio", "qpos"].cpu(), atol=1e-6)
    assert torch.allclose(a["task", "ee_pos"], b["task", "ee_pos"].cpu(), atol=1e-5)
    warp.close()


@pytest.mark.parametrize("device", DEVICES)
def test_same_seed_same_rollout(device):
    def run():
        env = warp_env(make_cfg(num_envs=3, noise=0.1), device)
        env.set_seed(7)
        torch.manual_seed(0)  # the random actions of rollout
        td = env.rollout(10)
        env.close()
        return td.cpu()

    a, b = run(), run()
    assert torch.allclose(a["next", "proprio", "qpos"], b["next", "proprio", "qpos"], atol=1e-5)
    assert torch.equal(a["task", "target_pos"], b["task", "target_pos"])


@pytest.mark.parametrize("device", DEVICES)
def test_injected_state(device):
    env = warp_env(make_cfg(num_envs=3), device)
    td = env.reset()
    td["target"] = torch.tensor([[0.3, 0.1, 0.2]] * 3, device=env.device)
    td["qpos"] = torch.tensor([[0.5, -0.5]] * 3, device=env.device)
    td["qvel"] = torch.tensor([[0.1, 0.2]] * 3, device=env.device)
    out = env.reset(td)
    assert torch.allclose(out["task", "target_pos"], td["target"])
    assert torch.allclose(out["proprio", "qpos"], td["qpos"], atol=1e-6)
    assert torch.allclose(out["proprio", "qvel"], td["qvel"], atol=1e-6)
    env.close()


@pytest.mark.parametrize("device", DEVICES)
def test_step_and_maybe_reset_over_a_short_episode(device):
    env = warp_env(make_cfg(num_envs=2, max_episode_steps=3, terminate=True), device)
    td = env.reset()
    for _ in range(7):
        td["action"] = torch.zeros(2, 2, device=env.device)
        _, td = env.step_and_maybe_reset(td)
    assert (td["episode_step"] <= 3).all() and torch.isfinite(td["proprio", "qpos"]).all()
    env.close()


def test_make_env_reports_the_missing_gpu_and_the_cpu_task_lists_warp():
    cfg = replace(make_cfg(), sim=SimConfig("warp", 2, "cuda:0", 0.002, "euler", 50, 0, "plain_floor", OPTIONS))
    if not torch.cuda.is_available():
        with pytest.raises(ConfigError, match="CUDA is not available"):
            make_env(cfg)


def test_the_task_stream_seed_is_shared_by_both_simulators():
    from robotdesigner.simulators.mujoco import env as cpu_env

    assert cpu_env.task_seed is task_seed and task_seed(0) == 0x5DEECE66D


@pytest.mark.render
@needs_gl
@pytest.mark.parametrize("device", DEVICES)
def test_render_matches_the_cpu_env_for_the_same_state(device):
    cfg = make_cfg(num_envs=2)
    cpu, warp = make_env(cfg), warp_env(cfg, device)
    state = {"qpos": torch.tensor([[0.4, -0.6]] * 2), "target": torch.tensor([[0.3, 0.1, 0.2]] * 2)}
    for env in (cpu, warp):
        td = env.reset()
        for k, v in state.items():
            td[k] = v.to(env.device)
        env.reset(td)
    a = cpu.render(1, size=(48, 64)).astype(int)
    b = warp.render(1, size=(48, 64)).astype(int)
    assert a.shape == b.shape == (48, 64, 3)
    assert np.abs(a - b).mean() < 1.0  # float32 rounding moves a few edge pixels at most
    warp.close()
    cpu.close()


@pytest.mark.parametrize("device", DEVICES)
def test_a_too_small_constraint_capacity_raises_capacity_error(device):
    from robotdesigner.config import Pose, TaskConfig
    from robotdesigner.simulators.warp.sim import CapacityError

    cfg = make_cfg(n=4, num_envs=4, max_episode_steps=100)
    cfg = replace(cfg, task=replace(cfg.task, mount=Pose((0.0, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0))))
    cfg = with_mode(replace(cfg, sim=replace(cfg.sim, scene="plain_floor")), "joint_torque")
    env = warp_env(cfg, device, WarpOptions(njmax=1, nconmax=16, cuda_graph=False))
    td = env.reset()
    action = torch.zeros(4, 4, device=env.device)
    action[:, 1], action[:, 3] = 0.6, 0.3
    with pytest.raises(CapacityError, match="njmax=1"):
        for _ in range(60):  # the arm presses onto the floor: a few constraint rows appear
            td["action"] = action
            td = env.step(td)["next"]
    env.close()
