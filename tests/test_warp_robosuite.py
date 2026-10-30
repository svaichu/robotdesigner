"""robosuite/Lift on the warp simulator. Needs the robosuite extra. Runs on Warp's CPU device and, with CUDA, on the GPU."""

from dataclasses import replace

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("mujoco")
pytest.importorskip("torchrl")
pytest.importorskip("mujoco_warp")
pytest.importorskip("robosuite")
pytestmark = pytest.mark.robosuite

from torchrl.envs.utils import check_env_specs  # noqa: E402

from robotdesigner.config import ConfigError  # noqa: E402
from robotdesigner.envs import make_env  # noqa: E402
from tests.parity import build, on_warp, twin, with_sim  # noqa: E402
from tests.test_robosuite_task import make_cfg  # noqa: E402
from tests.test_warp_env import DEVICES  # noqa: E402


def cube_z(env):
    """World height of the cube in every env."""
    body = env.task._by_name["cube"][1]
    return env._sim.body_pose(body, "world")[0][:, 2].cpu().double().numpy()


def settle(env, steps, n):
    td = env.reset()
    for _ in range(steps):
        td["action"] = torch.zeros(env.batch_size[0], n, device=env.device)
        td = env.step(td)["next"]
    return td


@pytest.mark.parametrize("device", DEVICES)
@pytest.mark.parametrize("n,axes", [(3, "zyy"), (7, "zyyzyyz")])
def test_builds_and_passes_check_env_specs(device, n, axes):
    env = build(on_warp(make_cfg(n=n, axes=axes, num_envs=64), device))
    check_env_specs(env)
    td = env.rollout(5)
    assert td.batch_size == torch.Size([64, 5]) and env.action_space == torch.Size([n])
    env.close()


@pytest.mark.parametrize("device", DEVICES)
def test_the_cube_at_rest_stays_at_rest_and_agrees_with_the_cpu(device):
    cfg = make_cfg(num_envs=3)
    warp, cpu = build(on_warp(cfg, device)), make_env(cfg)
    td = warp.reset()
    heights = []
    for step in range(100):
        td["action"] = torch.zeros(3, 3, device=warp.device)
        td = warp.step(td)["next"]
        if step in (49, 99):
            heights.append(cube_z(warp))
    assert np.abs(heights[1] - heights[0]).max() < 1e-3  # after the initial settling it does not move
    assert (heights[1] > 0.7).all()  # still on the table (its top is at 0.8)
    settle(cpu, 100, 3)
    assert np.allclose(heights[1], cube_z(cpu), atol=2e-3)
    warp.close()
    cpu.close()


@pytest.mark.parametrize("device", DEVICES)
def test_a_dropped_cube_rests_at_the_cpu_height(device):
    cfg = make_cfg(num_envs=2)
    rest = {}
    for name, c in (("cpu", twin(cfg)), ("warp", on_warp(cfg, device))):
        env = build(c)
        td = env.reset()
        _, body, adr = env.task._by_name["cube"]
        x, y, z = env._sim.body_pose(body, "world")[0][0].cpu().double().tolist()
        pose = torch.tensor([[x, y, z + 0.05, 1.0, 0.0, 0.0, 0.0]] * 2, dtype=torch.float64)  # released 5 cm up
        env._sim.write_qpos(adr, pose, torch.ones(2, dtype=torch.bool, device=env.device))
        for _ in range(60):
            td["action"] = torch.zeros(2, 3, device=env.device)
            td = env.step(td)["next"]
        rest[name] = cube_z(env)
        env.close()
    assert np.abs(rest["warp"] - rest["cpu"]).max() < 2e-3


def test_scene_problems_are_reported_together():
    cfg = replace(make_cfg(num_envs=2), sim=replace(make_cfg().sim, simulator="warp", device="cuda:0",
                                                   options=on_warp(make_cfg(), "cuda:0").sim.options, scene="plain_floor"))
    import torch as _t

    with pytest.raises(ConfigError) as e:
        make_env(cfg)
    msg = str(e.value)
    assert "sim.scene" in msg and ("sim.device" in msg or _t.cuda.is_available())


@pytest.mark.parametrize("device", DEVICES)
def test_a_too_small_contact_capacity_raises_capacity_error(device):
    from robotdesigner.config import WarpOptions
    from robotdesigner.simulators.warp.sim import CapacityError

    cfg = on_warp(make_cfg(num_envs=2), device)
    cfg = replace(cfg, sim=replace(cfg.sim, options=WarpOptions(njmax=64, nconmax=1, cuda_graph=False)))
    env = build(cfg)
    td = env.reset()
    with pytest.raises(CapacityError, match="nconmax=1"):
        for _ in range(20):  # the cube rests on the table: 4+ contacts per world against 1 allocated
            td["action"] = torch.zeros(2, 3, device=env.device)
            td = env.step(td)["next"]
    env.close()
