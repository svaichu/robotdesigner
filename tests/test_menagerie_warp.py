"""Menagerie robots on the Warp simulator against the CPU simulator (Warp's CPU device; CUDA where it exists, marker ``gpu``).

Only the cases that agree within the tolerances of ``test_warp_parity.py`` are asserted. The ones that do not are in
docs/simulators.md with their measured errors and the reason; they are not hidden by loosening a tolerance here.
"""

import warnings

import numpy as np
import pytest

mm = pytest.importorskip("mujoco_menagerie")
torch = pytest.importorskip("torch")
pytest.importorskip("mujoco")
pytest.importorskip("torchrl")
pytest.importorskip("mujoco_warp")

from robotdesigner.config import BaseEnvConfig, FixedGains, WarpOptions
from robotdesigner.robot.providers.menagerie import PROVIDER
from robotdesigner.simulators.warp.sim import CapacityError
from tests.parity import build, errors, on_warp, run, twin, with_sim
from tests.test_warp_env import DEVICES
from tests.test_warp_parity import TOL

pytestmark = pytest.mark.menagerie


def config(name, num_envs=4):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        robot = PROVIDER.describe(name)
    return robot, BaseEnvConfig().robot(robot).sim(num_envs=num_envs, scene="plain_floor")


def check(cfg, device, n, seed=0, scale=0.05, steps=30):
    actions = np.random.default_rng(seed).uniform(-1, 1, (steps, cfg.sim.num_envs, n)) * scale
    cpu, gpu = run(twin(cfg), actions, None), run(on_warp(cfg, device), actions, None)
    worst = errors(gpu, cpu)
    for key, err in worst.items():
        name = key.split("/")[-1]
        if name in TOL:
            assert err <= TOL[name], f"{key}: {err:.2e} > {TOL[name]:.0e}"
    return worst


@pytest.mark.parametrize("device", DEVICES)
@pytest.mark.parametrize("mode,scale", [("joint_pos_abs", None), ("joint_vel", 1.0), ("joint_torque", None)])
def test_panda_with_our_gains(device, mode, scale):
    robot, base = config("franka_emika_panda")
    check(base.control(gains=FixedGains(60.0, 6.0), mode=mode, action_scale=scale).resolve(), device, robot.n)


@pytest.mark.parametrize("device", DEVICES)
def test_ur5e_native_delta_and_our_velocity_mode(device):
    robot, base = config("universal_robots_ur5e")
    check(base.control(mode="joint_pos_delta", action_scale=0.05).resolve(), device, robot.n)
    check(base.control(gains=FixedGains(60.0, 6.0), mode="joint_vel", action_scale=1.0).resolve(), device, robot.n)


@pytest.mark.parametrize("device", DEVICES)
def test_go2_native_and_with_our_gains(device):
    robot, base = config("unitree_go2")
    check(base.resolve(), device, robot.n)  # plain motors, joint_torque, standing on the floor
    check(base.control(gains=FixedGains(60.0, 6.0), mode="joint_pos_abs").resolve(), device, robot.n)


def test_the_go2_root_and_ground_contact_start_the_same_in_both():
    _, base = config("unitree_go2", num_envs=3)
    cfg = base.resolve()
    cpu, warp = build(twin(cfg)), build(on_warp(cfg, "cpu"))
    cpu.reset(), warp.reset()
    a = cpu.interface.root_qpos_adr
    assert np.allclose(warp._sim._d.qpos.numpy()[:, a : a + 7], [*cpu.interface.home_root.pos, *cpu.interface.home_root.quat], atol=1e-6)
    assert np.allclose(warp._sim._d.qpos.numpy(), np.stack([d.qpos for d in cpu._sim.data]), atol=1e-6)
    cpu.close(), warp.close()


def test_a_go2_batch_with_a_small_contact_capacity_raises_an_error_that_names_it():
    _, base = config("unitree_go2")
    options = WarpOptions(njmax=512, nconmax=1, cuda_graph=False)
    env = build(with_sim(base.resolve(), simulator="warp", device="cpu", options=options))
    td = env.reset()
    with pytest.raises(CapacityError, match="sim.options.nconmax"):
        for _ in range(20):
            td["action"] = torch.zeros(4, 12)
            td = env.step(td)["next"]
    env.close()
