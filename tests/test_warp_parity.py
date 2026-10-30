"""CPU-versus-Warp parity from the same injected state (the CPU simulator is the reference).

Tolerances are measured, not guessed: the sweep behind them (4 control modes x 5 seeds, 3 other integrators, inertia-scaled
gains with n=7, an arm resting on the floor; 20-40 policy steps, 4-6 envs, run on Warp's CPU device) gave these largest
errors, and each tolerance sits about an order of magnitude above the largest one:

    quantity        largest measured   tolerance
    qpos            8.2e-06            1e-4      (float32 state against float64)
    qvel            9.0e-05            1e-3
    effort          9.5e-04            1e-2      (actuator force; gains of 60 / 6 amplify float32 rounding)
    ee_pos, reward  1.4e-06            1e-4
    target_pos      0 (exact)          0

A mismatch that is not float32 rounding is a bug or a capability to remove from ``Capabilities``; it is never a reason to
loosen a tolerance. Re-run the sweep on a CUDA device before trusting these numbers there (marker ``gpu``).
"""

from dataclasses import replace

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("mujoco")
pytest.importorskip("torchrl")
pytest.importorskip("mujoco_warp")

from robotdesigner.config import BaseEnvConfig, Chain, InertiaScaledGains, Pose, chain  # noqa: E402
from robotdesigner.simulators import get_simulator  # noqa: E402
from tests.parity import errors, on_warp, random_case, run, twin, with_control, with_sim  # noqa: E402
from tests.test_mujoco_env import make_cfg  # noqa: E402
from tests.test_warp_env import DEVICES  # noqa: E402

TOL = {"qpos": 1e-4, "qvel": 1e-3, "effort": 1e-2, "ee_pos": 1e-4, "reward": 1e-4, "target_pos": 0.0}
MODES = ["joint_pos_abs", "joint_pos_delta", "joint_vel", "joint_torque"]
INTEGRATORS = ["euler", "rk4", "implicit", "implicitfast"]
COMMON = dict(radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0)


def assert_parity(cfg, device, actions, inject, tol=TOL):
    cpu, gpu = run(twin(cfg), actions, inject), run(on_warp(cfg, device), actions, inject)
    for key, err in errors(gpu, cpu).items():
        name = key.split("/")[-1]
        if name in tol:
            assert err <= tol[name], f"{key}: {err:.2e} > {tol[name]:.0e}"
    return cpu, gpu


def reach(n=4, num_envs=6, **sim):
    return with_sim(make_cfg(n=n, num_envs=num_envs, max_episode_steps=100), scene="empty", **sim)


@pytest.mark.parametrize("device", DEVICES)
@pytest.mark.parametrize("mode", MODES)
def test_goal_task_in_every_control_mode(device, mode):
    actions, inject = random_case(4, 6, 20)
    assert_parity(with_control(reach(), mode), device, actions, inject)


@pytest.mark.parametrize("device", DEVICES)
@pytest.mark.parametrize("mode", MODES)
def test_contact_free_dynamics_of_the_empty_task(device, mode):
    cfg = BaseEnvConfig().robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0, range=(-2.5, 2.5))).sim(num_envs=5).resolve()
    actions, inject = random_case(3, 5, 20, seed=3)
    cpu, gpu = assert_parity(with_control(cfg, mode), device, actions, inject)
    assert not any(k.startswith("task/") for k in cpu)  # nothing but the robot is compared


@pytest.mark.parametrize("device", DEVICES)
def test_inertia_scaled_gains(device):
    arm = chain(7, "zyyzyyz", total_length=0.9, total_mass=3.0, range=(-2.5, 2.5), **COMMON)
    cfg = replace(reach(num_envs=4), robot=arm)
    cfg = with_control(cfg, "joint_pos_abs", InertiaScaledGains(bandwidth=20.0, damping_ratio=1.0))
    actions, inject = random_case(7, 4, 20, seed=1)
    assert_parity(cfg, device, actions, inject)


@pytest.mark.parametrize("device", DEVICES)
@pytest.mark.parametrize("integrator", INTEGRATORS)
def test_integrators(device, integrator):
    actions, inject = random_case(4, 6, 20)
    assert_parity(with_control(reach(integrator=integrator), "joint_pos_abs"), device, actions, inject)


@pytest.mark.parametrize("device", DEVICES)
def test_arm_pressed_onto_the_floor(device):
    cfg = make_cfg(n=4, num_envs=4, max_episode_steps=100)
    cfg = replace(cfg, task=replace(cfg.task, mount=Pose((0.0, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0))))
    cfg = with_control(with_sim(cfg, scene="plain_floor"), "joint_torque")
    actions = np.zeros((40, 4, 4))
    actions[:, :, 1], actions[:, :, 3] = 0.6, 0.3
    inject = {"qpos": np.zeros((4, 4)), "qvel": np.zeros((4, 4))}
    cpu, gpu = assert_parity(cfg, device, actions, inject)
    rest_z = cpu["task/ee_pos"][-1][:, 2] + 0.3  # world height of the tip: held up by the floor (soft contact: ~3 mm in)
    assert ((rest_z > 0.02) & (rest_z < 0.04)).all()
    assert np.allclose(gpu["task/ee_pos"][-1][:, 2] + 0.3, rest_z, atol=1e-4)


@pytest.mark.parametrize("device", DEVICES)
def test_random_resets_agree(device):
    cfg = with_control(make_cfg(n=4, num_envs=8, noise=0.1), "joint_pos_abs")
    actions = np.zeros((1, 8, 4))
    cpu, gpu = assert_parity(cfg, device, actions, inject=None)


def test_capabilities_list_exactly_what_passes():
    caps = get_simulator("warp").capabilities()
    assert caps.control_modes == frozenset(MODES) and caps.integrators == frozenset(INTEGRATORS)
