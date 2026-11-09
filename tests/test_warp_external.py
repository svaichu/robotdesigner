"""An ``ExternalRobot`` on the Warp simulator (Warp's CPU device here, CUDA where it exists: marker ``gpu``).

The robots are those of the test-only provider (no download); the Menagerie models are in ``test_menagerie_warp.py``.
"""

from dataclasses import replace

import numpy as np
import pytest

torch = pytest.importorskip("torch")
mujoco = pytest.importorskip("mujoco")
pytest.importorskip("torchrl")
pytest.importorskip("mujoco_warp")

from torchrl.envs.utils import check_env_specs

from robotdesigner.config import (
    ControlConfig,
    FixedGains,
    InertiaScaledGains,
    Pose,
    WarpOptions,
)
from robotdesigner.simulators import get_simulator
from robotdesigner.simulators.mujoco.external import compose
from robotdesigner.simulators.warp.sim import CapacityError
from tests.parity import build, on_warp, random_case, twin, with_sim
from tests.test_mujoco_external import ROTATED, idle_cfg, reach_cfg
from tests.test_warp_env import DEVICES
from tests.test_warp_parity import assert_parity
from tests.toy_robot_provider import PROVIDER

MODES = ["joint_pos_abs", "joint_pos_delta", "joint_vel", "joint_torque"]
SCALE = {"joint_pos_abs": None, "joint_pos_delta": 0.05, "joint_vel": 2.0, "joint_torque": None}


@pytest.fixture(scope="module")
def arm():
    return PROVIDER.describe("arm3d")  # damped servos: a stiff, undamped arm amplifies float32 rounding (see the docs)


@pytest.fixture(scope="module")
def floater():
    return PROVIDER.describe("floater")


def ours(cfg, mode, gains=None):
    return replace(cfg, control=ControlConfig(mode, gains or FixedGains(60.0, 6.0), SCALE[mode]))


def test_the_capability_and_the_check_of_the_model():
    assert get_simulator("warp").capabilities().robot_kinds == frozenset({"parametric", "external"})


@pytest.mark.parametrize("device", DEVICES)
@pytest.mark.parametrize("mode", ["joint_pos_abs", "joint_pos_delta"])
def test_native_gains_on_a_fixed_arm_at_a_rotated_mount(device, arm, mode):
    cfg = reach_cfg(arm, ROTATED, num_envs=4)
    if mode == "joint_pos_delta":
        cfg = replace(cfg, control=ControlConfig(mode, cfg.control.gains, 0.05))
    actions, inject = random_case(3, 4, 20)
    assert_parity(cfg, device, actions, inject)


@pytest.mark.parametrize("device", DEVICES)
@pytest.mark.parametrize("mode", MODES)
def test_our_gains_on_a_fixed_arm_in_every_mode(device, arm, mode):
    actions, inject = random_case(3, 4, 20, seed=1)
    if mode == "joint_torque":
        actions = actions * 0.1  # full-effort torques (20 to 40 N·m on 1 kg links) slam the arm into its joint limits
    assert_parity(ours(reach_cfg(arm, ROTATED, num_envs=4), mode), device, actions, inject)


@pytest.mark.parametrize("device", DEVICES)
def test_inertia_scaled_gains_on_an_external_arm(device, arm):
    actions, inject = random_case(3, 4, 20, seed=2)
    cfg = ours(reach_cfg(arm, num_envs=4), "joint_pos_abs", InertiaScaledGains(20.0, 1.0))
    assert_parity(cfg, device, actions, inject)


@pytest.mark.parametrize("device", DEVICES)
def test_a_floating_base_in_free_fall_agrees(device, floater):
    cfg = idle_cfg(floater, ROTATED, num_envs=4)
    actions, inject = random_case(2, 4, 12, seed=4)
    actions = actions * 0.02
    # The root starts 0.5 m up and lands after about 14 control steps. Until then there is no contact: the free joint, the
    # composed root pose and the legs agree to float32 rounding. After the landing the two tumble apart (see the docs): a
    # box and two capsules thrown onto the floor are a chaotic contact case, not a test of this robot kind.
    assert_parity(cfg, device, actions, inject)


def test_env_specs_and_held_actuators_on_warp(arm):
    armfinger = PROVIDER.describe("armfinger", driven=("j1", "j2"))
    env = build(on_warp(reach_cfg(armfinger, num_envs=2), "cpu"))
    check_env_specs(env)
    model = env._compiled.model
    finger = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "ext_af")
    td = env.reset()
    assert env.action_spec.shape == (2, 2)
    for _ in range(20):
        td["action"] = env.action_spec.rand()
        td = env.step(td)["next"]
        assert np.allclose(env._sim._d.ctrl.numpy()[:, finger], 0.03)
    env.close()


def test_a_masked_reset_returns_only_the_masked_roots(floater):
    env = build(on_warp(idle_cfg(floater, ROTATED, num_envs=4), "cpu"))
    td = env.reset()
    for _ in range(30):
        td["action"] = torch.ones(4, 2)
        td = env.step(td)["next"]
    a = env.interface.root_qpos_adr
    want = compose(ROTATED, floater.home_root)
    want = np.asarray([*want.pos, *want.quat], dtype=np.float32)
    before = env._sim._d.qpos.numpy()[:, a : a + 7].copy()
    assert not np.allclose(before, want, atol=1e-3)
    td["_reset"] = torch.tensor([[True], [False], [True], [False]])
    env.reset(td)
    after = env._sim._d.qpos.numpy()[:, a : a + 7]
    assert np.allclose(after[[0, 2]], want, atol=1e-6) and np.array_equal(after[[1, 3]], before[[1, 3]])
    env.close()


def test_the_reset_state_equals_the_cpu_reset(floater, arm):
    for robot, mode in ((floater, "joint_torque"), (PROVIDER.describe("armfinger", driven=("j1", "j2")), "joint_pos_abs")):
        cfg = idle_cfg(robot, ROTATED, num_envs=3) if robot is floater else reach_cfg(robot, ROTATED, num_envs=3)
        cpu, warp = build(twin(cfg)), build(on_warp(cfg, "cpu"))
        cpu.reset(), warp.reset()
        q_cpu = np.stack([d.qpos for d in cpu._sim.data])
        assert np.allclose(warp._sim._d.qpos.numpy(), q_cpu, atol=1e-6)
        assert np.allclose(warp._sim._d.ctrl.numpy(), np.stack([d.ctrl for d in cpu._sim.data]), atol=1e-6)
        cpu.close(), warp.close()


def test_too_small_a_contact_capacity_raises_an_error_that_names_the_option(floater):
    options = WarpOptions(njmax=128, nconmax=1, cuda_graph=False)
    env = build(with_sim(idle_cfg(floater, Pose.identity(), num_envs=4), simulator="warp", device="cpu", options=options))
    td = env.reset()
    with pytest.raises(CapacityError, match="sim.options.nconmax"):
        for _ in range(60):
            td["action"] = torch.zeros(4, 2)
            td = env.step(td)["next"]
    env.close()


def test_a_feature_the_library_does_not_run_is_a_problem_at_the_robot_path(arm, monkeypatch):
    from robotdesigner.simulators import warp as warp_module
    from robotdesigner.simulators.base import Problem
    from robotdesigner.simulators.warp import sim as warp_sim

    monkeypatch.setattr(warp_sim, "check_model_supported", lambda model: [Problem("task", "body plugins are not supported by the warp simulator")])
    problems = warp_module.WarpSimulator().validate(with_sim(reach_cfg(arm), simulator="warp", device="cuda:0", options=WarpOptions.default()))
    assert Problem("robot", "body plugins are not supported by the warp simulator") in problems


def test_provider_problems_reach_the_warp_validation_too(arm):
    from robotdesigner.simulators import warp as warp_module

    bad = reach_cfg(replace(arm, source="stale"))
    problems = warp_module.WarpSimulator().validate(with_sim(bad, simulator="warp", device="cuda:0", options=WarpOptions.default()))
    assert any(p.path == "robot.source" for p in problems)
