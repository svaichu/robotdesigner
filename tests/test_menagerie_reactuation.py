"""The gate of the re-actuation spike (#52) on real models: our own gains on a Menagerie arm (marker ``menagerie``)."""

import json
import warnings
from pathlib import Path

import numpy as np
import pytest

mm = pytest.importorskip("mujoco_menagerie")
pytest.importorskip("mujoco")
torch = pytest.importorskip("torch")
pytest.importorskip("torchrl")

from robotdesigner.config import BaseEnvConfig, FixedGains, InertiaScaledGains
from robotdesigner.robot.providers.menagerie import PROVIDER

pytestmark = pytest.mark.menagerie
EXPECTED = json.loads(Path(__file__).with_name("menagerie_expected.json").read_text())


def _base(robot):
    return BaseEnvConfig().robot(robot).sim(num_envs=1, scene="plain_floor")


@pytest.mark.parametrize("name", ["franka_emika_panda", "universal_robots_ur5e"])
def test_our_gains_with_the_models_numbers_reproduce_the_native_trajectory(name):
    """Same control targets, 200 steps: the actuators we add are the actuators the model has."""
    robot = PROVIDER.describe(name)
    native = _base(robot).build()
    kp, kd = native.interface.kp.copy(), native.interface.kd.copy()
    ours = _base(robot).control(gains=FixedGains(tuple(kp), tuple(kd))).build()
    assert np.allclose(ours.interface.kp, kp) and np.allclose(ours.interface.kd, kd)
    rng = np.random.default_rng(0)
    lo, hi = native.interface.joint_range[:, 0], native.interface.joint_range[:, 1]
    targets = [rng.uniform(lo, hi)[None] * 0.5 for _ in range(200)]  # the same targets for both, in ctrl space
    traces = []
    for env in (native, ours):
        env.reset()
        q = []
        for t in targets:
            env._sim.set_ctrl(t)
            env._sim.step()
            q.append(env._sim.data[0].qpos[env.interface.qpos_adr].copy())
        traces.append(np.array(q))
        env.close()
    assert np.abs(traces[0] - traces[1]).max() < 1e-6


@pytest.mark.parametrize("name", [n for n, s in EXPECTED.items() if s == "ok"])
def test_inertia_scaled_gains_compile_and_step_on_every_robot_that_runs(name):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        robot = PROVIDER.describe(name)
        env = _base(robot).control(gains=InertiaScaledGains(bandwidth=20.0, damping_ratio=1.0)).build()
        td = env.reset()
        for _ in range(10):
            td["action"] = env.action_spec.rand()
            td = env.step(td)["next"]
        assert torch.isfinite(td["proprio", "qpos"]).all()
        env.close()
