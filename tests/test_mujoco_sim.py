import numpy as np
import pytest

pytest.importorskip("mujoco")

from robotdesigner.simulators.mujoco.compile import compile_robot  # noqa: E402
from robotdesigner.simulators.mujoco.sim import BatchedSim  # noqa: E402
from robotdesigner.config import Pose, chain  # noqa: E402
from robotdesigner.config import kinematics as kin  # noqa: E402
from tests.test_mujoco_compile import COMMON, make_cfg  # noqa: E402


def make_sim(num_envs=3, mount=None):
    arm = chain(2, "zy", link_length=0.5, mass=1.0, range=(-2.0, 2.0), **COMMON)
    cfg = make_cfg(arm, mode="joint_torque", mount=mount, num_envs=num_envs)
    return BatchedSim(compile_robot(cfg), num_envs, cfg.sim.n_substeps), cfg, arm


def test_masked_reset_touches_only_selected():
    sim, _, _ = make_sim()
    sim.set_ctrl([[5.0, 1.0], [-5.0, 2.0], [3.0, -4.0]])
    for _ in range(5):
        sim.step()
    q, v, t = sim.qpos.copy(), sim.qvel.copy(), sim.time.copy()
    sim.reset(mask=np.array([True, False, False]))
    assert np.array_equal(sim.qpos[1:], q[1:]) and np.array_equal(sim.qvel[1:], v[1:])
    assert np.array_equal(sim.time[1:], t[1:])
    assert np.allclose(sim.qpos[0], 0.0) and np.allclose(sim.qvel[0], 0.0) and sim.time[0] == 0.0
    assert not np.any(sim.data[0].ctrl)


def test_determinism():
    seq = np.random.default_rng(0).uniform(-3, 3, size=(20, 3, 2))
    q0 = np.random.default_rng(1).uniform(-1, 1, size=(3, 2))
    out = []
    for _ in range(2):
        sim, _, _ = make_sim()
        sim.reset(qpos=q0)
        for c in seq:
            sim.set_ctrl(c)
            sim.step()
        out.append(sim.qpos)
    assert np.array_equal(out[0], out[1])


def test_time_bookkeeping():
    sim, cfg, _ = make_sim()
    for k in range(1, 4):
        sim.step()
        assert np.allclose(sim.time, k / cfg.sim.control_freq)


def test_ee_pos_base_matches_oracle_with_offset_base():
    base = Pose((0.3, 0.1, 0.5), (0.8, 0.2, 0.4, 0.1))
    sim, _, arm = make_sim(mount=base)
    q = np.random.default_rng(2).uniform(-1, 1, size=(3, 2))
    sim.reset(qpos=q)
    T = kin.pose_to_matrix(base)
    world = kin.forward_ee_batch(arm, q, base)
    expected = (world - T[:3, 3]) @ T[:3, :3]
    assert np.allclose(sim.ee_pos_base(), expected, atol=1e-9)


def test_shapes_and_validation():
    sim, _, _ = make_sim()
    assert sim.qpos.shape == (3, 2) and sim.actuator_force.shape == (3, 2) and sim.ee_pos_base().shape == (3, 3)
    with pytest.raises(ValueError):
        sim.set_ctrl(np.zeros((2, 2)))
    with pytest.raises(ValueError):
        sim.reset(mask=np.ones(2, dtype=bool))
