"""WarpBatchedSim. Runs on Warp's CPU device (what CI can do) and, with a CUDA device, on the GPU too (marker ``gpu``)."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")
mujoco = pytest.importorskip("mujoco")
pytest.importorskip("mujoco_warp")

from robotdesigner.config import Pose, WarpOptions, chain  # noqa: E402
from robotdesigner.config import kinematics as kin  # noqa: E402
from robotdesigner.simulators.mujoco.compile import compile_robot  # noqa: E402
from robotdesigner.simulators.warp.sim import CapacityError, WarpBatchedSim, check_model_supported  # noqa: E402
from robotdesigner.tasks.base import SimView  # noqa: E402
from tests.test_mujoco_compile import COMMON, MOUNT, make_cfg  # noqa: E402

DEVICES = ["cpu", pytest.param("cuda:0", marks=[pytest.mark.gpu, pytest.mark.skipif(
    not torch.cuda.is_available(), reason="needs CUDA")])]
OPTIONS = WarpOptions(njmax=64, nconmax=16, cuda_graph=False)


def make_sim(device, n=3, axes="zyy", num_envs=4, mode="joint_torque", options=OPTIONS, finalize=True):
    arm = chain(n, axes, total_length=0.9, total_mass=2.0, range=(-2.0, 2.0), **COMMON)
    cfg = make_cfg(arm, mode=mode, mount=MOUNT, num_envs=num_envs)
    sim = WarpBatchedSim(compile_robot(cfg), num_envs, cfg.sim.n_substeps, device, options)
    if finalize:
        sim.finalize()
    return sim, arm


@pytest.mark.parametrize("device", DEVICES)
def test_is_a_simview_and_nothing_is_allocated_before_finalize(device):
    sim, _ = make_sim(device, finalize=False)
    assert isinstance(sim, SimView) and sim._d is None
    with pytest.raises(RuntimeError):
        sim.step()
    sim.finalize()
    assert sim.qpos.shape == (4, 3) and sim.time.shape == (4,)


@pytest.mark.parametrize("device", DEVICES)
@pytest.mark.parametrize("n,axes", [(2, "zy"), (4, "zyyz")])
def test_end_effector_matches_the_kinematics_oracle(device, n, axes):
    sim, arm = make_sim(device, n=n, axes=axes, num_envs=5)
    q = kin.sample_joint_positions(arm, 5, np.random.default_rng(0))
    sim.write_qpos(int(sim.iface.qpos_adr[0]), torch.from_numpy(q), torch.ones(5, dtype=torch.bool))
    world = kin.forward_ee_batch(arm, q, MOUNT)
    p0, R = sim.base_frame()
    assert np.allclose(sim.site_pos(sim.iface.ee_site_id).cpu().numpy(), world, atol=1e-5)
    assert np.allclose(sim.site_pos(sim.iface.ee_site_id, "base").cpu().numpy(), (world - p0) @ R, atol=1e-5)


@pytest.mark.parametrize("device", DEVICES)
def test_masked_reset_leaves_other_worlds_bitwise_unchanged(device):
    sim, _ = make_sim(device)
    gen = torch.Generator().manual_seed(0)
    sim.reset(qpos=torch.rand(4, 3, generator=gen) - 0.5, qvel=torch.rand(4, 3, generator=gen))
    sim.set_ctrl(torch.rand(4, sim.model.nu, generator=gen) * 5)
    sim.step()
    sim.step()
    before = {k: getattr(sim, k).clone() for k in ("qpos", "qvel", "time", "actuator_force")}
    ctrl, warm = wp_torch(sim._d.ctrl).clone(), wp_torch(sim._d.qacc_warmstart).clone()
    mask = torch.tensor([True, False, False, True])
    sim.reset(mask=mask)
    keep = ~mask
    for k, v in before.items():
        assert torch.equal(getattr(sim, k)[keep], v[keep]), k
    assert torch.equal(wp_torch(sim._d.ctrl)[keep], ctrl[keep])
    assert torch.equal(wp_torch(sim._d.qacc_warmstart)[keep], warm[keep])
    assert torch.equal(sim.qpos[mask], sim._home.expand(2, 3))
    assert not sim.qvel[mask].any() and not sim.time[mask].any() and not wp_torch(sim._d.ctrl)[mask].any()
    assert not wp_torch(sim._d.qacc_warmstart)[mask].any()
    # injected values are read for the masked rows only
    inj = torch.full((4, 3), 0.25)
    sim.reset(mask=torch.tensor([False, True, False, False]), qpos=inj, qvel=inj)
    assert torch.allclose(sim.qpos[1], inj[1]) and torch.allclose(sim.qvel[1], inj[1])
    assert not torch.allclose(sim.qpos[2], inj[2])


def wp_torch(a):
    import warp as wp

    return wp.to_torch(a)


@pytest.mark.parametrize("device", DEVICES)
def test_step_moves_the_arm_and_reads_need_no_synchronisation(device):
    sim, _ = make_sim(device)
    sim.set_ctrl(torch.full((4, sim.model.nu), 3.0))
    sim.step()
    assert (sim.qvel.abs().sum(-1) > 0).all() and torch.allclose(sim.time, torch.full((4,), sim.control_dt), atol=1e-6)


@pytest.mark.parametrize("device", DEVICES)
def test_host_data_bridge_reproduces_the_ee_site(device):
    sim, _ = make_sim(device)
    sim.reset(qpos=torch.rand(4, 3) - 0.5)
    ee = sim.iface.ee_site_id
    for i in (0, 3):  # after a reset the derived state is current: mj_forward on the copy reproduces it
        d = sim.host_data(i)
        mujoco.mj_forward(sim.model, d)
        assert np.allclose(d.site_xpos[ee], sim.site_pos(ee)[i].cpu().numpy(), atol=1e-5)
    sim.set_ctrl(torch.ones(4, sim.model.nu))
    sim.step()
    for i in (0, 3):  # after a step both sides hold the state before the last integration, as the CPU simulator does
        d = sim.host_data(i)
        assert np.allclose(d.site_xpos[ee], sim.site_pos(ee)[i].cpu().numpy(), atol=1e-5)


@pytest.mark.parametrize("device", DEVICES)
def test_invalid_arguments(device):
    sim, _ = make_sim(device)
    with pytest.raises(ValueError):
        sim.body_pose(1, "tool")
    with pytest.raises(ValueError):
        sim.reset(mask=torch.ones(3, dtype=torch.bool))
    with pytest.raises(ValueError):
        sim.set_ctrl(torch.zeros(4, 99))


@pytest.mark.parametrize("device", DEVICES)
def test_capacity_overflow_is_reported(device):
    sim, _ = make_sim(device)
    sim.check_overflow()  # a free arm: nothing to report
    sim._d.overflow.fill_(1)
    with pytest.raises(CapacityError, match="NEFC"):
        sim.check_overflow()


def test_check_model_supported():
    sim, _ = make_sim("cpu", finalize=False)
    assert check_model_supported(sim.model) == []
    spec = mujoco.MjSpec()
    spec.option.enableflags |= int(mujoco.mjtEnableBit.mjENBL_FWDINV)
    problems = check_model_supported(spec.compile())
    assert len(problems) == 1 and "FWDINV" in problems[0].message
