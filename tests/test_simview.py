import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("mujoco")
pytest.importorskip("torchrl")

from robotdesigner.config import FixedBox, kinematics as kin  # noqa: E402
from robotdesigner.simulators.mujoco.compile import compile_robot  # noqa: E402
from robotdesigner.simulators.mujoco.sim import BatchedSim  # noqa: E402
from robotdesigner.tasks.base import SimView  # noqa: E402
from tests.test_mujoco_compile import COMMON, make_cfg  # noqa: E402
from tests.toy_provider.reach import ReachTask  # noqa: E402
from robotdesigner.config import Pose, chain  # noqa: E402
from tests.test_mujoco_env import make_cfg as env_cfg  # noqa: E402


def make_sim(num_envs=3):
    arm = chain(2, "zy", link_length=0.5, mass=1.0, range=(-2.0, 2.0), **COMMON)
    mount = Pose((0.1, 0.2, 0.3), (0.9, 0.1, 0.3, 0.3))
    cfg = make_cfg(arm, mode="joint_torque", mount=mount, num_envs=num_envs)
    sim = BatchedSim(compile_robot(cfg), num_envs, cfg.sim.n_substeps)
    sim.set_ctrl(np.random.default_rng(0).normal(size=(num_envs, sim.model.nu)))
    sim.step()
    return sim


def test_batched_sim_is_a_simview():
    assert isinstance(make_sim(), SimView)


def test_poses_match_the_numpy_formulas():
    sim = make_sim()
    pos_b, R = sim.base_frame()
    body, site = sim.model.nbody - 1, sim.iface.ee_site_id
    xpos = np.stack([d.xpos[body] for d in sim.data])
    p, q = sim.body_pose(body, "world")
    assert p.dtype == q.dtype == torch.float32 and p.shape == (3, 3) and q.shape == (3, 4)
    assert torch.equal(p, torch.from_numpy(xpos).float())
    p, q = sim.body_pose(body, "base")
    assert torch.equal(p, torch.from_numpy((xpos - pos_b) @ R).float())
    # base-frame orientation: rotating the base-frame quaternion back by the base rotation gives the world one
    assert torch.allclose(q.norm(dim=-1), torch.ones(3), atol=1e-6)
    world = np.stack([d.site_xpos[site] for d in sim.data])
    assert torch.equal(sim.site_pos(site), torch.from_numpy(world).float())
    assert torch.equal(sim.site_pos(site, "base"), torch.from_numpy(sim.ee_pos_base()).float())
    for call in (sim.body_pose, sim.site_pos):
        with pytest.raises(ValueError):
            call(1, "tool")


def test_write_qpos_on_a_mask():
    sim = make_sim()
    adr = int(sim.iface.qpos_adr[0])
    before_q, before_p = sim.qpos.copy(), sim.site_pos(sim.iface.ee_site_id).clone()
    mask = torch.tensor([True, False, True])
    sim.write_qpos(adr, torch.tensor([[0.5], [9.0], [-0.5]]), mask)
    q = sim.qpos
    assert np.allclose(q[[0, 2], 0], [0.5, -0.5]) and np.array_equal(q[1], before_q[1])
    p = sim.site_pos(sim.iface.ee_site_id)
    assert torch.equal(p[1], before_p[1]) and not torch.equal(p[0], before_p[0])  # derived state is current
    assert sim.host_data(0) is sim.data[0]


class FakeSim:
    """A SimView without an engine: the end effector sits at a fixed point."""

    def __init__(self, num_envs=4):
        self.num_envs, self.device = num_envs, torch.device("cpu")
        self.iface = type("I", (), {"ee_site_id": 0, "n": 2})()
        self.model = None

    def base_frame(self):
        return np.zeros(3), np.eye(3)

    def site_pos(self, site_id, frame="world"):
        return torch.tensor([[0.3, 0.0, 0.2]]).repeat(self.num_envs, 1)

    def body_pose(self, body_id, frame="world"):
        raise NotImplementedError

    def write_qpos(self, adr, values, mask):
        raise NotImplementedError

    def host_data(self, env_index):
        raise NotImplementedError


def test_a_task_runs_against_a_fake_simview():
    cfg = env_cfg(region=FixedBox((0.3, 0.0, 0.2), (0.3, 0.0, 0.2)))
    task, sim = ReachTask(cfg), FakeSim()
    assert isinstance(sim, SimView)
    task.bind(sim)
    gen = torch.Generator().manual_seed(0)
    task.reset(sim, torch.tensor([True, False, True, False]), lambda *s, dtype=torch.float64: torch.rand(*s, generator=gen, dtype=dtype), lambda k: None)
    obs = task.observe(sim)
    assert obs["ee_pos"].shape == (4, 3) and obs["target_pos"][0].tolist() == pytest.approx([0.3, 0.0, 0.2])
    assert obs["target_pos"][1].abs().sum() == 0  # unmasked row untouched
    success = task.success(sim, obs)
    assert success.dtype == torch.bool and success.shape == (4, 1) and success[0, 0] and not success[1, 0]
    assert task.reward(sim, obs).shape == (4, 1)
