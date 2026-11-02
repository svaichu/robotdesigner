import numpy as np
import pytest

torch = pytest.importorskip("torch")
mujoco = pytest.importorskip("mujoco")
pytest.importorskip("torchrl")

from torchrl.envs.utils import check_env_specs  # noqa: E402

from robotdesigner.config import TwoFingerGripper  # noqa: E402
from robotdesigner.envs import make_env  # noqa: E402
from robotdesigner.simulators.mujoco.compile import compile_robot  # noqa: E402
from tests.test_mujoco_env import COMMON, make_cfg  # noqa: E402,F401
from robotdesigner.robot import chain  # noqa: E402
from dataclasses import replace  # noqa: E402


def gripper_cfg(**kw):
    cfg = make_cfg(n=2, num_envs=2)
    arm = chain(2, "zy", total_length=0.6, total_mass=2.0, range=(-2.0, 2.0), gripper=TwoFingerGripper.default(**kw), **COMMON)
    return replace(cfg, robot=arm)


def test_compiles_with_one_extra_actuator():
    c = compile_robot(gripper_cfg())
    i = c.interface
    assert i.has_gripper and i.action_dim == 3 and c.model.nu == 3
    assert i.actuator_ids.tolist() == [0, 1] and i.gripper_actuator_id == 2
    assert mujoco.mj_name2id(c.model, mujoco.mjtObj.mjOBJ_SITE, "tcp") == i.tcp_site_id


def test_no_gripper_interface_is_unchanged():
    i = compile_robot(make_cfg(n=2)).interface
    assert not i.has_gripper and i.action_dim == 2


def test_env_specs_and_action_shape():
    env = make_env(gripper_cfg())
    assert env.action_spec.shape == (2, 3)
    check_env_specs(env)
    td = env.reset()
    assert td["proprio", "gripper"].shape == (2, 1) and td["proprio", "qpos"].shape == (2, 2)


def _run(env, g, steps=60):
    td = env.reset()
    a = torch.zeros(2, 3)
    a[:, 2] = g
    for _ in range(steps):
        td["action"] = a
        td = env.step(td)["next"]
    return td["proprio", "gripper"][:, 0]


def test_opening_follows_command_and_fingers_are_coupled():
    g = TwoFingerGripper.default()
    env = make_env(gripper_cfg())
    wide, shut = _run(env, 1.0), _run(env, -1.0)
    assert torch.allclose(wide, torch.full((2,), g.max_opening), atol=3e-3)
    assert torch.all(shut < 3e-3)
    d = env._sim.data[0]
    adr = env.interface.gripper_qpos_adr
    assert d.qpos[adr[0]] == pytest.approx(d.qpos[adr[1]], abs=1e-3)


def test_finger_stroke_in_the_gripper_frame():
    env = make_env(gripper_cfg())
    _run(env, 1.0)
    m, d = env._compiled.model, env._sim.data[0]
    f0 = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "finger_0")]
    f1 = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "finger_1")]
    g = TwoFingerGripper.default()
    assert np.linalg.norm(f0 - f1) == pytest.approx(2 * g.finger_radius + g.max_opening, abs=4e-3)
