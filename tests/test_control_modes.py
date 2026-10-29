import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("mujoco")
pytest.importorskip("torchrl")

from torchrl.envs.utils import check_env_specs  # noqa: E402

from robotdesigner.backends.mujoco.compile import compile_robot  # noqa: E402
from robotdesigner.config import (  # noqa: E402
    ControlConfig, EnvConfig, FixedBox, InertiaScaledGains, MujocoOptions, SimConfig,
    Pose, TaskConfig, chain,
)
from robotdesigner.tasks.native import ReachParams
from robotdesigner.envs import make_env  # noqa: E402

COMMON = dict(radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0)


def make_cfg(mode, scale, n=2, axes="zy", num_envs=2, rng=(-1.0, 1.0), bandwidth=20.0):
    arm = chain(n, axes, link_length=0.5, mass=1.0, range=rng,
                base_pose=Pose((0, 0, 0.5), (1, 0, 0, 0)), **COMMON)
    return EnvConfig(
        robot=arm,
        control=ControlConfig(mode, InertiaScaledGains(bandwidth, 1.0), scale),
        task=TaskConfig("native/reach", 100, 0.02, FixedBox((0.2, -0.3, 0.0), (0.6, 0.3, 0.4)),
                        ReachParams(0.0, False)),
        sim=SimConfig("mujoco", num_envs, "cpu", 0.002, "euler", 50, 0, "plain_floor"),
        backend_options=MujocoOptions(False),
    )


def test_gains_scale_with_inertia_and_are_deterministic():
    cfg = make_cfg("joint_pos_abs", None, axes="zz")
    a = compile_robot(cfg).interface
    b = compile_robot(cfg).interface
    assert a.kp[0] > a.kp[1]  # the shoulder sees the whole arm
    assert np.array_equal(a.kp, b.kp) and np.array_equal(a.kd, b.kd)
    assert np.all(a.kd >= 0)
    m = compile_robot(cfg).model
    assert np.allclose(m.actuator_gainprm[:, 0], a.kp) and np.allclose(m.actuator_biasprm[:, 2], -a.kd)


def test_gains_follow_formula():
    import mujoco

    cfg = make_cfg("joint_pos_abs", None, axes="zz", bandwidth=10.0)
    c = compile_robot(cfg)
    d = mujoco.MjData(c.model)
    mujoco.mj_forward(c.model, d)
    M = np.zeros((2, 2))
    mujoco.mj_fullM(c.model, d, M)
    kp = 100.0 * np.diag(M)
    kd = np.maximum(2 * np.sqrt(kp * np.diag(M)) - 0.5, 0.0)
    assert np.allclose(c.interface.kp, kp) and np.allclose(c.interface.kd, kd)


def test_delta_integrates_and_clips():
    env = make_env(make_cfg("joint_pos_delta", 0.05, rng=(-0.12, 0.12)))
    check_env_specs(env)
    td = env.reset()
    assert np.allclose(env.q_target, 0.0)
    for k in (1, 2):
        td["action"] = torch.ones(2, 2)
        td = env.step(td)["next"]
        assert np.allclose(env.q_target, 0.05 * k)
    for _ in range(3):
        td["action"] = torch.ones(2, 2)
        td = env.step(td)["next"]
    assert np.allclose(env.q_target, 0.12)  # clipped at the limit
    td["action"] = -torch.ones(2, 2)
    env.step(td)
    assert np.allclose(env.q_target, 0.07)


def test_delta_target_resets_with_masked_reset():
    env = make_env(make_cfg("joint_pos_delta", 0.05))
    td = env.reset()
    for _ in range(3):
        td["action"] = torch.ones(2, 2)
        td = env.step(td)["next"]
    td["_reset"] = torch.tensor([[True], [False]])
    env.reset(td)
    assert np.allclose(env.q_target[0], 0.0) and np.allclose(env.q_target[1], 0.15)


def test_velocity_mode_tracks():
    env = make_env(make_cfg("joint_vel", 0.8, axes="zz", rng=None, bandwidth=20.0))
    check_env_specs(env)
    td = env.reset()
    for _ in range(40):
        td["action"] = torch.ones(2, 2)
        td = env.step(td)["next"]
    qvel = td["proprio", "qvel"]
    assert torch.allclose(qvel, torch.full_like(qvel, 0.8), atol=0.15)
