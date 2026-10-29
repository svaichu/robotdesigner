import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from dataclasses import replace  # noqa: E402

from robotdesigner.backends.mujoco.compile import compile_robot  # noqa: E402
from robotdesigner.config import (  # noqa: E402
    ControlConfig, EnvConfig, FixedBox, FixedGains, MujocoOptions, Pose, SimConfig, TaskConfig, chain,
)
from robotdesigner.tasks.native import ReachParams
from robotdesigner.config import kinematics as kin  # noqa: E402

COMMON = dict(radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0)


def make_cfg(arm, mode="joint_pos_abs", **sim):
    sim_kw = dict(backend="mujoco", num_envs=2, device="cpu", dt=0.002, integrator="euler",
                  control_freq=50, seed=0, scene="plain_floor")
    sim_kw.update(sim)
    return EnvConfig(
        robot=arm,
        control=ControlConfig(mode=mode, gains=FixedGains(60.0, 6.0), action_scale=None),
        task=TaskConfig("native/reach", 100, 0.02, FixedBox((0.2, -0.3, 0.0), (0.6, 0.3, 0.4)),
                        ReachParams(0.0, False)),
        sim=SimConfig(**sim_kw),
        backend_options=MujocoOptions(self_collision=False),
    )


ARMS = {
    "zy2": lambda: chain(2, "zy", total_length=0.9, total_mass=2.0, range=[None, (-2.0, 2.0)],
                         base_pose=Pose((0.1, -0.2, 0.3), (0.9, 0.1, 0.3, 0.2)),
                         end_effector=Pose((0.05, 0.0, 0.02), (0.8, 0.0, 0.6, 0.0)), **COMMON),
    "zyyz4": lambda: chain(4, "zyyz", link_length=[0.3, 0.25, 0.2, 0.1], mass=[1.0, 0.8, 0.5, 0.2],
                           range=(-2.5, 2.5), **COMMON),
}


@pytest.mark.parametrize("name", ARMS)
def test_kinematics_oracle(name):
    arm = ARMS[name]()
    compiled = compile_robot(make_cfg(arm))
    model, iface = compiled.model, compiled.interface
    data = mujoco.MjData(model)
    rng = np.random.default_rng(0)
    for q in kin.sample_joint_positions(arm, 5, rng):
        data.qpos[iface.qpos_adr] = q
        mujoco.mj_forward(model, data)
        fk = kin.forward(arm, q).ee
        assert np.allclose(data.site_xpos[iface.ee_site_id], fk[:3, 3], atol=1e-9)
        assert np.allclose(data.site_xmat[iface.ee_site_id].reshape(3, 3), fk[:3, :3], atol=1e-9)


def test_dimensions_mass_and_interface():
    arm = ARMS["zyyz4"]()
    cfg = make_cfg(arm)
    compiled = compile_robot(cfg)
    m, iface = compiled.model, compiled.interface
    assert m.nv == m.nu == arm.n == iface.n
    assert m.body_mass.sum() == pytest.approx(sum(s.link.mass for s in arm.segments))
    assert iface.joint_names == tuple(f"joint_{i}" for i in range(4))
    assert np.array_equal(iface.qpos_adr, np.arange(4)) and np.array_equal(iface.qvel_adr, np.arange(4))
    assert np.array_equal(iface.actuator_ids, np.arange(4))
    assert np.allclose(iface.joint_range, [(-2.5, 2.5)] * 4)
    assert np.array_equal(iface.home_qpos, np.zeros(4))
    assert m.opt.timestep == cfg.sim.dt
    assert compile_robot(cfg).model.body_pos.tolist() == m.body_pos.tolist()  # deterministic


def test_unlimited_joint_range_is_infinite():
    iface = compile_robot(make_cfg(ARMS["zy2"]())).interface
    assert np.isinf(iface.joint_range[0]).all() and iface.joint_range[0, 0] < 0
    assert np.allclose(iface.joint_range[1], (-2.0, 2.0))


def test_actuators():
    arm = ARMS["zy2"]()
    pos = compile_robot(make_cfg(arm)).model
    assert np.allclose(pos.actuator_forcerange, [[-50, 50], [-50, 50]])
    assert np.allclose(pos.actuator_ctrlrange, [[-np.pi, np.pi], [-2.0, 2.0]])
    assert np.allclose(pos.actuator_gainprm[:, 0], 60.0)
    assert np.allclose(pos.actuator_biasprm[:, :3], [[0, -60, -6]] * 2)
    tq = compile_robot(make_cfg(arm, mode="joint_torque")).model
    assert np.allclose(tq.actuator_ctrlrange, [[-50, 50]] * 2)
    assert np.allclose(tq.actuator_gainprm[:, 0], 1.0) and np.allclose(tq.actuator_biasprm[:, :3], 0.0)


def test_explicit_inertia_and_empty_scene():
    arm = chain(1, "z", link_length=0.5, mass=1.0, **COMMON)
    seg = arm.segments[0]
    seg = replace(seg, link=replace(seg.link, inertia=(0.01, 0.02, 0.03)))
    arm = replace(arm, segments=(seg,))
    m = compile_robot(make_cfg(arm, scene="empty")).model
    assert np.allclose(m.body_inertia[2], (0.01, 0.02, 0.03)) and m.ngeom == 1


def test_contacts_robot_vs_scene_only():
    arm = chain(2, "zz", link_length=0.5, mass=1.0, **COMMON)
    m = compile_robot(make_cfg(arm)).model
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    g = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"geom_{i}") for i in range(2)]

    def collides(a, b):
        return bool(m.geom_contype[a] & m.geom_conaffinity[b]) or bool(m.geom_contype[b] & m.geom_conaffinity[a])

    assert collides(g[0], floor) and collides(g[1], floor)
    assert not collides(g[0], g[1])


@pytest.mark.parametrize("name", ARMS)
def test_joint_limits_compile_in_radians(name):
    # MjSpec reads angles as degrees unless told otherwise; the config and RobotInterface use radians.
    arm = ARMS[name]()
    compiled = compile_robot(make_cfg(arm))
    model, iface = compiled.model, compiled.interface
    limited = np.isfinite(iface.joint_range[:, 0])
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}") for i in range(arm.n)]
    assert limited.any()
    np.testing.assert_array_equal(model.jnt_limited[ids].astype(bool), limited)
    np.testing.assert_allclose(model.jnt_range[ids][limited], iface.joint_range[limited])
