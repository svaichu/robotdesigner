"""robosuite compatibility layer. Needs the robosuite extra; run with ``pytest -m robosuite``."""

import numpy as np
import pytest

pytestmark = pytest.mark.robosuite
suite = pytest.importorskip("robosuite")
mujoco = pytest.importorskip("mujoco")

from robotdesigner.tasks.robosuite import apply_compat  # noqa: E402

KNOWN_PATCHES = {"joint-address-enum", "mass-matrix"}


def _make_lift():
    return suite.make(
        "Lift",
        robots="Panda",
        has_renderer=False,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        control_freq=20,
    )


def test_apply_is_idempotent_and_reports_every_check():
    report = apply_compat()
    assert apply_compat() is report
    assert set(report.applied) | set(report.not_needed) == KNOWN_PATCHES
    assert not set(report.applied) & set(report.not_needed)
    assert report.mujoco_version == mujoco.__version__


def test_stock_panda_lift_resets_and_steps():
    apply_compat()
    env = _make_lift()
    try:
        obs = env.reset()
        rng = np.random.default_rng(0)
        for _ in range(10):
            obs, _, _, _ = env.step(rng.uniform(-1, 1, env.action_dim))
        assert env.action_dim == 7
        assert all(np.all(np.isfinite(v)) for v in obs.values())
    finally:
        env.close()


def test_controller_mass_matrix_matches_an_independent_computation():
    apply_compat()
    env = _make_lift()
    try:
        env.reset()
        rng = np.random.default_rng(1)
        for _ in range(5):
            env.step(rng.uniform(-1, 1, env.action_dim))
        part = env.robots[0].composite_controller.part_controllers["right"]
        part.update(force=True)
        model, data = env.sim.model._model, env.sim.data._data
        full = np.zeros((model.nv, model.nv))
        for i in range(model.nv):  # column i of M is M @ e_i
            column = np.zeros(model.nv)
            mujoco.mj_mulM(model, data, column, np.eye(model.nv)[i])
            full[:, i] = column
        expected = full[np.ix_(part.qvel_index, part.qvel_index)]
        assert part.mass_matrix.shape == (7, 7)
        np.testing.assert_allclose(part.mass_matrix, expected, atol=1e-9)
        assert np.all(np.diag(part.mass_matrix) > 0)
    finally:
        env.close()


def test_joint_addresses_match_mujoco_named_access():
    apply_compat()
    env = _make_lift()
    try:
        wrapper = env.sim.model
        model = wrapper._model
        kinds = {int(mujoco.mjtJoint.mjJNT_FREE): (7, 6), int(mujoco.mjtJoint.mjJNT_BALL): (4, 3)}
        checked = set()
        for j in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            if name is None:
                continue
            nq, nv = kinds.get(int(model.jnt_type[j]), (1, 1))
            q0, v0 = int(model.joint(name).qposadr[0]), int(model.joint(name).dofadr[0])
            expected_q = q0 if nq == 1 else (q0, q0 + nq)
            expected_v = v0 if nv == 1 else (v0, v0 + nv)
            assert wrapper.get_joint_qpos_addr(name) == expected_q
            assert wrapper.get_joint_qvel_addr(name) == expected_v
            checked.add(nq)
        assert {1, 7} <= checked  # hinge joints of the arm and the cube's free joint
    finally:
        env.close()


def test_mass_matrix_check_switches_off_when_robosuite_no_longer_reads_qM():
    from robotdesigner.tasks.robosuite import _compat

    class Fixed:
        class Controller:
            def update(self, force=False):
                return None  # a fixed robosuite no longer touches data.qM

    class Unfixed:
        class Controller:
            def update(self, force=False):
                return self.sim.data.qM

    model = mujoco.MjModel.from_xml_string(_compat._PROBE_XML)
    data = mujoco.MjData(model)
    try:  # MuJoCo 3.10 still has MjData.qM but already changed the mj_fullM signature, so test the call itself
        mujoco.mj_fullM(model, np.zeros((model.nv, model.nv)), data.qM)
        old_call_works = True
    except (AttributeError, TypeError):
        old_call_works = False

    assert _compat._mass_matrix_broken(mujoco, Fixed) is False
    assert _compat._mass_matrix_broken(mujoco, Unfixed) is (not old_call_works)
