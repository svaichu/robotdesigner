"""The ``menagerie`` robot provider against the real library (downloads models on first use; marker ``menagerie``).

``python -m tests.test_menagerie_provider`` rewrites ``tests/menagerie_expected.json`` from the sweep.
"""

import json
import warnings
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

mm = pytest.importorskip("mujoco_menagerie")
mujoco = pytest.importorskip("mujoco")
torch = pytest.importorskip("torch")
pytest.importorskip("torchrl")

from robotdesigner.config import BaseEnvConfig, ConfigError, Mount, Pose
from robotdesigner.config import kinematics as kin
from robotdesigner.robot.providers.menagerie import PROVIDER, cards

pytestmark = pytest.mark.menagerie

EXPECTED = Path(__file__).with_name("menagerie_expected.json")
CATEGORIES = ("arm", "quadruped", "mobile_manipulator")
SWEPT = [r.name for cat in CATEGORIES for r in mm.robots(cat)]


def err_of(build):
    with pytest.raises(ConfigError) as e:
        build()
    return e.value


# --- describe, against the survey ---------------------------------------------------------------------------


def test_panda_has_seven_driven_joints_and_an_end_effector_body():
    r = PROVIDER.describe("franka_emika_panda")
    assert r.n == 7 and r.actuator_kind == "position" and r.keyframe == "home" and r.home_root is None
    assert r.joints[0].name == "joint1" and r.joints[0].range == pytest.approx((-2.8973, 2.8973))
    assert r.end_effector.body == "hand" and r.end_effector.offset.pos == (0.0, 0.0, 0.1034)
    assert r.source == mm.get("franka_emika_panda").oid


def test_ur5e_has_six_and_go2_twelve_with_a_home_root():
    ur5e = PROVIDER.describe("universal_robots_ur5e")
    assert ur5e.n == 6 and ur5e.end_effector.site == "attachment_site" and ur5e.joints[0].name == "shoulder_pan_joint"
    go2 = PROVIDER.describe("unitree_go2")
    assert go2.n == 12 and go2.actuator_kind == "torque" and go2.end_effector is None
    assert go2.home_root.pos == (0.0, 0.0, 0.27) and go2.keyframe == "home"
    assert go2.joints[0].effort_limit == pytest.approx(23.7)  # a plain motor: its ctrlrange times its gain and gear


@pytest.mark.parametrize("name", ["franka_emika_panda", "universal_robots_ur5e", "unitree_go2"])
def test_make_env_runs_the_survey_robots(name):
    env = BaseEnvConfig().robot(PROVIDER.describe(name)).sim(num_envs=2, scene="plain_floor").build()
    td = env.reset()
    for _ in range(5):
        td["action"] = env.action_spec.rand()
        td = env.step(td)["next"]
    assert torch.isfinite(td["proprio", "qpos"]).all()
    env.close()


def test_a_finger_joint_actuator_is_driven_only_without_a_card():
    assert PROVIDER.describe("agilex_piper").n == 6  # the card leaves the finger out
    names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    assert PROVIDER.describe("agilex_piper", controlled=names).n == 7


def test_an_actuator_without_a_force_limit_needs_effort_limit():
    e = err_of(lambda: PROVIDER.describe("kuka_iiwa_14"))
    assert e.path == "effort_limit" and "'joint1'" in e.reason and "effort_limit=" in e.reason
    assert PROVIDER.describe("kuka_iiwa_14", effort_limit=200.0).joints[0].effort_limit == 200.0
    assert PROVIDER.describe("kuka_iiwa_14", effort_limit={f"joint{i}": 10.0 * i for i in range(1, 8)}).joints[2].effort_limit == 30.0
    # the limit of the joint (actfrcrange) is used when the actuator has none
    assert PROVIDER.describe("flexiv_rizon4").joints[0].effort_limit > 0


def test_a_home_value_outside_its_range_is_clipped_and_said():
    with pytest.warns(UserWarning, match="outside its range .* clipped"):
        r = PROVIDER.describe("google_robot")
    lo = next(j.range[0] for j in r.joints if j.name == "joint_finger_left")
    assert r.home_qpos[[j.name for j in r.joints].index("joint_finger_left")] == lo


def test_describe_refuses_a_scene_unknown_names_and_bad_arguments():
    assert err_of(lambda: PROVIDER.describe("universal_robots_ur5e", "scene")).path == "entry"
    e = err_of(lambda: PROVIDER.describe("franka_emika_pandaa"))
    assert e.path == "name" and "franka_emika_panda" in e.reason
    assert err_of(lambda: PROVIDER.describe("franka_emika_panda", "nope")).path == "entry"
    assert err_of(lambda: PROVIDER.describe("franka_emika_panda", controlled=["nope"])).path == "controlled"
    assert err_of(lambda: PROVIDER.describe("franka_emika_panda", keyframe="away")).path == "keyframe"
    assert err_of(lambda: PROVIDER.describe("unitree_go2", controlled=["FL_hip_joint", "NoSuch"])).path == "controlled"


# --- spec, validate, kinematics, licence ---------------------------------------------------------------------


def test_spec_refuses_a_changed_source():
    r = PROVIDER.describe("universal_robots_ur5e")
    assert PROVIDER.spec(r).compile().nu == 6
    e = err_of(lambda: PROVIDER.spec(replace(r, source="0" * 40)))
    assert e.path == "robot.source" and e.reason == "the model changed since this config was written"


def test_validate_problems_have_paths():
    r = PROVIDER.describe("universal_robots_ur5e")
    assert PROVIDER.validate(r, "mujoco") == []
    assert [p.path for p in PROVIDER.validate(replace(r, source="stale"), "mujoco")] == ["robot.source"]
    assert [p.path for p in PROVIDER.validate(replace(r, name="universal_robots_ur5"), "mujoco")] == ["robot.name"]
    assert "universal_robots_ur5e" in PROVIDER.validate(replace(r, name="universal_robots_ur5"), "mujoco")[0].message
    assert [p.path for p in PROVIDER.validate(replace(r, entry="nope"), "mujoco")] == ["robot.entry"]
    assert [p.path for p in PROVIDER.validate(replace(r, entry="scene"), "mujoco")] == ["robot.entry"]


def test_make_env_reports_a_stale_source_an_unknown_robot_and_a_wrong_kind():
    r = PROVIDER.describe("universal_robots_ur5e")
    for bad, where in ((replace(r, source="stale"), "robot.source"), (replace(r, name="nope_robot"), "robot.name")):
        e = err_of(lambda bad=bad: BaseEnvConfig().robot(bad).sim(scene="plain_floor").build())
        assert where in str(e)
    e = err_of(lambda: BaseEnvConfig().robot(replace(r, actuator_kind="torque")).control(mode="joint_torque").sim(scene="plain_floor").build())
    assert "robot.actuator_kind" in str(e)


def test_a_model_with_mixed_actuators_is_refused_with_the_way_out():
    r = PROVIDER.describe("pal_tiago")
    assert r.actuator_kind == "mixed"
    e = err_of(lambda: BaseEnvConfig().robot(r).resolve())
    assert e.path == "control.mode" and "restrict robot.joints" in e.reason
    arm = [j.name for j in r.joints if j.name.startswith("arm_")]
    sub = PROVIDER.describe("pal_tiago", controlled=arm)
    assert sub.actuator_kind == "position" and sub.n == 7


def test_the_oracle_matches_the_simulator():
    robot = PROVIDER.describe("universal_robots_ur5e")
    mount = Pose((0.3, -0.2, 0.1), (float(np.cos(0.3)), 0.0, 0.0, float(np.sin(0.3))))
    env = BaseEnvConfig().robot(robot).task(mount=Mount(mount)).sim(num_envs=1, scene="plain_floor").build()
    env.reset()
    sim_ee = env._sim.data[0].site_xpos[env.interface.ee_site_id]
    assert np.allclose(kin.forward_ee_batch(robot, np.array([robot.home_qpos]), mount)[0], sim_ee, atol=1e-6)
    q = kin.sample_joint_positions(robot, 4, np.random.default_rng(0))
    assert q.shape == (4, 6) and (q >= [j.range[0] for j in robot.joints]).all()
    env.close()


def test_licence_and_explain():
    assert PROVIDER.license("universal_robots_ur5e") == mm.get("universal_robots_ur5e").license == "BSD-3-Clause"
    cfg = BaseEnvConfig().robot(PROVIDER.describe("universal_robots_ur5e"))
    assert any("robot.license" in line and "BSD-3-Clause" in line for line in cfg.explain().splitlines())


# --- cards ----------------------------------------------------------------------------------------------------


def test_every_arm_and_quadruped_has_a_card():
    have = cards()
    assert {r.name for cat in ("arm", "quadruped") for r in mm.robots(cat)} <= set(have)
    assert set(have) <= set(mm.names())


@pytest.mark.parametrize("name", sorted(cards()))
def test_a_card_matches_its_model(name):
    card = cards()[name]
    meta = mm.get(name)
    entry = card.get("entry", meta.default_model)
    assert entry in meta.entry_names and meta.entry(entry).kind == "robot"
    model = meta.spec(entry).compile()
    O = mujoco.mjtObj
    joint = lambda n: mujoco.mj_name2id(model, O.mjOBJ_JOINT, n)
    transmitted = {int(model.actuator_trnid[a, 0]) for a in range(model.nu) if int(model.actuator_trntype[a]) == 0}
    for n in card.get("controlled", []):
        assert joint(n) >= 0 and joint(n) in transmitted, f"{name}: {n}"
    ee = card.get("end_effector")
    if ee is not None:
        assert (ee.get("site") is None) != (ee.get("body") is None)
        assert ee.get("site") is None or mujoco.mj_name2id(model, O.mjOBJ_SITE, ee["site"]) >= 0
        assert ee.get("body") is None or mujoco.mj_name2id(model, O.mjOBJ_BODY, ee["body"]) >= 0
    if card.get("keyframe") is not None:
        assert mujoco.mj_name2id(model, O.mjOBJ_KEY, card["keyframe"]) >= 0


# --- the coverage sweep -----------------------------------------------------------------------------------------


def status_of(name: str) -> str:
    """``ok``, or ``refused: <reason>`` with the reasons the expected file uses."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        try:
            robot = PROVIDER.describe(name)
        except ConfigError as e:
            return "refused: needs effort_limit" if e.path == "effort_limit" else f"refused: describe: {e.reason}"
        try:
            env = BaseEnvConfig().robot(robot).sim(num_envs=2, scene="plain_floor").build()
        except ConfigError as e:
            return "refused: mixed actuators" if "no mode fits" in str(e) else f"refused: make_env: {e}"
        td = env.reset()
        finite = lambda t: all(torch.isfinite(v).all() for v in t.values(True, True) if v.dtype.is_floating_point)
        assert finite(td), f"{name}: reset"
        for i in range(20):
            td["action"] = torch.zeros(2, env.action_spec.shape[-1]) if i < 10 else env.action_spec.rand()
            td = env.step(td)["next"]
            assert finite(td), f"{name}: step {i}"
        env.close()
        return "ok"


def test_the_sweep_matches_the_expected_status_of_every_robot():
    expected = json.loads(EXPECTED.read_text())
    assert sorted(expected) == sorted(SWEPT), "the library has robots the expected file does not list, or the reverse"
    flipped = {n: (expected[n], got) for n in SWEPT if (got := status_of(n)) != expected[n]}
    assert not flipped, f"status changed (expected, got): {flipped}; if intended, python -m tests.test_menagerie_provider"


if __name__ == "__main__":
    out = {n: status_of(n) for n in SWEPT}
    EXPECTED.write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    print(f"wrote {EXPECTED}: {sum(v == 'ok' for v in out.values())} ok, {sum(v != 'ok' for v in out.values())} refused")
