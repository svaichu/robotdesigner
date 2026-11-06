import json
import subprocess
import sys
from dataclasses import replace

import numpy as np
import pytest

from robotdesigner.config import (
    BaseEnvConfig,
    ConfigError,
    ControlConfig,
    EndEffectorRef,
    EnvConfig,
    ExternalJoint,
    ExternalRobot,
    FixedGains,
    InertiaScaledGains,
    NativeGains,
    ParametricArm,
    Pose,
    chain,
    get_robot_provider,
    list_robot_providers,
    register_robot_provider,
    robot_from_dict,
    unregister_robot_provider,
)
from robotdesigner.config import kinematics as kin
from robotdesigner.config.control import CONTROL_MODES, NATIVE_MODES
from tests.toy_robot_provider import PROVIDER, ToyRobots

KW = {"provider": "toyrobot", "name": "arm3", "entry": "arm3.xml", "source": "abc123"}


def ext(**over):
    values = {
        **KW,
        "joints": (ExternalJoint("j1", (-2.0, 2.0), 40.0), ExternalJoint("j2", (-1.5, 1.5), 30.0)),
        "home_qpos": (0.1, -0.2),
        "home_root": None,
        "keyframe": "home",
        "end_effector": EndEffectorRef(site="ee", body=None, offset=Pose.identity()),
        "actuator_kind": "position",
    }
    values.update(over)
    return ExternalRobot(**values)


def err(fn, *a, **k):
    with pytest.raises(ConfigError) as e:
        fn(*a, **k)
    return e.value


# --- validation ------------------------------------------------------------------------


@pytest.mark.parametrize(
    "over,path",
    [
        ({"provider": ""}, "provider"),
        ({"name": ""}, "name"),
        ({"entry": 3}, "entry"),
        ({"source": ""}, "source"),
        ({"provider": "nope"}, "provider"),
        ({"joints": ()}, "joints"),
        ({"joints": (ExternalJoint("j", None, 1.0), ExternalJoint("j", None, 1.0)), "home_qpos": (0.0, 0.0)}, "joints[1].name"),
        ({"joints": ("j1",), "home_qpos": (0.0,)}, "joints[0]"),
        ({"home_qpos": (0.1,)}, "home_qpos"),
        ({"home_qpos": (3.0, 0.0)}, "home_qpos[0]"),
        ({"actuator_kind": "magic"}, "actuator_kind"),
        ({"home_root": (0, 0, 0)}, "home_root"),
        ({"end_effector": "ee"}, "end_effector"),
        ({"keyframe": ""}, "keyframe"),
        ({"label": 3}, "label"),
    ],
)
def test_validation_errors_carry_their_path(over, path):
    assert err(ext, **over).path == path


def test_unknown_provider_lists_the_registered_ones():
    assert "toyrobot" in err(ext, provider="nope").reason


def test_joint_validation():
    assert err(ExternalJoint, "", None, 1.0).path == "name"
    assert err(ExternalJoint, "j", (1.0, 0.0), 1.0).path == "range"
    assert err(ExternalJoint, "j", None, 0.0).path == "effort_limit"


def test_end_effector_ref_needs_exactly_one_of_site_and_body():
    assert err(EndEffectorRef, site="ee", body="hand", offset=Pose.identity()).reason == "give exactly one of site and body"
    assert err(EndEffectorRef, site=None, body=None, offset=Pose.identity()).reason == "give exactly one of site and body"
    assert err(EndEffectorRef, site="ee", body=None, offset=Pose((0.1, 0, 0), (1, 0, 0, 0))).path == "offset"
    EndEffectorRef(site=None, body="hand", offset=Pose((0.1, 0, 0), (1, 0, 0, 0)))


def test_n_and_unlimited_joint():
    r = ext(joints=(ExternalJoint("j1", None, 1.0),), home_qpos=(5.0,))
    assert r.n == 1


# --- round trip and hash -----------------------------------------------------------------


def test_round_trip_dict_and_json():
    r = ext(home_root=Pose((0, 0, 0.5), (1, 0, 0, 0)), label="x")
    d = r.to_dict()
    assert d["kind"] == "external_robot"
    assert ExternalRobot.from_dict(d) == r
    assert ExternalRobot.from_dict(json.loads(json.dumps(d))) == r
    assert robot_from_dict(d) == r


def test_hash_follows_source_not_label():
    a = ext()
    assert ext(source="other").full_hash != a.full_hash
    assert ext(label="renamed").full_hash == a.full_hash
    assert "label" not in a.hash_dict() and len(a.config_hash) == 12 and a.full_hash.startswith(a.config_hash)


def test_robot_from_dict_dispatches_and_rejects_unknown_kinds():
    arm = chain(2, "zy", total_length=0.6, total_mass=2.0, radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0)
    assert robot_from_dict(arm.to_dict()) == arm
    e = err(robot_from_dict, {"kind": "mystery"})
    assert e.path == "kind" and "external_robot" in e.reason and "parametric_arm" in e.reason
    assert err(robot_from_dict, {}).path == "kind"


def test_from_dict_errors_are_prefixed():
    d = ext().to_dict()
    d["joints"][1]["effort_limit"] = -1
    assert err(ExternalRobot.from_dict, d).path == "joints[1].effort_limit"
    d = ext().to_dict()
    d["end_effector"] = {"site": "a", "body": "b", "offset": Pose.identity().to_dict()}
    assert err(ExternalRobot.from_dict, d).path == "end_effector"


# --- the registry --------------------------------------------------------------------------


def test_registry_register_replace_unregister():
    assert "toyrobot" in list_robot_providers()
    assert isinstance(get_robot_provider("toyrobot"), ToyRobots)
    try:
        register_robot_provider("scratch", ToyRobots)
        assert "scratch" in list_robot_providers() and isinstance(get_robot_provider("scratch"), ToyRobots)
        assert err(register_robot_provider, "scratch", PROVIDER).reason.endswith("pass replace=True to override it")
        register_robot_provider("scratch", PROVIDER, replace=True)
    finally:
        unregister_robot_provider("scratch")
    assert "scratch" not in list_robot_providers()
    assert err(get_robot_provider, "scratch").path == "robot.provider"
    assert err(register_robot_provider, "Bad-Name", PROVIDER).path == ""


def test_a_provider_must_implement_the_protocol():
    try:
        register_robot_provider("broken", object())
        assert "does not implement" in err(get_robot_provider, "broken").reason
    finally:
        unregister_robot_provider("broken")


# --- gains and modes (rules 6 and 7) ---------------------------------------------------------


def _env(robot, mode, gains, scale=None):
    base = BaseEnvConfig().resolve()
    return EnvConfig(robot=robot, control=ControlConfig(mode=mode, gains=gains, action_scale=scale), task=base.task, sim=base.sim)


def _scale(mode):
    return 0.1 if mode in ("joint_pos_delta", "joint_vel") else None


ARM = chain(2, "zy", total_length=0.6, total_mass=2.0, radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0)


@pytest.mark.parametrize("mode", [m for m in CONTROL_MODES if not m.startswith("ee_")])
def test_native_gains_with_a_parametric_arm_is_an_error(mode):
    assert err(_env, ARM, mode, NativeGains(), _scale(mode)).path == "control.gains"


@pytest.mark.parametrize("kind", sorted(NATIVE_MODES))
@pytest.mark.parametrize("mode", CONTROL_MODES)
def test_native_gains_mode_table(kind, mode):
    robot = ext(actuator_kind=kind)
    if mode in NATIVE_MODES[kind]:
        assert _env(robot, mode, NativeGains(), _scale(mode)).control.gains == NativeGains()
    else:
        e = err(_env, robot, mode, NativeGains(), _scale(mode))
        assert e.path == "control.mode" and (
            all(m in e.reason for m in NATIVE_MODES[kind]) if NATIVE_MODES[kind] else "no mode fits" in e.reason
        )


@pytest.mark.parametrize("gains", [FixedGains(10.0, 1.0), InertiaScaledGains(10.0, 1.0)])
@pytest.mark.parametrize("kind", sorted(NATIVE_MODES))
@pytest.mark.parametrize("mode", [m for m in CONTROL_MODES if not m.startswith("ee_")])
def test_our_own_gains_fit_any_mode_and_any_actuator_kind(gains, kind, mode):
    # the compile replaces the model's actuators with ours, so the model's kind does not matter
    cfg = _env(ext(actuator_kind=kind), mode, gains, _scale(mode))
    assert cfg.control.gains == gains


def test_per_joint_gains_of_the_wrong_length_are_still_an_error():
    assert err(_env, ext(), "joint_pos_abs", FixedGains((1.0, 2.0, 3.0), 1.0)).path == "control.gains.kp"


def test_native_gains_round_trip():
    assert NativeGains.from_dict(NativeGains().to_dict()) == NativeGains()
    assert ControlConfig.from_dict(ControlConfig("joint_pos_abs", NativeGains(), None).to_dict()).gains == NativeGains()
    assert err(NativeGains.from_dict, {"kind": "fixed"}).path == "kind"


def test_env_config_with_an_external_robot_round_trips_and_hashes():
    cfg = _env(ext(), "joint_pos_abs", NativeGains())
    back = EnvConfig.from_dict(json.loads(json.dumps(cfg.to_dict())))
    assert back == cfg and back.env_hash == cfg.env_hash
    assert replace(cfg, robot=ext(source="other")).env_hash != cfg.env_hash
    assert err(EnvConfig, robot="arm", control=cfg.control, task=cfg.task, sim=cfg.sim).reason == "expected ParametricArm or ExternalRobot"


# --- the builder -------------------------------------------------------------------------------


def test_builder_defaults_to_native_gains_and_says_where_they_come_from():
    cfg = BaseEnvConfig().robot(ext())
    env = cfg.resolve()
    assert env.robot == ext() and env.control.gains == NativeGains() and env.control.mode == "joint_pos_abs"
    gains_row = next(line for line in cfg.explain().splitlines() if "control.gains" in line)
    assert "NativeGains()" in gains_row and gains_row.endswith("[default (robot)]")
    assert not cfg.validate()


def test_builder_picks_the_first_mode_that_fits():
    cfg = BaseEnvConfig().robot(ext(actuator_kind="torque"))
    assert cfg.resolve().control.mode == "joint_torque"
    assert "[default (robot)]" in next(line for line in cfg.explain().splitlines() if "control.mode" in line)
    assert BaseEnvConfig().robot(ext(actuator_kind="velocity")).control(action_scale=0.5).resolve().control.mode == "joint_vel"


def test_builder_keeps_a_written_mode_and_reports_a_misfit():
    ok = BaseEnvConfig().robot(ext()).control(mode="joint_pos_delta", action_scale=0.1)
    assert ok.resolve().control.mode == "joint_pos_delta"
    bad = BaseEnvConfig().robot(ext(actuator_kind="torque")).control(mode="joint_pos_abs")
    assert [p.path for p in bad.validate()] == ["control.mode"]


def test_builder_takes_fixed_gains_written_later_and_native_gains_again():
    cfg = BaseEnvConfig().robot(ext()).control(gains=FixedGains(10.0, 1.0))
    assert cfg.resolve().control.gains == FixedGains(10.0, 1.0)
    assert BaseEnvConfig().robot(ext()).control(gains=NativeGains()).resolve().control.gains == NativeGains()
    assert "[set]" in next(line for line in cfg.explain().splitlines() if "control.gains" in line)


def test_builder_native_gains_with_a_chain_is_an_error():
    assert [p.path for p in BaseEnvConfig().control(gains=NativeGains()).validate()] == ["control.gains"]


def test_builder_rejects_chain_fields_on_an_external_robot():
    assert err(BaseEnvConfig().robot, ext(), n=3).path == "robot"
    assert err(BaseEnvConfig().robot, 3).path == "robot"


def test_builder_dict_round_trip_and_diff():
    cfg = BaseEnvConfig().robot(ext())
    d = json.loads(json.dumps(cfg.to_dict()))
    assert d["robot"]["kind"] == "external_robot"
    back = BaseEnvConfig.from_dict(d)
    assert back == cfg and back.resolve().env_hash == cfg.resolve().env_hash
    assert "robot" in cfg.diff() and "control.gains" not in cfg.diff()
    assert err(BaseEnvConfig.from_dict, {"robot": {"kind": "mystery"}}).path == "robot.kind"


def test_builder_with_overrides_and_sweep():
    cfg = BaseEnvConfig().robot(ext())
    moved = cfg.with_overrides({"robot.home_qpos": [0.3, 0.0]})
    assert moved.resolve().robot.home_qpos == (0.3, 0.0)
    assert err(cfg.with_overrides, {"robot.home_qpos": [9.0, 0.0]}).path == "robot.home_qpos[0]"
    assert err(cfg.with_overrides, {"robot.n": 3}).path == "robot.n"
    fam = cfg.sweep(grid={"robot.home_qpos": [[0.0, 0.0], [0.3, 0.0]]})
    assert len(fam) == 2 and len({c.env_hash for c in fam.configs()}) == 2


def test_builder_switching_between_robot_kinds():
    cfg = BaseEnvConfig().robot(ext()).robot(ARM)
    assert cfg.resolve().robot == ARM and cfg.resolve().control.gains == FixedGains(60.0, 6.0)
    assert isinstance(BaseEnvConfig().robot(ARM).robot(ext()).resolve().robot, ExternalRobot)


# --- kinematics (rule 10) -------------------------------------------------------------------------


def test_kinematics_dispatch_to_the_provider_oracle():
    robot = PROVIDER.describe("arm3")
    q = np.zeros((2, 3))
    assert np.allclose(kin.forward_ee_batch(robot, q), [[0.9, 0.0, 0.1]] * 2)
    base = Pose((1.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    assert np.allclose(kin.forward_ee_batch(robot, q, base), [[1.9, 0.0, 0.1]] * 2)
    samples = kin.sample_joint_positions(robot, 50, np.random.default_rng(0))
    assert samples.shape == (50, 3)
    lo, hi = [j.range[0] for j in robot.joints], [j.range[1] for j in robot.joints]
    assert (samples >= lo).all() and (samples <= hi).all()


def test_the_oracle_gives_own_reachable_targets_for_an_external_robot():
    # What OwnReachable does in a goal task: sample q in the joint ranges, push it through forward kinematics.
    robot = PROVIDER.describe("arm3")
    q = kin.sample_joint_positions(robot, 200, np.random.default_rng(1))
    targets = kin.forward_ee_batch(robot, q)
    reach = np.linalg.norm(targets - [0.0, 0.0, 0.1], axis=1)
    assert reach.max() <= 0.9 + 1e-9 and reach.min() > 0.0


def test_forward_is_parametric_only():
    with pytest.raises(TypeError, match="needs a ParametricArm"):
        kin.forward(PROVIDER.describe("arm3"), np.zeros(3))
    assert isinstance(ARM, ParametricArm) and kin.forward(ARM, np.zeros(2)).ee.shape == (4, 4)


def test_kinematics_without_the_engine_is_a_clear_config_error():
    code = (
        "import sys\n"
        "sys.modules['mujoco'] = None\n"
        "import tests.conftest\n"
        "import numpy as np\n"
        "from robotdesigner.config import ConfigError, ExternalJoint, ExternalRobot, EndEffectorRef, Pose, kinematics as kin\n"
        "r = ExternalRobot(provider='toyrobot', name='arm3', entry='arm3.xml', source='s', joints=(ExternalJoint('j1', None, 1.0),),\n"
        "    home_qpos=(0.0,), home_root=None, keyframe=None, end_effector=EndEffectorRef('ee', None, Pose.identity()), actuator_kind='position')\n"
        "try:\n"
        "    kin.forward_ee_batch(r, np.zeros((1, 1)))\n"
        "except ConfigError as e:\n"
        "    assert e.path == 'robot' and 'mujoco' in e.reason and 'install' in e.reason, str(e)\n"
        "else:\n"
        "    raise SystemExit('no error')\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_describe_of_the_toy_provider_matches_a_hand_written_robot():
    robot = PROVIDER.describe("arm3")
    assert robot.joints[0] == ExternalJoint("j1", (-2.0, 2.0), 40.0) and robot.actuator_kind == "position"
    floater = PROVIDER.describe("floater")
    assert floater.home_root == Pose((0.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0)) and floater.end_effector is None
    assert floater.actuator_kind == "torque"
    assert BaseEnvConfig().robot(floater).resolve().control.mode == "joint_torque"
