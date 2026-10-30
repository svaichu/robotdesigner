import copy
import pickle

import pytest

from robotdesigner.config import (
    NOT_SET, BaseEnvConfig, Chain, ConfigError, ControlConfig, EnvConfig, FixedBox, FixedGains,
    MujocoOptions, Pose, SimConfig, TaskConfig, chain, register_simulator_options,
)
from robotdesigner.config import defaults
from robotdesigner.config.sim import SIMULATOR_OPTIONS
from tests.toy_provider import ReachParams


def hand_built():
    arm = chain(1, "z", total_length=0.5, total_mass=1.0, radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0,
                effort_limit=50.0, range=(-2.5, 2.5))
    from robotdesigner.tasks.empty import EmptyParams

    return EnvConfig(
        robot=arm,
        control=ControlConfig("joint_pos_abs", FixedGains(60.0, 6.0), None),
        task=TaskConfig(EmptyParams("idle"), 200, None, None, Pose.identity()),
        sim=SimConfig("mujoco", 1, "cpu", 0.002, "euler", 50.0, 0, "empty", MujocoOptions(False)),
    )


# 1. runs and equals the hand-built config --------------------------------------------------------------

def test_the_base_resolves_to_the_explicit_config_with_the_same_hash():
    cfg = BaseEnvConfig().resolve()
    assert cfg == hand_built() and cfg.env_hash == hand_built().env_hash
    assert EnvConfig.from_dict(cfg.to_dict()) == cfg


def test_the_base_builds_and_runs():
    pytest.importorskip("torch")
    pytest.importorskip("mujoco")
    pytest.importorskip("torchrl")
    env = BaseEnvConfig().sim(num_envs=2).build()
    assert env.env_hash == BaseEnvConfig().sim(num_envs=2).resolve().env_hash
    assert env.rollout(3).shape[0] == 2


# 2. every call returns a new config ----------------------------------------------------------------------

def test_calls_return_new_configs_and_leave_the_receiver_alone():
    base = BaseEnvConfig()
    before = copy.deepcopy(base)
    for new in (base.robot(n=2, axes="zy"), base.control(mode="joint_torque"), base.task(horizon=5), base.sim(seed=3)):
        assert new is not base and new != base
    assert base == before and base.diff() == {}
    only_sim = base.sim(num_envs=4).resolve()
    assert only_sim.robot == base.resolve().robot and only_sim.control == base.resolve().control
    assert only_sim.task == base.resolve().task


# 3. calls accumulate --------------------------------------------------------------------------------------

def test_calls_accumulate():
    cfg = BaseEnvConfig().sim(num_envs=8).sim(dt=0.001).resolve().sim
    assert cfg.num_envs == 8 and cfg.dt == 0.001
    c = BaseEnvConfig().task(ReachParams("reach", 0.0, False), success_tolerance=0.02,
                             target_region=FixedBox((0.1, -0.1, 0.0), (0.4, 0.1, 0.3))).task(horizon=7).resolve()
    assert c.task.horizon == 7 and c.task.params == ReachParams("reach", 0.0, False) and c.task.success_tolerance == 0.02
    base = BaseEnvConfig().robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0, damping=1.5))
    five = base.robot(Chain(n=5, axes="zyyyy")).resolve().robot
    assert five.n == 5 and five.segments[0].joint.damping == 1.5  # the rest carries
    assert sum(s.link.mass for s in five.segments) == pytest.approx(3.0)
    assert sum(s.link.tip_offset[0] for s in five.segments) == pytest.approx(0.9)


def test_exclusive_pairs_clear_each_other():
    r = BaseEnvConfig().robot(n=2, axes="zy").robot(link_length=0.3).resolve().robot
    assert [s.link.tip_offset[0] for s in r.segments] == [0.3, 0.3]  # total_length of the base is cleared
    r = BaseEnvConfig().robot(link_length=0.3).robot(Chain(total_length=0.9)).resolve().robot
    assert r.segments[0].link.tip_offset[0] == 0.9
    m = BaseEnvConfig().robot(mass=2.0).resolve().robot
    assert m.segments[0].link.mass == 2.0
    both = Chain(link_length=0.3).merge(Chain(total_length=1.0))
    assert both.link_length is NOT_SET and both.total_length == 1.0


def test_a_ready_arm_is_a_fixed_robot_and_a_chain_replaces_it():
    arm = chain(2, "zy", total_length=0.6, total_mass=2.0, radius=0.02, damping=0.1, armature=0.0, frictionloss=0.0,
                effort_limit=10.0)
    b = BaseEnvConfig().robot(arm)
    assert b.resolve().robot == arm
    assert b.robot(n=3, axes="zyy").resolve().robot.n == 3  # back to the default chain with the fields on top
    with pytest.raises(ConfigError, match="^robot"):
        b.robot(arm, n=3)


def test_the_arm_of_a_chain_is_the_function_chain():
    kw = dict(radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0, range=(-2.0, 2.0))
    assert Chain(n=3, axes="zyy", link_length=0.3, mass=1.0, **kw).arm() == chain(3, "zyy", link_length=0.3, mass=1.0, **kw)
    with pytest.raises(ConfigError, match="^radius: not set"):
        Chain(n=1, axes="z").arm()


# 4. errors carry dotted paths -----------------------------------------------------------------------------

def test_unknown_keys_sections_and_types():
    base = BaseEnvConfig()
    for call, path in ((lambda: base.sim(num_env=2), "sim.num_env"), (lambda: base.robot(lenght=1), "robot.lenght"),
                       (lambda: base.control(gain=1), "control.gain"), (lambda: base.task(horizons=1), "task.horizons"),
                       (lambda: base.robot(3), "robot"), (lambda: base.sim(3), "sim"), (lambda: base.control(3), "control"),
                       (lambda: base.sim(simulator="nope"), "sim.simulator")):
        with pytest.raises(ConfigError) as e:
            call()
        assert e.value.path == path, (path, e.value)


def test_resolve_lists_every_problem_with_its_path():
    bad = BaseEnvConfig().robot(n=3, damping=-1.0).sim(num_envs=0).control(mode="warp")
    problems = bad.validate()
    assert {p.path for p in problems} == {"robot.axes", "sim.num_envs", "control.mode"}
    with pytest.raises(ConfigError) as e:
        bad.resolve()
    assert e.value.problems == tuple(problems) and "sim.num_envs" in str(e.value) and "control.mode" in str(e.value)
    with pytest.raises(ConfigError, match="^sim.dt") as one:
        BaseEnvConfig().sim(dt="fast").resolve()
    assert len(one.value.problems) == 1
    with pytest.raises(ConfigError, match="^control.gains.kp: expected 1 per-joint"):  # EnvConfig's cross check
        BaseEnvConfig().control(gains=FixedGains((1.0, 2.0), 1.0)).resolve()
    with pytest.raises(ConfigError, match="^task.params"):
        BaseEnvConfig().task(MujocoOptions(False)).resolve()
    with pytest.raises(ConfigError, match="^robot.axes"):  # n without axes: one letter per joint, no cycling
        BaseEnvConfig().robot(n=3).resolve()


# 5. explain, diff, defaults versions --------------------------------------------------------------------------

def test_explain_and_diff():
    cfg = BaseEnvConfig().robot(n=2, axes="zy").sim(num_envs=8)
    text = cfg.explain()
    assert "robot.n" in text and "2  [set]" in text and "sim.dt" in text and "[default]" in text
    assert "task.mount" in text and "[default (task)]" in text
    assert cfg.diff() == {"robot.n": (1, 2), "robot.axes": ("z", "zy"), "sim.num_envs": (1, 8)}
    assert BaseEnvConfig().diff() == {}


def test_a_change_of_a_default_changes_the_hash_but_a_stored_dict_still_reproduces(monkeypatch):
    stored = BaseEnvConfig().resolve()
    new_sim = {**defaults.DEFAULTS["sim"], "dt": 0.001}
    monkeypatch.setitem(defaults.DEFAULTS, "sim", new_sim)
    after = BaseEnvConfig().resolve()
    assert after.env_hash != stored.env_hash  # a config that relied on the default changed
    assert EnvConfig.from_dict(stored.to_dict()).env_hash == stored.env_hash  # the stored resolved dict is exact


# 6. the mount ----------------------------------------------------------------------------------------------

def test_the_mount_is_the_default_of_the_task_until_pinned():
    class Moved:
        provider = "empty"
        uses_goal = False
        name = "idle"

    base = BaseEnvConfig()
    assert base.resolve().task.mount == Pose.identity()
    pin = Pose((0.3, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0))
    pinned = base.task(mount=pin)
    assert pinned.resolve().task.mount == pin and "[set]" in [l for l in pinned.explain().splitlines() if "task.mount" in l][0]
    # a pinned mount survives a task change; an unpinned one follows the provider's default
    other = pinned.task(ReachParams("reach", 0.0, False), success_tolerance=0.02,
                        target_region=FixedBox((0.1, -0.1, 0.0), (0.4, 0.1, 0.3)))
    assert other.resolve().task.mount == pin
    assert base.task(ReachParams("reach", 0.0, False), success_tolerance=0.02,
                     target_region=FixedBox((0.1, -0.1, 0.0), (0.4, 0.1, 0.3))).resolve().task.mount == Pose.identity()


def test_robosuite_default_mount_is_filled_in():
    pytest.importorskip("robosuite")
    from robotdesigner.tasks.robosuite import LiftObject, RobosuiteParams

    cfg = BaseEnvConfig().task(RobosuiteParams("Lift", {}, LiftObject("cube", 0.84), 0.0, False)).sim(scene="empty").resolve()
    assert cfg.task.mount.pos == pytest.approx((-0.56, 0.0, 0.912))


def test_goal_fields_are_reset_when_the_task_changes_class():
    goal = BaseEnvConfig().task(ReachParams("reach", 0.0, False), success_tolerance=0.02,
                                target_region=FixedBox((0.1, -0.1, 0.0), (0.4, 0.1, 0.3)))
    back = goal.task(__import__("robotdesigner.tasks.empty", fromlist=["EmptyParams"]).EmptyParams("idle"))
    assert back.resolve().task.target_region is None and back.resolve().task.success_tolerance is None


# 7. the simulator resets the options --------------------------------------------------------------------------

def test_changing_the_simulator_resets_the_options():
    class DOpts:
        def __init__(self, k=1):
            self.k = k

        @classmethod
        def default(cls):
            return cls(7)

        def to_dict(self):
            return {"k": self.k}

        @classmethod
        def from_dict(cls, d):
            return cls(**d)

    class NoDefault(DOpts):
        default = None

    saved = dict(SIMULATOR_OPTIONS)
    try:
        register_simulator_options("dummy", DOpts)
        register_simulator_options("nodef", NoDefault)
        base = BaseEnvConfig().sim(options=MujocoOptions(True))
        assert base.sim(num_envs=2).resolve().sim.options == MujocoOptions(True)  # same simulator: options carry
        assert base.sim(simulator="dummy")._sim["options"].k == 7  # a new simulator: its defaults
        assert base.sim(simulator="dummy", options=DOpts(3))._sim["options"].k == 3
        with pytest.raises(ConfigError, match="^sim.options.*has no default options"):
            base.sim(simulator="nodef").resolve()
        assert base.sim(simulator="nodef", options=NoDefault(2))._sim["options"].k == 2
    finally:
        SIMULATOR_OPTIONS.clear()
        SIMULATOR_OPTIONS.update(saved)


def test_not_set_is_a_singleton_that_survives_copies_and_pickles():
    assert copy.deepcopy(NOT_SET) is NOT_SET and pickle.loads(pickle.dumps(NOT_SET)) is NOT_SET
    assert repr(NOT_SET) == "NOT_SET" and Chain().written() == ()
    assert MujocoOptions.default() == MujocoOptions(False)
