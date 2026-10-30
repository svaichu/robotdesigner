import copy
import pickle

import pytest

from robotdesigner.config import (
    NOT_SET, BaseEnvConfig, Chain, ConfigError, ControlConfig, EnvConfig, FixedBox, FixedGains, InertiaScaledGains,
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


# --- with_overrides ------------------------------------------------------------------------------------------

def goal_cfg():
    return BaseEnvConfig().task(ReachParams("reach", 0.0, False), success_tolerance=0.02,
                                target_region=FixedBox((0.1, -0.1, 0.0), (0.4, 0.1, 0.3))).robot(
        Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0))


def test_overrides_edit_nested_fields_and_leave_the_rest_of_the_section():
    cfg = goal_cfg()
    new = cfg.with_overrides({"sim.options.self_collision": True, "task.params.reset_noise": 0.1,
                              "robot.total_length": 1.2, "sim.num_envs": 64, "control.gains.kp": 99.0,
                              "task.target_region.low": [0.0, -0.1, 0.0]})
    r, o = new.resolve(), cfg.resolve()
    assert r.sim.options.self_collision is True and r.sim.num_envs == 64 and r.sim.dt == o.sim.dt
    assert r.task.params == ReachParams("reach", 0.1, False) and r.task.horizon == o.task.horizon
    assert r.task.target_region.low == (0.0, -0.1, 0.0) and r.task.target_region.high == o.task.target_region.high
    assert sum(s.link.tip_offset[0] for s in r.robot.segments) == pytest.approx(1.2)
    assert r.robot.n == 3 and r.control.gains.kp == 99.0 and r.control.gains.kd == o.control.gains.kd
    assert cfg.resolve() == o  # the receiver is unchanged


def test_cli_strings_are_json_literals_or_strings():
    new = BaseEnvConfig().with_overrides(["sim.num_envs=64", "robot.n=5", "robot.axes=zyyyy", "sim.integrator=rk4",
                                          "robot.range=[-2.0, 2.0]", "sim.options.self_collision=true",
                                          "control.action_scale=null", "sim.dt=1e-3"])
    r = new.resolve()
    assert r.sim.num_envs == 64 and r.robot.n == 5 and r.sim.integrator == "rk4" and r.sim.dt == 0.001
    assert r.robot.segments[0].joint.range == (-2.0, 2.0) and r.sim.options.self_collision is True
    assert r.control.action_scale is None
    assert new.with_overrides({"sim.num_envs": 64}) == new.with_overrides(["sim.num_envs=64"])
    for bad in ("sim.num_envs", "=3", 3):
        with pytest.raises(ConfigError):
            BaseEnvConfig().with_overrides([bad])


def test_override_errors_name_the_dotted_path():
    cfg = goal_cfg()
    for ov, path in (({"sim.num_env": 1}, "sim.num_env"), ({"nope.x": 1}, "nope"), ({"sim": 1}, "sim"),
                     ({"sim.options.zzz": 1}, "sim.options.zzz"), ({"task.params.zzz": 1}, "task.params.zzz"),
                     ({"robot.lenght": 1}, "robot.lenght"), ({"sim.dt.x": 1}, "sim.dt"),
                     ({"task.params.reset_noise": -1.0}, "task.params.reset_noise"), ({"sim.num_envs": 0}, "sim.num_envs")):
        with pytest.raises(ConfigError) as e:
            cfg.with_overrides(ov)
        assert e.value.path == path, (ov, e.value)


def test_overrides_clear_the_partner_of_an_exclusive_pair():
    r = BaseEnvConfig().with_overrides({"robot.link_length": 0.3}).resolve().robot
    assert r.segments[0].link.tip_offset[0] == 0.3
    r = BaseEnvConfig().with_overrides({"robot.mass": 2.0}).resolve().robot
    assert r.segments[0].link.mass == 2.0
    with pytest.raises(ConfigError, match="link_length/total_length"):  # both written in one call: reported
        BaseEnvConfig().with_overrides({"robot.link_length": 0.3, "robot.total_length": 1.0}).resolve()


def test_a_simulator_override_resets_the_options():
    saved = dict(SIMULATOR_OPTIONS)

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

    try:
        register_simulator_options("dummy", DOpts)
        base = BaseEnvConfig().sim(options=MujocoOptions(True))
        assert base.with_overrides({"sim.simulator": "dummy"})._sim["options"].k == 7
        both = base.with_overrides({"sim.options.k": 3, "sim.simulator": "dummy"})  # options edit the new simulator's
        assert both._sim["options"].k == 3
        assert base.with_overrides({"sim.simulator": "mujoco"}).resolve().sim.options == MujocoOptions(True)
    finally:
        SIMULATOR_OPTIONS.clear()
        SIMULATOR_OPTIONS.update(saved)


def test_a_ready_arm_and_the_pinned_mount_through_overrides():
    arm = chain(2, "zy", total_length=0.6, total_mass=2.0, radius=0.02, damping=0.1, armature=0.0, frictionloss=0.0,
                effort_limit=10.0)
    cfg = BaseEnvConfig().robot(arm).with_overrides({"robot.label": "mine", "task.mount": Pose((1, 0, 0), (1, 0, 0, 0))})
    r = cfg.resolve()
    assert r.robot.label == "mine" and r.task.mount.pos == (1.0, 0.0, 0.0)
    assert "[set]" in [l for l in cfg.explain().splitlines() if "task.mount" in l][0]


# --- dict, JSON and YAML round trips --------------------------------------------------------------------------

def test_dict_and_json_round_trips_keep_the_hash(tmp_path):
    cfg = goal_cfg().sim(num_envs=8).task(mount=Pose((0.1, 0.0, 0.2), (1.0, 0.0, 0.0, 0.0))).control(
        gains=InertiaScaledGains(20.0, 1.0))
    d = cfg.to_dict()
    assert d["defaults_version"] == defaults.DEFAULTS_VERSION and d["task"]["provider"] == "toy"
    assert "mount" not in goal_cfg().to_dict()["task"]  # the deferred default is omitted
    back = BaseEnvConfig.from_dict(d)
    assert back == cfg and back.resolve().env_hash == cfg.resolve().env_hash and back.diff() == cfg.diff()
    path = tmp_path / "run.json"
    cfg.dump(path)
    assert BaseEnvConfig.load(path).resolve().env_hash == cfg.resolve().env_hash
    assert BaseEnvConfig.from_dict(BaseEnvConfig().to_dict()) == BaseEnvConfig()
    arm = chain(2, "zy", total_length=0.6, total_mass=2.0, radius=0.02, damping=0.1, armature=0.0, frictionloss=0.0,
                effort_limit=10.0)
    fixed = BaseEnvConfig().robot(arm)
    assert BaseEnvConfig.from_dict(fixed.to_dict()).resolve().robot == arm


def test_short_dicts_take_the_base_for_what_they_leave_out():
    cfg = BaseEnvConfig.from_dict({"sim": {"num_envs": 4}, "robot": {"n": 2, "axes": "zy", "link_length": 0.3}})
    r = cfg.resolve()
    assert r.sim.num_envs == 4 and r.sim.dt == 0.002 and r.robot.n == 2
    assert r.robot.segments[0].link.tip_offset[0] == 0.3 and r.robot.segments[0].joint.damping == 0.5
    for bad, path in (({"sim": {"nope": 1}}, "sim"), ({"nope": {}}, ""), ({"robot": {"lenght": 1}}, "robot"),
                      ({"sim": {"num_envs": 0}}, "sim.num_envs"), ({"task": {"params": {"name": "nope"}}}, "task")):
        with pytest.raises(ConfigError) as e:
            BaseEnvConfig.from_dict(bad)
        assert e.value.path.startswith(path), (bad, e.value)


def test_a_defaults_version_mismatch_warns():
    d = {**BaseEnvConfig().to_dict(), "defaults_version": 0}
    with pytest.warns(UserWarning, match="defaults version 0"):
        BaseEnvConfig.from_dict(d)


def test_yaml_round_trip_and_the_missing_extra(tmp_path, monkeypatch):
    cfg = goal_cfg().sim(num_envs=8)
    try:
        import yaml  # noqa: F401
    except ImportError:
        with pytest.raises(ImportError, match=r"robotdesigner\[yaml\]"):
            cfg.dump(tmp_path / "run.yaml")
        return
    path = tmp_path / "run.yaml"
    cfg.dump(path)
    assert BaseEnvConfig.load(path).resolve().env_hash == cfg.resolve().env_hash
    import sys

    monkeypatch.setitem(sys.modules, "yaml", None)
    with pytest.raises(ImportError, match=r"robotdesigner\[yaml\]"):
        cfg.dump(tmp_path / "again.yml")
    with pytest.raises(ConfigError, match="^format"):
        cfg.dump(tmp_path / "x.json", format="toml")


def test_the_config_layer_imports_with_numpy_only():
    import subprocess
    import sys

    code = ("import sys\nfor m in ['torch','torchrl','tensordict','mujoco','robosuite','yaml']: sys.modules[m] = None\n"
            "from robotdesigner.config import BaseEnvConfig\n"
            "c = BaseEnvConfig().with_overrides(['sim.num_envs=2']); assert c.resolve().sim.num_envs == 2\n"
            "import json; BaseEnvConfig.from_dict(json.loads(json.dumps(c.to_dict())))\n")
    subprocess.run([sys.executable, "-c", code], check=True)


# --- sweep ----------------------------------------------------------------------------------------------------

PAIRED = {"robot.n": [2, 3, 4, 5], "robot.axes": ["zy", "zyy", "zyyy", "zyyyy"]}


def test_sweep_size_order_and_stable_distinct_hashes():
    cfg = BaseEnvConfig().robot(total_length=0.9, total_mass=3.0)
    family = cfg.sweep(paired=PAIRED, grid={"sim.seed": range(3), "sim.num_envs": [1, 2]})
    assert len(family) == 4 * 3 * 2 and family.notes == []
    rows = family.table()
    # the paired group first, then the grid axes in the order given, the last one fastest
    assert [(r["robot.n"], r["sim.seed"], r["sim.num_envs"]) for r in rows][:7] == [
        (2, 0, 1), (2, 0, 2), (2, 1, 1), (2, 1, 2), (2, 2, 1), (2, 2, 2), (3, 0, 1)]
    assert [r["index"] for r in rows] == list(range(24))
    hashes = [r["env_hash"] for r in rows]
    assert len(set(hashes)) == 24
    assert hashes == [r["env_hash"] for r in cfg.sweep(paired=PAIRED, grid={"sim.seed": range(3), "sim.num_envs": [1, 2]}).table()]
    assert [e.env_hash for e in family.configs()] == hashes and all(isinstance(e, EnvConfig) for e in family.configs())
    assert [c.resolve().robot.n for c in family][:1] == [2] and family[6].resolve().robot.n == 3
    assert family.overrides()[0] == {"robot.n": 2, "robot.axes": "zy", "sim.seed": 0, "sim.num_envs": 1}
    for env in family.configs():  # a worker rebuilds from the resolved dict
        assert EnvConfig.from_dict(env.to_dict()).env_hash == env.env_hash
    assert cfg.resolve() == BaseEnvConfig().robot(total_length=0.9, total_mass=3.0).resolve()  # the base is unchanged


def test_sweep_with_only_one_kind_of_axis():
    assert len(BaseEnvConfig().sweep(grid={"sim.seed": [0, 1]})) == 2
    assert len(BaseEnvConfig().sweep(paired=PAIRED)) == 4
    assert len(BaseEnvConfig().sweep()) == 1  # the base itself


def test_sweep_drops_repeated_configs_with_a_note():
    with pytest.warns(UserWarning, match="repeats item 0"):
        family = BaseEnvConfig().sweep(grid={"sim.seed": [0, 1, 0], "sim.num_envs": [1]})
    assert len(family) == 2 and len(family.notes) == 1 and "item 2" in family.notes[0]
    assert [r["sim.seed"] for r in family.table()] == [0, 1]


def test_sweep_errors_name_the_item_and_the_override():
    base = BaseEnvConfig()
    with pytest.raises(ConfigError, match="equal lengths") as e:
        base.sweep(paired={"robot.n": [2, 3], "robot.axes": ["zy"]})
    assert e.value.path == "sweep.paired"
    with pytest.raises(ConfigError, match="^robot.axes.*sweep item 1.*'robot.n': 3") as e:  # n without axes
        base.sweep(paired={"robot.n": [1, 3]})
    assert e.value.path == "robot.axes"
    with pytest.raises(ConfigError) as e:  # per-joint kp of the wrong length
        base.robot(n=2, axes="zy").sweep(grid={"control.gains.kp": [[1.0, 2.0], [1.0, 2.0, 3.0]]})
    assert e.value.path == "control.gains.kp" and "sweep item 1" in str(e.value)
    with pytest.raises(ConfigError, match="^sim.nope.*sweep item 0"):
        base.sweep(grid={"sim.nope": [1]})
    with pytest.raises(ConfigError, match="both paired and grid"):
        base.sweep(paired={"sim.seed": [1]}, grid={"sim.seed": [2]})
    with pytest.raises(ConfigError, match="^sweep.grid.sim.seed: expected a sequence"):
        base.sweep(grid={"sim.seed": 3})
    with pytest.raises(ConfigError, match="has no values"):
        base.sweep(grid={"sim.seed": []})


def test_sweeping_the_simulator_resets_the_options():
    saved = dict(SIMULATOR_OPTIONS)

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

    try:
        register_simulator_options("dummy", DOpts)
        base = BaseEnvConfig().sim(options=MujocoOptions(True))
        family = base.sweep(grid={"sim.simulator": ["mujoco", "dummy"]})
        assert [c._sim["options"] for c in family][0] == MujocoOptions(True) and family[1]._sim["options"].k == 7
        varied = base.sweep(grid={"sim.simulator": ["dummy"], "sim.options.k": [1, 2]})
        assert [c._sim["options"].k for c in varied] == [1, 2]
    finally:
        SIMULATOR_OPTIONS.clear()
        SIMULATOR_OPTIONS.update(saved)
