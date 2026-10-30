import copy

import pytest

from robotdesigner.config import (
    Pose,
    ConfigError,
    ControlConfig,
    EnvConfig,
    FixedBox,
    FixedGains,
    MujocoOptions,
    OwnReachable,
    SimConfig,
    TaskConfig,
    chain,
)
from tests.toy_provider import ReachParams


def make_cfg(dt=0.002, num_envs=8, horizon=200, kp=60.0, mass=3.0, region=None):
    arm = chain(
        3, "zyz", total_length=0.9, total_mass=mass,
        radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0,
    )
    return EnvConfig(
        robot=arm,
        control=ControlConfig(mode="joint_pos_abs", gains=FixedGains(kp=kp, kd=6.0), action_scale=None),
        task=TaskConfig(
            params=ReachParams(name="reach", reset_noise=0.0, terminate_on_success=False),
            horizon=horizon,
            success_tolerance=0.02,
            target_region=region or FixedBox(low=(0.2, -0.3, 0.0), high=(0.6, 0.3, 0.4)),
            mount=Pose.identity(),
        ),
        sim=SimConfig(simulator="mujoco", num_envs=num_envs, device="cpu", dt=dt, integrator="euler",
                      control_freq=50, seed=0, scene="plain_floor", options=MujocoOptions(self_collision=False)),
    )


def test_round_trip_resolve_and_hash():
    cfg = make_cfg()
    back = EnvConfig.from_dict(cfg.to_dict())
    assert back == cfg and back.env_hash == cfg.env_hash
    assert len(cfg.env_hash) == 12 and cfg.full_env_hash.startswith(cfg.env_hash)
    assert cfg.resolve().resolve() == cfg.resolve()
    assert cfg.resolve().env_hash == cfg.env_hash
    assert make_cfg(dt=0.001).env_hash != cfg.env_hash
    assert cfg.sim.n_substeps == 10


def test_own_reachable_round_trip():
    cfg = make_cfg(region=OwnReachable())
    assert EnvConfig.from_dict(cfg.to_dict()) == cfg


def test_hash_sensitivity_and_invariance():
    base = make_cfg()
    for other in (make_cfg(num_envs=4), make_cfg(horizon=100), make_cfg(kp=61.0), make_cfg(mass=3.1)):
        assert other.env_hash != base.env_hash
    d = base.to_dict()
    assert EnvConfig.from_dict(dict(reversed(list(d.items())))).env_hash == base.env_hash
    d["robot"]["label"] = "renamed"
    assert EnvConfig.from_dict(d).env_hash == base.env_hash


def test_validation_errors_with_paths():
    with pytest.raises(ConfigError, match="^control_freq"):
        SimConfig(simulator="mujoco", num_envs=1, device="cpu", dt=0.003, integrator="euler",
                  control_freq=50, seed=0, scene="plain_floor", options=MujocoOptions(False))
    with pytest.raises(ConfigError, match="^action_scale"):
        ControlConfig(mode="joint_pos_delta", gains=FixedGains(1.0, 1.0), action_scale=None)
    with pytest.raises(ConfigError, match="^action_scale"):
        ControlConfig(mode="joint_pos_abs", gains=FixedGains(1.0, 1.0), action_scale=0.1)
    with pytest.raises(ConfigError, match=r"^low\[0\]"):
        FixedBox(low=(1, 0, 0), high=(0, 1, 1))
    d = make_cfg().to_dict()
    d["sim"]["dt"] = 0.003
    with pytest.raises(ConfigError, match=r"^sim\.control_freq"):
        EnvConfig.from_dict(d)
    d = make_cfg().to_dict()
    d["task"]["target_region"]["low"] = [1, 0, 0]
    with pytest.raises(ConfigError, match=r"^task\.target_region\.low\[0\]"):
        EnvConfig.from_dict(d)


def test_cross_field_checks():
    cfg = make_cfg()
    with pytest.raises(ConfigError, match="control.gains.kp"):
        EnvConfig(cfg.robot, ControlConfig("joint_pos_abs", FixedGains((1.0, 2.0), 1.0), None),
                  cfg.task, cfg.sim)
    with pytest.raises(ConfigError, match="^sim: expected SimConfig"):
        EnvConfig(cfg.robot, cfg.control, cfg.task, cfg.sim.options)
    d = copy.deepcopy(cfg.to_dict())
    d["sim"]["simulator"] = "nope"
    with pytest.raises(ConfigError, match="sim.simulator"):
        EnvConfig.from_dict(d)


def test_the_options_belong_to_the_simulator():
    from dataclasses import replace

    cfg = make_cfg()
    assert cfg.sim.options == MujocoOptions(self_collision=False)
    assert cfg.to_dict()["sim"]["options"] == {"self_collision": False}
    assert "backend_options" not in cfg.to_dict() and list(cfg.to_dict()) == ["robot", "control", "task", "sim"]
    with pytest.raises(ConfigError, match="^options: simulator 'mujoco' requires MujocoOptions"):
        replace(cfg.sim, options=ReachParams("reach", 0.0, False))
    with pytest.raises(ConfigError, match="^simulator: no options registered for simulator 'nope'"):
        replace(cfg.sim, simulator="nope")
    d = copy.deepcopy(cfg.to_dict())
    d["sim"]["options"]["self_collision"] = "yes"
    with pytest.raises(ConfigError, match=r"^sim\.options\.self_collision"):
        EnvConfig.from_dict(d)
    d = copy.deepcopy(cfg.to_dict())
    del d["sim"]["options"]
    with pytest.raises(ConfigError, match=r"^sim"):
        EnvConfig.from_dict(d)
    on = replace(cfg, sim=replace(cfg.sim, options=MujocoOptions(self_collision=True)))
    assert on.env_hash != cfg.env_hash and EnvConfig.from_dict(on.to_dict()) == on


def test_inertia_scaled_gains_round_trip_and_validation():
    from dataclasses import replace

    from robotdesigner.config import InertiaScaledGains

    cfg = make_cfg()
    cfg2 = replace(cfg, control=ControlConfig("joint_pos_delta", InertiaScaledGains(20.0, 1.0), 0.05))
    back = EnvConfig.from_dict(cfg2.to_dict())
    assert back == cfg2 and back.env_hash == cfg2.env_hash != cfg.env_hash
    with pytest.raises(ConfigError, match="^bandwidth"):
        InertiaScaledGains(0.0, 1.0)


def test_task_config_rules_come_from_the_provider():
    ok = ReachParams("reach", 0.0, False)
    with pytest.raises(ConfigError, match="success_tolerance"):
        TaskConfig(ok, 10, None, None, mount=Pose.identity())  # a goal task needs its tolerance
    with pytest.raises(ConfigError, match="target_region"):
        TaskConfig(ok, 10, 0.02, None, mount=Pose.identity())  # and its target region
    with pytest.raises(ConfigError, match="horizon"):
        TaskConfig(ok, 0, 0.02, OwnReachable(), mount=Pose.identity())
    with pytest.raises(ConfigError, match="^params: expected a task provider's params object"):
        TaskConfig(MujocoOptions(False), 10, 0.02, OwnReachable(), mount=Pose.identity())
    for bad in ("", "nope", "Reach"):
        with pytest.raises(ConfigError, match="^params.name: unknown task"):
            TaskConfig(ReachParams(bad, 0.0, False), 10, 0.02, OwnReachable(), mount=Pose.identity())
    d = make_cfg().task.to_dict()
    with pytest.raises(ConfigError, match="^provider: unknown task provider 'nope'"):
        TaskConfig.from_dict({**d, "provider": "nope"})
    with pytest.raises(ConfigError, match="^params.name: unknown task 'toy/nope'"):
        TaskConfig.from_dict({**d, "params": {**d["params"], "name": "nope"}})
    with pytest.raises(ConfigError, match="^params.reset_noise"):
        TaskConfig.from_dict({**d, "params": {**d["params"], "reset_noise": -1.0}})
    with pytest.raises(ConfigError, match="unknown keys: id"):
        TaskConfig.from_dict({**d, "id": "toy/reach"})  # the id is derived, never stored


def test_task_id_is_derived_and_the_dict_names_the_provider():
    task = make_cfg().task
    assert task.id == "toy/reach"
    with pytest.raises((AttributeError, TypeError)):  # read-only
        task.id = "toy/other"
    d = task.to_dict()
    assert d["provider"] == "toy" and d["params"]["name"] == "reach" and "id" not in d
    assert TaskConfig.from_dict(d) == task
    reordered = EnvConfig.from_dict({**make_cfg().to_dict(), "task": dict(reversed(list(d.items())))})
    assert reordered.env_hash == make_cfg().env_hash


def test_task_params_validate_themselves():
    with pytest.raises(ConfigError, match="^reset_noise"):
        ReachParams("reach", -1.0, False)
    with pytest.raises(ConfigError, match="terminate_on_success"):
        ReachParams("reach", 0.0, "yes")
    spelled = TaskConfig(ReachParams("reach", 0.05, True), 200, 0.02, OwnReachable(), mount=Pose.identity())
    assert TaskConfig.from_dict(spelled.to_dict()) == spelled


def test_the_mount_belongs_to_the_task_not_the_robot():
    from dataclasses import replace

    cfg = make_cfg()
    moved = replace(cfg, task=replace(cfg.task, mount=Pose((0.1, 0.2, 0.3), (1.0, 0.0, 0.0, 0.0))))
    assert moved.robot == cfg.robot and moved.robot.config_hash == cfg.robot.config_hash  # one embodiment, whatever the mount
    assert moved.env_hash != cfg.env_hash and moved.to_dict()["task"]["mount"]["pos"] == [0.1, 0.2, 0.3]
    assert EnvConfig.from_dict(moved.to_dict()) == moved
    assert "base_pose" not in cfg.robot.to_dict()
    with pytest.raises(ConfigError, match="^task.mount"):
        EnvConfig.from_dict({**cfg.to_dict(), "task": {**cfg.to_dict()["task"], "mount": [0, 0, 0]}})
