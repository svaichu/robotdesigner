import copy

import pytest

from robotdesigner.config import (
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
from robotdesigner.tasks.native import ReachParams


def make_cfg(dt=0.002, num_envs=8, horizon=200, kp=60.0, mass=3.0, region=None):
    arm = chain(
        3, "zyz", total_length=0.9, total_mass=mass,
        radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0,
    )
    return EnvConfig(
        robot=arm,
        control=ControlConfig(mode="joint_pos_abs", gains=FixedGains(kp=kp, kd=6.0), action_scale=None),
        task=TaskConfig(
            id="native/reach",
            horizon=horizon,
            success_tolerance=0.02,
            target_region=region or FixedBox(low=(0.2, -0.3, 0.0), high=(0.6, 0.3, 0.4)),
            params=ReachParams(reset_noise=0.0, terminate_on_success=False),
        ),
        sim=SimConfig(backend="mujoco", num_envs=num_envs, device="cpu", dt=dt, integrator="euler",
                      control_freq=50, seed=0, scene="plain_floor"),
        backend_options=MujocoOptions(self_collision=False),
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
        SimConfig(backend="mujoco", num_envs=1, device="cpu", dt=0.003, integrator="euler",
                  control_freq=50, seed=0, scene="plain_floor")
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
                  cfg.task, cfg.sim, cfg.backend_options)
    with pytest.raises(ConfigError, match="backend_options"):
        EnvConfig(cfg.robot, cfg.control, cfg.task, cfg.sim, ReachParams(0.0, False))
    d = copy.deepcopy(cfg.to_dict())
    d["sim"]["backend"] = "nope"
    with pytest.raises(ConfigError, match="sim.backend"):
        EnvConfig.from_dict(d)


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
    ok = ReachParams(0.0, False)
    with pytest.raises(ConfigError, match="success_tolerance"):
        TaskConfig("native/reach", 10, None, None, ok)  # a goal task needs its tolerance
    with pytest.raises(ConfigError, match="target_region"):
        TaskConfig("native/reach", 10, 0.02, None, ok)  # and its target region
    with pytest.raises(ConfigError, match="horizon"):
        TaskConfig("native/reach", 0, 0.02, OwnReachable(), ok)
    with pytest.raises(ConfigError, match="params: task 'native/reach' requires ReachParams"):
        TaskConfig("native/reach", 10, 0.02, OwnReachable(), MujocoOptions(False))
    for bad in ("native/", "native/nope", "nope/reach", "reach", "", "Native/reach", 3):
        with pytest.raises(ConfigError, match="unknown task"):
            TaskConfig(bad, 10, 0.02, OwnReachable(), ok)
    with pytest.raises(ConfigError, match="unknown task"):
        TaskConfig.from_dict({**make_cfg().task.to_dict(), "id": "nope/reach"})
