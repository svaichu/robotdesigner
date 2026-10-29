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
    ReachParams,
    SimConfig,
    TaskConfig,
    chain,
)


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


def test_robosuite_task_config_round_trip_hash_and_validation():
    from dataclasses import replace

    from robotdesigner.config import InertiaScaledGains, RobosuiteOptions, RobosuiteParams

    cfg = make_cfg()
    params = RobosuiteParams({"reward_shaping": True, "table_full_size": [0.8, 0.8, 0.05]}, 0.01, False)
    task = TaskConfig("robosuite/Lift", 100, None, None, params)
    sim = SimConfig("robosuite", 2, "cpu", 0.002, "euler", 20.0, 0, "env")
    rs = replace(
        cfg, task=task, sim=sim, backend_options=RobosuiteOptions("env_default"),
        control=ControlConfig("joint_pos_delta", InertiaScaledGains(7.0, 1.0), 0.05),
    )
    back = EnvConfig.from_dict(rs.to_dict())
    assert back == rs and back.env_hash == rs.env_hash and hash(back.task) == hash(task)
    assert back.task.params.kwargs["table_full_size"] == (0.8, 0.8, 0.05)
    reordered = RobosuiteParams({"table_full_size": (0.8, 0.8, 0.05), "reward_shaping": True}, 0.01, False)
    assert replace(rs, task=replace(task, params=reordered)).env_hash == rs.env_hash  # key order is not config
    changed = RobosuiteParams({"reward_shaping": False, "table_full_size": [0.8, 0.8, 0.05]}, 0.01, False)
    assert replace(rs, task=replace(task, params=changed)).env_hash != rs.env_hash

    ok = RobosuiteParams({}, 0.0, False)
    with pytest.raises(ConfigError, match="unknown task"):
        TaskConfig("robosuite/", 10, None, None, ok)
    with pytest.raises(ConfigError, match="success_tolerance: must be None"):
        TaskConfig("robosuite/Lift", 10, 0.02, None, ok)  # the library defines success
    with pytest.raises(ConfigError, match="env_kwargs.*set from the robot"):
        RobosuiteParams({"robots": "Panda"}, 0.0, False)  # robot, controller and gripper come from our config
    with pytest.raises(ConfigError, match="success_tolerance"):
        TaskConfig("native/reach", 10, None, None, ReachParams(0.0, False))  # reach still needs its goal
    with pytest.raises(ConfigError, match="base_placement"):
        RobosuiteOptions("nowhere")
