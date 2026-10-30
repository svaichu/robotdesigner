"""Config side of the robosuite task provider. Needs neither robosuite nor an engine."""

from dataclasses import replace

import pytest

from robotdesigner.config import (
    ConfigError, ControlConfig, EnvConfig, FixedGains, MujocoOptions, OwnReachable, Pose, SimConfig, TaskConfig, chain,
)
from robotdesigner.tasks.robosuite import LiftObject, ReachObject, RobosuiteParams
from robotdesigner.tasks.robosuite.params import RESERVED_ENV_KWARGS, objective_from_dict


def make_cfg(params=None, name="Lift"):
    arm = chain(3, "zyy", total_length=0.9, total_mass=3.0, radius=0.03, damping=0.5, armature=0.01,
                frictionloss=0.0, effort_limit=50.0, range=(-2.5, 2.5), base_pose=Pose((-0.56, 0.0, 1.2), (1, 0, 0, 0)))
    params = params or RobosuiteParams({"table_full_size": [0.8, 0.8, 0.05]}, LiftObject("cube", 0.84), 0.0, False)
    return EnvConfig(
        robot=arm,
        control=ControlConfig("joint_pos_abs", FixedGains(60.0, 6.0), None),
        task=TaskConfig(f"robosuite/{name}", 100, None, None, params),
        sim=SimConfig("mujoco", 2, "cpu", 0.002, "euler", 50, 0, "empty"),
        backend_options=MujocoOptions(False),
    )


def test_round_trip_and_hash():
    cfg = make_cfg()
    back = EnvConfig.from_dict(cfg.to_dict())
    assert back == cfg and back.env_hash == cfg.env_hash and hash(back.task) == hash(cfg.task)
    assert back.task.params.kwargs["table_full_size"] == (0.8, 0.8, 0.05)
    assert cfg.task.to_dict()["params"]["objective"] == {"kind": "lift_object", "object": "cube", "height": 0.84}


def test_hash_follows_every_field_except_key_order():
    cfg = make_cfg()
    p = cfg.task.params
    reordered = RobosuiteParams({"table_full_size": (0.8, 0.8, 0.05)}, p.objective, 0.0, False)
    assert replace(cfg, task=replace(cfg.task, params=reordered)).env_hash == cfg.env_hash
    for changed in (
        replace(p, env_kwargs={"table_full_size": [0.9, 0.8, 0.05]}),
        replace(p, objective=LiftObject("cube", 0.9)),
        replace(p, objective=ReachObject("cube", 0.05)),
        replace(p, reset_noise=0.1),
        replace(p, terminate_on_success=True),
    ):
        assert replace(cfg, task=replace(cfg.task, params=changed)).env_hash != cfg.env_hash
    assert make_cfg(name="Stack").env_hash != cfg.env_hash


def test_validation_errors_carry_paths():
    ok = RobosuiteParams({}, ReachObject("cube", 0.1), 0.0, False)
    with pytest.raises(ConfigError, match="env_kwargs.*set from the robot"):
        RobosuiteParams({"robots": "Panda"}, ReachObject("cube", 0.1), 0.0, False)
    assert {"robots", "control_freq", "seed", "has_renderer"} <= RESERVED_ENV_KWARGS
    with pytest.raises(ConfigError, match="^env_kwargs.x"):
        RobosuiteParams({"x": object()}, ReachObject("cube", 0.1), 0.0, False)
    with pytest.raises(ConfigError, match="^env_kwargs: keys"):
        RobosuiteParams({"not valid": 1}, ReachObject("cube", 0.1), 0.0, False)
    with pytest.raises(ConfigError, match="^objective"):
        RobosuiteParams({}, "lift", 0.0, False)
    with pytest.raises(ConfigError, match="^tolerance"):
        ReachObject("cube", 0.0)
    with pytest.raises(ConfigError, match="^object"):
        LiftObject("", 0.8)
    with pytest.raises(ConfigError, match="^reset_noise"):
        RobosuiteParams({}, ReachObject("cube", 0.1), -1.0, False)
    with pytest.raises(ConfigError, match="unknown objective"):
        objective_from_dict({"kind": "juggle"})
    with pytest.raises(ConfigError, match="unknown task"):
        TaskConfig("robosuite/", 10, None, None, ok)
    with pytest.raises(ConfigError, match="success_tolerance: must be None"):
        TaskConfig("robosuite/Lift", 10, 0.02, None, ok)  # the objective defines success
    with pytest.raises(ConfigError, match="target_region: must be None"):
        TaskConfig("robosuite/Lift", 10, None, OwnReachable(), ok)
    with pytest.raises(ConfigError, match="requires RobosuiteParams"):
        TaskConfig("robosuite/Lift", 10, None, None, LiftObject("cube", 0.84))
    d = make_cfg().to_dict()
    d["task"]["params"]["objective"]["kind"] = "juggle"
    with pytest.raises(ConfigError, match=r"^task\.params\.objective\.kind"):
        EnvConfig.from_dict(d)
