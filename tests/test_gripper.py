import json
from dataclasses import replace

import numpy as np
import pytest

from robotdesigner.config import BaseEnvConfig, ConfigError, ParametricArm, ToolSpec, TwoFingerGripper, chain
from robotdesigner.config.sim import SimConfig  # noqa: F401

COMMON = dict(radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0)


def arm(**kw):
    return chain(2, "zy", total_length=0.6, total_mass=2.0, range=(-2.0, 2.0), **COMMON, **kw)


def test_robot_package_is_the_home_of_the_robot_classes():
    import robotdesigner.config as cfg
    import robotdesigner.robot as robot

    assert cfg.ParametricArm is robot.ParametricArm and cfg.TwoFingerGripper is robot.TwoFingerGripper
    assert cfg.chain is robot.chain and cfg.Pose is robot.Pose


def test_no_gripper_leaves_the_hash_unchanged():
    a = arm()
    assert a.gripper is None and "gripper" not in a.to_dict()
    assert a.config_hash == arm(gripper=None).config_hash


def test_gripper_is_hashed_and_round_trips():
    g = TwoFingerGripper.default()
    a = arm(gripper=g)
    assert a.config_hash != arm().config_hash
    assert a.config_hash != arm(gripper=TwoFingerGripper.default(stroke=0.05)).config_hash
    b = ParametricArm.from_dict(json.loads(json.dumps(a.to_dict())))
    assert b == a and b.config_hash == a.config_hash
    assert g.max_opening == pytest.approx(0.08)


@pytest.mark.parametrize("field", ["finger_length", "finger_radius", "stroke", "finger_mass", "palm_length", "palm_mass", "force_limit"])
def test_gripper_validation(field):
    with pytest.raises(ConfigError, match=field):
        TwoFingerGripper.default(**{field: 0.0})
    with pytest.raises(ConfigError, match="kp"):
        TwoFingerGripper.default(kp=-1.0)


def test_gripper_and_tool_are_exclusive():
    with pytest.raises(ConfigError, match="not both"):
        arm(gripper=TwoFingerGripper.default(), tool=ToolSpec("PandaGripper", (0, 0, 0), (1, 0, 0, 0)))


def test_base_env_config_accepts_a_gripper():
    base = BaseEnvConfig().robot(n=3, axes="zyy", total_length=0.9, total_mass=2.0, gripper=TwoFingerGripper.default())
    assert base.resolve().robot.gripper == TwoFingerGripper.default()
