"""The mount: the scene owns a frame, the task adds an offset (``Mount``) or a spawn and heading (``MobileMount``)."""

import math
from dataclasses import replace

import numpy as np
import pytest

from robotdesigner.config import (
    BaseEnvConfig,
    ConfigError,
    ExternalJoint,
    ExternalRobot,
    MobileMount,
    Mount,
    Pose,
    TaskConfig,
    world_mount,
)
from robotdesigner.config import (
    kinematics as kin,
)
from robotdesigner.config.mount import compose, mount_from_dict
from tests.toy_provider.framed import FRAME, FramedParams


def make_task(provider_params, mount):
    from robotdesigner.config import OwnReachable

    return TaskConfig(provider_params, 10, 0.02, OwnReachable(), mount)


# --- the types -------------------------------------------------------------------------------------------------------


def test_mount_defaults_to_snapped_and_round_trips():
    assert Mount().offset == Pose.identity()
    for m in (Mount(), Mount(Pose((0.1, 0.0, 0.0), (1, 0, 0, 0))), MobileMount((1.0, 2.0), 0.5), MobileMount((1, 2, 0.3))):
        assert mount_from_dict(m.to_dict()) == m


def test_mobile_mount_normalises_spawn_and_validates():
    assert MobileMount((1, 2)).spawn == (1.0, 2.0, 0.0)
    for bad in ((1,), (1, 2, 3, 4), "xy", (float("nan"), 0.0)):
        with pytest.raises(ConfigError, match="spawn"):
            MobileMount(bad)
    with pytest.raises(ConfigError, match="heading"):
        MobileMount((0, 0), float("inf"))
    with pytest.raises(ConfigError, match="offset"):
        Mount(offset=(0, 0, 0))


def test_the_two_types_never_collide_in_the_hash():
    cfg = BaseEnvConfig().resolve()
    base = cfg.env_hash
    shifted = replace(cfg, task=replace(cfg.task, mount=Mount(Pose((0.1, 0, 0), (1, 0, 0, 0)))))
    assert shifted.env_hash != base
    assert Mount().to_dict()["kind"] != MobileMount().to_dict()["kind"]
    assert MobileMount((1, 2), 0.1).to_dict() != MobileMount((1, 2), 0.2).to_dict()


def test_the_old_absolute_forms_are_rejected_with_the_replacement():
    cfg = BaseEnvConfig().resolve()
    with pytest.raises(ConfigError, match=r"^mount: .*Mount\(offset=Pose"):
        replace(cfg.task, mount=Pose.identity())
    with pytest.raises(ConfigError, match=r"^mount: .*relative to the scene"):
        TaskConfig.from_dict({**cfg.task.to_dict(), "mount": Pose.identity().to_dict()})
    with pytest.raises(ConfigError, match="^mount"):
        TaskConfig.from_dict({**cfg.task.to_dict(), "mount": {"pos": [0, 0, 0], "quat": [1, 0, 0, 0]}})


# --- the frame and the composition ---------------------------------------------------------------------------------------


def test_world_mount_is_the_frame_applied_to_the_offset():
    offset = Pose((0.5, 0.0, 0.2), (1, 0, 0, 0))
    t = make_task(FramedParams("reach", 0.0, False), Mount(offset))
    T = kin.pose_to_matrix(FRAME)
    want = T[:3, :3] @ np.array(offset.pos) + T[:3, 3]
    got = world_mount(t)
    assert np.allclose(got.pos, want)
    assert np.allclose(kin.pose_to_matrix(got), T @ kin.pose_to_matrix(offset))


def test_default_mount_snaps_onto_the_frame():
    assert world_mount(make_task(FramedParams("reach", 0.0, False), Mount())) == FRAME


def test_world_end_effector_is_world_mount_applied_to_the_base_frame_one():
    arm = BaseEnvConfig().resolve().robot
    t = make_task(FramedParams("reach", 0.0, False), Mount(Pose((0.1, 0.2, 0.0), (1, 0, 0, 0))))
    q = np.array(arm.home_qpos)
    world = kin.forward(arm, q, world_mount(t)).ee[:3, 3]
    base = kin.forward(arm, q, None).ee[:3, 3]
    T = kin.pose_to_matrix(world_mount(t))
    assert np.allclose(world, T[:3, :3] @ base + T[:3, 3])


def test_mobile_mount_is_a_yaw_and_a_translation_in_the_frame():
    t = make_task(FramedParams("reach", 0.0, False), MobileMount((1.0, 0.5, 0.2), math.pi / 2))
    got = kin.pose_to_matrix(world_mount(t))
    local = np.eye(4)
    local[:3, 3] = (1.0, 0.5, 0.2)
    local[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    assert np.allclose(got, kin.pose_to_matrix(FRAME) @ local)


def test_compose_is_associative_and_has_an_identity():
    a, b, c = FRAME, Pose((1, 0, 0), (0.7071067811865476, 0.7071067811865476, 0, 0)), Pose((0, 2, 0), (1, 0, 0, 0))
    assert np.allclose(kin.pose_to_matrix(compose(compose(a, b), c)), kin.pose_to_matrix(compose(a, compose(b, c))))
    assert compose(Pose.identity(), a) == a == compose(a, Pose.identity())


def test_empty_idle_frame_is_the_origin():
    assert world_mount(BaseEnvConfig().resolve()) == Pose.identity()


# --- the builder --------------------------------------------------------------------------------------------------------


def test_builder_has_no_mount_special_case():
    cfg = BaseEnvConfig()
    assert "[default (task)]" not in cfg.explain()
    assert cfg.resolve().task.mount == Mount()
    assert cfg.to_dict()["task"]["mount"] == Mount().to_dict()
    shifted = cfg.task(mount=Mount(Pose((0.1, 0, 0), (1, 0, 0, 0))))
    assert "task.mount" in shifted.diff() and "task.mount" not in cfg.diff()
    assert BaseEnvConfig.from_dict(shifted.to_dict()) == shifted


def test_a_mount_offset_can_be_written_by_dotted_path():
    cfg = BaseEnvConfig().with_overrides({"task.mount.offset.pos": [0.25, 0.0, 0.0]})
    assert cfg.resolve().task.mount.offset.pos == (0.25, 0.0, 0.0)


# --- the robot decides which type fits --------------------------------------------------------------------------------


def floater():
    return ExternalRobot(
        provider="toyrobot", name="floater", entry="m.xml", source="x",
        joints=(ExternalJoint("j", (-1.0, 1.0), 10.0),), home_qpos=(0.0,),
        home_root=Pose((0, 0, 0.3), (1, 0, 0, 0)), keyframe=None, end_effector=None, actuator_kind="position",
    )


def test_mount_and_robot_kind_must_agree():
    cfg = BaseEnvConfig().resolve()
    with pytest.raises(ConfigError, match=r"^task.mount: .*fixed-base"):
        replace(cfg, task=replace(cfg.task, mount=MobileMount()))
    robot = floater()
    with pytest.raises(ConfigError, match=r"^task.mount: .*floating base.*MobileMount"):
        replace(cfg, robot=robot, task=replace(cfg.task, mount=Mount()))
    replace(cfg, robot=robot, task=replace(cfg.task, mount=MobileMount((1, 2), 0.3)))  # fits
