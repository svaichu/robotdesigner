"""The compile snaps the robot base onto the scene's mount marker (``TaskImpl.mount_site``)."""

import math

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from robotdesigner.config import (  # noqa: E402
    BaseEnvConfig, ConfigError, ExternalRobot, MobileMount, Mount, Pose, chain, kinematics as kin, world_mount,
)
from robotdesigner.config.mount import compose  # noqa: E402
from robotdesigner.simulators.mujoco.compile import compile_robot  # noqa: E402
from tests.toy_provider import ReachParams  # noqa: E402
from tests.toy_provider.framed import ELSEWHERE, BadParams, FRAME, FramedParams  # noqa: E402

MARKER = "mount_marker"

ARM = chain(3, "zyy", total_length=0.9, total_mass=3.0, radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0,
            effort_limit=50.0, range=(-2.5, 2.5))
OFFSET = Pose((0.1, -0.2, 0.05), (math.cos(0.3), 0.0, 0.0, math.sin(0.3)))


def cfg_of(params, mount, robot=ARM):
    from robotdesigner.config import OwnReachable

    return (
        BaseEnvConfig().robot(robot)
        .task(params, mount=mount, success_tolerance=0.02, target_region=OwnReachable())
        .sim(num_envs=2, scene="plain_floor").resolve()
    )


def base_pose(compiled):
    m = compiled.model
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    return d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy()


def marker_at(pose):
    """What a task's ``attach`` does: a site in a static (nested) body at ``pose``."""
    def attach(spec):
        holder = spec.worldbody.add_body(name="mount_holder", pos=list(pose.pos), quat=list(pose.quat))
        holder.add_site(name=MARKER, size=[0.01, 0.01, 0.01])
    return attach


def compiled_for(cfg, at=FRAME):
    return compile_robot(cfg, scene_hook=marker_at(at), mount_site=MARKER)


def test_default_mount_puts_the_base_body_on_the_marker():
    cfg = cfg_of(FramedParams("reach", 0.0, False), Mount())
    compiled = compiled_for(cfg)
    pos, R = base_pose(compiled)
    assert np.allclose(pos, FRAME.pos, atol=1e-9) and np.allclose(R, kin.pose_to_matrix(FRAME)[:3, :3], atol=1e-9)
    assert np.allclose(compiled.interface.mount.pos, FRAME.pos)


def test_an_offset_moves_the_base_in_the_markers_axes():
    cfg = cfg_of(FramedParams("reach", 0.0, False), Mount(OFFSET))
    pos, R = base_pose(compiled_for(cfg))
    T = kin.pose_to_matrix(FRAME) @ kin.pose_to_matrix(OFFSET)
    assert np.allclose(pos, T[:3, 3], atol=1e-9) and np.allclose(R, T[:3, :3], atol=1e-9)
    assert np.allclose(pos - np.asarray(FRAME.pos), kin.pose_to_matrix(FRAME)[:3, :3] @ np.asarray(OFFSET.pos), atol=1e-9)


def test_the_world_end_effector_follows_the_oracle():
    cfg = cfg_of(FramedParams("reach", 0.0, False), Mount(OFFSET))
    compiled = compiled_for(cfg)
    model, data = compiled.model, mujoco.MjData(compiled.model)
    q = np.array([0.4, -0.3, 0.6])
    data.qpos[compiled.interface.qpos_adr] = q
    mujoco.mj_forward(model, data)
    want = kin.forward(cfg.robot, q, world_mount(cfg)).ee[:3, 3]
    assert np.allclose(data.site_xpos[compiled.interface.ee_site_id], want, atol=1e-9)


def test_the_base_frame_end_effector_is_the_same_in_two_scenes():
    """The point of the mount: the same arm at the same q sees the same base-frame end effector wherever the scene puts it."""
    pytest.importorskip("torchrl")
    from robotdesigner.envs import make_env

    def ee_in_base(cfg):
        env = make_env(cfg)
        env.reset()
        out = env._sim.ee_pos_base()[0].copy()
        env.close()
        return out

    framed = ee_in_base(cfg_of(FramedParams("reach", 0.0, False), Mount()))
    plain = ee_in_base(cfg_of(ReachParams("reach", 0.0, False), Mount()))  # a scene with no marker, frame at the origin
    assert np.allclose(framed, plain, atol=1e-9)


def test_no_marker_means_the_providers_frame_and_the_presets_behave_as_before():
    cfg = BaseEnvConfig().robot(ARM).task(mount=Mount(OFFSET)).sim(num_envs=1, scene="plain_floor").resolve()
    compiled = compile_robot(cfg)
    pos, _ = base_pose(compiled)
    assert np.allclose(pos, OFFSET.pos, atol=1e-9) and compiled.interface.mount == OFFSET


def test_a_marker_that_disagrees_with_mount_frame_is_refused_naming_both_poses():
    cfg = cfg_of(BadParams("reach", 0.0, False), Mount())
    with pytest.raises(ConfigError) as e:
        compiled_for(cfg, at=ELSEWHERE)
    msg = str(e.value)
    assert "mount_marker" in msg and "0.4" in msg and "0.1" in msg and "mount_frame" in msg


def test_a_marker_the_scene_does_not_have_is_refused():
    cfg = cfg_of(FramedParams("reach", 0.0, False), Mount())
    with pytest.raises(ConfigError, match="no site of that name"):
        compile_robot(cfg, scene_hook=lambda spec: None, mount_site="missing")


def test_the_built_env_records_the_world_mount_and_the_marker_stays_out_of_the_spaces():
    pytest.importorskip("torchrl")
    from robotdesigner.envs import make_env

    cfg = cfg_of(FramedParams("reach", 0.0, False), Mount(OFFSET))
    env = make_env(cfg)
    assert np.allclose(env.world_mount.pos, compose(FRAME, OFFSET).pos, atol=1e-9)
    assert env.action_spec.shape[-1] == ARM.n
    env.close()


def test_a_floating_robot_spawns_at_the_world_mount_with_its_heading():
    from robotdesigner.config import get_robot_provider

    floater = get_robot_provider("toyrobot").describe("floater")
    assert isinstance(floater, ExternalRobot) and floater.home_root is not None
    mount = MobileMount((0.5, 0.2, 0.1), 0.7)
    cfg = BaseEnvConfig().robot(floater).task(mount=mount).sim(num_envs=1, scene="plain_floor").resolve()
    compiled = compile_robot(cfg)  # empty/idle: no marker, the frame is the origin
    heading = Pose((0.5, 0.2, 0.1), (math.cos(0.35), 0.0, 0.0, math.sin(0.35)))
    want = compose(heading, floater.home_root)
    assert np.allclose(compiled.interface.mount.pos, heading.pos, atol=1e-9)
    assert np.allclose(compiled.interface.home_root.pos, want.pos, atol=1e-9)
    assert np.allclose(compiled.interface.home_root.quat, want.quat, atol=1e-9)
