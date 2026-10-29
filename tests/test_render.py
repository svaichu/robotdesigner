import numpy as np
import pytest

pytest.importorskip("torchrl")
mujoco = pytest.importorskip("mujoco")


def _has_gl() -> bool:
    try:
        with mujoco.Renderer(mujoco.MjModel.from_xml_string("<mujoco/>"), 16, 16):
            return True
    except Exception:
        return False


needs_gl = pytest.mark.skipif(not _has_gl(), reason="no OpenGL context (try MUJOCO_GL=egl)")


def _env(num_envs=2, **kw):
    from robotdesigner.envs import make_env
    from tests.test_reach_env import make_cfg

    return make_env(make_cfg(num_envs=num_envs, **kw))


@pytest.mark.render
@needs_gl
def test_render_rgb_shape_and_dtype():
    from robotdesigner.backends.mujoco.render import render_rgb

    env = _env(2)
    env.reset()
    frame = render_rgb(env, env_index=1, size=(48, 64))
    assert frame.shape == (48, 64, 3) and frame.dtype == np.uint8


def test_render_rejects_bad_env_index():
    from robotdesigner.backends.mujoco.render import render_rgb

    env = _env(2)
    with pytest.raises(IndexError):
        render_rgb(env, env_index=5)
    with pytest.raises(IndexError):
        env.render(2)


@pytest.mark.render
@needs_gl
def test_env_render_is_gymnasium_style():
    env = _env(2)
    env.reset()
    frame = env.render()  # env 0, 240x320
    assert frame.shape == (240, 320, 3) and frame.dtype == np.uint8 and frame.mean() > 1.0
    assert env.render(1, size=(48, 64)).shape == (48, 64, 3)
    with pytest.raises(ValueError):
        env.render(0, size=(0, 10))


@pytest.mark.render
@needs_gl
def test_rendering_leaves_the_simulation_untouched():
    import torch

    n_envs = 2
    a, b = _env(n_envs), _env(n_envs)
    ta, tb = a.reset(), b.reset()
    action = torch.full((n_envs, 2), 0.7)
    for _ in range(8):
        ta["action"], tb["action"] = action.clone(), action.clone()
        ta, tb = a.step(ta)["next"], b.step(tb)["next"]
        for i in range(n_envs):  # only env a is rendered
            a.render(i, size=(32, 48))
    assert np.array_equal(a._sim.qpos, b._sim.qpos) and np.array_equal(a._sim.qvel, b._sim.qvel)
    for da, db in zip(a._sim.data, b._sim.data):
        assert da.time == db.time and np.array_equal(da.qacc_warmstart, db.qacc_warmstart)


def _channel_dominant(frame):
    """Counts of (green, red) dominant pixels: the colours of the reached / not reached target marker."""
    f = frame.astype(int)
    r, g, b = f[..., 0], f[..., 1], f[..., 2]
    return int(((g > r + 40) & (g > b + 40)).sum()), int(((r > g + 60) & (r > b + 60)).sum())


@pytest.mark.render
@needs_gl
def test_overlays_draw_the_target_and_region():
    import torch
    from tensordict import TensorDict

    env = _env(1)
    env.reset(TensorDict({"target": torch.tensor([[0.4, 0.0, 0.2]])}, batch_size=[1]))
    with_overlays, without = env.render(0, overlays=True), env.render(0, overlays=False)
    assert (with_overlays != without).any(axis=-1).sum() > 100  # pixels that changed


@pytest.mark.render
@needs_gl
def test_region_overlay_casts_no_shadow():
    env = _env(1)
    env.reset()
    on = env.render(0, size=(240, 320)).astype(int)
    off = env.render(0, size=(240, 320), overlays=False).astype(int)
    darker = ((off.sum(-1) - on.sum(-1)) > 90).mean()
    assert darker < 0.01  # a filled translucent box darkened about 4 % of the frame with its shadow


@pytest.mark.render
@needs_gl
def test_target_marker_turns_green_once_reached():
    import torch
    from tensordict import TensorDict

    env = _env(1)
    env.reset()
    ee = env._sim.ee_pos_base()[0]  # the arm returns to the same home pose on every reset (no reset noise)

    def reset_with_target(target):
        td = TensorDict({"target": torch.tensor(np.asarray(target)[None], dtype=torch.float32)}, batch_size=[1])
        out = env.reset(td)
        return out, env.render(0, size=(480, 640))

    out, reached = reset_with_target(ee)
    assert bool(out["success"].all())
    out, away = reset_with_target(ee + np.array([0.0, 0.25, 0.0]))
    assert not bool(out["success"].any())
    green_reached, red_reached = _channel_dominant(reached)
    green_away, red_away = _channel_dominant(away)
    assert green_reached > 20 and red_reached == 0  # visible although the sphere is inside the arm tip
    assert green_away == 0 and red_away > 20


@pytest.mark.render
@needs_gl
def test_named_views_and_custom_cameras():
    from robotdesigner.viz import VIEWS, auto_camera

    env = _env(1)
    env.reset()
    size = (60, 80)
    frames = {v: env.render(0, camera=v, size=size) for v in VIEWS}
    assert len({f.tobytes() for f in frames.values()}) == len(VIEWS)  # every view looks different
    spec = auto_camera(env.cfg, view="side", fovy=float(env._sim.model.vis.global_.fovy), aspect=80 / 60)
    assert np.array_equal(env.render(0, camera=spec, size=size), frames["side"])
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(env._sim.model, cam)
    assert env.render(0, camera=cam, size=size).shape == (60, 80, 3)


@pytest.mark.render
@needs_gl
def test_frame_larger_than_the_default_framebuffer():
    env = _env(1)
    env.reset()
    assert env.render(0, size=(720, 1280)).shape == (720, 1280, 3)  # MuJoCo's default buffer is 640x480
    assert env.render(0, size=(60, 80)).shape == (60, 80, 3)  # and the renderer follows size changes


@pytest.mark.render
@needs_gl
def test_close_releases_the_renderer():
    env = _env(1)
    env.reset()
    env.render()
    assert env._renderer is not None
    env.close()
    assert env._renderer is None
    env.close(raise_if_closed=False)


@pytest.mark.render
@needs_gl
def test_record_rollout_end_to_end(tmp_path):
    pytest.importorskip("PIL")  # labels are drawn when several envs are tiled
    from robotdesigner.viz import record_rollout

    env = _env(2)
    traj, clip = record_rollout(env, 6, env_index="all", size=(48, 64))
    assert traj.shape[-1] == 6 and len(clip) == 7  # initial state + 6 steps
    assert clip.fps == 50.0 and clip.size == (48 + 2 * 2, 2 * 64 + 3 * 2)
    assert clip.save(tmp_path / "run.gif").stat().st_size > 0


@pytest.mark.render
@needs_gl
def test_record_rollout_stops_with_the_rollout():
    from robotdesigner.viz import record_rollout

    env = _env(1, horizon=3)  # truncates after 3 steps
    traj, clip = record_rollout(env, 10, size=(32, 48))
    assert len(clip) == traj.shape[-1] + 1
