"""Clip, frame helpers, camera framing and the recorder. No OpenGL; Pillow / imageio-ffmpeg tests skip if absent."""

import subprocess
from types import SimpleNamespace

import numpy as np
import pytest

import robotdesigner.viz.clip as clipmod
from robotdesigner.config import (
    ControlConfig, EnvConfig, FixedBox, FixedGains, MujocoOptions, Pose, ReachParams, SimConfig, TaskConfig, chain,
)
from robotdesigner.viz import (
    CameraSpec, Clip, FrameRecorder, annotate, auto_camera, record_rollout, tile, tile_clips, workspace_points,
)


def _frames(t=6, h=24, w=32):
    return np.random.default_rng(0).integers(0, 255, (t, h, w, 3), dtype=np.uint8)


def _cfg(n=3, length=0.9):
    arm = chain(
        n, ("zy" * n)[:n], total_length=length, total_mass=2.0, radius=0.03, damping=0.5, armature=0.01,
        frictionloss=0.0, effort_limit=50.0, range=(-2.0, 2.0), base_pose=Pose((0.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0)),
    )
    return EnvConfig(
        robot=arm,
        control=ControlConfig("joint_pos_abs", FixedGains(60.0, 6.0), None),
        task=TaskConfig("native/reach", 20, 0.02, FixedBox((0.2, -0.3, 0.0), (0.6, 0.3, 0.4)), ReachParams(0.0, False)),
        sim=SimConfig("mujoco", 2, "cpu", 0.002, "euler", 50, 0, "plain_floor"),
        backend_options=MujocoOptions(False),
    )


# --- Clip ---------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "frames, fps",
    [
        (np.zeros((3, 4, 4, 3), np.float32), 25),  # wrong dtype
        (np.zeros((4, 4, 3), np.uint8), 25),  # missing time axis
        (np.zeros((3, 4, 4, 4), np.uint8), 25),  # RGBA
        (np.zeros((0, 4, 4, 3), np.uint8), 25),  # empty
        (np.zeros((3, 4, 4, 3), np.uint8), 0),  # bad fps
        (np.zeros((3, 4, 4, 3), np.uint8), float("nan")),
    ],
)
def test_clip_validation(frames, fps):
    with pytest.raises(ValueError):
        Clip(frames, fps)


def test_clip_basics():
    clip = Clip(_frames(50, 24, 32), 25)
    assert len(clip) == 50 and clip.size == (24, 32) and clip.duration == 2.0
    assert repr(clip) == "Clip(50 frames, 24x32, 25 fps, 2.00 s)"
    assert clip.with_fps(50).duration == 1.0


def test_save_rejects_unknown_format(tmp_path):
    with pytest.raises(ValueError, match="mp4"):
        Clip(_frames(), 25).save(tmp_path / "a.webm")


def test_gif_roundtrip_and_exact_delay(tmp_path):
    pytest.importorskip("PIL")
    from PIL import Image

    path = Clip(_frames(6), 50).save(tmp_path / "sub" / "a.gif")  # creates the folder
    with Image.open(path) as im:
        assert im.n_frames == 6 and im.size == (32, 24)
        assert im.info["duration"] == 20 and im.info["loop"] == 0


def test_gif_rate_that_gif_cannot_store_warns(tmp_path):
    pytest.importorskip("PIL")
    from PIL import Image

    with pytest.warns(UserWarning, match="10 ms steps"):
        path = Clip(_frames(3), 30).save(tmp_path / "a.gif")
    with Image.open(path) as im:
        assert im.info["duration"] == 30


def test_mp4_is_h264_yuv420p_and_odd_sizes_are_padded_not_rescaled(tmp_path):
    imageio_ffmpeg = pytest.importorskip("imageio_ffmpeg")
    pytest.importorskip("imageio")
    path = Clip(_frames(8, 31, 47), 25).save(tmp_path / "a.mp4")
    probe = subprocess.run(
        [imageio_ffmpeg.get_ffmpeg_exe(), "-hide_banner", "-i", str(path)], capture_output=True, text=True
    ).stderr
    line = next(l for l in probe.splitlines() if "Video:" in l)
    assert "h264" in line and "yuv420p" in line and "48x32" in line  # 31x47 padded to 32x48
    n, secs = imageio_ffmpeg.count_frames_and_secs(str(path))
    assert n == 8 and abs(secs - 8 / 25) < 0.05


def test_to_bytes_matches_file(tmp_path):
    pytest.importorskip("PIL")
    clip = Clip(_frames(4), 25)
    assert clip.to_bytes("gif") == clip.save(tmp_path / "a.gif").read_bytes()


def test_repr_html_prefers_mp4_and_falls_back_to_gif(monkeypatch):
    pytest.importorskip("PIL")
    clip = Clip(_frames(4), 25)
    if clipmod.have("imageio") and clipmod.have("imageio_ffmpeg"):
        html = clip._repr_html_()
        assert "<video" in html and "data:video/mp4;base64," in html and "controls" in html
    monkeypatch.setattr(clipmod, "have", lambda name: name == "PIL")
    html = clip._repr_html_()
    assert "<img" in html and "data:image/gif;base64," in html and "showing a GIF" in html


def test_repr_html_without_any_encoder_explains(monkeypatch):
    monkeypatch.setattr(clipmod, "have", lambda name: False)
    monkeypatch.setattr(clipmod, "need", lambda module, what: (_ for _ in ()).throw(ImportError("no encoder")))
    assert "no encoder" in Clip(_frames(2), 25)._repr_html_()


# --- frames -------------------------------------------------------------------------------------------


def test_tile_layout():
    frames = [np.full((10, 12, 3), 10 * (k + 1), np.uint8) for k in range(5)]
    grid = tile(frames, ncols=3, pad=2)
    assert grid.shape == (2 * 10 + 3 * 2, 3 * 12 + 4 * 2, 3)
    assert grid[2, 2, 0] == 10 and grid[2 + 12, 2 + 14, 0] == 50  # tile 0 and tile 4 (second row, second column)
    assert (grid[:2] == 0).all()  # border
    with pytest.raises(ValueError):
        tile([np.zeros((4, 4, 3), np.uint8), np.zeros((5, 4, 3), np.uint8)])


def test_tile_clips_truncates_and_checks():
    a, b = Clip(_frames(6), 25), Clip(_frames(4), 25)
    both = tile_clips([a, b], ncols=2)
    assert len(both) == 4 and both.fps == 25 and both.size == (24 + 4, 2 * 32 + 6)
    with pytest.raises(ValueError, match="fps"):
        tile_clips([a, b.with_fps(10)])
    with pytest.raises(ValueError, match="size"):
        tile_clips([a, Clip(_frames(4, 20, 32), 25)])
    with pytest.raises(ValueError, match="labels"):
        tile_clips([a, b], labels=["only one"])


def test_annotate_marks_one_corner_only():
    pytest.importorskip("PIL")
    frame = np.full((60, 300, 3), 100, np.uint8)
    out = annotate(frame, "env 0  step 12")
    assert out.shape == frame.shape and out.dtype == np.uint8
    assert (out[:16, :60] != 100).any() and (out[:16, :60] == 255).any()  # white text on a dark box
    assert (out[30:, :] == 100).all() and (out[:, 200:] == 100).all()  # elsewhere unchanged
    assert (annotate(frame, "x", corner="br")[-16:, -20:] != 100).any()
    with pytest.raises(ValueError):
        annotate(frame, "x", corner="middle")


def test_tile_clips_with_labels():
    pytest.importorskip("PIL")
    a, b = Clip(_frames(3), 25), Clip(_frames(3), 25)
    plain, labelled = tile_clips([a, b]), tile_clips([a, b], labels=["one", "two"])
    assert plain.size == labelled.size and not np.array_equal(plain.frames, labelled.frames)


# --- camera -------------------------------------------------------------------------------------------


def test_auto_camera_covers_workspace():
    cfg = _cfg(3)
    cam = auto_camera(cfg, view="iso", fovy=45.0, aspect=4 / 3)
    pts = workspace_points(cfg)
    half = np.arctan(np.tan(np.radians(45.0) / 2))
    reach = np.linalg.norm(pts - np.array(cam.lookat), axis=1).max()
    assert cam.distance * np.sin(half) > reach  # margin included
    assert (cam.azimuth, cam.elevation) == (135.0, -25.0)


def test_auto_camera_portrait_needs_more_distance():
    cfg = _cfg(3)
    assert auto_camera(cfg, aspect=0.5).distance > auto_camera(cfg, aspect=4 / 3).distance


def test_shared_camera_covers_every_embodiment():
    small, big = _cfg(2, length=0.6), _cfg(4, length=1.4)
    shared = auto_camera(small, big)
    half = np.arctan(np.tan(np.radians(45.0) / 2))
    for cfg in (small, big):
        far = np.linalg.norm(workspace_points(cfg) - np.array(shared.lookat), axis=1).max()
        assert far < shared.distance * np.sin(half)  # everything of every embodiment is inside the view
    assert auto_camera(small).distance < auto_camera(big).distance  # their own cameras differ in scale


def test_auto_camera_accepts_a_bare_arm_and_rejects_bad_input():
    assert isinstance(auto_camera(_cfg(2).robot), CameraSpec)
    with pytest.raises(ValueError):
        auto_camera()
    with pytest.raises(ValueError, match="unknown view"):
        auto_camera(_cfg(2), view="underwater")


def test_workspace_points_do_not_touch_global_random_state():
    np.random.seed(5)
    before = np.random.get_state()[1].copy()
    workspace_points(_cfg(3))
    assert (np.random.get_state()[1] == before).all()


# --- recorder (fake env: no GL, no torch) -------------------------------------------------------------


class FakeEnv:
    """Just the contract the recorder relies on: batch_size, cfg.sim.control_freq and render()."""

    def __init__(self, n=3, freq=50.0):
        self.batch_size = (n,)
        self.cfg = SimpleNamespace(sim=SimpleNamespace(control_freq=freq))
        self.calls = []

    def render(self, env_index=0, *, camera=None, size=(8, 10), overlays=True):
        self.calls.append((env_index, camera, size, overlays))
        return np.full((*size, 3), 10 + env_index, np.uint8)


def test_recorder_every_and_finish_do_not_duplicate():
    env = FakeEnv()
    rec = FrameRecorder(env, every=2, size=(8, 10))
    rec.capture(0)
    for _ in range(5):  # torchrl calls the callback after steps 1..5 of a 6-step rollout
        rec(env, object())
    rec.finish(6)
    assert rec.steps == [0, 2, 4, 6]
    rec.finish(6)
    assert rec.steps == [0, 2, 4, 6]
    clip = rec.clip()
    assert len(clip) == 4 and clip.fps == 25.0 and clip.size == (8, 10)


def test_recorder_finish_adds_last_state_when_every_does_not_divide():
    env = FakeEnv()
    rec = FrameRecorder(env, every=4, size=(8, 10))
    rec.capture(0)
    for _ in range(5):
        rec()
    rec.finish(6)
    assert rec.steps == [0, 4, 6]


def test_recorder_tiles_selected_envs_and_labels():
    pytest.importorskip("PIL")
    env = FakeEnv(n=4)
    rec = FrameRecorder(env, "all", size=(40, 60), ncols=2)
    rec.capture(0)
    assert [c[0] for c in env.calls] == [0, 1, 2, 3]
    assert rec.frames[0].shape == (2 * 40 + 3 * 2, 2 * 60 + 3 * 2, 3)
    assert FrameRecorder(env, [3, 1]).indices == [3, 1] and FrameRecorder(env, 2).label is False


def test_recorder_validates():
    env = FakeEnv(n=2)
    with pytest.raises(IndexError):
        FrameRecorder(env, 2)
    with pytest.raises(ValueError):
        FrameRecorder(env, "some")
    with pytest.raises(ValueError):
        FrameRecorder(env, every=0)
    with pytest.raises(RuntimeError, match="no frames"):
        FrameRecorder(env).clip()
    env.cfg = None
    with pytest.raises(ValueError, match="fps"):
        rec = FrameRecorder(env)
        rec.capture(0)
        rec.clip()


def test_record_rollout_follows_the_torchrl_callback_contract():
    """TorchRL calls callback(env, td) after steps 1..k-1 of a k-step rollout; the recorder adds frames 0 and k."""

    class Env(FakeEnv):
        def reset(self, td=None):
            self.reset_arg = td
            return "td0"

        def rollout(self, steps, policy=None, *, callback, auto_reset, tensordict, **kw):
            assert auto_reset is False and tensordict == "td0" and kw == {"break_when_any_done": False}
            for _ in range(steps - 1):
                callback(self, object())
            return SimpleNamespace(shape=(self.batch_size[0], steps))

    env = Env()
    traj, clip = record_rollout(env, 5, size=(8, 10), reset_td="inject", break_when_any_done=False)
    assert env.reset_arg == "inject" and traj.shape == (3, 5)
    assert len(clip) == 6 and clip.fps == 50.0  # steps + 1 frames
    with pytest.raises(TypeError, match="callback"):
        record_rollout(env, 5, callback=print)
