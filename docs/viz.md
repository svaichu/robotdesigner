# Recording and showing videos (`robotdesigner.viz`)

Render a rollout and turn it into a GIF or MP4: save it, or show it inline in a notebook. Recording is
Gymnasium-style: the live state is rendered after every step while the rollout runs.

```python
from robotdesigner.viz import record_rollout

traj, clip = record_rollout(env, 150, env_index=0)   # trajectory (a TensorDict) and a Clip
clip                       # last expression of a notebook cell: plays inline
clip.save("run.mp4")       # the format follows the suffix: .mp4 or .gif
clip.show("gif")           # show inline in a chosen format
```

Related pages: [simulators](simulators.md) (the env contract, `env.render`), [robosuite scenes](tasks/robosuite.md) (the arena cameras). The walkthrough is `examples/scene_viz.ipynb`; `examples/record_scene.py` is the same thing as a headless script.

## Install and headless machines

```bash
pip install -e ".[env,viz]"
```

`viz` adds Pillow (GIF) and imageio-ffmpeg (MP4: H.264, yuv420p; it ships its own ffmpeg binary, so no system ffmpeg
is needed). `import robotdesigner.viz` itself needs only numpy; the other libraries load when a file is written.

Rendering needs an OpenGL context. On a headless Linux machine set `MUJOCO_GL=egl` **before the first import of
mujoco** (in a notebook: before the first cell that imports it, or restart the kernel). With several GPUs,
`MUJOCO_EGL_DEVICE_ID` picks one. EGL rendering worked on a CPU-only cluster node; `osmesa` and `glfw` did not there.
Without a context, `env.render()` raises a `RuntimeError` that says this.

## What a frame shows

`env.render(env_index=0, *, camera=None, size=(240, 320), overlays=True)` returns one `uint8 (H, W, 3)` frame of the
live state of one env, so any step loop can collect frames itself.

- **The whole MuJoCo scene**: floor, robot, and anything else in the compiled model.
- **Overlays** (`overlays=True`), drawn by the task's `overlays` hook from task state that is not part of the model (a
  goal marker, say). A task that keeps everything in the model draws none: the robosuite scenes have none, their
  objects are bodies of the model. Overlays are added to a scratch scene, never to the model.
- Rendering never changes the simulation. It works on a scratch copy of the state (a test checks that rendering every
  step leaves `qpos`, `qvel`, time and warm start bit-identical).
- The scene presets have no skybox, so the background above the floor is black.

## Cameras

`camera=` takes:

| value | meaning |
|---|---|
| `None` or `"iso"` | default view, framed on the workspace |
| `"front"`, `"side"`, `"top"` | other named views (world frame, `+x` forward) |
| `CameraSpec(lookat, distance, azimuth, elevation)` | a free camera, angles in degrees |
| `mujoco.MjvCamera`, a model camera name or id | passed to MuJoCo unchanged |

The named views are framed on a bounding sphere of the robot's workspace: base, floor point under the base,
end-effector positions for 512 random joint configurations (own random generator, so an env's random stream is not
touched), the home pose and the corners of a `FixedBox` region.

**Comparing embodiments needs one camera for all of them.** Each embodiment's own default camera is framed on its own
workspace, so a short arm looks as big as a long one. `auto_camera(cfg_a, cfg_b, ...)` returns one `CameraSpec` that
covers every config; pass it as `camera=` to each recording. Frames then share a scale, and a small arm looks small.

```python
shared = auto_camera(cfg_small, cfg_big)
_, a = record_rollout(env_small, 60, policy_small, camera=shared, every=2, label=False)
_, b = record_rollout(env_big, 60, policy_big, camera=shared, every=2, label=False)
tile_clips([a, b], labels=["n=2", "n=5"])
```

With the same `sim.seed` and `num_envs` every embodiment gets the same scene at every reset, whatever its number of joints
([seeding](simulators.md#the-environment-contract)); `reset_td` pins injected state instead. A fixed camera of the scene, such
as the robosuite arena's `"rs_sideview"`, also shares a scale across embodiments.

## Reference

**`record_rollout(env, steps, policy=None, *, env_index=0, every=1, camera=None, size=(240, 320), fps=None, label=None,
overlays=True, ncols=None, reset_td=None, **rollout_kwargs) -> (trajectory, Clip)`**
Resets (with `reset_td` if given), runs `env.rollout(steps, policy, ...)` and renders after every `every`-th step.
The clip has the initial state, every `every`-th state and the final state: `steps + 1` frames at `every=1`, fewer if
the rollout stops early. `env_index` is an int, a list of ints or `"all"`; several envs are tiled (`ncols`) and
labelled (`label`, default on for several envs). The clip plays at `control_freq / every` fps unless `fps` is given.
Extra keyword arguments (`break_when_any_done=...`) go to `env.rollout`; `callback`, `auto_reset` and `tensordict` are
set by `record_rollout`.

**`Clip(frames, fps)`**: `frames` is `uint8 (T, H, W, 3)`. Properties `len()`, `size` (height, width), `duration`;
`with_fps(fps)`, `save(path, fmt=None) -> Path`, `to_bytes(fmt)`, `show(fmt=None)`. As the last expression of a notebook
cell a clip shows itself: an MP4 if imageio-ffmpeg is installed, otherwise a GIF with a note. Embedding more than
25 MB warns; save the file instead.

**`FrameRecorder(env, env_index=0, *, every=1, ...)`**: what `record_rollout` uses; it is a rollout callback with
`capture(step=None)`, `finish(steps)` and `clip()`. TorchRL calls a rollout callback after every step except the last
and except the step that stops a rollout early, so a hand-written loop must call `capture(0)` after the reset and `finish(k)` after
a rollout of `k` steps (the notebook shows this).

**Camera and frames**: `auto_camera(*cfgs, view="iso", fovy=45.0, aspect=4/3, margin=1.15)`, `CameraSpec`, `VIEWS`,
`workspace_points(cfg)`; `tile(frames, ncols=None, pad=2)`, `tile_clips(clips, ncols=None, labels=None)` (equal frame
size and fps; as long as the shortest clip), `annotate(frame, text, corner="tl")` (white text on a dark box).

**On the env**: `env.render(...)`, `env.close()` (frees the renderer), and in `robotdesigner.simulators.mujoco.render`:
`render_rgb(env, env_index, camera, size)` (the same as `env.render`) and `launch_viewer(env)` (interactive viewer, needs
a display).

## Rates, sizes and file formats

- **GIF** stores delays in 10 ms steps (browsers play delays under 20 ms at 100 ms), so only some rates are exact:
  50, 25, 20, 10 fps. Other rates warn and are stored as the nearest 10 ms delay (a 30 fps request is stored as 30 ms and
  plays at 33.3 fps). GIF files are much larger than MP4 files of the same clip.
- **MP4** pads odd frame sizes to even by repeating the last row / column (yuv420p needs even sizes) and does not
  rescale.
- **Frame size**: MuJoCo's offscreen buffer is 640x480 by default. A larger `size=` raises the model's visual buffer
  setting (`model.vis.global_.offwidth/offheight`) automatically; this does not touch the physics.
- Frames are kept in memory as uint8 arrays: 150 frames at 240x320 are about 35 MB per env.
- Rendering several envs costs one render per env per frame (about 14 ms each at 240x320 on a CPU node).

## Adding rendering to another simulator

`FrameRecorder` and `record_rollout` need only this from an env:

```python
env.batch_size[0]                                   # number of envs
env.render(env_index, *, camera, size, overlays)    # -> uint8 (H, W, 3)
env.cfg.sim.control_freq                            # or pass fps= to record_rollout
```

## Not included

- Only the CPU MuJoCo simulator renders so far.
- Frames are rendered while the rollout runs; replaying recorded states afterwards is not implemented.
- TorchRL's own `torchrl.render` stack is not wired in (see `.agent/future_ideas.md`).
- No depth or segmentation output.
- Tested with Python 3.13 (CI also runs 3.10 and 3.12).
