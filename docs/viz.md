# Rendering and video: `robotdesigner.viz`

Every environment renders the **live state** of one of its worlds, so any loop can collect frames, and `robotdesigner.viz` turns frames into clips you can show in a notebook or save as MP4 or GIF.
Rendering never changes the simulation. Both simulators render (a Warp environment renders from a host copy of the world).

| need | for |
|---|---|
| an OpenGL context | any `env.render` |
| `pip install -e ".[env,viz]"` | saving: Pillow writes GIFs and annotates frames, `imageio-ffmpeg` writes MP4s |

`import robotdesigner.viz` imports numpy only; Pillow, imageio and MuJoCo load when first needed.

## Headless machines

Without a display (a cluster node, a container) use EGL, and set it **before MuJoCo is first imported** (in a notebook: before any cell that imports it, or restart the kernel):

```bash
export MUJOCO_GL=egl
export MUJOCO_EGL_DEVICE_ID=0     # on a multi-GPU node: pin the GPU that renders; pick an idle one
```

OSMesa and GLFW did not work in headless tests of this package; EGL is the supported route. Rendering is for inspection and figures, not for the training loop.

## One frame

```python no-run
frame = env.render(0, camera="iso", size=(240, 320))        # uint8 (H, W, 3)
```

`env.render(env_index=0, *, camera=None, size=(240, 320), overlays=True)`. `camera` is:

| value | meaning |
|---|---|
| `None` or `"iso"` | an isometric view framed on the workspace |
| `"front"`, `"side"`, `"top"` | the other preset views (`viz.VIEWS`) |
| a `CameraSpec` | a free camera: `lookat`, `distance`, `azimuth`, `elevation` (degrees) |
| a name or id | a camera in the model, e.g. `"rs_agentview"` from a [robosuite scene](tasks/robosuite.md) |
| an `mujoco.MjvCamera` | used as given |

`overlays=True` also draws what the task adds that is not in the model (a goal marker).

## Cameras that compare embodiments

An `"iso"` camera framed on each arm separately makes a 0.5 m arm and a 2 m arm look the same size, which hides exactly what you want to see. `auto_camera` returns **one** camera that covers the workspace of every
config you give it, so clips of different embodiments are drawn at the same scale:

```python
import numpy as np

from robotdesigner.config import BaseEnvConfig, Chain
from robotdesigner.viz import VIEWS, Clip, auto_camera, tile

small = BaseEnvConfig().robot(Chain(n=2, axes="zy", total_length=0.5, total_mass=1.0)).resolve()
large = BaseEnvConfig().robot(Chain(n=4, axes="zyyy", total_length=1.5, total_mass=4.0)).resolve()   # configs or bare arms both work

cam_both = auto_camera(small, large, view="iso")
assert cam_both.distance > auto_camera(small, view="iso").distance       # the shared camera has to be farther back than the small arm needs
assert (cam_both.azimuth, cam_both.elevation) == VIEWS["iso"]
```

With the same seed the arms also get the same task draws ([seeding](simulators.md#seeding)), so the scenes are the same except for the robot. Or use a fixed arena camera of the scene.

## Recording a rollout

`record_rollout(env, steps, policy=None, *, env_index=0, every=1, camera=None, size=(240, 320), fps=None, label=None, overlays=True, ncols=None, reset_td=None, **rollout_kwargs)`
resets, runs `env.rollout`, renders after every `every`-th step and returns `(trajectory, clip)`. The clip holds the initial state, every `every`-th state and the final state (`steps + 1` frames at `every=1`).

```python no-run
from robotdesigner.viz import record_rollout

traj, clip = record_rollout(env, 150, env_index=0, camera="rs_sideview")
clip                          # the last expression of a notebook cell plays inline
clip.save("run.mp4")          # the format follows the suffix: .mp4 or .gif

traj, tiled = record_rollout(env, 150, env_index=[0, 1, 2, 3], ncols=2)   # several envs of one batch in one clip, labelled "env i  step t"
```

- `env_index` is one index, a list, or `"all"`; several are tiled into a grid (`ncols`) and labelled (`label=False` to turn that off).
- `fps` defaults to `control_freq / every`, so a clip plays in real time.
- `policy` is anything `env.rollout` takes; `reset_td` goes to `env.reset` (to inject a target, say). Other keyword arguments go to `env.rollout`.
- `callback`, `auto_reset` and `tensordict` are set by the function and cannot be passed.

To record inside your own loop use `FrameRecorder(env, ...)` : call `capture()` after the reset, use the recorder as the `callback` of `env.rollout` (TorchRL calls it after every step but the last, never before the first),
call `finish(steps)` at the end, then `clip()`.

## `Clip`

A `Clip` is `frames` (`uint8 (T, H, W, 3)`) plus `fps`, immutable.

| | |
|---|---|
| `len(clip)`, `clip.size` `(H, W)`, `clip.duration` | frames, frame size, seconds |
| `clip.save(path)` | writes `.mp4` (H.264, needs `imageio-ffmpeg`) or `.gif` (needs Pillow); creates parent directories |
| `clip.to_bytes(fmt)` | the encoded file in memory |
| `clip.show()` or a bare `clip` | plays inline in a notebook |
| `clip.with_fps(fps)` | the same frames at another rate |

Side by side, for comparing embodiments: `tile_clips([clip_a, clip_b], ncols=2, labels=["n=2", "n=4"])` (same frame size and fps, as long as the shortest), and `tile(frames, ncols)` /
`annotate(frame, text, corner="tl")` for single frames.

```python
frames = np.zeros((10, 48, 64, 3), dtype=np.uint8)
clip = Clip(frames, fps=50.0)
assert len(clip) == 10 and clip.size == (48, 64) and abs(clip.duration - 0.2) < 1e-12
assert tile([frames[0], frames[1]], ncols=2).shape[0] == 48 + 2 * 2                       # two frames side by side, padded
try:
    Clip(frames.astype(float), 50.0)
except ValueError:
    pass
```

A worked notebook: `examples/scene_viz.ipynb`.
