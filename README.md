# robotdesigner

Describe robot arms as validated, hashable data and run them in batched [TorchRL](https://github.com/pytorch/rl)
environments, so the same control task can be compared across embodiments.

## Install

```bash
pip install -e .              # config layer only (numpy)
pip install -e ".[env]"       # + torch, torchrl, tensordict, mujoco: the CPU MuJoCo backend
pip install -e ".[env,robosuite]"   # + robosuite: its arenas and objects as tasks
pip install -e ".[env,dev]"   # + pytest, ruff
```

Python >= 3.10. Other extras: `robosuite` (the task provider: robosuite arenas and objects as tasks; the quick start needs it),
`viz` (Pillow, imageio-ffmpeg: GIF and MP4 output) and `warp` (a later milestone; declared only).
`import robotdesigner.config` works with numpy alone and never imports an engine.

## The idea

- **The robot is data.** A `ParametricArm` is frozen, validated and serializable. It has a stable
  `config_hash` (12 characters, SHA-256 prefix) that ignores labels and float noise.
- **Everything that changes behaviour is in `EnvConfig`**: robot, control, task, sim and backend options. There
  are no hidden defaults. `env_hash` identifies a run.
- **`env = make_env(cfg)`** is the one step from config to environment, for every task.
- **Simulators and tasks are separate.** `sim.backend = "mujoco"` names the simulator (the CPU reference backend). A
  task id is `"<namespace>/<name>"` and comes from a **task provider**: the package knows no task by name, the one
  provider it ships is `robosuite`, others register themselves (see [TaskConfig](docs/configs/task.md#where-task-ids-come-from)). An unsupported combination
  (control mode, device, task on this simulator, ...) is rejected up front with every problem listed, never
  approximated.
- **One env contract.** Every environment is a TorchRL `EnvBase` with `batch_size = [num_envs]`. The env never
  autoresets; reset rows with the `_reset` mask.

## Documentation

Reference pages in [`docs/`](docs/README.md): [RobotConfig](docs/configs/robot.md), [ControlConfig](docs/configs/control.md),
[TaskConfig](docs/configs/task.md), [SimConfig and backend options](docs/configs/sim.md), [EnvConfig](docs/configs/env.md),
[backends and the environment contract](docs/backends.md), the task provider [robosuite scenes](docs/tasks/robosuite.md), and the [design spec](docs/spec_robotconfig_env.md).
The Python examples in them are executed by the tests.

## Quick start

A three-link arm in the robosuite `Lift` scene (a table and a cube): robosuite supplies the assets, the simulator named
in `sim.backend` steps everything.

```python
from robotdesigner import make_env
from robotdesigner.config import (
    ControlConfig, EnvConfig, FixedGains, MujocoOptions, SimConfig, TaskConfig, chain,
)
from robotdesigner.tasks.robosuite import LiftObject, RobosuiteParams, default_mount   # a task comes from the provider that owns it

arm = chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0, radius=0.03,
            damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0, range=(-2.5, 2.5),
            base_pose=default_mount("Lift"))        # where robosuite mounts a stock arm in this scene

cfg = EnvConfig(
    robot=arm,
    control=ControlConfig("joint_pos_abs", FixedGains(kp=60.0, kd=6.0), action_scale=None),
    task=TaskConfig("robosuite/Lift", horizon=200, success_tolerance=None, target_region=None,
                    params=RobosuiteParams({}, LiftObject("cube", height=0.84),
                                           reset_noise=0.0, terminate_on_success=False)),
    sim=SimConfig(backend="mujoco", num_envs=8, device="cpu", dt=0.002, integrator="euler",
                  control_freq=50, seed=0, scene="empty"),     # the arena brings its own floor
    backend_options=MujocoOptions(self_collision=False),
)

env = make_env(cfg)
print(env.robot_hash, env.env_hash)
traj = env.rollout(50)          # random policy, TorchRL's own rollout
print(traj.shape)               # torch.Size([8, 50])
```

The full script is `examples/robosuite_scene.py` (headless; `python examples/robosuite_scene.py`). A walkthrough notebook is
`examples/scene_walkthrough.ipynb` (embodiment comparison, control modes, rendering). Recording, saving and showing
videos: `examples/scene_viz.ipynb`.

Step by hand:

```python
td = env.reset()
td["action"] = env.action_spec.rand()          # [8, n] in [-1, 1]
td = env.step(td)                              # outputs under ("next", ...)

# or, with driver-side autoreset (the env itself never resets):
data, td = env.step_and_maybe_reset(td)
```

## robosuite scenes as tasks

robosuite supplies the arena, the objects and their placement; our MuJoCo simulation steps everything, with your robot
and your control law. The task id is `robosuite/<EnvName>` and the config says what counts as success (the quick start
above is one). Full details: [robosuite scenes](docs/tasks/robosuite.md).

- **Assets, not physics.** The arena, objects and their placement rule are robosuite's; the time step, integrator,
  control modes and gains are yours. robosuite's controllers, robots, grippers and rewards are not used.
- **Success and reward** are the `objective` in `RobosuiteParams` (`LiftObject`, `ReachObject`).
- **Tried on:** `Lift`, `Stack`, `NutAssembly*`, `PickPlace*`, `Door`, `ToolHang`; the random-object `*Single` variants and
  two-arm environments are refused. The table is in the [robosuite page](docs/tasks/robosuite.md#which-environments-work).
- **Versions:** robosuite 1.5.x does not run on MuJoCo >= 3.10 as released; `apply_compat()` patches that in the running
  process and is called automatically.

## Control modes and targets

Four joint-space modes (`joint_pos_abs`, `joint_pos_delta`, `joint_vel`, `joint_torque`) with `FixedGains` or
`InertiaScaledGains`, and, for providers whose tasks are defined by a goal point, target regions `FixedBox` or `OwnReachable`. What each mode and
region means, and which simulator supports what, is in [ControlConfig](docs/configs/control.md), [TaskConfig](docs/configs/task.md) and
[backends](docs/backends.md). Reset can inject `qpos` and `qvel` through the reset TensorDict, and whatever the task
accepts (a task documents its own keys).

## Rendering and video (optional)

Rendering needs an OpenGL context; on headless machines set `MUJOCO_GL=egl` before the first import of mujoco.
Saving videos needs the `viz` extra (`pip install -e ".[env,viz]"`): Pillow writes GIFs, imageio-ffmpeg writes MP4s.

```python
from robotdesigner.viz import record_rollout

traj, clip = record_rollout(env, 150, env_index=0)   # renders the live state while the rollout runs
clip                      # last expression of a notebook cell: plays inline (MP4, or a GIF if MP4 support is missing)
clip.save("run.mp4")      # the format follows the suffix: .mp4 or .gif
clip.show("gif")          # show inline in a chosen format
```

`env.render(env_index=0, camera=None, size=(240, 320), overlays=True)` returns one `uint8 (H, W, 3)` frame of the live
state, like Gymnasium's `rgb_array`, so any step loop can collect its own frames. A frame shows the whole MuJoCo scene
(floor, robot, arena and objects of the task) plus the overlays the task draws for state that is not in the model (a task that keeps everything in the model, like
the robosuite scenes, draws none). Rendering works on a scratch copy of the state, so it never changes the simulation.
Only the CPU MuJoCo backend renders so far.

- **Which envs.** `env_index` is an int, a list of ints or `"all"`. Several envs are tiled and labelled.
- **Camera.** A view name (`"iso"` is the default, `"front"`, `"side"`, `"top"`; world frame, `+x` forward), a
  `CameraSpec`, a `mujoco.MjvCamera`, or a camera name in the model. The named views are framed on the robot's
  reachable workspace (and a box target region, if the task has one).
- **Comparing embodiments.** Use one camera for all of them: `cam = auto_camera(cfg_a, cfg_b)`, then
  `record_rollout(env_a, 150, camera=cam)`, the same for `env_b`, and `tile_clips([clip_a, clip_b], labels=[...])`.
  With the same `sim.seed` and `num_envs` they get the same scene: the task draws from its own random stream, which does
  not depend on the robot.
- **Rate.** The clip plays at `control_freq / every` fps. A GIF stores delays in 10 ms steps, so use 50, 25, 20 or
  10 fps for an exact rate (a warning says when it is not exact). MP4 files are much smaller than GIFs.
- **Low level.** `FrameRecorder` is the rollout callback behind `record_rollout`; `render_rgb(env, ...)` is
  `env.render` under its old name; `launch_viewer(env)` opens the interactive viewer (needs a display).

`examples/record_scene.py` records a four-env rollout to `scene.mp4` and `scene.gif`. The full guide is `docs/viz.md`.

## Layout

```
src/robotdesigner/
    config/      numpy only: configs, chain() generator, validation, hashing, kinematics oracle
    backends/    simulators: protocol, capabilities and lazy registry; mujoco/ is the CPU backend and its generic env
    tasks/       task providers, one subpackage per namespace (robosuite/), and the task contract (base.py)
    envs/        make_env(cfg)
    viz/         Clip (save GIF / MP4, show inline), frame recorder, camera framing; numpy only
```

Documentation is in `docs/` (start at `docs/README.md`); the design record is `docs/spec_robotconfig_env.md`.

## Tests

```bash
pytest -m "not gpu and not robosuite and not render"
```

Markers: `gpu`, `robosuite` (needs the robosuite extra: `pytest -m robosuite`), `render` (skipped when no OpenGL context
is available). `tests/test_docs.py` runs the Python
examples in `docs/` and checks that every link and anchor resolves; it needs no simulator.
