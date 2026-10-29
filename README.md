# robotdesigner

Describe robot arms as validated, hashable data and run them in batched [TorchRL](https://github.com/pytorch/rl)
environments, so the same control task can be compared across embodiments.

## Install

```bash
pip install -e .              # config layer only (numpy)
pip install -e ".[env]"       # + torch, torchrl, tensordict, mujoco: the CPU MuJoCo backend
pip install -e ".[env,dev]"   # + pytest, ruff
```

Python >= 3.10. Other extras: `viz` (Pillow, imageio-ffmpeg: GIF and MP4 output), `robosuite` (the robosuite
backend, below) and `warp` (a later milestone; declared only).
`import robotdesigner.config` works with numpy alone and never imports an engine.

## The idea

- **The robot is data.** A `ParametricArm` is frozen, validated and serializable. It has a stable
  `config_hash` (12 characters, SHA-256 prefix) that ignores labels and float noise.
- **Everything that changes behaviour is in `EnvConfig`**: robot, control, task, sim and backend options. There
  are no hidden defaults. `env_hash` identifies a run.
- **Backends realize configs.** `sim.backend = "mujoco"` is the CPU reference backend. An unsupported combination
  (control mode, device, task, ...) is rejected up front with every problem listed, never approximated.
- **One env contract.** Every backend returns a TorchRL `EnvBase` with `batch_size = [num_envs]`. The env never
  autoresets; reset rows with the `_reset` mask.

## Quick start

```python
from robotdesigner.config import (
    ControlConfig, EnvConfig, FixedBox, FixedGains, MujocoOptions, Pose, ReachParams,
    SimConfig, TaskConfig, chain,
)
from robotdesigner.envs import make_env

arm = chain(n=3, axes="zyz", total_length=0.9, total_mass=3.0, radius=0.03,
            damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0, range=(-2.5, 2.5),
            base_pose=Pose((0.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0)))

cfg = EnvConfig(
    robot=arm,
    control=ControlConfig("joint_pos_abs", FixedGains(kp=60.0, kd=6.0), action_scale=None),
    task=TaskConfig("native/reach", horizon=200, success_tolerance=0.02,
                    target_region=FixedBox(low=(0.2, -0.3, -0.2), high=(0.6, 0.3, 0.2)),
                    params=ReachParams(reset_noise=0.0, terminate_on_success=False)),
    sim=SimConfig(backend="mujoco", num_envs=8, device="cpu", dt=0.002, integrator="euler",
                  control_freq=50, seed=0, scene="plain_floor"),
    backend_options=MujocoOptions(self_collision=False),
)

env = make_env(cfg)
print(env.robot_hash, env.env_hash)
traj = env.rollout(50)          # random policy, TorchRL's own rollout
print(traj.shape)               # torch.Size([8, 50])
```

The full script is `examples/reach_cpu.py` (headless; `python examples/reach_cpu.py`). A walkthrough notebook is `examples/reach_cpu.ipynb` (embodiment comparison, control modes, rendering). Recording, saving and showing videos: `examples/reach_viz.ipynb`.

Step by hand:

```python
td = env.reset()
td["action"] = env.action_spec.rand()          # [8, n] in [-1, 1]
td = env.step(td)                              # outputs under ("next", ...)

# or, with driver-side autoreset (the env itself never resets):
data, td = env.step_and_maybe_reset(td)
```

## robosuite backend

Any single-arm robosuite environment, with the robot built from our `ParametricArm`. robosuite keeps the task, scene,
gripper and reward; robotdesigner supplies the arm.

```python
from robotdesigner.config import (ControlConfig, EnvConfig, InertiaScaledGains, Pose, RobosuiteOptions,
                                  RobosuiteParams, SimConfig, TaskConfig, ToolSpec, chain)
from robotdesigner.envs import make_env

arm = chain(4, "zyyy", link_length=[0.1, 0.4, 0.4, 0.15], total_mass=8.0, radius=0.04, damping=1.0, armature=0.05,
            frictionloss=0.0, effort_limit=80.0, range=(-2.9, 2.9), home_qpos=(0.0, -1.06, 1.91, 0.72),
            base_pose=Pose((0.0, 0.0, 1.1), (1.0, 0.0, 0.0, 0.0)),            # shoulder height above the stock mount
            end_effector=Pose((0.0, 0.0, 0.0), (0.7071068, 0.0, 0.7071068, 0.0)),   # gripper axis along the last link
            tool=ToolSpec("PandaGripper", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
cfg = EnvConfig(
    robot=arm,
    control=ControlConfig("joint_pos_delta", InertiaScaledGains(bandwidth=7.0, damping_ratio=1.0), action_scale=0.05),
    task=TaskConfig("robosuite/Lift", horizon=200, success_tolerance=None, target_region=None,
                    params=RobosuiteParams({"reward_shaping": True}, reset_noise=0.0, terminate_on_success=False)),
    sim=SimConfig("robosuite", num_envs=4, device="cpu", dt=0.002, integrator="euler", control_freq=20.0, seed=0,
                  scene="env"),
    backend_options=RobosuiteOptions(base_placement="env_default"),
)
env = make_env(cfg)   # a TorchRL env with the same contract as native/reach
```

- **Tasks:** `robosuite/<EnvName>` for any registered single-arm env (`Lift`, `Stack`, `Door`, `PickPlace`, `Wipe`, ...).
  Task arguments go in `RobosuiteParams.env_kwargs`; robosuite's own defaults for the rest are recorded in
  `env.resolved_env_kwargs` and are not part of the config hash. Two-arm envs are refused, as are
  `PickPlaceSingle` / `NutAssemblySingle`, which pick a random object per episode (fix it with
  `single_object_mode=2, object_type=...`).
- **Robot:** `ToolSpec.gripper` must be a robosuite gripper (`Wipe` insists on `WipingGripper`). The action is
  `n` joint increments followed by the gripper's dimensions. `base_placement="env_default"` mounts the arm where the
  environment mounts a stock arm and `base_pose` is relative to that point; `"explicit"` puts the root at the arena
  origin and `base_pose` is the world pose.
- **Control:** `joint_pos_delta` with `InertiaScaledGains`, with the same accumulated target as the native backend.
  robosuite's controller adds gravity compensation and re-evaluates the inertia every step, so dynamics are close to,
  not identical with, the native servo.
- **Fixed by robosuite:** `sim.dt` is 0.002, the integrator is MuJoCo's default (`"euler"`), and `sim.scene` is `"env"`.
- **Observations:** `task` holds the environment's own observations, `proprio` the robot's (`qpos`, `qvel` first).
- **Versions:** robosuite 1.5.x does not run on MuJoCo >= 3.10 as released; `apply_compat()` patches that in the running
  process (it applies only what the installed MuJoCo needs) and is called automatically.
- **Cost:** one robosuite environment per batch element, stepped in turn on one CPU core.

## Control modes (CPU backend)

| mode | action `a in [-1, 1]` |
|---|---|
| `joint_pos_abs` | finite range mapped affinely onto `[lo, hi]`; unlimited joints map to `a * pi` |
| `joint_pos_delta` | `q_target += a * action_scale`, clipped to the joint range |
| `joint_vel` | velocity target `a * action_scale` (rad/s) |
| `joint_torque` | `a * effort_limit` |

Gains are `FixedGains(kp, kd)` or `InertiaScaledGains(bandwidth, damping_ratio)`: `kp_i = bandwidth^2 * M_ii`,
`kd_i = max(2 * damping_ratio * sqrt(kp_i * M_ii) - damping_i, 0)`, with `M` the joint-space mass matrix at
`home_qpos` (an approximation, since `M` changes with configuration). The effective gains are stored on the
compiled model's `RobotInterface` (`kp`, `kd`).

## Targets

`FixedBox(low, high)` samples targets uniformly in the base frame. `OwnReachable()` uses end-effector positions of
uniformly sampled joint configurations, so targets are always reachable. Reset can inject `target`, `qpos` and
`qvel` through the reset TensorDict.

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
(floor, robot, anything else in the model) plus overlays drawn from task state: the target (a sphere of radius
`success_tolerance` with a cross-hair, red until reached, then green) and, for a `FixedBox` region, its edges as a
wireframe box.
Rendering works on a scratch copy of the state, so it never changes the simulation.
Only the CPU MuJoCo backend (`native/reach`) renders so far.

- **Which envs.** `env_index` is an int, a list of ints or `"all"`. Several envs are tiled and labelled.
- **Camera.** A view name (`"iso"` is the default, `"front"`, `"side"`, `"top"`; world frame, `+x` forward), a
  `CameraSpec`, a `mujoco.MjvCamera`, or a camera name in the model. The named views are framed on the robot's
  reachable workspace and the target region.
- **Comparing embodiments.** Use one camera for all of them: `cam = auto_camera(cfg_a, cfg_b)`, then
  `record_rollout(env_a, 150, camera=cam)`, the same for `env_b`, and `tile_clips([clip_a, clip_b], labels=[...])`.
  Pass the same `reset_td` (for example one that injects `target`) to give them the same task.
- **Rate.** The clip plays at `control_freq / every` fps. A GIF stores delays in 10 ms steps, so use 50, 25, 20 or
  10 fps for an exact rate (a warning says when it is not exact). MP4 files are much smaller than GIFs.
- **Low level.** `FrameRecorder` is the rollout callback behind `record_rollout`; `render_rgb(env, ...)` is
  `env.render` under its old name; `launch_viewer(env)` opens the interactive viewer (needs a display).

`examples/reach_record.py` records a four-env rollout to `reach.mp4` and `reach.gif`. The full guide is `docs/viz.md`.

## Layout

```
src/robotdesigner/
    config/      numpy only: configs, chain() generator, validation, hashing, kinematics oracle
    backends/    protocol, capabilities and lazy registry; mujoco/ is the CPU backend, robosuite/ wraps robosuite tasks
    envs/        make_env(cfg)
    viz/         Clip (save GIF / MP4, show inline), frame recorder, camera framing; numpy only
```

The design spec is in `docs/spec_robotconfig_env.md`.

## Tests

```bash
pytest -m "not gpu and not robosuite and not render"
```

Markers: `gpu`, `robosuite`, `render` (skipped when no OpenGL context is available).
