# Simulators and the environment contract

`env = make_env(cfg)` turns an [`EnvConfig`](configs/env.md) into a batched [TorchRL](https://github.com/pytorch/rl) environment. The **simulator** named by `cfg.sim.simulator` is the physics engine; the **task** comes
from a [provider](configs/task.md#where-tasks-come-from) and is written against an engine-neutral view of the simulation, so one task runs on every simulator.

| simulator | engine | devices | environment class | extra |
|---|---|---|---|---|
| `mujoco` | MuJoCo on the CPU: the reference and debugging simulator. The `num_envs` worlds are stepped one after another in one Python process | `cpu` | `MujocoEnv` | `env` |
| `warp` | MuJoCo Warp on a CUDA GPU: all worlds in one device allocation, tensors stay on the device | `cuda:N` | `WarpEnv` | `warp` |

Both build the *same* compiled MuJoCo model of the robot and run the same control law, resets and seeding. The CPU simulator is the reference: parity tests compare the GPU one against it.

```python
from robotdesigner.config import BaseEnvConfig
from robotdesigner.simulators import check_capabilities, get_simulator, list_simulators

assert list_simulators() == ("mujoco", "warp")
caps = get_simulator("warp").capabilities()                    # importing these checks needs neither torch nor MuJoCo
assert caps.devices == frozenset({"cuda"}) and "joint_torque" in caps.control_modes

cfg = BaseEnvConfig().control(mode="ee_pos_delta").sim(simulator="warp", device="cpu").resolve()
problems = check_capabilities(cfg, caps, "warp")
assert {p.path for p in problems} == {"control.mode", "sim.device"}
```

## Capabilities and validation

`make_env(cfg)` collects **every** reason the config cannot run and raises one `ConfigError` listing all of them, each with the config path to change. Nothing is approximated or
silently substituted. The sources of problems are:

1. the simulator's **capabilities**: control modes, devices, integrators, robot kinds (`check_capabilities`);
2. the simulator's own checks (`Simulator.validate`): `mujoco` checks that the scene exists; `warp` checks that `mujoco-warp` is installed and has the same release as `mujoco`, that CUDA
   and the device index are available, that the robot has no gripper, and that the scene exists;
3. the task provider's checks (`TaskProvider.validate`): the task runs on this simulator, the scene the task needs, the assets exist, and so on.

| | `mujoco` | `warp` |
|---|---|---|
| control modes | `joint_pos_abs`, `joint_pos_delta`, `joint_vel`, `joint_torque` | the same four |
| `ee_pos_delta`, `ee_pose_delta` | not implemented | not implemented |
| integrators | `euler`, `implicit`, `implicitfast`, `rk4` | the same |
| `TwoFingerGripper` | built; one extra action dimension | **rejected** (`robot.gripper`) |
| devices | `cpu` | `cuda:N` |

What the capabilities list is what the parity tests have shown to match, not what the engine could conceivably do.

```text
ConfigError: simulator 'warp' cannot run this config (2 problems):
  - control.mode: simulator 'warp' does not support control mode 'ee_pos_delta'; supported: [...]
  - sim.device: simulator 'warp' does not support device 'cpu'; supported: ['cuda']
```

## The environment

Every environment is a TorchRL `EnvBase` with `batch_size = [num_envs]` and `batch_locked = True`. Specs below are for `N = num_envs` and an arm with `n` joints.

**Action.** `action_spec` is `Bounded(-1, 1)` of shape `[N, n]` (`[N, n + 1]` with a gripper, the last component being the opening). How it becomes a command: [control](configs/control.md#modes).

**Observation** (nested; nothing is flattened):

| key | shape, dtype | content |
|---|---|---|
| `task/...` | provider's | the task's group, e.g. `ee_pos`, `<object>_pos` ([robosuite](tasks/robosuite.md)); empty for `empty/idle` |
| `proprio/qpos` | `[N, n]` f32 | joint positions (rad) |
| `proprio/qvel` | `[N, n]` f32 | joint velocities |
| `proprio/effort` | `[N, n]` f32 | actuator forces |
| `proprio/gripper` | `[N, 1]` f32 | opening (m), only with a gripper |
| `success` | `[N, 1]` bool | the task's success test on the current state |
| `episode_step` | `[N, 1]` i64 | steps since the env's last reset |

`env.observation_space` and `env.action_space` are plain **per-env shapes**: a nested dict of `torch.Size` mirroring the observation, and a `torch.Size`. They are not Gymnasium `Space`
objects; dtypes and bounds stay in `observation_spec` / `action_spec`, and the batched shape is `env.batch_size + env.action_space`.

**Step output** (under `("next", ...)` in a TorchRL step): the observation, `reward` `[N, 1]` f32, and `done`, `terminated`, `truncated` `[N, 1]` bool.
`truncated` is set once `episode_step >= max_episode_steps`; `terminated` is `success` if the task's params have `terminate_on_success = True`, else `False`; `done = terminated | truncated`.

**Reset.** The environment **never autoresets**. Reset only some rows with the `_reset` mask; unselected rows are untouched.

```python no-run
td = env.reset()                               # every row
td["action"] = env.action_spec.rand()          # [N, n] in [-1, 1]
td = env.step(td)                              # outputs under ("next", ...)

# driver-side autoreset: rows that are done are reset by the caller, never by the env
data, td = env.step_and_maybe_reset(td)

# reset rows 0 and 3 only, and inject a posture and a task value for them
td_reset = env.reset(TensorDict({"_reset": mask, "qpos": q0}, batch_size=env.batch_size))
traj = env.rollout(50)                         # TorchRL's rollout with a random policy; `traj.shape == [N, 50]`
```

A reset can inject `qpos` and `qvel` (`[N, n]`, read for the masked rows) and whatever the task documents (`injected(key)`). Without injection the robot goes to `home_qpos` plus uniform noise of
`reset_noise` (when the task's params have it), clipped to the joint ranges.

### Seeding

There are two seeded streams, both set from `sim.seed` and by `env.set_seed(seed)`:

- the **robot's** stream draws the reset noise (its shape depends on `n`);
- the **task's** stream (`rand` in `TaskImpl.reset`) is derived from the seed by a fixed offset and **does not depend on the robot**.

So with the same seed and `num_envs`, every embodiment sees the same task draws (the same cube positions, the same targets): that is what makes a comparison across
embodiments a comparison. The Warp simulator draws from the same CPU generators and moves the result to the device, so a seed gives the same draws on both simulators.
Physics on the GPU is deterministic per device and library release but not bitwise (the contact solver), so compare trajectories with tolerances.

### Attributes and methods

| | |
|---|---|
| `env.cfg`, `env.robot_hash`, `env.env_hash` | the resolved config and its identity |
| `env.interface` | `RobotInterface`: joint names and addresses, ranges, effort limits, `home_qpos`, mount, effective `kp` / `kd`, gripper fields |
| `env.task` | the engine side of the task |
| `env.q_target` | position targets `[N, n]`, meaningful in `joint_pos_delta` |
| `env.render(env_index=0, *, camera=None, size=(240, 320), overlays=True)` | one `uint8 (H, W, 3)` frame of the live state; see [viz](viz.md) |
| `env.close()` | frees the renderer (and the Warp allocation) |

## CPU MuJoCo specifics

- Worlds are stepped serially in the calling process: throughput scales with the speed of one core, not with `num_envs`. Use it for correctness, debugging and small studies; run several processes (see [cluster](cluster.md)), or use `warp`, for volume.
- Float64 physics; observations are converted to float32.
- Supports the gripper.

## GPU MuJoCo Warp specifics

- Needs the `warp` extra, an NVIDIA GPU, and `mujoco` and `mujoco-warp` of **one release** (otherwise `make_env` reports it).
- `sim.device` is `cuda:N`; tensors (observations, actions, rewards) live on that device and are float32.
- `WarpOptions` sets the per-world capacities and the CUDA graph ([sim](configs/sim.md#warpoptionsnjmax-nconmax-cuda_graph)). The environment checks the capacities after every step and raises
  `CapacityError` instead of silently clipping; `WarpBatchedSim.capacity_usage()` reports how much of an allocation the last step used. A scene with many contacts needs larger options.
- With the CUDA graph on, the physics sub-steps of one policy step are replayed as one launch (about 12 to 13 times faster per step than stepping kernel by kernel in the Lift sweep of `benchmarks/bench_warp.py`); resets are not captured.
- Rendering uses a host copy of one world's state, so it costs a device-to-host copy.
- The model must be one the parity tests cover (`check_model_supported` lists what is not).
- No gripper yet.

```python no-run
cfg = cfg.sim(simulator="warp", device="cuda:0", num_envs=4096)     # options become WarpOptions.default()
env = cfg.build()                                                   # env.device == cuda:0, env.batch_size == [4096]
family = cfg.sweep(paired={"sim.simulator": ["mujoco", "warp"], "sim.device": ["cpu", "cuda:0"]})   # one env_hash each
```

## What a task sees: `SimView`

Tasks never reach into a simulator. They get a `SimView` (`robotdesigner.tasks.base`): `num_envs`, `device`, `iface`, `model` (editable only in `bind`), `base_frame()`, `body_pose()`,
`site_pos()`, `write_qpos()` and `host_data()`. Both `BatchedSim` (CPU) and `WarpBatchedSim` implement it, which is why a provider lists the simulators it was tested on and nothing more.
How to write a task: [writing a provider](tasks/writing-a-provider.md).
