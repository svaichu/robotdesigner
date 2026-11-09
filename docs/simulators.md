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
| robot kinds | `parametric`, `external` | `parametric`, `external` |
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
- Runs an [`ExternalRobot`](configs/robot.md#externalrobot-a-model-that-lives-elsewhere), see below.

### External robots on the CPU simulator

The robot provider's `MjSpec` is attached to the scene and driven by **its own actuators** (`NativeGains`) or, with `FixedGains` / `InertiaScaledGains`, by ours in their place ([control](configs/control.md#our-gains-on-an-external-robot)); the same environment, tasks and `SimView` run on it.

- **Placement.** The compile adds an empty, welded body `base` at the mount (so `SimView`'s `"base"` frame works as for a `ParametricArm`) and attaches the model into a frame of the
  world body at the same pose, with the prefix `ext_` (`ext_j1`, `ext_a1`, `ext_ee`). The model's own world frame is therefore the base frame: a fixed arm stands at the mount, and a
  floating robot (a free root joint, which MuJoCo only allows on a direct child of the world) starts at the mount composed with `ExternalRobot.home_root`. New code reads the pose from
  `iface.mount`.
- **Joints and actuators.** `iface.joint_names`, `qpos_adr` and `qvel_adr` are the prefixed `robot.joints`, in order. Each driven joint has exactly one actuator with a joint
  transmission, and `action_dim == n`. The other actuators (a gripper, a coupled finger) are **held** at the `ctrl` of the model keyframe named by `robot.keyframe` (zero when `None`) on every step.
- **Reset.** `mj_resetData`, then the whole keyframe state (finger and passive joints get their model values), then the driven joints at `home_qpos` plus the reset noise (driven joints
  only, clipped to their ranges), then the root pose of a free joint. A masked reset leaves the other envs, roots included, untouched.
- **End effector.** `iface.ee_site_id` is the site named by `EndEffectorRef.site`, or a site `ee` added to `EndEffectorRef.body` at `offset`. It is `None` when the robot has no end effector.
- **Native control.** The action in `[-1, 1]` maps to the actuator's `ctrlrange`, not to the joint range: the two are separate fields of a model and need not be equal. For a position
  servo without a `ctrlrange` the joint range is used. `joint_pos_delta` accumulates a target clipped to the joint range, as for an arm; `joint_torque` scales by the ends of the actuator's
  `ctrlrange`. `iface.kp` and `kd` are read from the model for position servos (`gainprm[0]` and `-biasprm[2]`) and are `None` otherwise. (Measured on MuJoCo Menagerie: the Panda's first
  servo has gain 4500 and bias `(0, -4500, -450)`, the UR5e's shoulder servo gain 2000 and bias `(0, -2000, -400)`; for both, `ctrlrange` equals the joint range.)
- **`dt` and the integrator** stay the authority of `SimConfig`: the scene carries them and the model's own are not used. A `sim.dt` more than twice the model's own `timestep` is reported
  with a warning. The contact and solver options of the model (`cone`, `impratio`, solver, noslip) are the scene's: when the two differ, `MjSpec.attach` keeps the scene's value.
- **Spawn pose.** The mount is where the model's world frame goes (above), so it is the spawn pose: a fixed arm stands at the mount, and a floating robot starts at the mount composed with
  `home_root`. Tested on the UR5e at a yawed and shifted mount; on Go1 and Go2 at the default mount, at a mount rotated 90° about z and shifted in x and y (in the base frame the robot starts at its
  model's home root), and at a mount 0.3 m up (they start above the floor, fall and land, and never go through it); and on TidyBot, whose base is made of joints (two slides and a hinge) under position
  servos and which has no free joint: a constant `joint_pos_delta` action on its base x slide moves the base along the mount's x axis.
- **Option policy: the scene's options win.** `MjSpec.attach` keeps the scene's value where the model's own `<option>` differs (the legged robots: `cone` elliptic and `impratio` 100 against the scene's
  pyramidal cone and `impratio` 1). `external.CARRY_MODEL_OPTIONS` (`_model_options`) can carry them over; the default is not to. Measured with `benchmarks/menagerie_options.py` (2 s from the reset
  pose, `joint_pos_delta`, zero action):

  | robot | model's options carried | `sim.dt` | root drift | tilt | max penetration | mean contacts |
  |---|---|---|---|---|---|---|
  | Go1 | no | 0.002 (own) / 0.004 | -0.003 m | 0.26° | 0.014 m | 4.0 |
  | Go1 | yes | 0.002 (own) / 0.004 | -0.005 m | 0.18° | 0.015 m | 4.0 |
  | A1 | no | 0.002 (own) / 0.004 | -0.019 m | 0.43° | 0.010 m | 4.2 |
  | A1 | yes | 0.002 (own) / 0.004 | -0.019 m | 0.23° | 0.012 m | 7.9 |

  Both stand, with the same drift and under a degree of tilt; carrying the options gives a little less tilt, and doubling `dt` changes nothing visible. Carrying them is not worth what it costs: the solver
  options are then different for every robot, so the same task (the cube of `robosuite/Lift`, a table) would be simulated with a different friction cone for each embodiment, which is what a comparison of
  embodiments must not do. The scene's options stay, `sim.dt` and the integrator stay `SimConfig`'s, and no field was added to `ExternalRobot`. The test pins the default (Go1 drifts less than 1 cm and A1 less than
  3 cm, both tilt under 1°). MuJoCo's "Attach conflict" warning about the dropped options is silenced in the compile, because it is the policy.
- **Frame limitation.** `SimView(frame="base")` is the static mount frame, **not** the moving trunk of a floating robot. A `"root"` frame does not exist yet.
- **Problems, not silent changes.** `make_env` reports, with the config path: a model with no actuators; driven actuators of another kind than `robot.actuator_kind`; a stale `robot.source`
  (the provider's check: "the model changed since this config was written"); an unknown joint, entry or keyframe; a free root without `home_root`, or `home_root` without one.
- **Kinematics.** `robotdesigner.simulators.mujoco.external.MujocoKinematics` is the host-side oracle that a MuJoCo-based robot provider returns from `RobotProvider.kinematics`
  (`forward_ee_batch(q, base)` by `mj_kinematics`, `sample_joint_positions`), so `OwnReachable` targets and the camera framing of [viz](viz.md) work for external robots.
## GPU MuJoCo Warp specifics

- Needs the `warp` extra, an NVIDIA GPU, and `mujoco` and `mujoco-warp` of **one release** (otherwise `make_env` reports it).
- `sim.device` is `cuda:N`; tensors (observations, actions, rewards) live on that device and are float32.
- `WarpOptions` sets the per-world capacities and the CUDA graph ([sim](configs/sim.md#warpoptionsnjmax-nconmax-cuda_graph)). The environment checks the capacities after every step and raises
  `CapacityError` instead of silently clipping; `WarpBatchedSim.capacity_usage()` reports how much of an allocation the last step used. A scene with many contacts needs larger options.
- With the CUDA graph on, the physics sub-steps of one policy step are replayed as one launch (about 12 to 13 times faster per step than stepping kernel by kernel in the Lift sweep of `benchmarks/bench_warp.py`); resets are not captured.
- Rendering uses a host copy of one world's state, so it costs a device-to-host copy.
- The model must be one the parity tests cover (`check_model_supported` lists what is not).
- No gripper yet.

### External robots on Warp

An [`ExternalRobot`](configs/robot.md#externalrobot-a-model-that-lives-elsewhere) runs on Warp with the same compile as on the CPU simulator ([above](#external-robots-on-the-cpu-simulator)). The Warp
reset writes the same state as the CPU reset: the model keyframe, the driven joints at `home_qpos` plus noise, the composed root pose of a free joint and the held `ctrl` of the actuators that are not driven,
each per world and masked. `validate` runs `check_model_supported` on the compiled robot model and reports what the installed MuJoCo Warp does not run at the path `robot`; the provider's own checks
(a stale `source`) are reported too. The action mapping is the CPU simulator's (the actuator's `ctrlrange`).

**Parity** against the CPU simulator, Warp's CPU device, 4 envs, 30 policy steps of random actions of 0.05 of the action range, from the home state (largest absolute error; tolerances `qpos` 1e-4, `qvel` 1e-3,
effort 1e-2, as in `tests/test_warp_parity.py`). Measured with warp-lang 1.18.0 and mujoco-warp 3.17.0. **These numbers are from Warp's CPU device; the CUDA path is not verified for external robots**
(the `gpu`-marked copies of the tests are written and skip without a device).

| robot | gains, mode | qpos | qvel | effort | within |
|---|---|---|---|---|---|
| Panda | ours (kp 60, kd 6), `joint_pos_abs` | 4.8e-7 | 9.2e-6 | 7.0e-5 | yes |
| Panda | ours, `joint_vel` | 6.0e-7 | 1.7e-5 | 5.1e-5 | yes |
| Panda | ours, `joint_torque` | 2.9e-6 | 1.4e-5 | 0 | yes |
| Panda | native, `joint_pos_delta` | 6.7e-4 | 1.2e-1 | 13 | **no** |
| Panda | native, `joint_pos_abs` | 3.9e-2 | 2.3 | 1.7e2 | **no** |
| UR5e | native, `joint_pos_delta` | 6.0e-7 | 2.0e-5 | 3.3e-3 | yes |
| UR5e | ours, `joint_vel` | 1.8e-6 | 3.9e-5 | 2.4e-4 | yes |
| UR5e | ours, `joint_pos_abs` | 9.0e-3 | 2.6e-1 | 1.8 | **no** |
| UR5e | ours, `joint_torque` | 1.6e-4 | 1.4e-3 | 0 | **no** (slightly) |
| UR5e | native, `joint_pos_abs` | 1.5e-2 | 1.9 | 3.0e2 | **no** |
| Go2 | native (`joint_torque`, plain motors) | 3.1e-6 | 4.8e-5 | 2.4e-7 | yes |
| Go2 | ours, `joint_pos_abs` | 9.5e-7 | 1.6e-5 | 1.5e-4 | yes |
| Go2 | ours, `joint_vel` | 2.9e-6 | 6.1e-5 | 4.0e-4 | yes |

Where it is outside the tolerances, the two simulators agree until something amplifies float32 rounding, which is a property of the case, not a unit mismatch:

- **Stiff native servos (Panda).** Its native gains are kp 4500 and kd 450. With no contact at all, the error grows from 2e-7 at the first step to 9e-3 by step 6. The same model with kp 60 agrees to 5e-7.
- **Mesh contacts (UR5e).** With our gains the trajectory agrees to 5e-7 until the arm touches the floor or itself (step 15), and then differs by 1e-3 to 5e-3; the contact sets differ (for example 2 contacts on the CPU, 1 on Warp).
  `joint_pos_abs` of a native robot also starts by moving to the middle of the range, which causes such contacts.
- **A body thrown onto the floor.** A free root in free fall agrees (tested), but once a box and two capsules land and tumble the two simulators diverge: a chaotic contact case.
- **An undamped servo arm.** A toy arm with kp 100, kd 0 and no armature differs by 3e-2 in 20 steps; adding armature and damping brings it to 3e-7. Parity needs a model that is not marginally stable.

**Capacities** (`WarpOptions`; the default is `njmax=128`, `nconmax=32` per world; the numbers are the most constraint rows / the most contacts in one world, native control, 8 envs, plain floor):

| robot | at home | random actions, 0.5 of the range | random actions, full range |
|---|---|---|---|
| Panda | 3 / 0 | 53 / 13 | 419 / 104 |
| UR5e | 20 / 5 | 28 / 7 | 36 / 9 |
| Go2 | 92 / 14 | 72 / 11 | 108 / 20 |

The defaults fit the UR5e and the Go2, and the Panda unless it is driven through its whole range, where its self-contacts need `njmax` of about 512 and `nconmax` of about 128. `CapacityError` says which option to raise
(`sim.options.njmax` for constraint rows, `sim.options.nconmax` for contacts). `WarpOptions.default()` is unchanged.

```python no-run
cfg = cfg.sim(simulator="warp", device="cuda:0", num_envs=4096)     # options become WarpOptions.default()
env = cfg.build()                                                   # env.device == cuda:0, env.batch_size == [4096]
family = cfg.sweep(paired={"sim.simulator": ["mujoco", "warp"], "sim.device": ["cpu", "cuda:0"]})   # one env_hash each
```

## What a task sees: `SimView`

Tasks never reach into a simulator. They get a `SimView` (`robotdesigner.tasks.base`): `num_envs`, `device`, `iface`, `model` (editable only in `bind`), `base_frame()`, `body_pose()`,
`site_pos()`, `write_qpos()` and `host_data()`. Both `BatchedSim` (CPU) and `WarpBatchedSim` implement it, which is why a provider lists the simulators it was tested on and nothing more.
How to write a task: [writing a provider](tasks/writing-a-provider.md).
