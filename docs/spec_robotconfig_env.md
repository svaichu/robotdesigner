# RobotConfig, EnvConfig and backends: design spec (v4, streamlined)

> **Status: draft for discussion. Nothing is implemented.**
> Branch `feat/quotient`. Written 2026-10-27 and streamlined the same day. Companion to `docs/quotient_space.md` (untouched). Working notes: `.agent/memory.md`.
> Derived from requirements and library documentation only. The existing package code, dependencies, lockfile, CI and examples are placeholders (user, 2026-10-27): no default value, name, layout or test in this spec is inherited from them.

## 1. Purpose and scope

"Quotient" is the downstream research project that studies relationships between embodiments performing the **same control task**. `robotdesigner` provides:

1. an embodiment described as validated, hashable data (`RobotConfig`);
2. batched RL environments that instantiate it on several simulation backends;
3. task descriptions that do not depend on the embodiment;
4. traceability: every run maps to a configuration hash.

From `docs/quotient_space.md`: an n-DoF arm in an RL environment with `n` as a parameter, and the robot configuration kept separate from the env.

**Non-goals (v1):** training code or trainer-specific glue; JAX / MJX and Playground backends (shelved); multi-robot, mobile or legged robots; autodiff through the simulator; non-MuJoCo simulators; a Gymnasium-native contract (an adapter may follow).

## 2. Principles

1. **The robot is data.** `RobotConfig` is frozen, validated, hashable and serializable, and imports no engine.
2. **Backends realize configs; envs consume them.** Dependencies point from backends to the config layer, never back.
3. **One batched env contract:** every backend returns a TorchRL `EnvBase` over TensorDicts, with `batch_size = [num_envs]` and a fixed `device`.
4. **The task is embodiment-independent and recorded.**
5. **Everything that changes behaviour is in `EnvConfig` and its hash.** No hidden defaults.

## 3. Architecture and packaging

```
robotdesigner.config               numpy only: configs, generators, validation, hashing, kinematics oracle
robotdesigner.envs                 registration (namespaced ids), shared transforms
robotdesigner.backends.mujoco      CPU reference
robotdesigner.backends.warp        GPU training
robotdesigner.backends.robosuite   CPU, library tasks
```

- `config` imports no engine, no torch and no robosuite. Backends are imported lazily by name (`sim.backend`). Each backend holds its own env class.
- `config.kinematics` is a pure-numpy forward-kinematics implementation of `RobotConfig`. It serves two purposes: the reachable-workspace estimator, and an engine-independent oracle against which every compiled model is tested.

**Packaging** (versions read from PyPI on 2026-10-27):

```
base        numpy>=1.26                                     # robotdesigner.config
[env]       torch>=2.7, tensordict>=0.14.3,<0.15, torchrl>=0.14.1,<0.15, mujoco>=3.13
[warp]      [env] + mujoco-warp, warp-lang>=1.15            # NVIDIA GPU; lock mujoco and mujoco-warp to the same release
[robosuite] [env] + robosuite>=1.5,<1.6
[viz]       imageio
[dev]       pytest, ruff
```

- `requires-python = ">=3.10"`: every key dependency declares it, and torch, torchrl and mujoco publish Linux x86_64 wheels for CPython 3.10 to 3.14. CI covers 3.10, 3.12 and 3.13; development default 3.12.
- torchrl 0.14.1 pins tensordict below 0.15, and tensordict 0.15.3 needs torch 2.11 or newer, so the 0.14 line is the one that serves torch 2.7 to 2.8 environments. Widen the pins only after CI passes on a newer line.
- numpy 1.26 is the floor because older downstream environments pin it on Python 3.10; mujoco 3.13 is the floor because mujoco-warp requires it. CUDA builds of torch are chosen by the consuming environment.
- uv with a committed lockfile, src layout.

## 4. Configuration objects

```
EnvConfig(robot, control, task, sim, backend_options)
```

### 4.1 RobotConfig

`RobotConfig` is one of two kinds. SI units throughout.

**`ParametricArm`** (the main kind):

```
schema_version
segments: tuple[Segment]      # kinematic chain order; n = len(segments)
base_pose                     # fixed base
home_qpos                     # length n
end_effector: Pose            # offset from the last link tip
tool: None | ToolSpec(gripper, mount_pos, mount_quat)

Segment
  joint: axis (unit 3-vector in the parent frame), range (lo, hi) | None,
         damping, armature, frictionloss, effort_limit            # hinge joints in v1
  link:  joint_frame (pose in the previous link's end frame), tip_offset (next frame in the link frame),
         mass, geometry (capsule(radius) in v1), inertia (derived from geometry and mass, or explicit)
```

- **Explicit everything.** Every dynamic parameter is required. A backend or library must never fill in a value on its own; robosuite, for example, does fill missing joint attributes with its own defaults.
- **Canonical names:** `joint_{i}`, `link_{i}`, `base`, `ee`; backends map them to their own naming.
- **Validation:** `n >= 1`; axes non-zero (normalized on construction); `lo < hi`; masses, radii and effort limits positive; `home_qpos` inside the ranges; unknown keys rejected; supported `schema_version`.
- **Canonical form and hash:** JSON with sorted keys, floats rounded to 12 significant digits, unit axes and quaternions with `w >= 0`, `-0.0` mapped to `0.0`. `config_hash` is the SHA-256 of that JSON; the first 12 hex characters are used for display and generated names. Labels are metadata and excluded.
- **Generators** produce `ParametricArm` objects; `n` is a generator argument, not a field:

  ```
  chain(n, axes, link_length=... | total_length=..., mass=... | total_mass=...,
        radius, damping, armature, frictionloss, effort_limit, range)
  ```

  `axes` is explicit (a sequence of length `n`, or a pattern the caller expands). Exactly one of each length and mass pair is given, so a study can hold total length or total mass fixed while `n` changes.

**`ExternalRobot`:** a reference to an existing model: an MJCF file plus an interface descriptor (end-effector site, joint names, actuator names, home pose), or a robosuite stock robot by name. Hashed by content or name and version. In v1 it exists for baselines (the stock robosuite Panda); a backend may refuse it.

### 4.2 ControlConfig

```
mode:  joint_pos_abs | joint_pos_delta | joint_torque | joint_vel | ee_pos_delta | ee_pose_delta
gains: fixed(kp, kd) | inertia_scaled(bandwidth, damping_ratio)
action scaling
```

- Hardware limits live in `RobotConfig` (`effort_limit`); the control interface lives here.
- `inertia_scaled`: `kp_i = w^2 * M_ii`, `kd_i = 2 * zeta * sqrt(kp_i * M_ii)`, with `M` taken at `home_qpos` (an approximation, since the mass matrix changes with configuration). Any damping already in the joint is subtracted and clipped at zero.
- Native backends run position and torque control in the simulator's actuators; delta and velocity modes are implemented in the env layer; end-effector modes are out of scope for native backends in v1. robosuite offers joint delta position, velocity, torque and OSC position / pose controllers.
- Every backend declares its supported modes; an unsupported mode is rejected at validation, never approximated.

### 4.3 TaskConfig

```
id: namespaced (native/reach, robosuite/Lift, ...)
params, horizon, success_tolerance
target_region: fixed_box(low, high) | own_reachable(margin) | family_reachable(family, margin)
```

- A task never mentions `n` or link lengths. Targets are expressed in the robot base frame.
- **Declared versus resolved.** `EnvConfig.resolve()` turns region policies into concrete numbers. Workspace estimation uses `config.kinematics`: sample joint positions inside the ranges (±pi for unlimited joints), compute end-effector positions, and for a family intersect the reachable sets and shrink by `margin`, so every member can in principle reach every target. The resolved config is what is hashed and logged.
- **`native/reach`:** the end effector must reach a target; dense reward `-||ee - target||`; success within `success_tolerance`; `truncated` at `horizon`; `terminate_on_success` is a parameter, off by default.

### 4.4 SimConfig and backend options

```
SimConfig:       backend, num_envs, device, dt, integrator, control_freq, seed, render, scene
backend_options: one typed class per backend, part of the hash
```

- `dt` and `integrator` are explicit for native backends; their defaults are chosen at milestone 1 after parity tests and recorded here. `control_freq` must be a multiple of `1/dt`. `scene` selects a preset; the robot spec never contains scene elements.
- Options: `MujocoOptions` (viewer, render); `WarpOptions` (contact and constraint capacities, CUDA graph capture); `RobosuiteOptions` (arena, cameras, reset behaviour).

### 4.5 Resolved config, hash and RobotInterface

`env_hash` is the SHA-256 of the canonical JSON of the resolved config (robot hash, control, resolved task, sim, backend, options). Envs expose `robot_hash` and `env_hash` as attributes.

`RobotInterface` is derived, not configured: `n`, joint and actuator indices and names, end-effector frame, ranges, effort limits, home pose, base frame. Backends hand it to tasks and wrappers.

## 5. Backends

A backend implements `capabilities()` (control modes, task namespaces, devices, robot kinds), `validate(cfg)` (list of problems, before anything is built) and `make_env(cfg)`.

| | `mujoco` | `warp` | `robosuite` |
|---|---|---|---|
| Role | reference, debugging | training | library tasks |
| Device | CPU | CUDA | CPU |
| Robot | ours | ours | ours (generated) or stock |
| Scene, task | ours | ours | library |
| Control | ours (actuators + env layer) | ours (actuators + env layer) | library controllers, configured from `ControlConfig` |
| Stepping | N `MjData` stepped in a loop | MuJoCo Warp, `nworld = num_envs` | one robosuite env per sub-env |

- **`mujoco`:** `ParametricArm` compiles to a robot-only `MjSpec`, is attached to a scene spec and compiled to one `MjModel`. The batched interface is identical to `warp`'s. It is the development backend and the parity oracle.
- **`warp`:** the same `MjModel` goes through `put_model` and `make_data(nworld, ...)`; state is exposed as torch tensors. Per-world model fields (`put_model(batch_sizes=...)`) may let same-topology embodiments with different lengths or masses share one sim; different `n` needs a separate sim per topology group. Masked reset should map to `reset_data` with a mask. Limits: NVIDIA GPU, no autodiff.
- **`robosuite`:** robosuite supplies the task, scene, controllers and observations; we supply the robot (Appendix A). Pin `robosuite>=1.5,<1.6`.

## 6. Env contract (TorchRL)

```
env.batch_size = [num_envs]       # batch-locked: the simulators have a fixed size
env.device
env.reset(td=None) -> td          # td["_reset"], bool [num_envs, 1], selects envs; absent = reset all
env.step(td)       -> td          # reads td["action"]; outputs live under ("next", ...)
env.step_and_maybe_reset(td)      # driver-side autoreset: reads the done keys, writes "_reset"
env.rollout(max_steps, policy)    # TorchRL's own; this package ships no rollout helper
env.set_seed(seed)
env.robot_hash, env.env_hash
```

Subclasses implement `_reset`, `_step` and `_set_seed` and declare the specs; they never override `reset` or `step`. Shapes include the batch dimension:

```
observation_spec = Composite(task = Composite(...), proprio = Composite(qpos, qvel, [effort]))
action_spec      = Bounded(-1, 1, shape=(num_envs, action_dim))     # float32 on `device`
reward_spec      = Unbounded(shape=(num_envs, 1))
done_spec        = done, terminated, truncated: Binary(shape=(num_envs, 1))
```

- **Observation keys are declared per backend.** `qpos` and `qvel` are required in `proprio`; `effort` (applied joint torque) and task-specific keys are optional; the `task` group depends on the task. Analyses that must run everywhere use the required subset.
- **No autoreset inside the env.** The driver (`step_and_maybe_reset`, `rollout`, a collector) reads the done keys and writes `_reset`.
- **Partial reset** honours the `_reset` mask. TorchRL does not verify that an env implements it, so each backend needs its own test.
- **Injected state:** optional entries in the reset TensorDict (`target`, `qpos`, `qvel`) override random initialization, for parity tests and for replaying a world on another backend.
- **Seeding:** `set_seed` seeds a torch generator on `device`. Same backend, device and seed give the same results; random resets differ across backends, so parity uses injected state.
- **Views:** nested by default; a transform concatenates chosen keys, in a declared order, into a flat `observation`; an optional transform pads to a maximum `n` and adds a mask.
- **Heterogeneous families:** envs with different `n` have different specs. TorchRL can combine such envs with lazy stacking (for example `AsyncEnvPool(..., stack="lazy")`); to be verified. Padding stays optional.
- **Conformance:** `check_env_specs(env)` must pass for every backend. TorchRL renamed its spec classes between releases, so `torchrl` and `tensordict` are pinned and CI runs the check.

## 7. Verification strategy

- **Config:** validation errors; hash stability (key order, `-0.0`, scaled axes); generator invariants; `config` imports without torch or any engine.
- **Kinematics oracle:** for random configurations and joint positions, the end-effector pose of every compiled model matches `config.kinematics`.
- **Env:** `check_env_specs`; masked-reset test; seeding determinism; tensors on the declared device.
- **Parity:** CPU backend against `warp` from injected initial state, same action sequence, with tolerances (Warp state appears to be float32, CPU MuJoCo is float64; short horizons).
- **Debug tiers:** CPU backend first; then move one world between device and host (`get_data_into(..., world_id)`, `put_data`); then CPU-versus-Warp runs. GPU tests carry a marker and skip on CPU-only runners; robosuite tests are optional.

## 8. Knowledge status

Read from library source or documentation, nothing executed: robosuite's robot model, registry, controller and gripper handling and the `Lift` task; TorchRL's `EnvBase` conventions; the `mujoco_warp` API surface and docstrings; PyPI metadata.

Unchecked, to settle during implementation:

- a generated robot built through `suite.make`;
- Warp masked reset and per-world `batch_sizes` fields;
- the torch to Warp zero-copy helper in warp-lang 1.18, and Warp on a CPU device;
- CPU versus Warp numeric parity;
- `_reset` handling and `check_env_specs` on our envs, and lazy-stacked heterogeneous families;
- torchrl 0.14.1 and tensordict 0.14.3 with torch 2.7 and 2.14;
- the cost of the workspace estimator.

## 9. Decisions

**Adopted:**

1. Segment-based `ParametricArm` with generators; `ExternalRobot` only for baselines.
2. Control mode lives in `ControlConfig`, not in the robot.
3. `config` is numpy-only; the env layer (CPU backend included) requires torch, tensordict and torchrl.
4. The env contract is TorchRL `EnvBase`; the env never autoresets; nested observations are primary.
5. GPU backend: MuJoCo Warp driven from torch. JAX is shelved. CPU MuJoCo is the reference and debugging backend.
6. Frozen dataclasses with hand-written validation; packaging as in section 3.
7. First robosuite target: `Lift`, with the stock Panda as a baseline before generated arms.

**Open:**

- `Lift` details (Appendix A): base placement and mount for generic arms, which gripper, reward default (sparse or shaped), and the smallest `n` that can physically solve it.
- Family batching on the GPU: separate sim per topology group, lazy stacking, or padding.
- Whether `.agent/` is ignored or committed.

## 10. Milestones

1. **Config layer and CPU backend.** Validation and hash tests pass; the kinematics oracle matches compiled models for several `n`, axis patterns and parameter sets; `config` imports without torch or engines; `dt` and `integrator` defaults chosen and recorded.
2. **Native Reach on CPU (TorchRL).** `check_env_specs` passes; masked `_reset` works; seeding is deterministic; injected state works; registration under a namespaced id.
3. **`warp` backend.** Parity with the CPU backend from injected state; masked reset; throughput measured and reported.
4. **robosuite `Lift`.** The stock Panda runs through our contract first; then a generated `chain(n)` builds and steps `robosuite/Lift`, `reset` randomizes the cube, the action dimension is `n + 1` and the success flag works.
5. **Later:** Playground and JAX backends.

## Appendix A. `robosuite/Lift`

Read from the robosuite 1.5.1 source; nothing has been run.

**The task.** One single-arm robot at a 0.8 x 0.8 x 0.05 m table whose top is at z = 0.8 m. A small cube (half-size 0.020 to 0.022 m) is placed within 3 cm of the table center. Success when the cube's height exceeds the table height by more than 0.04 m. Reward is sparse (2.25, normalized to `reward_scale`) or shaped (reaching term `1 - tanh(10 d)` plus 0.25 for a grasp). Defaults: 20 Hz control, 1000-step horizon; `done` is horizon-only; `step` returns `(obs, reward, done, info)`.

**Configuration sketch.**

```
robot:   chain(n, axes=..., total_length=<Panda-scale>, ...), home_qpos with the flange pointing down,
         tool = ToolSpec("PandaGripper", mount_pos, mount_quat)
control: joint_pos_delta (+ gripper)
task:    id="robosuite/Lift", horizon=1000, params={reward_shaping, reward_scale, terminate_on_success=False}
sim:     backend="robosuite", num_envs=N, device="cpu", control_freq=20, seed
backend_options: RobosuiteOptions(table_full_size, table_friction,
                                  use_camera_obs=False, has_offscreen_renderer=False, hard_reset=False)
```

**Mapping.**

- **Robot:** a generated `ManipulatorModel` subclass (name from `robot_hash`, `arms = ["right"]`, `init_qpos` from `home_qpos`, `arm_type = "single"`, base-placement properties), registered inside the env constructor with `register_robot_class("FixedBaseRobot")` because robosuite's registries are process-global. Its XML has a `base` root body, a `right_hand` end-effector body, explicit joint attributes and torque motors with `ctrlrange` from `effort_limit`. `Lift` calls the robot's `base_xpos_offset["table"]` rule with the table length; the model must also define the `bins` and `empty` entries. For a 0.8 m table the stock Panda rule puts its base 0.56 m from the table center; our rule derives the offset from the base geometry and a clearance. Open: stock mount or none, and the base height.
- **Gripper:** the library mounts it on the `right_hand` body. Grasping needs the flange to point down at the home pose, so `home_qpos` and `tool.mount_quat` matter. Without a gripper the robot builds (a null gripper is used), but `Lift`'s reward and grasp check use the gripper.
- **Control:** a composite controller dict with the arm `JOINT_POSITION` controller (delta) and the `GRIP` gripper controller. Action layout `[arm (n), gripper (1)]`, so `action_spec` has `(num_envs, n + 1)`.
- **Observations:** `proprio` from joint position and velocity (robosuite does not provide applied torque, so `effort` is absent here); end-effector pose and gripper state as extra keys; `task` from the cube pose and the gripper-to-cube vector.
- **Reward and done:** reward becomes `[N, 1]`; `truncated` is robosuite's `done`; `terminated` is False unless `terminate_on_success`; a `success` entry comes from the task's success check.
- **Seeding and reset:** the seed is a constructor argument, so each sub-env gets `seed + index`. robosuite's default `hard_reset=True` rebuilds the model at every reset; we set it to False (semantics to verify).
- **Physics:** robosuite owns the timestep and integrator, so the backend must reject `sim.dt` and `integrator` overrides (to verify where robosuite sets them).
- **Camera defaults:** camera observations and the offscreen renderer default to on; state-only runs switch both off, otherwise a GL context is required on a headless node.

**Constraints on the embodiment family.** Scale: the task layout is built around Panda-sized arms, so `chain` uses `total_length`. Degrees of freedom: whether a 3-DoF arm can grasp a 4 cm cube is unknown and is itself a study result; physical infeasibility must be separable from failures of learning. Gains: robosuite's default joint-position gains were tuned for Panda inertias. Compute: robosuite's `Lift` runs on CPU MuJoCo. A GPU-parallel version is possible in principle as a separate native task (`native/lift`). MuJoCo Warp consumes an ordinary `MjModel`, so the scene (table, gripper, cube) can be composed from MJCF assets: robosuite ships its table arena and gripper models as plain XML files, and the cube is just a box. What must be re-implemented in torch is the logic robosuite runs in Python: cube placement, reward and success (the shaped reward needs a contact-based grasp check; the sparse reward only needs the cube height), observations, and the controller loop (robosuite repeats `step1`, controller torques, `step2` for every physics substep of a policy step). It is outside v1 because of that effort and because `native/lift` results are comparable with `robosuite/Lift` only after a parity check; the contact behaviour of the gripper models on Warp is unverified.
