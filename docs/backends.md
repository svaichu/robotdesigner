# Backends and the environment contract

A **backend** turns an [`EnvConfig`](configs/env.md) into a batched TorchRL environment. The config says what to run;
the backend decides whether it can, and builds it.

```python no-run
from robotdesigner.envs import make_env
env = make_env(cfg)        # cfg.sim.backend picks the backend
```

| backend | what it runs | needs |
|---|---|---|
| `mujoco` | `native/reach` on a robot-only MuJoCo model compiled from the config. The reference and debugging backend | `[env]` extra |
| `robosuite` | any single-arm robosuite environment with the robot built from the config | `[robosuite]` extra |
| `warp` | planned: the same models on the GPU. Declared as an extra only, not implemented | |

## Protocol and registry

```python no-run
class Backend(Protocol):
    name: str
    def capabilities(self) -> Capabilities: ...   # what it can run
    def validate(self, cfg) -> list[Problem]: ... # engine-specific checks, before anything is built
    def make_env(self, cfg): ...                  # a torchrl EnvBase
```

- `robotdesigner.backends` is a registry: `register_backend(name, target)`, `get_backend(name)`, `list_backends()`.
  `target` is a backend instance or a lazy `"module:attr"` string, resolved on first use, so importing
  `robotdesigner.backends` never imports MuJoCo, torch or robosuite. An unknown name raises `ConfigError` with path
  `sim.backend`.
- `Problem(path, message)` names the config field at fault, like `ConfigError.path`.

## Capabilities

`Capabilities` lists what a backend supports: `control_modes`, `task_ids`, `devices`, `robot_kinds`, `integrators`. A task
id entry ending in `*` matches by prefix (`"robosuite/*"`); a device entry `"cuda"` matches any `cuda:N`.

`make_env(cfg)` first compares the config with the capabilities (control mode, task, integrator, robot kind, device), then
calls `backend.validate(cfg)`, and raises one `ConfigError` that lists **every** problem. Nothing is built, approximated
or silently changed.

```text
backend 'robosuite' cannot run this config (5 problems):
  - sim.dt: robosuite fixes the physics timestep at 0.002, got 0.001
  - sim.scene: must be 'env': the robosuite environment builds its own scene
  - control.gains: robosuite's joint controller scales its gains by the joint inertia; use InertiaScaledGains (bandwidth = sqrt(kp), damping_ratio)
  - robot.tool: robosuite environments need a gripper; give the arm a ToolSpec
  - task.id: 'TwoArmLift' needs more than one arm; single-arm environments: ['Door', 'Lift', ...]
```

| | `mujoco` | `robosuite` |
|---|---|---|
| control modes | `joint_pos_abs`, `joint_pos_delta`, `joint_torque`, `joint_vel` | `joint_pos_delta` |
| gains | `FixedGains`, `InertiaScaledGains` | `InertiaScaledGains` |
| tasks | `native/reach` | `robosuite/<EnvName>`, single-arm manipulation environments |
| device | `cpu` | `cpu` |
| integrators | `euler`, `implicit`, `implicitfast`, `rk4` | `euler` |
| time step `dt` | any | `0.002` |
| scenes | `empty`, `plain_floor` | `env` |
| robot kinds | `parametric` | `parametric` |
| `robot.tool` | ignored | required: the gripper |
| backend options | `MujocoOptions` | `RobosuiteOptions` |

## The environment contract

Every backend returns a TorchRL `EnvBase` with these properties.

- `batch_size = [num_envs]`, batch-locked, on the CPU for the current backends.
- **No autoreset.** `step` never resets an environment. Reset chosen rows with a boolean mask `td["_reset"]` of shape
  `[num_envs, 1]` (absent: reset all), or use `env.step_and_maybe_reset(td)` to let the driver do it from the done keys.
- **Step** reads `td["action"]`, shape `[num_envs, action_dim]` in `[-1, 1]`, and returns the outputs under `("next", ...)`.
- **Done keys:** `done`, `terminated`, `truncated`, each `[num_envs, 1]` bool. `truncated` is the time limit (`horizon`).
- **Reward:** `[num_envs, 1]` float32.
- **Always present** in the observation: `success` `[num_envs, 1]` bool and `episode_step` `[num_envs, 1]` int64, a
  `task` group and a `proprio` group with at least `qpos` and `qvel`, each `[num_envs, n]`.
- **Injected state:** the reset TensorDict may carry `qpos` and `qvel` (and `target` on `native/reach`), which override
  the random initialization. This is how two backends are compared from the same start, or an episode is replayed.
- **Seeding:** `env.set_seed(seed)`. The same backend, device and seed give the same episodes.
- **Attributes:** `env.cfg` (the resolved config), `env.robot_hash`, `env.env_hash`.
- `check_env_specs(env)` passes for every backend (the tests run it).

### `native/reach` on `mujoco`

| key | shape | meaning |
|---|---|---|
| `task/ee_pos` | `[N, 3]` | end effector, in the robot's base frame |
| `task/target_pos` | `[N, 3]` | target, in the base frame |
| `proprio/qpos`, `proprio/qvel` | `[N, n]` | joint position and velocity |
| `proprio/effort` | `[N, n]` | actuator force (N·m) at the last physics sub-step of the step |
| action | `[N, n]` | one value per joint; the meaning is the control mode (see [ControlConfig](configs/control.md#modes)) |

Extra attributes: `env.interface` (a `RobotInterface`, below) and `env.q_target` (the position targets in
`joint_pos_delta` mode). `env.render(...)` returns an RGB frame of the live state; see [viz](viz.md).

### `robosuite/<EnvName>` on `robosuite`

One robosuite environment per batch element, stepped one after another in this process.

| key | shape | meaning |
|---|---|---|
| `task/<name>` | `[N, d]` | every robosuite observation that does not belong to the robot (`cube_pos`, `handle_pos`, ...); the names and sizes are the environment's own |
| `proprio/qpos`, `proprio/qvel` | `[N, n]` | joint position and velocity |
| `proprio/<name>` | `[N, d]` | the robot's other observations with the `robot0_` prefix removed (`eef_pos`, `gripper_qpos`, ...). There is no `effort` |
| action | `[N, n + g]` | `n` joint increments, then the gripper's `g` dimensions (`g = 1` for the Panda gripper, `0` for the wiping tool) |

The aggregate `*-state` vectors robosuite builds are dropped as redundant; scalar observations become length-1 vectors.
The observation keys are read from the environment at construction, so they differ between tasks; an environment whose
keys change between episodes is refused.

Behaviour worth knowing:

- **Control.** `joint_pos_delta` keeps an accumulated target, `q_target += action * action_scale`, exactly as the native
  backend does, and hands it to robosuite's joint controller as an absolute target. The controller adds gravity
  compensation and re-evaluates the inertia every step, so the dynamics are close to, not identical with, the native
  servo. See [ControlConfig](configs/control.md#inertiascaledgainsbandwidth-damping_ratio).
- **Reset.** robosuite's `hard_reset` is off: a reset re-randomizes the task without rebuilding the simulation. robosuite's
  own initial-state noise is zeroed; the robot starts at `home_qpos` plus `reset_noise` from our generator, or at the
  injected `qpos`.
- **Randomness.** Each sub-environment `i` has its own random stream, seeded with `seed + i`, so an environment's
  episodes do not depend on resets of the others.
- **No rendering yet.** This backend turns camera observations and the renderers off; `env.render()` and
  `robotdesigner.viz` work with the native backend only (see [viz](viz.md#adding-rendering-to-another-backend)).
- **Library defaults.** `env.resolved_env_kwargs` is the complete argument set the environment ran with. `env.env_name`,
  `env.robosuite_version`, `env.robosuite_envs` (the underlying environments) and `env.q_target` are also available.
- **Versions.** robosuite 1.5.x as released fails on MuJoCo 3.10 and later. `robotdesigner.backends.robosuite.apply_compat()`
  patches the two affected code paths in the running process, applies only what the installed MuJoCo needs, and is called
  for you. Call it yourself in worker processes started with `spawn`.

## `RobotInterface`

What tasks and wrappers need to know about a compiled robot, without engine-specific names (native backend):

| field | meaning |
|---|---|
| `n`, `joint_names` | number of joints and their names |
| `qpos_adr`, `qvel_adr`, `actuator_ids` | indices into the simulation arrays, `(n,)` |
| `ee_site_id` | the end-effector site |
| `joint_range` | `(n, 2)`, `-inf`/`+inf` where unlimited |
| `effort_limit`, `home_qpos` | `(n,)` |
| `base_pose` | the base frame |
| `kp`, `kd` | effective gains `(n,)`; `None` for torque control |

## Adding a backend

1. Write a class with `name`, `capabilities()`, `validate(cfg)` and `make_env(cfg)`. Import the engine inside
   `validate` and `make_env`, not at module level.
2. Register it: `register_backend("name", "your.module:YourBackend")`.
3. Register its options class: `register_backend_options("name", YourOptions)` (it needs `to_dict` and `from_dict`).
4. Return an `EnvBase` that follows the contract above: nested `task` / `proprio` observations, `[num_envs, ...]` shapes,
   no autoreset, `_reset` mask, injected state, deterministic seeding. Test it with `check_env_specs`, a masked-reset
   test and a seeding test.
5. Add a task id the same way if the backend brings its own tasks (see [TaskConfig](configs/task.md#adding-a-task-id)).
