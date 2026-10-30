# Backends and the environment contract

A **backend** is a simulator. It turns an [`EnvConfig`](configs/env.md) into a batched TorchRL environment. The config says
what to run; the backend and the task's provider decide whether they can, and build it. Tasks are not part of a backend:
they come from [task providers](configs/task.md#where-task-ids-come-from) and run on any backend their provider accepts.

```python no-run
from robotdesigner import make_env
env = make_env(cfg)        # cfg.sim.backend picks the simulator, cfg.task.id the provider that supplies the task
```

| backend | what it is | needs |
|---|---|---|
| `mujoco` | CPU MuJoCo: a model compiled from the config (robot, a scene preset, the task's assets). The reference and debugging backend | `[env]` extra |
| `warp` | planned: the same models on the GPU. Declared as an extra only, not implemented | |

Where a task's scene comes from is a provider matter: [robosuite scenes](tasks/robosuite.md) load a robosuite arena and
its objects into the MuJoCo model; a task that needs no assets adds none and runs on `sim.scene`.

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

`Capabilities` lists what a backend supports: `control_modes`, `devices`, `robot_kinds`, `integrators`. A device entry
`"cuda"` matches any `cuda:N`. A backend never lists tasks.

`make_env(cfg)` collects problems from three places and raises one `ConfigError` that lists **every** one:

1. the config against the backend's capabilities (control mode, integrator, robot kind, device);
2. `backend.validate(cfg)` (engine-specific checks, for example the scene preset);
3. `provider.validate(cfg, backend.name)`: the task's own checks against this backend.

Nothing is built, approximated or silently changed.

```text
backend 'mujoco' cannot run this config (3 problems):
  - control.mode: backend 'mujoco' does not support control mode 'ee_pos_delta'; supported: ['joint_pos_abs', ...]
  - sim.device: backend 'mujoco' does not support device 'cuda:0'; supported: ['cpu']
  - sim.scene: must be 'empty' for robosuite tasks: the arena brings its own floor
```

| | `mujoco` |
|---|---|
| control modes | `joint_pos_abs`, `joint_pos_delta`, `joint_torque`, `joint_vel` |
| gains | `FixedGains`, `InertiaScaledGains` |
| device | `cpu` |
| integrators | `euler`, `implicit`, `implicitfast`, `rk4` |
| time step `dt` | any (`1 / (control_freq · dt)` must be an integer) |
| scenes | `empty`, `plain_floor` (a task's own scene is added on top) |
| robot kinds | `parametric` |
| `robot.tool` | ignored |
| backend options | `MujocoOptions` |

## The environment contract

Every backend returns a TorchRL `EnvBase` with these properties, whatever the task.

- `batch_size = [num_envs]`, batch-locked, on the CPU for the current backends.
- **No autoreset.** `step` never resets an environment. Reset chosen rows with a boolean mask `td["_reset"]` of shape
  `[num_envs, 1]` (absent: reset all), or use `env.step_and_maybe_reset(td)` to let the driver do it from the done keys.
- **Step** reads `td["action"]`, shape `[num_envs, n]` in `[-1, 1]` (one value per joint), and returns the outputs under
  `("next", ...)`.
- **Done keys:** `done`, `terminated`, `truncated`, each `[num_envs, 1]` bool. `truncated` is the time limit (`horizon`).
- **Reward:** `[num_envs, 1]` float32, from the task.
- **Always present** in the observation: `success` `[num_envs, 1]` bool and `episode_step` `[num_envs, 1]` int64, a
  `task` group (whatever the task observes) and a `proprio` group with `qpos`, `qvel` and `effort`, each `[num_envs, n]`.
- **Injected state:** the reset TensorDict may carry `qpos` and `qvel`, and whatever the task accepts (a task
  documents its own keys, for example a goal point), which override the random initialization. This is how two
  backends are compared from the same start, or an episode is replayed.
- **Seeding:** `env.set_seed(seed)`. The same backend, device and seed give the same episodes. The reset noise of the
  robot and the draws of the task come from two separate streams, both seeded from `sim.seed`: what a task draws (an
  object's placement, a goal) is the same for every embodiment run with the same seed and `num_envs`.
- **Attributes:** `env.cfg` (the resolved config), `env.robot_hash`, `env.env_hash`, `env.task` (the task's engine side),
  `env.interface` (a `RobotInterface`, below), `env.q_target` (position targets in `joint_pos_delta` mode).
- `env.render(...)` returns an RGB frame of the live state, with the overlays the task draws; see [viz](viz.md).
- `check_env_specs(env)` passes for every task (the tests run it).

`proprio/effort` is the actuator force (N·m) at the last physics sub-step of the step. The action's meaning is the
control mode (see [ControlConfig](configs/control.md#modes)). The keys of the `task` group are the task's own and are
documented on its page.

## `RobotInterface`

What tasks and wrappers need to know about a compiled robot, without engine-specific names:

| field | meaning |
|---|---|
| `n`, `joint_names` | number of joints and their names |
| `qpos_adr`, `qvel_adr`, `actuator_ids` | indices into the simulation arrays, `(n,)` |
| `ee_site_id` | the end-effector site |
| `joint_range` | `(n, 2)`, `-inf`/`+inf` where unlimited |
| `effort_limit`, `home_qpos` | `(n,)` |
| `base_pose` | the base frame |
| `kp`, `kd` | effective gains `(n,)`; `None` for torque control |

## How an environment is put together

```
make_env(cfg)
  ├─ backend  = get_backend(cfg.sim.backend)              the simulator
  ├─ provider = provider_for(cfg.task.id)                 owner of the task id's namespace
  ├─ problems = capabilities + backend.validate + provider.validate     (all at once)
  └─ backend.make_env(cfg)
        ├─ task = provider.build(cfg, backend.name)       engine side of the task (a TaskImpl)
        └─ MujocoEnv(cfg, task)
              compile_robot(cfg, scene_hook=task.attach)  scene preset, then the task's assets, then the robot
              BatchedSim, control law, masked reset, episode counters, rendering   (shared by every task)
              task.reset / observe / success / reward                                (the task's part)
```

## Adding a backend

1. Write a class with `name`, `capabilities()`, `validate(cfg)` and `make_env(cfg)`. Import the engine inside
   `validate` and `make_env`, not at module level. `make_env` gets the task with
   `provider_for(cfg.task.id)[0].build(cfg, self.name)`.
2. Register it: `register_backend("name", "your.module:YourBackend")`.
3. Register its options class: `register_backend_options("name", YourOptions)` (it needs `to_dict` and `from_dict`).
4. Return an `EnvBase` that follows the contract above: nested `task` / `proprio` observations, `[num_envs, ...]` shapes,
   no autoreset, `_reset` mask, injected state, deterministic seeding. Test it with `check_env_specs`, a masked-reset
   test and a seeding test.
5. Tell the providers it exists: a provider accepts a backend by name in its `validate`, and builds the engine side the
   backend needs in `build` (a provider that only knows MuJoCo reports `sim.backend` as a problem on yours).
