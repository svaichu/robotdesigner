# Simulators and the environment contract

Related pages: [SimConfig](configs/sim.md), [TaskConfig](configs/task.md), [EnvConfig](configs/env.md), [writing a task provider](tasks/writing-a-provider.md), [viz](viz.md).

A **simulator** is a physics engine realized as code: `robotdesigner.simulators.<name>`, loaded by name when `sim.simulator`
names it. It turns an [`EnvConfig`](configs/env.md) into a batched TorchRL environment. The config says
what to run; the simulator and the task's provider decide whether they can, and build it. Tasks are not part of a simulator:
they come from [task providers](configs/task.md#where-tasks-come-from) and run on any simulator their provider accepts.

```python no-run
from robotdesigner import make_env
env = make_env(cfg)        # cfg.sim.simulator picks the simulator, cfg.task.params the provider that supplies the task
```

| simulator | what it is | needs |
|---|---|---|
| `mujoco` | CPU MuJoCo: a model compiled from the config (robot, a scene preset, the task's assets). The reference and debugging simulator | `[env]` extra |
| `warp` | planned: the same models on the GPU. Declared as an extra only, not implemented | |

Where a task's scene comes from is a provider matter: [robosuite scenes](tasks/robosuite.md) load a robosuite arena and
its objects into the MuJoCo model; a task that needs no assets adds none and runs on `sim.scene`.

## Protocol and registry

```python no-run
class Simulator(Protocol):
    name: str
    def capabilities(self) -> Capabilities: ...   # what it can run
    def validate(self, cfg) -> list[Problem]: ... # engine-specific checks, before anything is built
    def make_env(self, cfg): ...                  # a torchrl EnvBase
```

- `robotdesigner.simulators` is a registry: `register_simulator(name, target)`, `get_simulator(name)`, `list_simulators()`.
  `target` is a simulator instance or a lazy `"module:attr"` string, resolved on first use, so importing
  `robotdesigner.simulators` never imports MuJoCo, torch or robosuite. An unknown name raises `ConfigError` with path
  `sim.simulator`.
- `Problem(path, message)` names the config field at fault, like `ConfigError.path`.

## Capabilities

`Capabilities` lists what a simulator supports: `control_modes`, `devices`, `robot_kinds`, `integrators`. A device entry
`"cuda"` matches any `cuda:N`. A simulator never lists tasks.

`make_env(cfg)` collects problems from three places and raises one `ConfigError` that lists **every** one:

1. the config against the simulator's capabilities (control mode, integrator, robot kind, device);
2. `simulator.validate(cfg)` (engine-specific checks, for example the scene preset);
3. `provider.validate(cfg, simulator.name)`: the task's own checks against this simulator.

Nothing is built, approximated or silently changed.

```text
simulator 'mujoco' cannot run this config (3 problems):
  - control.mode: simulator 'mujoco' does not support control mode 'ee_pos_delta'; supported: ['joint_pos_abs', ...]
  - sim.device: simulator 'mujoco' does not support device 'cuda:0'; supported: ['cpu']
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
| `sim.options` | `MujocoOptions` |

## The environment contract

Every simulator returns a TorchRL `EnvBase` with these properties, whatever the task.

- `batch_size = [num_envs]`, batch-locked, on the CPU for the current simulators.
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
  simulators are compared from the same start, or an episode is replayed.
- **Seeding:** `env.set_seed(seed)`. The same simulator, device and seed give the same episodes. The reset noise of the
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
| `mount` | the base pose in the world (`task.mount`) |
| `kp`, `kd` | effective gains `(n,)`; `None` for torque control |

## How an environment is put together

```
make_env(cfg)
  ├─ simulator = get_simulator(cfg.sim.simulator)                 the simulator named in the config
  ├─ provider  = get_task_provider(cfg.task.params.provider)      owner of the task's namespace
  ├─ problems  = capabilities + simulator.validate + provider.validate   (all at once)
  └─ simulator.make_env(cfg)
        ├─ task = provider.build(cfg, simulator.name)       engine side of the task (a TaskImpl)
        └─ MujocoEnv(cfg, task)
              compile_robot(cfg, scene_hook=task.attach)  scene preset, then the task's assets, then the robot
              BatchedSim, control law, masked reset, episode counters, rendering   (shared by every task)
              task.reset / observe / success / reward                                (the task's part)
```

## Adding a simulator

1. Write a class with `name`, `capabilities()`, `validate(cfg)` and `make_env(cfg)`. Import the engine inside
   `validate` and `make_env`, not at module level. `make_env` gets the task with
   `get_task_provider(cfg.task.params.provider).build(cfg, self.name)`.
2. Register it: `register_simulator("name", "your.module:YourSimulator")`.
3. Register its options class: `register_simulator_options("name", YourOptions)` (it needs `to_dict` and `from_dict`). It
   is the type of `sim.options`, which `SimConfig` checks against `sim.simulator`.
4. Return an `EnvBase` that follows the contract above: nested `task` / `proprio` observations, `[num_envs, ...]` shapes,
   no autoreset, `_reset` mask, injected state, deterministic seeding. Test it with `check_env_specs`, a masked-reset
   test and a seeding test.
5. Tell the providers it exists: a provider accepts a simulator by name in its `validate`, and builds the engine side the
   simulator needs in `build` (a provider that only knows MuJoCo reports `sim.simulator` as a problem on yours).
