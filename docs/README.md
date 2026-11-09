# robotdesigner documentation

A framework for loading different robot configurations ("embodiments") into one batched RL environment, so the same control task can be compared across robots.
The model in one paragraph: a robot is validated, hashable **data**; an `EnvConfig` (robot, control, task, simulation) holds every value that changes behaviour; `make_env(cfg)` is the single
step from config to a batched TorchRL environment; simulators and tasks are separate, and tasks come from providers.

```python
from robotdesigner.config import BaseEnvConfig, Chain

cfg = BaseEnvConfig().robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0)).sim(num_envs=8)
env_cfg = cfg.resolve()                         # robot + control + task + sim, every value explicit
assert (env_cfg.robot.config_hash, env_cfg.env_hash) == (cfg.resolve().robot.config_hash, cfg.resolve().env_hash)
# env = cfg.build()                             # make_env(env_cfg): needs the `env` extra
```

```
                   robot ──┐
                 control ──┤
                    task ──┼──►  EnvConfig ──► env_hash ──►  make_env(cfg)  ──►  batched TorchRL env
                     sim ──┘     (frozen)                        │
                                                                 ├─ simulator named by sim.simulator  (mujoco | warp)
                                                                 └─ task from its provider            (empty/idle, robosuite/<Env>, yours)
```

## Start here

| | |
|---|---|
| [Getting started](getting-started.md) | install, a first robot, a first environment, a sweep, the GPU, a video |
| [Design](design.md) | the principles, what is held fixed when embodiments are compared, what is not built, the vocabulary |

## Reference

| page | covers |
|---|---|
| [Robot](configs/robot.md) | `ParametricArm`, `chain()`, the gripper, `ExternalRobot` and robot providers, hashing, forward kinematics |
| [Control](configs/control.md) | `ControlConfig`: modes, gains, action scaling |
| [Task](configs/task.md) | `TaskConfig`: params, episode length, mount, goal regions, where tasks come from |
| [Simulation](configs/sim.md) | `SimConfig`, scenes, `MujocoOptions`, `WarpOptions` |
| [Robot provider: menagerie](robots/menagerie.md) | MuJoCo Menagerie robots as `ExternalRobot`s: `describe`, cards, licences, which robots run |
| [Environment](configs/env.md) | `EnvConfig`: the complete description, `env_hash`, serialization |
| [Builder](configs/base-env.md) | `BaseEnvConfig`: defaults, refinement, overrides, `sweep`, files |
| [Simulators and the environment contract](simulators.md) | `make_env`, capabilities, specs, reset, seeding, CPU and GPU specifics |
| [Rendering and video](viz.md) | `env.render`, cameras, `record_rollout`, `Clip` |

## Tasks

| page | covers |
|---|---|
| [`empty/idle`](tasks/empty.md) | the one task the package names |
| [robosuite scenes](tasks/robosuite.md) | robosuite arenas and objects as tasks, which environments work |
| [Writing a task provider](tasks/writing-a-provider.md) | bring your own tasks, end to end |

## Robots from a library

| page | covers |
|---|---|
| [`menagerie`](robots/menagerie.md) | MuJoCo Menagerie robots as `ExternalRobot`s: `describe`, cards, what is driven and held, spawn pose, licences, which robots run |
| [Writing a robot provider](robots/writing-a-robot-provider.md) | bring your own source of robot models, end to end |

## Working with it

| page | covers |
|---|---|
| [Running on a cluster](cluster.md) | one resolved dict per job, choosing the simulator, environment checklist |
| [Development](development.md) | layout, tests and markers, how the docs are tested, adding a simulator |

The Python examples in these pages are executed by `tests/test_docs.py` (blocks fenced `python no-run` need an engine and are not), and every link and anchor is checked.
Runnable notebooks are in [`examples/`](../examples).

## Words used in these docs

**embodiment**: a robot configuration. **simulator**: a physics engine (`mujoco`, `warp`). **task** / **task provider**: `"<namespace>/<name>"` and the owner of the namespace.
**robot provider** / **external robot** / **native gains**: the owner of a robot namespace; a reference to one of its models (`ExternalRobot`); `NativeGains`, which drives such a model with its own actuators.
**scene**: a simulator preset (`sim.scene`), or, for robosuite, an arena with objects. **mount frame**: the scene's own place for a robot (a world pose, from the task provider). **mount**: what the task adds to it: `Mount(offset)` for a fixed-base arm, `MobileMount(spawn, heading)` for a floating-base robot. **resolve**: turn the builder into the frozen `EnvConfig`.
**config side / engine side**: numpy-only code versus code that needs torch and MuJoCo. The full table is in [Design](design.md#vocabulary).
