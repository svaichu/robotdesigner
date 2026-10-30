# EnvConfig

The complete, declarative description of one environment: robot, control, task and sim (which includes the options of the chosen simulator), with one hash.
Everything that changes behaviour is in here, so a run is identified by its `env_hash`. Importing it needs numpy only.
An `EnvConfig` is data: `env = make_env(cfg)` is the one step that turns it into an environment (see
[building the environment](#building-the-environment)).

```python
import json
from dataclasses import replace

from robotdesigner.config import (
    ConfigError, ControlConfig, EnvConfig, FixedBox, FixedGains, MujocoOptions, Pose, SimConfig, TaskConfig, chain,
)
from robotdesigner.tasks.robosuite import LiftObject, RobosuiteParams
```

Related pages: [RobotConfig](robot.md), [ControlConfig](control.md), [TaskConfig](task.md), [SimConfig](sim.md),
[simulators](../simulators.md).

## `EnvConfig(robot, control, task, sim)`

| field | type | |
|---|---|---|
| `robot` | `ParametricArm` | [RobotConfig](robot.md) |
| `control` | `ControlConfig` | [ControlConfig](control.md) |
| `task` | `TaskConfig` | [TaskConfig](task.md) |
| `sim` | `SimConfig` | [SimConfig](sim.md): the simulator, the batch, the time step, the seed, the scene and the simulator's options |

Checks across the sections, on top of each section's own:

- each field has the right class (error path: the field name);
- per-joint `kp` and `kd` in `FixedGains` have exactly `robot.n` entries (path `control.gains.kp` or `.kd`).

Whether the simulator can run the combination (a control mode, a device, the time step) and whether the task's provider
accepts it on that simulator is not checked here: `make_env` reports it when the environment is built.

## Example

```python
arm = chain(3, "zyy", total_length=0.9, total_mass=3.0, radius=0.03, damping=0.5, armature=0.01,
            frictionloss=0.0, effort_limit=50.0, range=(-2.5, 2.5), label="three-joint arm",
            base_pose=Pose((-0.56, 0.0, 0.912), (1.0, 0.0, 0.0, 0.0)))   # the mount of a stock arm in robosuite's Lift
cfg = EnvConfig(
    robot=arm,
    control=ControlConfig("joint_pos_abs", FixedGains(kp=60.0, kd=6.0), action_scale=None),
    task=TaskConfig(id="robosuite/Lift", horizon=200, success_tolerance=None, target_region=None,   # a task of the
                    params=RobosuiteParams({}, LiftObject("cube", height=0.84),                        # `robosuite` provider
                                           reset_noise=0.0, terminate_on_success=False)),
    sim=SimConfig(simulator="mujoco", num_envs=8, device="cpu", dt=0.002, integrator="euler",
                  control_freq=50.0, seed=0, scene="empty",                # the arena brings its own floor
                  options=MujocoOptions(self_collision=False)),            # the settings of "mujoco" only
)
assert list(cfg.to_dict()) == ["robot", "control", "task", "sim"]
```

## Serialization

`to_dict()` gives a plain structure and `from_dict()` rebuilds it, validating everything on the way in. Unknown keys,
missing keys and wrong types raise `ConfigError`, and the path of the error is counted from the `EnvConfig`.

```python
text = json.dumps(cfg.to_dict(), indent=2)           # store this next to your results
back = EnvConfig.from_dict(json.loads(text))
assert back == cfg and back.env_hash == cfg.env_hash

broken = json.loads(text)
broken["sim"]["dt"] = -1
try:
    EnvConfig.from_dict(broken)
except ConfigError as e:
    assert e.path == "sim.dt"
else:
    raise AssertionError("no error")
```

## Hashing

```python
assert len(cfg.env_hash) == 12 and cfg.full_env_hash.startswith(cfg.env_hash) and len(cfg.full_env_hash) == 64
assert list(cfg.hash_dict()) == ["robot_hash", "control", "task", "sim"]
```

`full_env_hash` is the SHA-256 of the canonical JSON (sorted keys, floats rounded to 12 significant digits, `-0.0` as
`0.0`) of `hash_dict()`: the robot's `full_hash`, then the control, task and sim dicts (the simulator's options are part of `sim`). `env_hash` is its
first 12 characters.

| changes `env_hash` | does not |
|---|---|
| any physical or control parameter, the task, the time step, the integrator, the scene, the options | the robot's `label` |
| `num_envs` and `seed` (they define the run) | key order in a dict, `-0.0` versus `0.0`, scaling an axis or a quaternion |
| the simulator name and the task id | |

Not in the hash, because they are not part of the config: the version of MuJoCo, robosuite or this package, and robosuite's
own defaults for arguments you did not set (those are recorded on the environment as `resolved_env_kwargs`).

```python
assert replace(cfg, robot=replace(arm, label="renamed")).env_hash == cfg.env_hash
assert replace(cfg, sim=replace(cfg.sim, seed=1)).env_hash != cfg.env_hash
assert replace(cfg, sim=replace(cfg.sim, num_envs=16)).env_hash != cfg.env_hash
assert replace(cfg, control=ControlConfig("joint_pos_abs", FixedGains(kp=61.0, kd=6.0), None)).env_hash != cfg.env_hash
reordered = json.loads(json.dumps(cfg.to_dict(), sort_keys=True))
assert EnvConfig.from_dict(reordered).env_hash == cfg.env_hash
```

`resolve()` returns a concrete config in which region policies are turned into numbers before hashing. Nothing needs
resolving yet, so it returns the config itself; it is idempotent, and the environments and the hash call it for you.

```python
assert cfg.resolve() is cfg or cfg.resolve() == cfg
```

## Errors

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

# the options class must match the simulator (checked by SimConfig, so the path is relative to it)
assert error_of(lambda: replace(cfg, sim=replace(cfg.sim, options=FixedBox((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))))).path == "options"
# a per-joint gain list must have n entries (the arm has 3 joints)
assert error_of(lambda: replace(cfg, control=ControlConfig("joint_pos_abs", FixedGains((1.0, 2.0), 1.0), None))).path == "control.gains.kp"
# an unknown simulator has no options class
assert error_of(lambda: replace(cfg, sim=replace(cfg.sim, simulator="nope"))).path == "simulator"
```

## Building the environment

```python no-run
# needs the env extra (torch, torchrl, mujoco)
from robotdesigner import make_env
env = make_env(cfg)
print(env.robot_hash, env.env_hash)   # the 12-character hashes of the robot and of this whole config
```

`make_env(cfg)` is the only way an `EnvConfig` becomes an environment, and it works the same for every task: the simulator
is the one named in `cfg.sim.simulator`, the task comes from the provider that owns `cfg.task.id`
(see [TaskConfig](task.md#where-task-ids-come-from)). `EnvConfig` itself has no `make()` method, so the config layer
stays free of engines. Everything the simulator or the task cannot do with this config is reported in one `ConfigError`
([simulators](../simulators.md#capabilities)).

Every environment exposes `cfg`, `robot_hash` (`robot.config_hash`) and `env_hash`. Keep the JSON from `to_dict()` with
each run: `EnvConfig.from_dict(json.load(f)).env_hash` must equal the recorded `env_hash`.
