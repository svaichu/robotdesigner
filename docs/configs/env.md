# EnvConfig

The complete, declarative description of one environment: robot, control, task, sim and backend options, with one hash.
Everything that changes behaviour is in here, so a run is identified by its `env_hash`. Importing it needs numpy only.

```python
import json
from dataclasses import replace

from robotdesigner.config import (
    ConfigError, ControlConfig, EnvConfig, FixedBox, FixedGains, MujocoOptions, Pose, ReachParams,
    RobosuiteOptions, SimConfig, TaskConfig, chain,
)
```

Related pages: [RobotConfig](robot.md), [ControlConfig](control.md), [TaskConfig](task.md), [SimConfig](sim.md),
[backends](../backends.md).

## `EnvConfig(robot, control, task, sim, backend_options)`

| field | type | |
|---|---|---|
| `robot` | `ParametricArm` | [RobotConfig](robot.md) |
| `control` | `ControlConfig` | [ControlConfig](control.md) |
| `task` | `TaskConfig` | [TaskConfig](task.md) |
| `sim` | `SimConfig` | [SimConfig](sim.md) |
| `backend_options` | the options class of `sim.backend` | `MujocoOptions` for `"mujoco"`, `RobosuiteOptions` for `"robosuite"` |

Checks across the sections, on top of each section's own:

- each field has the right class (error path: the field name);
- `backend_options` is an instance of the class registered for `sim.backend` (path `backend_options`), and the backend
  has options registered at all (path `sim.backend`);
- per-joint `kp` and `kd` in `FixedGains` have exactly `robot.n` entries (path `control.gains.kp` or `.kd`).

Whether the backend can run the combination (a control mode, a task, a device, the time step) is not checked here: the
backend reports it when the environment is built.

## Example

```python
arm = chain(3, "zyz", total_length=0.9, total_mass=3.0, radius=0.03, damping=0.5, armature=0.01,
            frictionloss=0.0, effort_limit=50.0, range=(-2.5, 2.5), label="three-joint arm")
cfg = EnvConfig(
    robot=arm,
    control=ControlConfig("joint_pos_abs", FixedGains(kp=60.0, kd=6.0), action_scale=None),
    task=TaskConfig("native/reach", horizon=200, success_tolerance=0.02,
                    target_region=FixedBox(low=(0.2, -0.3, -0.2), high=(0.6, 0.3, 0.2)),
                    params=ReachParams(reset_noise=0.0, terminate_on_success=False)),
    sim=SimConfig(backend="mujoco", num_envs=8, device="cpu", dt=0.002, integrator="euler",
                  control_freq=50.0, seed=0, scene="plain_floor"),
    backend_options=MujocoOptions(self_collision=False),
)
assert list(cfg.to_dict()) == ["robot", "control", "task", "sim", "backend_options"]
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
assert list(cfg.hash_dict()) == ["robot_hash", "control", "task", "sim", "backend_options"]
```

`full_env_hash` is the SHA-256 of the canonical JSON (sorted keys, floats rounded to 12 significant digits, `-0.0` as
`0.0`) of `hash_dict()`: the robot's `full_hash`, then the control, task, sim and backend-option dicts. `env_hash` is its
first 12 characters.

| changes `env_hash` | does not |
|---|---|
| any physical or control parameter, the task, the time step, the integrator, the scene, the options | the robot's `label` |
| `num_envs` and `seed` (they define the run) | key order in a dict, `-0.0` versus `0.0`, scaling an axis or a quaternion |
| the backend name | |

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

# the options class must match the backend
assert error_of(lambda: replace(cfg, backend_options=RobosuiteOptions("explicit"))).path == "backend_options"
# a per-joint gain list must have n entries (the arm has 3 joints)
assert error_of(lambda: replace(cfg, control=ControlConfig("joint_pos_abs", FixedGains((1.0, 2.0), 1.0), None))).path == "control.gains.kp"
# an unknown backend has no options class
assert error_of(lambda: replace(cfg, sim=replace(cfg.sim, backend="nope"))).path == "sim.backend"
```

## Building the environment

```python no-run
# needs the env extra (torch, torchrl, mujoco)
from robotdesigner.envs import make_env
env = make_env(cfg)
print(env.robot_hash, env.env_hash)   # the 12-character hashes of the robot and of this whole config
```

Every environment exposes `cfg`, `robot_hash` (`robot.config_hash`) and `env_hash`. Keep the JSON from `to_dict()` with
each run: `EnvConfig.from_dict(json.load(f)).env_hash` must equal the recorded `env_hash`.
