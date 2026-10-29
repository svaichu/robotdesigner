# SimConfig and backend options

How the simulation runs: which backend, how many environments, the time step and the seed. Nothing here has a default
value. Importing it needs numpy only.

```python
from robotdesigner.config import (
    ConfigError, INTEGRATORS, MujocoOptions, RobosuiteOptions, SimConfig, register_backend_options,
)
from robotdesigner.config.sim import BACKEND_OPTIONS, BASE_PLACEMENTS
```

Related pages: [RobotConfig](robot.md), [ControlConfig](control.md), [TaskConfig](task.md), [EnvConfig](env.md),
[backends](../backends.md).

## `SimConfig(backend, num_envs, device, dt, integrator, control_freq, seed, scene)`

| field | type | rule |
|---|---|---|
| `backend` | str | non-empty; the registered name of a backend (`"mujoco"`, `"robosuite"`) |
| `num_envs` | int | `>= 1`; the batch size of the environment |
| `device` | str | `"cpu"` or `"cuda:N"` |
| `dt` | float | `> 0`; physics time step in seconds |
| `integrator` | str | one of `INTEGRATORS`: `euler`, `implicit`, `implicitfast`, `rk4` |
| `control_freq` | float | `> 0`; policy steps per second |
| `seed` | int | seeds the environment's random draws |
| `scene` | str | non-empty; a scene preset known to the backend |

**Sub-steps.** One policy step runs `1 / (control_freq · dt)` physics steps, and that number must be a positive integer
(checked to 1e-9). `sim.n_substeps` returns it.

**What a backend accepts.** The config only checks its own consistency. Whether the backend supports the device,
integrator, time step and scene is checked by the backend when the environment is built, and every problem is reported
at once (see [backends](../backends.md#capabilities)).

| field | `mujoco` | `robosuite` |
|---|---|---|
| `device` | `cpu` | `cpu` |
| `dt` | any | fixed by robosuite at `0.002` |
| `integrator` | all four | `euler` (MuJoCo's default, which robosuite does not change) |
| `scene` | `empty` or `plain_floor` | `env`: the scene is whatever the robosuite environment builds |
| `num_envs` | N simulations stepped one after another in one process | N robosuite environments stepped one after another |
| `seed` | seeds the generator behind target and reset-noise draws | sub-environment `i` is seeded with `seed + i`; the generator behind reset noise uses `seed` |

Scene presets of the native backend: `empty` has nothing but the robot; `plain_floor` adds a floor plane at `z = 0` that
robot geoms collide with, and a light. Same backend, device and seed reproduce the same results; random draws differ
between backends.

```python
sim = SimConfig(backend="mujoco", num_envs=4, device="cpu", dt=0.002, integrator="euler",
                control_freq=50.0, seed=0, scene="plain_floor")
assert sim.n_substeps == 10
assert sim.to_dict() == {
    "backend": "mujoco", "num_envs": 4, "device": "cpu", "dt": 0.002, "integrator": "euler",
    "control_freq": 50.0, "seed": 0, "scene": "plain_floor",
}
assert SimConfig.from_dict(sim.to_dict()) == sim
assert INTEGRATORS == ("euler", "implicit", "implicitfast", "rk4")
```

## Backend options

Each backend has one options class. It is required, it is part of the hash, and `EnvConfig` rejects the wrong class for
the backend named in `sim.backend`.

### `MujocoOptions(self_collision)`

| field | type | meaning |
|---|---|---|
| `self_collision` | bool | `False`: robot links collide with the scene only. `True`: they also collide with each other (non-adjacent links) |

### `RobosuiteOptions(base_placement)`

| field | type | meaning |
|---|---|---|
| `base_placement` | `"env_default"` or `"explicit"` | where the robot goes in the robosuite scene |

- `"env_default"`: the arm's root sits where the environment mounts a stock arm (robosuite's own placement rule for the
  environment, for example 0.56 m behind the table centre for a table task), and `robot.base_pose` is relative to that
  point, for example the shoulder height above it.
- `"explicit"`: the root sits at the arena origin and `robot.base_pose` is the pose of the arm base in the arena frame.

```python
assert MujocoOptions(self_collision=False).to_dict() == {"self_collision": False}
assert RobosuiteOptions("env_default").to_dict() == {"base_placement": "env_default"}
assert BASE_PLACEMENTS == ("env_default", "explicit")
assert BACKEND_OPTIONS["mujoco"] is MujocoOptions and BACKEND_OPTIONS["robosuite"] is RobosuiteOptions
```

A backend defined outside this package registers its options with `register_backend_options("name", OptionsClass)`.
The class needs `to_dict()` and a `from_dict()` classmethod, like every config class here.

## Errors

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

args = dict(backend="mujoco", num_envs=4, device="cpu", dt=0.002, integrator="euler", control_freq=50.0, seed=0,
            scene="plain_floor")
assert error_of(lambda: SimConfig(**{**args, "control_freq": 70.0})).path == "control_freq"   # 1/(70*0.002) is not an integer
assert error_of(lambda: SimConfig(**{**args, "num_envs": 0})).path == "num_envs"
assert error_of(lambda: SimConfig(**{**args, "device": "gpu"})).path == "device"
assert error_of(lambda: SimConfig(**{**args, "integrator": "leapfrog"})).path == "integrator"
assert error_of(lambda: SimConfig(**{**args, "dt": 0.0})).path == "dt"
assert error_of(lambda: RobosuiteOptions("nowhere")).path == "base_placement"
```
