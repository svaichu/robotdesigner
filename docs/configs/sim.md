# SimConfig and simulator options

How the simulation runs: which simulator, how many environments, the time step and the seed. Nothing here has a default
value. Importing it needs numpy only. `simulator` names a **simulator** (a physics engine); where the task's scene and
objects come from is the business of the [task](task.md) and its provider.

```python
from robotdesigner.config import (
    ConfigError, INTEGRATORS, MujocoOptions, SimConfig, WarpOptions, register_simulator_options,
)
from robotdesigner.config.sim import SIMULATOR_OPTIONS
```

Related pages: [RobotConfig](robot.md), [ControlConfig](control.md), [TaskConfig](task.md), [EnvConfig](env.md),
[simulators](../simulators.md).

## `SimConfig(simulator, num_envs, device, dt, integrator, control_freq, seed, scene, options)`

| field | type | rule |
|---|---|---|
| `simulator` | str | non-empty; the registered name of a simulator (`"mujoco"`) and of its options class |
| `num_envs` | int | `>= 1`; the batch size of the environment |
| `device` | str | `"cpu"` or `"cuda:N"` |
| `dt` | float | `> 0`; physics time step in seconds |
| `integrator` | str | one of `INTEGRATORS`: `euler`, `implicit`, `implicitfast`, `rk4` |
| `control_freq` | float | `> 0`; policy steps per second |
| `seed` | int | seeds the environment's random streams (see the table below) |
| `scene` | str | non-empty; a scene preset known to the simulator |
| `options` | the options class of `simulator` | settings that exist for that simulator only, [below](#simulator-options); `MujocoOptions` for `"mujoco"` |

**Sub-steps.** One policy step runs `1 / (control_freq · dt)` physics steps, and that number must be a positive integer
(checked to 1e-9). `sim.n_substeps` returns it.

**What a simulator accepts.** The config only checks its own consistency. Whether the simulator supports the device,
integrator, time step and scene is checked by the simulator when the environment is built, and every problem is reported
at once (see [simulators](../simulators.md#capabilities)).

| field | `mujoco` | `warp` |
|---|---|---|
| `device` | `cpu` | `cuda:N` |
| `dt` | any | any |
| `integrator` | all four | all four |
| `scene` | `empty` or `plain_floor`; a task that brings its own floor (an arena) asks for `empty` | the same |
| `num_envs` | N simulations stepped one after another in one process | N worlds in one device allocation |
| `seed` | two streams: the robot's reset noise, and separately the task's own draws (placements, targets), so the task's draws do not depend on the robot | the same two streams, drawn on the CPU and copied to the device, so a reset draws the same numbers as `mujoco` |

Scene presets (both simulators): `empty` has nothing but the robot; `plain_floor` adds a floor plane at `z = 0` that
robot geoms collide with, and a light. Same simulator, device and seed reproduce the same results. Across simulators the draws
are equal and the state agrees to float32 rounding (`warp` stores float32), but a long contact-rich rollout can diverge.

```python
sim = SimConfig(simulator="mujoco", num_envs=4, device="cpu", dt=0.002, integrator="euler",
                control_freq=50.0, seed=0, scene="plain_floor", options=MujocoOptions(self_collision=False))
assert sim.n_substeps == 10
assert sim.to_dict() == {
    "simulator": "mujoco", "num_envs": 4, "device": "cpu", "dt": 0.002, "integrator": "euler",
    "control_freq": 50.0, "seed": 0, "scene": "plain_floor", "options": {"self_collision": False},
}
assert SimConfig.from_dict(sim.to_dict()) == sim
assert INTEGRATORS == ("euler", "implicit", "implicitfast", "rk4")
```

## Simulator options

Each simulator has one options class. It is required, it is part of the hash, and `SimConfig` rejects a class that does
not belong to `simulator` (error path `options`), and a `simulator` name that has no options class registered (path
`simulator`). The options sit in `SimConfig` because they are the settings of the simulator that `simulator` names: when
you change the simulator you change both together.

### `MujocoOptions(self_collision)`

| field | type | meaning |
|---|---|---|
| `self_collision` | bool | `False`: robot links collide with the scene only. `True`: they also collide with each other (non-adjacent links) |

```python
assert MujocoOptions(self_collision=False).to_dict() == {"self_collision": False}
assert SIMULATOR_OPTIONS["mujoco"] is MujocoOptions
```

### `WarpOptions(njmax, nconmax, cuda_graph)`

Options of the `"warp"` simulator (MuJoCo Warp on an NVIDIA GPU). The capacities are per world, so `sim.num_envs` can change
in a sweep without re-tuning them.

| field | type | meaning |
|---|---|---|
| `njmax` | int | `>= 1`; constraint rows allocated per world; no world may exceed it |
| `nconmax` | int | `>= 1`; contacts allocated per world; the simulation allocates `nconmax * num_envs` |
| `cuda_graph` | bool | capture the physics sub-steps of one policy step in a CUDA graph |

```python
o = WarpOptions(njmax=64, nconmax=16, cuda_graph=True)
assert WarpOptions.from_dict(o.to_dict()) == o
assert SIMULATOR_OPTIONS["warp"] is WarpOptions
assert WarpOptions.default().njmax >= 1     # provisional numbers, to be set from measurements
```

`WarpOptions.default()` is `WarpOptions(njmax=64, nconmax=16, cuda_graph=True)`: provisional numbers, to be replaced by measurements.
Scenes with many contacts need more (see [capacity overflow](../simulators.md#capacity-overflow)). The config side needs no GPU,
torch or MuJoCo; building the environment (`make_env`) reports a missing CUDA device or library together with every other problem.

```python
warp = SimConfig(simulator="warp", num_envs=4096, device="cuda:0", dt=0.002, integrator="euler", control_freq=50.0,
                 seed=0, scene="empty", options=WarpOptions(njmax=128, nconmax=32, cuda_graph=True))
assert SimConfig.from_dict(warp.to_dict()) == warp
try:
    SimConfig(**{**{k: v for k, v in warp.to_dict().items() if k != "options"}, "options": MujocoOptions(self_collision=False)})
except ConfigError as e:
    assert e.path == "options"
else:
    raise AssertionError("no error")
```

A simulator defined outside this package registers its options with `register_simulator_options("name", OptionsClass)`.
The class needs `to_dict()` and a `from_dict()` classmethod, like every config class here.

## Errors

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

args = dict(simulator="mujoco", num_envs=4, device="cpu", dt=0.002, integrator="euler", control_freq=50.0, seed=0,
            scene="plain_floor", options=MujocoOptions(self_collision=False))
assert error_of(lambda: SimConfig(**{**args, "control_freq": 70.0})).path == "control_freq"   # 1/(70*0.002) is not an integer
assert error_of(lambda: SimConfig(**{**args, "num_envs": 0})).path == "num_envs"
assert error_of(lambda: SimConfig(**{**args, "device": "gpu"})).path == "device"
assert error_of(lambda: SimConfig(**{**args, "integrator": "leapfrog"})).path == "integrator"
assert error_of(lambda: SimConfig(**{**args, "dt": 0.0})).path == "dt"
assert error_of(lambda: SimConfig(**{**args, "options": True})).path == "options"          # not the class of "mujoco"
assert error_of(lambda: SimConfig(**{**args, "simulator": "nope"})).path == "simulator"   # no options class registered
```
