# Simulation: `SimConfig` and simulator options

`SimConfig` chooses the physics engine and every number that changes how time advances. Nothing has a default here; `BaseEnvConfig` supplies defaults ([base-env](base-env.md)).

```python
from robotdesigner.config import ConfigError, MujocoOptions, SimConfig, WarpOptions, INTEGRATORS

sim = SimConfig(simulator="mujoco", num_envs=8, device="cpu", dt=0.002, integrator="euler", control_freq=50.0, seed=0,
                scene="empty", options=MujocoOptions(self_collision=False))
assert sim.n_substeps == 10                       # 1 / (control_freq * dt) physics steps per policy step
assert SimConfig.from_dict(sim.to_dict()) == sim
```

| field | rule |
|---|---|
| `simulator` | the name of a registered simulator: `"mujoco"` (CPU) or `"warp"` (GPU). A simulator is a physics engine, never a source of tasks |
| `num_envs` | `>= 1`; the batch size of the environment |
| `device` | `"cpu"` or `"cuda:N"`; the simulator decides which it supports |
| `dt` | physics time step, seconds, `> 0` |
| `integrator` | one of `INTEGRATORS`: `euler`, `implicit`, `implicitfast`, `rk4` |
| `control_freq` | policy steps per second, `> 0` |
| `seed` | integer; seeds the reset noise and, through a derived value, the task's own stream ([simulators](../simulators.md#seeding)) |
| `scene` | a preset of the simulator (below) |
| `options` | an instance of the options class registered for `simulator` |

**Substeps.** `1 / (control_freq * dt)` must be a positive integer (within 1e-9), so every policy step is a whole number of physics steps. 50 Hz at `dt = 0.002` is 10 substeps. `n_substeps` is that number.

## Scenes

`sim.scene` selects what the simulator adds around the robot before the task adds its own elements:

| scene | adds |
|---|---|
| `empty` | nothing |
| `plain_floor` | a 10 m × 10 m floor plane (collides with the robot, not with other scene geoms) and a light |

Both simulators use the same presets. A task can require a scene: `robosuite/*` tasks need `empty`, since the arena brings its own floor.
`empty` as a *scene* has nothing to do with the `empty` task provider.

## Options

`options` carries the settings that exist for one simulator only, as a typed object. A mismatch (`WarpOptions` with `simulator="mujoco"`) is an error at `options`.

### `MujocoOptions(self_collision)`

`self_collision=False` (the default of the builder) makes the robot's geoms collide with the scene but not with each other.

### `WarpOptions(njmax, nconmax, cuda_graph)`

| field | meaning |
|---|---|
| `njmax` | constraint rows allocated **per world** |
| `nconmax` | contacts allocated per world; the simulation allocates `nconmax * num_envs` in total |
| `cuda_graph` | capture the physics sub-steps of one policy step as a CUDA graph and replay them (CUDA only; resets are not captured) |

Capacities are per world, so `num_envs` can change without touching them. `WarpOptions.default()` is `njmax=128, nconmax=32, cuda_graph=True`: a plain choice measured
on table-top scenes with one or two objects (`benchmarks/bench_warp.py`), with headroom, not a recommendation for every scene. A scene with many contacts
needs more. When a capacity is exceeded the library would silently clip, so the environment checks after every step and raises
`robotdesigner.simulators.warp.sim.CapacityError` naming the field and what was used. It is never silent.

```python
assert WarpOptions.default() == WarpOptions(njmax=128, nconmax=32, cuda_graph=True)
big = SimConfig("warp", 4096, "cuda:0", 0.002, "euler", 50.0, 0, "empty", WarpOptions(njmax=512, nconmax=128, cuda_graph=True))
assert big.options.nconmax == 128
```

The options are part of the `env_hash`. Changing the capacities, or the CUDA graph switch, gives a different hash even though a correct run does not change physically.

### Options of another simulator

`register_simulator_options(name, cls)` declares the options class of a simulator you add. It needs `to_dict()`, `from_dict(d)`, and may offer a `default()` classmethod, which
`BaseEnvConfig.sim(simulator=...)` uses to reset the options when the simulator changes. See [development](../development.md#adding-a-simulator).

## Errors

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

common = dict(num_envs=2, device="cpu", dt=0.002, integrator="euler", control_freq=50.0, seed=0, scene="empty")
mj = MujocoOptions(False)
assert error_of(lambda: SimConfig("mujoco", **{**common, "control_freq": 40.0}, options=mj)).path == "control_freq"   # 12.5 substeps
assert error_of(lambda: SimConfig("mujoco", **{**common, "device": "gpu"}, options=mj)).path == "device"
assert error_of(lambda: SimConfig("mujoco", **{**common, "integrator": "rk5"}, options=mj)).path == "integrator"
assert error_of(lambda: SimConfig("mujoco", **{**common, "num_envs": 0}, options=mj)).path == "num_envs"
assert error_of(lambda: SimConfig("mujoco", **common, options=WarpOptions.default())).path == "options"
assert error_of(lambda: SimConfig("physx", **common, options=mj)).path == "simulator"
assert error_of(lambda: WarpOptions(0, 32, True)).path == "njmax"
```

Whether a simulator can actually *run* a config (the device exists, the control mode is implemented, the scene name is known) is checked by `make_env`, not here:
see [simulators](../simulators.md#capabilities-and-validation).
