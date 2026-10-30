# BaseEnvConfig

A complete, runnable environment description that you refine with section calls, in the ergonomics of RLlib's
`AlgorithmConfig` (nothing else is borrowed: no `ray`, no trainer). `BaseEnvConfig()` is a one-link arm in the `empty`
scene doing the [`empty/idle`](../tasks/empty.md) task on the CPU MuJoCo simulator. `.robot()`, `.control()`, `.task()` and `.sim()` each
return a **new** config that changes what the call writes; `resolve()` returns the frozen, validated
[`EnvConfig`](env.md) with its `env_hash`. Importing it needs numpy only; `build()` imports the engine.

```python
from robotdesigner.config import (
    NOT_SET, BaseEnvConfig, Chain, ConfigError, EnvConfig, FixedGains, Pose,
)
from robotdesigner.config.defaults import DEFAULTS, DEFAULTS_VERSION
```

Related pages: [EnvConfig](env.md), [RobotConfig](robot.md), [TaskConfig](task.md), [SimConfig](sim.md), [simulators](../simulators.md).

## The rules

1. **`BaseEnvConfig()` is complete and runnable.** Its defaults are explicit values in one table (`config/defaults.py`, `DEFAULTS` and
   `DEFAULTS_VERSION`), shown by `explain()`. Defaults live in this builder layer only: `EnvConfig`, `SimConfig`, `ControlConfig`, `TaskConfig`
   and `ParametricArm` still require every field.
2. **Every call returns a new config**; the receiver is never changed, so one base serves a whole sweep. Sections not called keep the base's value.
3. **Calls accumulate.** A call changes only what it writes: a second `.sim(dt=...)` keeps the first call's `num_envs`.
4. **`NOT_SET`** means "argument not passed". `None` is a meaningful value in the configs (`action_scale=None`, `target_region=None`).
5. **`resolve()`** applies the deferred default (the mount), validates every section and raises **one** `ConfigError` that lists every problem
   with its dotted path (`.problems` holds them). `validate()` returns the same problems without raising.
6. **Store `resolve().to_dict()` with every run, checkpoint and dataset.** `env_hash` is computed on the resolved `EnvConfig`, so changing a
   default changes the hash of configs that relied on it; the stored dict holds every value and reproduces after the defaults change.

```python
base = BaseEnvConfig()
cfg = base.resolve()
assert isinstance(cfg, EnvConfig) and cfg.robot.n == 1 and cfg.task.id == "empty/idle" and cfg.sim.scene == "empty"
assert DEFAULTS_VERSION == 1 and DEFAULTS["sim"]["control_freq"] == 50.0
assert EnvConfig.from_dict(cfg.to_dict()).env_hash == cfg.env_hash      # the stored resolved dict reproduces
```

## The section calls

| call | what it takes | what it does |
|---|---|---|
| `.robot(robot=, **fields)` | a `Chain`, a ready `ParametricArm`, or `Chain` fields as keywords | a `Chain` is partial: written fields change, the rest carries. A `ParametricArm` is a fixed robot and replaces the section |
| `.control(control=, **fields)` | `mode`, `gains`, `action_scale`, or a `ControlConfig` | assigns the fields written |
| `.task(params=, **fields)` | a provider's params object, `horizon`, `success_tolerance`, `target_region`, `mount`, or a `TaskConfig` | the params object is the task. A *new params class* resets `success_tolerance` and `target_region` (they belong to goal tasks) unless the call writes them |
| `.sim(sim=, **fields)` | the fields of `SimConfig` (including `options`), or a `SimConfig` | assigns the fields written; `simulator=` replaces `options` with that simulator's defaults (below) |

An unknown field is a `ConfigError` at the call, with the dotted path (`sim.num_env`). Values are validated by `resolve()`.

```python
cfg = (
    BaseEnvConfig()
    .robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0))
    .control(gains=FixedGains(kp=60.0, kd=6.0))
    .task(horizon=100)
    .sim(num_envs=8)
)
fine = cfg.sim(dt=0.001)                 # a new config; cfg is unchanged; num_envs stays 8
assert cfg.resolve().sim.dt == 0.002 and fine.resolve().sim.dt == 0.001 and fine.resolve().sim.num_envs == 8
assert cfg.resolve().task.horizon == 100
```

### `Chain`

`Chain` is a frozen spec that mirrors `chain()` without `base_pose` (the mount belongs to the task): `n`, `axes`, `link_length` or
`total_length`, `mass` or `total_mass`, `radius`, `damping`, `armature`, `frictionloss`, `effort_limit`, `range`, `home_qpos`, `end_effector`, `tool`,
`label`. Every field is optional at the call; the base holds a complete `Chain`; `Chain.arm()` returns the `ParametricArm`; `chain()` stays the function.

- Writing one member of an exclusive pair (`link_length` / `total_length`, `mass` / `total_mass`) **clears the other**.
- `n` and `axes` change together: one letter per joint, no cycling. A per-joint sequence must have `n` entries after the merge; otherwise `resolve()` reports
  its path (`robot.axes`, `robot.radius`, ...).

```python
three = cfg.robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0, damping=1.5))
five = three.robot(Chain(n=5, axes="zyyyy"))          # lengths, masses and damping carry
arm = five.resolve().robot
assert arm.n == 5 and arm.segments[0].joint.damping == 1.5
assert abs(sum(s.link.tip_offset[0] for s in arm.segments) - 0.9) < 1e-12

short_links = three.robot(link_length=0.2)            # clears total_length
assert short_links.resolve().robot.segments[0].link.tip_offset[0] == 0.2
assert Chain(link_length=0.3).merge(Chain(total_length=1.0)).link_length is NOT_SET
```

### The mount

`mount` is "the task's default" until written: `resolve()` fills it from `TaskProvider.default_mount(name)` and the resolved value is what is
hashed. `.task(mount=Pose(...))` pins it, and a pinned mount survives later task changes.

```python
pin = Pose((0.3, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0))
assert BaseEnvConfig().resolve().task.mount == Pose.identity()              # the empty task: the origin
assert BaseEnvConfig().task(mount=pin).resolve().task.mount == pin
```

### The simulator and its options

`.sim(simulator="other")` replaces `sim.options` with that simulator's defaults, because the options of one simulator cannot be kept for another;
pass `options=` to give them. The defaults come from an optional `default()` classmethod on the options class (`MujocoOptions` has one);
without it, `options=` is required. Staying on the same simulator keeps the options.

## Overrides by dotted path

`with_overrides` returns a new config and has the semantics of the section calls (they accumulate), with paths that reach nested fields:
`sim.options.*`, `task.params.*`, `control.gains.*`, `robot.*`. It takes a mapping or a list of CLI strings `path=value`; the value is parsed as a
JSON literal and kept as a string if it is not one (`64` is an int, `[-2.0, 2.0]` a list, `true` a bool, `null` is `None`, `zyy` and `rk4` are strings).

- Each touched section is converted to a dict, edited, and rebuilt with its own `from_dict` and validation, so providers need no extra code. The rebuilt
  section is validated at once: `{"sim.num_envs": 0}` raises `sim.num_envs`. Several edits of one call are applied before the section is rebuilt.
- An unknown section or key raises `ConfigError` with the dotted path (`sim.options.zzz`, `task.params.zzz`, `robot.lenght`).
- Writing one member of an exclusive pair clears the other (unless both are written in the same call, which `resolve()` reports).
- `sim.simulator` resets `sim.options` to that simulator's defaults unless `sim.options.*` is given in the same call (the options then edit the new simulator's).

```python
cfg = BaseEnvConfig().robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0))
big = cfg.with_overrides({"sim.num_envs": 64, "sim.options.self_collision": True, "robot.total_length": 1.2})
r = big.resolve()
assert r.sim.num_envs == 64 and r.sim.options.self_collision is True and r.robot.n == 3    # the rest of each section stays
assert abs(sum(s.link.tip_offset[0] for s in r.robot.segments) - 1.2) < 1e-12

five = cfg.with_overrides(["sim.num_envs=64", "robot.n=5", "robot.axes=zyyyy", "sim.integrator=rk4"])   # CLI strings
assert five.resolve().robot.n == 5 and five.resolve().sim.integrator == "rk4"
assert five.with_overrides({"sim.num_envs": 64}) == five.with_overrides(["sim.num_envs=64"])
```

## Sweeps

`sweep(paired=, grid=)` expands a config into a family, one config (and one `env_hash`) per embodiment or setting. Keys are dotted paths with the semantics
of `with_overrides`. `grid` axes form a Cartesian product; `paired` axes are zipped (equal lengths, otherwise an error) and the paired group multiplies
with the grid. The order is deterministic: the paired group first, then the grid axes in the order given, the last one fastest.

- Items whose resolved `env_hash` repeats are dropped, with a warning and an entry in `family.notes`, so a sweep never runs the same config twice.
- An invalid item raises at expansion, with the path of the problem and the item index and overrides (for example `n` changed without `axes`, or a per-joint
  `kp` of the wrong length).
- Sweeping `sim.simulator` alone is fine: `sim.options` becomes that simulator's defaults; add `sim.options.*` axes to vary them.
- `Family` is iterable (each item a `BaseEnvConfig`); `configs()` gives the `EnvConfig`s, `table()` one row per item (`index`, the override values, `env_hash`),
  `overrides()` the overrides.
- Jobs and workers should receive the resolved dict (`config.resolve().to_dict()`) and rebuild with `EnvConfig.from_dict`: the task and robot registries are
  process-global. Launching the jobs is up to the project that uses the family; `examples/embodiment_sweep.ipynb` compares arms with 2 to 5 joints.

```python
def error_of_sweep(build):
    try:
        build()
    except ConfigError as e:
        return e.path
    raise AssertionError("no error")
```

```python
family = cfg.sweep(                       # 4 embodiments x 3 seeds = 12 configs
    paired={"robot.n": [2, 3, 4, 5], "robot.axes": ["zy", "zyy", "zyyy", "zyyyy"]},
    grid={"sim.seed": range(3)},
)
assert len(family) == 12 and len(family.configs()) == 12
rows = family.table()
assert [(r["robot.n"], r["sim.seed"]) for r in rows][:4] == [(2, 0), (2, 1), (2, 2), (3, 0)]    # the last axis is fastest
assert len({r["env_hash"] for r in rows}) == 12                                                  # distinct
assert all(isinstance(c, EnvConfig) for c in family.configs()) and family[0].resolve().robot.n == 2

assert error_of_sweep(lambda: cfg.sweep(paired={"robot.n": [2, 3], "robot.axes": ["zy"]})) == "sweep.paired"   # unequal lengths
assert error_of_sweep(lambda: cfg.sweep(paired={"robot.n": [2]})) == "robot.axes"                             # n without axes
```

## Dict, JSON and YAML

`to_dict()` holds every builder field and `defaults_version`; a mount that was not pinned is omitted (it is the task's default, applied by `resolve()`).
`from_dict()` reads it back, takes the base's value for sections and keys left out (so a hand-written file can be short), and **warns when
`defaults_version` differs** from the package's. `dump(path)` / `BaseEnvConfig.load(path)` write and read JSON; a `.yaml` / `.yml` path uses YAML,
which needs the optional extra `pip install "robotdesigner[yaml]"` (without it the error names the extra; the base install stays numpy only).

The record to keep with a run is still `resolve().to_dict()`: it holds every value, the mount included, and reproduces after the defaults change.

```python
import json, tempfile, pathlib

d = cfg.sim(num_envs=8).to_dict()
assert d["defaults_version"] == DEFAULTS_VERSION and "mount" not in d["task"]
back = BaseEnvConfig.from_dict(json.loads(json.dumps(d)))
assert back == cfg.sim(num_envs=8) and back.resolve().env_hash == cfg.sim(num_envs=8).resolve().env_hash

short = BaseEnvConfig.from_dict({"sim": {"num_envs": 4}, "robot": {"n": 2, "axes": "zy"}})   # the rest is the base
assert short.resolve().sim.num_envs == 4 and short.resolve().sim.dt == 0.002

with tempfile.TemporaryDirectory() as tmp:
    path = pathlib.Path(tmp) / "run.json"
    cfg.dump(path)
    assert BaseEnvConfig.load(path).resolve().env_hash == cfg.resolve().env_hash
```

```python no-run
cfg.dump("run.yaml")                     # needs the `yaml` extra
cfg = BaseEnvConfig.load("run.yaml")
```

## Errors

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

assert error_of(lambda: BaseEnvConfig().sim(num_env=2)).path == "sim.num_env"          # unknown field
assert error_of(lambda: BaseEnvConfig().robot(3)).path == "robot"                       # not a Chain or an arm
assert error_of(lambda: BaseEnvConfig().robot(n=3).resolve()).path == "robot.axes"      # three joints, one axis letter

bad = BaseEnvConfig().robot(n=3).sim(num_envs=0).control(mode="warp")
assert {p.path for p in bad.validate()} == {"robot.axes", "sim.num_envs", "control.mode"}   # all at once
err = error_of(bad.resolve)
assert err.path == "" and len(err.problems) == 3
assert error_of(lambda: BaseEnvConfig().with_overrides({"sim.options.zzz": 1})).path == "sim.options.zzz"
assert error_of(lambda: BaseEnvConfig().with_overrides(["nope.x=1"])).path == "nope"     # unknown section
```

## Looking at a config

```python
cfg = BaseEnvConfig().robot(n=2, axes="zy").sim(num_envs=8)
assert cfg.diff() == {"robot.n": (1, 2), "robot.axes": ("z", "zy"), "sim.num_envs": (1, 8)}   # against the pristine base
text = cfg.explain()
assert "robot.n" in text and "2  [set]" in text and "[default (task)]" in text
```

- `explain()` lists every field with its value and its source: `set`, `default` or `default (task)` (the mount).
- `diff()` returns `{path: (default, value)}` against the pristine `BaseEnvConfig()` of this defaults version; a field only one side has is `NOT_SET` on the other.

## Building

`build()` is a one-line delegate to `make_env(self.resolve())`; `make_env(cfg)` stays the one construction path and `EnvConfig` has no `make()`.

```python no-run
env = BaseEnvConfig().robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0)).sim(num_envs=8).build()
```

## The defaults of version 1

The numbers are plain choices, not recommendations (the control and simulator numbers are the README quick start's):

| section | value |
|---|---|
| robot | one `z` joint, 0.5 m, 1 kg, radius 0.03, damping 0.5, armature 0.01, friction loss 0, effort limit 50, range ±2.5 |
| control | `joint_pos_abs`, `FixedGains(kp=60, kd=6)`, `action_scale=None` |
| task | `empty/idle`, horizon 200, no tolerance or region, the provider's default mount |
| sim | `mujoco` with `MujocoOptions.default()`, 1 env, `cpu`, `dt` 0.002, `euler`, 50 Hz control, seed 0, scene `empty` |
