# The builder: `BaseEnvConfig`

`EnvConfig` requires every value. `BaseEnvConfig` is the layer on top of it for people: a **complete, runnable default** that you refine by saying only what differs.
It is the one place in the package that holds defaults, and it ends in a frozen `EnvConfig` (`resolve()`) and an environment (`build()`).

```python
import os
import tempfile

from robotdesigner.config import BaseEnvConfig, Chain, ConfigError, EnvConfig, FixedGains, Pose

base = BaseEnvConfig()                                   # a one-link arm doing empty/idle on the CPU MuJoCo simulator
cfg = (
    base
    .robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0))   # written fields change, the rest carries over
    .control(gains=FixedGains(kp=60.0, kd=6.0))
    .sim(num_envs=8, seed=1)
)
env_cfg = cfg.resolve()                                  # the frozen, validated EnvConfig
assert isinstance(env_cfg, EnvConfig) and env_cfg.robot.n == 3 and env_cfg.sim.num_envs == 8
assert base.resolve().robot.n == 1                       # every call returns a new config; the receiver is unchanged
```

`cfg.build()` is `make_env(cfg.resolve())`: it is the only call that imports an engine.

## Defaults

`BaseEnvConfig()` (defaults version 2). These are plain choices to start from, not recommendations, and none of them is hidden: each is listed here and in `src/robotdesigner/config/defaults.py`.

| section | default |
|---|---|
| robot | `Chain(n=1, axes="z", total_length=0.5, total_mass=1.0, radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0, range=(-2.5, 2.5))` |
| control | `joint_pos_abs`, `FixedGains(kp=60.0, kd=6.0)`, `action_scale=None` |
| task | `empty/idle`, `max_episode_steps=200`, no goal fields; the mount is the task's default mount |
| sim | `mujoco`, 1 env, `cpu`, `dt=0.002`, `euler`, `control_freq=50.0`, `seed=0`, scene `empty`, the simulator's default options |

Changing a default changes the `env_hash` of every config that relied on it, so `DEFAULTS_VERSION` is raised with it. The version is written into every file
`to_dict()` makes, and loading a file from another version warns.

## Refining: `robot`, `control`, `task`, `sim`

Each section call takes either a whole object of that section or keyword fields, writes only what you give, and returns a **new** config.

| call | takes | notes |
|---|---|---|
| `.robot(...)` | a `Chain`, a ready `ParametricArm`, or `Chain` fields as keywords | a `Chain` is merged field by field. Writing `total_length` clears `link_length` (and `total_mass` clears `mass`), and the other way round. A ready arm replaces the section; a later `.robot(n=...)` on it starts from the default chain again |
| `.control(...)` | a `ControlConfig` or `mode`, `gains`, `action_scale` | switching to a scaled mode needs `action_scale=` in the same chain of calls, or `resolve()` reports it |
| `.task(...)` | a provider's params object, a `TaskConfig`, or `max_episode_steps`, `success_tolerance`, `target_region`, `mount` | giving a params object of a **different class** resets `success_tolerance` and `target_region`, which belong to goal tasks, unless you write them |
| `.sim(...)` | a `SimConfig` or any `SimConfig` field | changing `simulator` resets `options` to that simulator's `default()` unless `options=` is given |

`Chain` is `chain()` with every argument optional (`NOT_SET` marks "not written"); `Chain.arm()` builds the arm and raises if a required value is missing.

```python
five = cfg.robot(Chain(n=5, axes="zyyyy"))                     # total_length and total_mass carry over
assert five.resolve().robot.n == 5 and cfg.resolve().robot.n == 3
swapped = cfg.robot(link_length=0.2)                           # the total is cleared: exactly one of the pair stays
assert swapped.resolve().robot.segments[0].link.tip_offset[0] == 0.2
```

### The mount

Unless you pin one with `.task(mount=Pose(...))`, `resolve()` uses `default_mount(name)` of the task's provider, so the arm stands where the scene expects it.
`to_dict()` leaves an unpinned mount out, which keeps a stored file valid when the provider's idea of the default changes. Pin it when the exact
position matters to your result.

```python
pinned = cfg.task(mount=Pose((0.1, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
assert pinned.resolve().task.mount.pos == (0.1, 0.0, 0.0)
assert "mount" not in cfg.to_dict()["task"] and "mount" in pinned.to_dict()["task"]
```

## Checking: `validate`, `resolve`

`validate()` returns every problem of the config as `Problem(path, message)` without raising; `resolve()` raises one `ConfigError` that lists them all (`.problems` holds
the list). Paths are dotted and rooted in the section: `robot.axes`, `control.action_scale`, `sim.control_freq`.

```python
bad = cfg.robot(axes="zy").control(mode="joint_vel").sim(control_freq=40.0)
problems = bad.validate()
assert {p.path for p in problems} == {"robot.axes", "control.action_scale", "sim.control_freq"}
try:
    bad.resolve()
except ConfigError as e:
    assert len(e.problems) == 3
```

`validate` checks the config only. What the installed simulator or the task's provider cannot run is reported by `make_env` ([simulators](../simulators.md#capabilities-and-validation)).

## Overrides by path

`with_overrides` applies dotted paths with the semantics of the section calls (they accumulate, and each touched section is rebuilt and validated by its own classes).
It takes a mapping or a list of `"path=value"` strings, which is what a command line gives you; the value is parsed as a JSON literal and kept as a string if it is not one.
Paths reach nested fields: `sim.options.self_collision`, `task.params.<field>`, `control.gains.kp`, `robot.total_length`. An unknown key raises `ConfigError` with the path.

```python
big = cfg.with_overrides(["sim.num_envs=64", "sim.options.self_collision=true", "robot.axes=zyy"])
assert big.resolve().sim.num_envs == 64 and big.resolve().sim.options.self_collision is True
assert cfg.with_overrides({"robot.total_length": 1.2}).resolve().robot.segments[0].link.tip_offset[0] == 1.2 / 3
try:
    cfg.with_overrides(["sim.num_env=2"])
except ConfigError as e:
    assert e.path == "sim.num_env"
```

## Families of embodiments: `sweep`

```python
family = cfg.sweep(
    paired={"robot.n": [2, 3, 4], "robot.axes": ["zy", "zyy", "zyyy"]},   # zipped: equal lengths, varied together
    grid={"sim.seed": range(2)},                                           # Cartesian product, multiplied with the paired group
)
assert len(family) == 6
assert [row["robot.n"] for row in family.table()][:2] == [2, 2]            # paired group first, the last grid axis fastest
assert all(abs(sum(s.link.mass for s in e.robot.segments) - 3.0) < 1e-12 for e in family.configs())   # total mass is held fixed
```

A `Family` is a list of `BaseEnvConfig` (`len`, iteration, indexing) with `configs()` (the resolved `EnvConfig`s), `overrides()` and `table()` (one row per item: index, the
override values and `env_hash`). Items whose resolved `env_hash` repeats are dropped, and `Family.notes` says which. An invalid item raises at expansion and names its index and
overrides. A sweep only **expands**: it launches nothing. Running a family on many processes or nodes is [cluster](../cluster.md).

## Inspecting: `explain`, `diff`

`explain()` returns a table of every field with its value and where it came from (`set`, `default`, or `default (task)` for an unpinned mount). `diff()` returns
`{path: (default, value)}` against a pristine `BaseEnvConfig()`.

```python
assert set(cfg.diff()) >= {"robot.n", "robot.axes", "robot.total_length", "sim.num_envs", "sim.seed"}
assert "robot.n" in cfg.explain() and "[set]" in cfg.explain()
```

## Files and reproducibility

`to_dict()` writes the builder's fields (with `defaults_version`); `from_dict` is the inverse and accepts **short** files: sections and keys left out take the default.
`dump(path)` and `load(path)` use JSON, or YAML for `.yaml` / `.yml` with the `yaml` extra (`pip install "robotdesigner[yaml]"`).

```python
with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, "run.json")
    cfg.dump(path)
    assert BaseEnvConfig.load(path) == cfg
assert BaseEnvConfig.from_dict({"sim": {"num_envs": 4}}).resolve().sim.num_envs == 4
```

**Store the resolved dict with every run.** `cfg.resolve().to_dict()` holds every value, rebuilds with `EnvConfig.from_dict`, and keeps meaning the same after a default
changes; the builder file, being short, does not. The `env_hash` of the resolved config is the run's identity.
