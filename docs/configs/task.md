# TaskConfig

What the robot is asked to do. The **params object is the task**: its class names the provider and its `name` field names
the task within it, so nothing is written twice and no string id is accepted anywhere. A task never mentions the number of joints or the link lengths, so the same task config
can be given to different embodiments. Importing it needs numpy only.

```python
from dataclasses import dataclass
from typing import ClassVar

from robotdesigner.config import (
    ConfigError, FixedBox, OwnReachable, Pose, TaskConfig, list_task_providers, register_task_provider,
    unregister_task_provider,
)
```

Related pages: [RobotConfig](robot.md), [ControlConfig](control.md), [SimConfig](sim.md), [EnvConfig](env.md),
[simulators](../simulators.md), and the pages of the task providers that ship with the package:
[robosuite scenes](../tasks/robosuite.md) and [the `empty` placeholder](../tasks/empty.md).

## `TaskConfig(params, max_episode_steps, success_tolerance, target_region, mount)`

| field | type | rule |
|---|---|---|
| `params` | the provider's params class | the task: the class attribute `provider` is the namespace of a **task provider**, the field `name` is the task within it, and the provider decides which names exist. Validated and hashed with the config |
| `max_episode_steps` | int | `>= 1`; the episode is truncated after this many steps. Gymnasium's name for the time limit (`gym.make(..., max_episode_steps=...)`, `TimeLimit`); it was called `horizon` before, and `from_dict` still reads that key (below) |
| `success_tolerance` | float or `None` | `> 0` when the task's params class sets `uses_goal = True`; must be `None` otherwise |
| `target_region` | `FixedBox`, `OwnReachable` or `None` | required when `uses_goal` is true; must be `None` otherwise |
| `mount` | `Pose` | the robot base in the world: where the scene expects the arm. Required; part of the task's dict and of `env_hash`, and **not** of the robot, so `robot_hash` identifies the embodiment alone |

`TaskConfig.id` is a derived, read-only property, `"<provider>/<name>"`, for messages, logs and docs; it is not a field and
not part of the dict. There is no `provider=` argument: a second argument could contradict the class.

**The old key.** `from_dict()` (and with it `EnvConfig.from_dict`, `BaseEnvConfig.from_dict`, `load`) reads the key `horizon` as `max_episode_steps` and
emits a `DeprecationWarning`; giving both keys is an error at `horizon`. Saved files keep loading. The `env_hash` of every config changed once with the rename.

`to_dict()` writes `provider` (taken from the class) next to `params` (which holds `name`), `max_episode_steps`, `success_tolerance`,
`target_region` and `mount`; `from_dict()` reads `provider` and `params.name`, asks the registry for that params class and builds
it. Hashes (`env_hash`) are over the canonical form of the dict, so key order never matters.

`success_tolerance` and `target_region` exist only for tasks that are defined by a goal point. A task that defines its own
success (an objective on a scene object, say) takes `None` for both; passing a value is an error rather than silently
ignored.

An unknown provider is rejected at `provider` (in `from_dict`) with the list of registered providers; an unknown name at
`params.name` with the list of every registered id. Which simulator can run a task is decided when the
environment is built, by the task's provider and the simulator together (see [simulators](../simulators.md#capabilities)),
never here.

## Where tasks come from

The package names exactly one task, the placeholder [`empty/idle`](../tasks/empty.md) (so that a default environment can run), and the
only real provider that ships with it is `robosuite` (arenas and objects of robosuite environments). A **task provider** owns a namespace: it names its tasks, gives each a typed params
class, checks a config against a simulator, and builds the task for it. Providers are registered, not listed:

- every subpackage of `robotdesigner.tasks` is a provider, named like the subpackage (`PROVIDER` is its entry point);
- another installed package adds one through the entry-point group `robotdesigner.task_providers`;
- `register_task_provider(namespace, provider)` adds one in the running process and takes precedence over the first two.

`list_task_providers()` returns the namespaces. Importing a provider's config side imports no simulator, torch or
robosuite, so `TaskConfig` stays numpy-only.

## Target regions

Goal tasks draw a target from a region, in the **robot's base frame**, so a region means the same wherever the base is
placed.

| region | targets |
|---|---|
| `FixedBox(low, high)` | uniform in an axis-aligned box, in the base frame; `low <= high` componentwise |
| `OwnReachable()` | the end-effector position of a uniformly sampled joint configuration of the arm itself (inside the joint ranges, `±pi` for unlimited joints), expressed in the base frame, so the target is always reachable |

Both are drawn by the task's own seeded random stream (`sim.seed`), which does not depend on the robot's number of joints,
except that `OwnReachable` samples that many joint angles by definition.

```python
box = FixedBox(low=(0.2, -0.3, 0.0), high=(0.6, 0.3, 0.4))
assert box.to_dict() == {"kind": "fixed_box", "low": [0.2, -0.3, 0.0], "high": [0.6, 0.3, 0.4]}
assert OwnReachable().to_dict() == {"kind": "own_reachable"}
```

## A task config, end to end

The page registers a toy provider so that the example needs no engine. Real providers do the same with more behind
`validate` and `build` ([writing a task provider](../tasks/writing-a-provider.md)).

```python
@dataclass(frozen=True)
class DemoParams:
    provider: ClassVar[str] = "demo"       # the namespace this class belongs to
    uses_goal: ClassVar[bool] = False      # the task defines its own success: no tolerance, no region

    name: str                              # which task of the provider
    reset_noise: float

    def to_dict(self):
        return {"name": self.name, "reset_noise": self.reset_noise}

    @classmethod
    def from_dict(cls, d):
        return cls(**d)


class DemoTasks:
    def task_names(self):
        return ("hold",)

    def params_class(self, name):
        return DemoParams if name == "hold" else None

    def default_mount(self, name):       # where this task expects the robot base
        return Pose.identity()

    def validate(self, cfg, simulator):
        return []

    def build(self, cfg, simulator):
        raise NotImplementedError("a real provider returns a tasks.base.TaskImpl here")


register_task_provider("demo", DemoTasks(), replace=True)
assert "demo" in list_task_providers()

task = TaskConfig(params=DemoParams("hold", 0.0), max_episode_steps=200, success_tolerance=None, target_region=None,
                  mount=Pose((0.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0)))
assert task.id == "demo/hold"              # derived and read-only
assert task.to_dict() == {
    "provider": "demo",
    "params": {"name": "hold", "reset_noise": 0.0},
    "max_episode_steps": 200,
    "success_tolerance": None,
    "target_region": None,
    "mount": {"pos": [0.0, 0.0, 0.5], "quat": [1.0, 0.0, 0.0, 0.0]},
}
assert TaskConfig.from_dict(task.to_dict()) == task
```

## Errors

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

ok = DemoParams("hold", 0.0)
d = task.to_dict()
assert error_of(lambda: TaskConfig.from_dict({**d, "provider": "nope"})).path == "provider"       # no such provider
assert error_of(lambda: TaskConfig(DemoParams("nope", 0.0), 10, None, None, Pose.identity())).path == "params.name"   # no such task
assert error_of(lambda: TaskConfig.from_dict({**d, "params": {**d["params"], "name": "nope"}})).path == "params.name"
import warnings

old = {k: v for k, v in d.items() if k != "max_episode_steps"} | {"horizon": 200}
with warnings.catch_warnings(record=True) as caught:
    warnings.simplefilter("always")
    assert TaskConfig.from_dict(old) == task                                  # the old key still loads ...
assert any(issubclass(w.category, DeprecationWarning) for w in caught)       # ... with a warning
assert error_of(lambda: TaskConfig.from_dict({**d, "horizon": 5})).path == "horizon"   # both keys: an error
assert error_of(lambda: TaskConfig.from_dict({**d, "id": "demo/hold"}))                                # `id` is not a field
M = Pose.identity()
assert error_of(lambda: TaskConfig(ok, 0, None, None, M)).path == "max_episode_steps"
assert error_of(lambda: TaskConfig(ok, 10, 0.02, None, M)).path == "success_tolerance"
assert error_of(lambda: TaskConfig(ok, 10, None, OwnReachable(), M)).path == "target_region"
assert error_of(lambda: TaskConfig(box, 10, None, None, M)).path == "params"   # not a provider's params object
assert error_of(lambda: TaskConfig(ok, 10, None, None, (0, 0, 0))).path == "mount"      # the mount is a Pose
assert error_of(lambda: FixedBox((1.0, 0.0, 0.0), (0.0, 1.0, 1.0))).path == "low[0]"
assert "demo/hold" in str(error_of(lambda: TaskConfig.from_dict({**d, "params": {"name": "x"}})))  # the message lists the known ids

unregister_task_provider("demo")
assert "demo" not in list_task_providers()
```

## Adding a task provider

A provider has a config side (the params class of each task, numpy only) and an engine side (a `TaskImpl`), and it registers by namespace. The walk-through,
with runnable code for the config side, is [writing a task provider](../tasks/writing-a-provider.md). `tests/toy_provider/` is the complete small provider the
engine tests use, registered under the namespace `toy` by `tests/conftest.py`; it is not part of the installed package. Copy it as a starting point.
