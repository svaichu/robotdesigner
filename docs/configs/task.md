# TaskConfig

What the robot is asked to do. A task never mentions the number of joints or the link lengths, so the same task config
can be given to different embodiments. Importing it needs numpy only.

```python
from dataclasses import dataclass

from robotdesigner.config import (
    ConfigError, FixedBox, OwnReachable, TaskConfig, list_task_providers, register_task_provider,
    unregister_task_provider,
)
```

Related pages: [RobotConfig](robot.md), [ControlConfig](control.md), [SimConfig](sim.md), [EnvConfig](env.md),
[simulators](../simulators.md), and the page of the task provider that ships with the package:
[robosuite scenes](../tasks/robosuite.md).

## `TaskConfig(id, horizon, success_tolerance, target_region, params)`

| field | type | rule |
|---|---|---|
| `id` | str | `"<namespace>/<name>"`. The namespace selects a **task provider**, which decides what names exist |
| `horizon` | int | `>= 1`; the episode is truncated after this many steps |
| `success_tolerance` | float or `None` | `> 0` when the task's params class sets `uses_goal = True`; must be `None` otherwise |
| `target_region` | `FixedBox`, `OwnReachable` or `None` | required when `uses_goal` is true; must be `None` otherwise |
| `params` | the provider's params class for this id | validated and hashed with the config |

`success_tolerance` and `target_region` exist only for tasks that are defined by a goal point. A task that defines its own
success (an objective on a scene object, say) takes `None` for both; passing a value is an error rather than silently
ignored.

An unknown id is rejected with the list of every registered id. Which simulator can run a task is decided when the
environment is built, by the task's provider and the simulator together (see [simulators](../simulators.md#capabilities)),
never here.

## Where task ids come from

The package itself knows no task by name, and the one provider that ships with it is `robosuite` (arenas and objects of
robosuite environments). A **task provider** owns a namespace: it names its tasks, gives each a typed params
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
`validate` and `build` ([adding a task provider](#adding-a-task-provider)).

```python
@dataclass(frozen=True)
class DemoParams:
    uses_goal = False                      # the task defines its own success: no tolerance, no region

    reset_noise: float

    def to_dict(self):
        return {"reset_noise": self.reset_noise}

    @classmethod
    def from_dict(cls, d):
        return cls(**d)


class DemoTasks:
    def task_names(self):
        return ("hold",)

    def params_class(self, name):
        return DemoParams if name == "hold" else None

    def validate(self, cfg, simulator):
        return []

    def build(self, cfg, simulator):
        raise NotImplementedError("a real provider returns a tasks.base.TaskImpl here")


register_task_provider("demo", DemoTasks(), replace=True)
assert "demo" in list_task_providers()

task = TaskConfig(id="demo/hold", horizon=200, success_tolerance=None, target_region=None, params=DemoParams(0.0))
assert task.to_dict() == {
    "id": "demo/hold",
    "horizon": 200,
    "success_tolerance": None,
    "target_region": None,
    "params": {"reset_noise": 0.0},
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

ok = DemoParams(0.0)
assert error_of(lambda: TaskConfig("nope/hold", 10, None, None, ok)).path == "id"       # no such provider
assert error_of(lambda: TaskConfig("demo/nope", 10, None, None, ok)).path == "id"       # the provider has no such task
assert error_of(lambda: TaskConfig("demo", 10, None, None, ok)).path == "id"            # not <namespace>/<name>
assert error_of(lambda: TaskConfig("demo/hold", 0, None, None, ok)).path == "horizon"
assert error_of(lambda: TaskConfig("demo/hold", 10, 0.02, None, ok)).path == "success_tolerance"
assert error_of(lambda: TaskConfig("demo/hold", 10, None, OwnReachable(), ok)).path == "target_region"
assert error_of(lambda: TaskConfig("demo/hold", 10, None, None, box)).path == "params"   # the wrong params class
assert error_of(lambda: FixedBox((1.0, 0.0, 0.0), (0.0, 1.0, 1.0))).path == "low[0]"
assert "demo/hold" in str(error_of(lambda: TaskConfig("nope/hold", 10, None, None, ok)))   # the message lists the known ids

unregister_task_provider("demo")
assert "demo" not in list_task_providers()
```

## Adding a task provider

A provider has two halves, kept apart so that the config layer never imports an engine.

1. **Config side.** One params class per task: a frozen dataclass with `to_dict()` and `from_dict()`, validated in
   `__post_init__`, with the class attribute `uses_goal` (`True` if the task is defined by a target region and a
   tolerance). If the params have `reset_noise` (radians of uniform noise on the initial joint positions) or
   `terminate_on_success`, the environment applies them. Use `robotdesigner.config._canon.freeze` / `thaw` for free-form
   mappings.
2. **Provider object.** `task_names()` (names or patterns, for error messages), `params_class(name)`,
   `validate(cfg, simulator) -> list[Problem]` (every reason this task cannot run on that simulator, collected with the
   simulator's own problems into one `ConfigError`) and `build(cfg, simulator)`, which imports the engine and returns the
   task.
3. **Engine side.** A `robotdesigner.tasks.base.TaskImpl` (subclass `BaseTask` to skip the optional hooks): `attach(spec)`
   adds scene elements to the MuJoCo spec before it is compiled, `bind(sim)` resolves names to ids, `observation_spec`
   and `observe` give the `task` observation group, `reset` places the task for the masked envs, `success` and `reward`
   score a step. The robot, the control law, batching, masked resets, episode counters and rendering belong to the
   environment, so a task states only what it adds.
4. **Register.** A subpackage of `robotdesigner.tasks` exposing `PROVIDER`, or an entry point in the group
   `robotdesigner.task_providers` in another package (`name = "your.package:PROVIDER"`), or `register_task_provider`.

Keep the module that defines the provider free of engine imports (put them inside `validate` and `build`, or in the
module that `build` imports). Test it with `check_env_specs`, a masked-reset test and a seeding test.

`tests/toy_provider/` is a complete small provider, a goal task that moves the end effector to a target point
(`params.py` is the config side, `reach.py` the engine side, `__init__.py` the provider object). The engine tests use it,
registered under the namespace `toy` by `tests/conftest.py`; it is not part of the installed package. Copy it as a
starting point.
