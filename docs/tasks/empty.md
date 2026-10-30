# The `empty` placeholder task

`empty/idle` is a **placeholder**, shipped so that a default environment can run: the robot stands in the simulator's scene and nothing
is asked of it. It is the one task the package names itself (decision log of the [design spec](../spec_robotconfig_env.md#9-decision-log)); real tasks come from
providers such as [robosuite](robosuite.md), or your own ([writing a task provider](writing-a-provider.md)).

```python
from robotdesigner.config import Pose, TaskConfig, get_task_provider
from robotdesigner.tasks.empty import EmptyParams
```

Related pages: [TaskConfig](../configs/task.md), [simulators](../simulators.md).

## `EmptyParams(name)`

| field | type | rule |
|---|---|---|
| `name` | str | `"idle"`, the only task |

`uses_goal` is false, so `TaskConfig` takes `success_tolerance=None` and `target_region=None`. `max_episode_steps` is the episode length; the mount
defaults (provider side) to the origin with the identity quaternion.

```python
task = TaskConfig(
    params=EmptyParams(name="idle"), max_episode_steps=100, success_tolerance=None, target_region=None, mount=Pose.identity(),
)
assert task.id == "empty/idle"
assert task.to_dict()["provider"] == "empty" and task.to_dict()["params"] == {"name": "idle"}
assert TaskConfig.from_dict(task.to_dict()) == task
assert get_task_provider("empty").default_mount("idle") == Pose.identity()
```

## What the task does

- **Scene:** adds nothing. Whatever `sim.scene` provides is all there is.
- **Observations:** the `task` group is empty; the robot's own observations are unchanged.
- **Reward** is 0, success is never set and `terminated` never fires; `truncated` fires at `max_episode_steps`.
- **Randomness:** draws nothing from the task's random stream.
- **Simulators:** `mujoco`.

## Not the `empty` scene

`sim.scene = "empty"` is a preset of the simulator (no floor, no objects); `empty/idle` is a task owned by a provider. The names coincide
and nothing else does: a robosuite task needs the `empty` scene, `empty/idle` runs on any scene. The test that keeps the core free of
task names exempts this one namespace for that reason.
