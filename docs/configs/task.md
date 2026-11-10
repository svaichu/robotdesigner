# Task: `TaskConfig`

A task says what the world adds to the robot: scene objects, resets, observations, success and reward. Which tasks exist is decided by **task providers**,
never by the core package. `TaskConfig` never mentions the number of joints or link lengths, so the same task object can be given to any embodiment.

```python
from robotdesigner.config import (
    BaseEnvConfig, ConfigError, FixedBox, MobileMount, Mount, OwnReachable, Pose, TaskConfig, get_task_provider, list_task_providers,
    snapped, world_mount,
)
from robotdesigner.tasks.empty import EmptyParams
```

## Fields

`TaskConfig(params, max_episode_steps, success_tolerance, target_region, mount)`; every field is required.

| field | meaning |
|---|---|
| `params` | the provider's params object. **It is the task**: its class says which provider, its `name` field says which task |
| `max_episode_steps` | episode length; the environment sets `truncated` after this many steps (Gymnasium's name for a time limit). `>= 1` |
| `success_tolerance`, `target_region` | the goal of a *goal task* (below); `None` for every other task |
| `mount` | `Mount(offset)` for a fixed-base arm, `MobileMount(spawn, heading)` for a floating-base robot: the base relative to the scene's mount frame. Target regions are in the robot's base frame and follow it |

`task.id` is derived, `"<provider>/<name>"`, and used in messages and logs; it is never stored (the dict holds `provider` and `params.name`).

```python
idle = TaskConfig(params=EmptyParams(name="idle"), max_episode_steps=200, success_tolerance=None, target_region=None,
                  mount=Mount())
assert idle.id == "empty/idle"
assert idle.to_dict()["provider"] == "empty"
assert TaskConfig.from_dict(idle.to_dict()) == idle      # the provider and the name pick the params class on the way back in
```

The old key `horizon` is still read as `max_episode_steps`, with a `DeprecationWarning`; giving both is an error.

### Mount

The arm has no pose. The scene owns where a robot goes: `get_task_provider(ns).mount_frame(params)` is the scene's **mount frame**, a world pose that depends on the
task's params (a table top at z = 0.8, a bench at the origin; for `empty/idle`, the world origin). `mount` holds only what the task adds to it, so the same task
config puts any embodiment in the same place in every scene:

- `Mount(offset=Pose(...))`, for a fixed-base arm. `Mount()` snaps the base exactly onto the frame; `offset` is expressed in the frame's axes.
- `MobileMount(spawn=(x, y), heading=0.0)`, for a robot with a floating base (a legged robot): where it starts, in the frame's axes, and which way it faces
  (a yaw in radians about the frame's +z; 0 faces the frame's +x). `spawn` may also be `(x, y, z)`: `z` is added to the robot's own home height, so the robot
  can start in the air. A `Mount` on a floating-base robot, or a `MobileMount` on a fixed-base one, is an error at `task.mount`.

`snapped(robot)` is the mount that fits a robot (`Mount()` or `MobileMount()`): `.task(mount=snapped(robot))` is what a script over many robots writes.

`world_mount(cfg)` (`cfg` an `EnvConfig` or a `TaskConfig`) is the only place that composes them: `T_world_base = T_frame @ T_mount`. The simulators, `viz` and the
kinematics oracle all use it. A `Pose` at `task.mount` (the old absolute form) is rejected with a message that shows the replacement; it cannot be converted without
the scene.

```python
box_task = TaskConfig(EmptyParams("idle"), 10, None, None, Mount(offset=Pose((0.1, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))))
assert world_mount(box_task) == Pose((0.1, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))   # the empty scene's frame is the origin
assert MobileMount(spawn=(1.0, 2.0), heading=1.0).spawn == (1.0, 2.0, 0.0)
assert snapped(BaseEnvConfig().resolve().robot) == Mount()          # a fixed-base arm; a legged robot gets MobileMount()
```

### Goal tasks

A params class with the class attribute `uses_goal = True` defines its success by a region of targets and a distance:

- `success_tolerance` (metres, `> 0`) and `target_region` are then **required**;
- otherwise (`uses_goal = False`) they must be `None`, because the task defines its own success.

`target_region` is one of two values, both in the robot's base frame:

| region | targets |
|---|---|
| `FixedBox(low, high)` | uniform in an axis-aligned box; `low <= high` componentwise |
| `OwnReachable()` | end-effector positions of uniformly sampled joint configurations of **that** arm, so a short arm is never given an unreachable target |

```python
box = FixedBox(low=(0.2, -0.3, -0.2), high=(0.6, 0.3, 0.2))
assert FixedBox.from_dict(box.to_dict()) == box and OwnReachable().to_dict() == {"kind": "own_reachable"}
```

The two providers shipped with the package (`empty`, `robosuite`) are not goal tasks. Goal tasks come from providers you add
([writing a provider](../tasks/writing-a-provider.md) builds one, `mine/reach`, end to end).

## Where tasks come from

A provider owns a namespace (`[a-z][a-z0-9_]*`) and offers `task_names()`, `params_class(name)`, `mount_frame(params)`, `validate(cfg, simulator)` and
`build(cfg, simulator)` (the `TaskProvider` protocol). Providers are found lazily, from three sources:

1. `register_task_provider(namespace, provider, replace=False)`: explicit, also what tests use; it wins over the two discovered sources;
2. the subpackages of `robotdesigner.tasks`: the package name is the namespace and the package exposes `PROVIDER`;
3. the entry-point group `robotdesigner.task_providers` (entry-point name = namespace, value `"module:attr"`): how another project brings its own tasks.
   If an entry point reuses the namespace of a subpackage, the subpackage is the one that loads.

```python
assert {"empty", "robosuite"} <= set(list_task_providers())          # lists the namespaces; loads none of them
provider = get_task_provider("empty")
assert provider.task_names() == ("idle",) and provider.params_class("idle") is EmptyParams
```

| provider | tasks | page |
|---|---|---|
| `empty` | `empty/idle`: the robot alone, nothing to do (a placeholder, so that a default environment can run) | [empty](../tasks/empty.md) |
| `robosuite` | `robosuite/<EnvName>`: robosuite's arena and objects, our simulator, our objective | [robosuite](../tasks/robosuite.md) |

Importing a provider's module and its params class imports no simulator. The engine side (torch, MuJoCo, robosuite) loads when an environment is built.
The registry is **per process**: a worker process must register custom providers itself (see [cluster](../cluster.md)).

## Errors

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

assert error_of(lambda: TaskConfig(EmptyParams("idle"), 0, None, None, Mount())).path == "max_episode_steps"
assert error_of(lambda: TaskConfig(EmptyParams("idle"), 10, 0.1, None, Mount())).path == "success_tolerance"   # a non-goal task has none
assert error_of(lambda: TaskConfig(EmptyParams("idle"), 10, None, box, Mount())).path == "target_region"
assert error_of(lambda: TaskConfig("empty/idle", 10, None, None, Mount())).path == "params"                   # not a params object
assert error_of(lambda: TaskConfig(EmptyParams("idle"), 10, None, None, None)).path == "mount"
assert error_of(lambda: TaskConfig(EmptyParams("idle"), 10, None, None, Pose.identity())).path == "mount"   # an absolute Pose is the old form
assert error_of(lambda: EmptyParams("walk")).path == "name"
assert error_of(lambda: FixedBox((1, 0, 0), (0, 1, 1))).path == "low[0]"
assert error_of(lambda: TaskConfig.from_dict({**idle.to_dict(), "provider": "nowhere"})).path == "provider"
```
