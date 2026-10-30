# Native tasks

The provider `robotdesigner.tasks.native` holds the tasks that need no third-party assets. Today that is one task,
`reach`, which runs on the [MuJoCo backend](../backends.md). Its params class lives next to the provider, not in the
core config:

```python
from robotdesigner.config import FixedBox, OwnReachable, TaskConfig
from robotdesigner.tasks.native import ReachParams, reach_task
```

Related pages: [TaskConfig](../configs/task.md), [backends](../backends.md), [robosuite scenes](robosuite.md).

## `reach`

Move the end effector to a target point.

- **Target:** drawn at every reset from `target_region`, in the **robot's base frame**.
- **Reward:** `-‖ee − target‖` (negative distance, metres), every step.
- **Success:** `‖ee − target‖ < success_tolerance`.
- **End of episode:** `truncated` when the step count reaches `horizon`; `terminated` when successful, only if
  `terminate_on_success` is true. The environment never resets itself.

| field | type | rule |
|---|---|---|
| `id` | str | `native/reach` |
| `success_tolerance` | float | `> 0` (a goal task) |
| `target_region` | `FixedBox` or `OwnReachable` | required, see [target regions](../configs/task.md#target-regions) |
| `params` | `ReachParams` | below |

### `ReachParams(reset_noise, terminate_on_success)`

| field | type | rule |
|---|---|---|
| `reset_noise` | float | `>= 0`; at reset each joint starts at `home_qpos + u`, with `u` uniform in `[-reset_noise, reset_noise]` rad, clipped to the joint range |
| `terminate_on_success` | bool | |

The reset can also inject `target`, `qpos` and `qvel` through the reset TensorDict, which overrides the random draw
(used for parity tests and replays).

```python
reach = TaskConfig(
    id="native/reach",
    horizon=200,
    success_tolerance=0.02,
    target_region=FixedBox(low=(0.2, -0.3, 0.0), high=(0.6, 0.3, 0.4)),
    params=ReachParams(reset_noise=0.0, terminate_on_success=False),
)
assert reach.to_dict() == {
    "id": "native/reach",
    "horizon": 200,
    "success_tolerance": 0.02,
    "target_region": {"kind": "fixed_box", "low": [0.2, -0.3, 0.0], "high": [0.6, 0.3, 0.4]},
    "params": {"reset_noise": 0.0, "terminate_on_success": False},
}
assert TaskConfig.from_dict(reach.to_dict()) == reach

own = TaskConfig("native/reach", 200, 0.02, OwnReachable(), ReachParams(0.05, True))
assert own.to_dict()["target_region"] == {"kind": "own_reachable"}
```

`reach_task(horizon, success_tolerance, target_region, reset_noise, terminate_on_success)` builds the same `TaskConfig`
without spelling the id, which is what the other pages use:

```python
assert reach_task(200, 0.02, FixedBox(low=(0.2, -0.3, 0.0), high=(0.6, 0.3, 0.4)), 0.0, False) == reach
assert reach_task(200, 0.02, OwnReachable(), 0.05, True) == own
```

### Observations

| key | shape | meaning |
|---|---|---|
| `task/ee_pos` | `[N, 3]` | end effector, in the robot's base frame |
| `task/target_pos` | `[N, 3]` | target, in the base frame |

`env.render(...)` draws the target (a sphere of radius `success_tolerance` with a cross-hair, red until reached, then
green) and, for a `FixedBox` region, its edges as a wireframe box.

## Errors

```python
def error_of(build):
    try:
        build()
    except Exception as e:
        return e
    raise AssertionError("no error")

assert error_of(lambda: TaskConfig("native/lift", 10, 0.02, OwnReachable(), ReachParams(0.0, False))).path == "id"
assert error_of(lambda: TaskConfig("native/reach", 0, 0.02, OwnReachable(), ReachParams(0.0, False))).path == "horizon"
assert error_of(lambda: TaskConfig("native/reach", 10, None, OwnReachable(), ReachParams(0.0, False))).path == "success_tolerance"
assert error_of(lambda: TaskConfig("native/reach", 10, 0.02, None, ReachParams(0.0, False))).path == "target_region"
assert error_of(lambda: ReachParams(-1.0, False)).path == "reset_noise"
```
