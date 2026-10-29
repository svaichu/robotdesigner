# TaskConfig

What the robot is asked to do. A task never mentions the number of joints or the link lengths, so the same task config
can be given to different embodiments. Importing it needs numpy only.

```python
from robotdesigner.config import (
    ConfigError, FixedBox, OwnReachable, ReachParams, RobosuiteParams, TaskConfig,
)
from robotdesigner.config.task import RESERVED_ENV_KWARGS
```

Related pages: [RobotConfig](robot.md), [ControlConfig](control.md), [SimConfig](sim.md), [EnvConfig](env.md),
[backends](../backends.md).

## `TaskConfig(id, horizon, success_tolerance, target_region, params)`

| field | type | rule |
|---|---|---|
| `id` | str | `native/reach`, or `robosuite/<EnvName>` for any robosuite environment name |
| `horizon` | int | `>= 1`; the episode is truncated after this many steps |
| `success_tolerance` | float or `None` | `> 0` for tasks defined by a goal (`native/reach`); must be `None` for `robosuite/*` |
| `target_region` | `FixedBox`, `OwnReachable` or `None` | required for `native/reach`; must be `None` for `robosuite/*` |
| `params` | the id's parameter class | `ReachParams` for `native/reach`, `RobosuiteParams` for `robosuite/*` |

`success_tolerance` and `target_region` exist only for tasks that are defined by a goal point. A robosuite task defines
its own goal and success, so both must be `None` there; passing a value is an error rather than silently ignored.

An unknown id is rejected with the list of known ones. Which backend can run which id is decided by the backend, not
here (see [backends](../backends.md#capabilities)).

## `native/reach`

Move the end effector to a target point. Runs on the native MuJoCo backend.

- **Target:** drawn at every reset from `target_region`, in the **robot's base frame**, so a region means the same thing
  wherever the base is placed.
- **Reward:** `-‖ee − target‖` (negative distance, metres), every step.
- **Success:** `‖ee − target‖ < success_tolerance`.
- **End of episode:** `truncated` when the step count reaches `horizon`; `terminated` when successful, only if
  `terminate_on_success` is true. The environment never resets itself.

### `ReachParams(reset_noise, terminate_on_success)`

| field | type | rule |
|---|---|---|
| `reset_noise` | float | `>= 0`; at reset each joint starts at `home_qpos + u`, with `u` uniform in `[-reset_noise, reset_noise]` rad, clipped to the joint range |
| `terminate_on_success` | bool | |

### Target regions

| region | targets |
|---|---|
| `FixedBox(low, high)` | uniform in an axis-aligned box, in the base frame; `low <= high` componentwise |
| `OwnReachable()` | the end-effector position of a uniformly sampled joint configuration of the arm itself (inside the joint ranges, `±pi` for unlimited joints), expressed in the base frame, so the target is always reachable |

Both are drawn by the environment's seeded generator. The reset can also inject `target`, `qpos` and `qvel` through the
reset TensorDict, which overrides the random draw (used for parity tests and replays).

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

## `robosuite/<EnvName>`

Any single-arm manipulation environment that robosuite registers: `Lift`, `Stack`, `Door`, `PickPlace`, `Wipe`,
`NutAssembly`, `ToolHang` and their variants. robosuite owns the scene, the objects, the reward and the success check;
robotdesigner supplies the robot and the control. Runs on the robosuite backend.

### `RobosuiteParams(env_kwargs, reset_noise, terminate_on_success)`

| field | type | rule |
|---|---|---|
| `env_kwargs` | mapping | keyword arguments for the robosuite environment constructor (`reward_shaping`, `table_full_size`, `single_object_mode`, ...). Keys must be argument names; values must be `bool`, `int`, `float`, `str`, `None`, lists or mappings of those. The backend rejects names the environment does not accept |
| `reset_noise` | float | `>= 0`, as for `native/reach` |
| `terminate_on_success` | bool | `terminated` when robosuite's success check passes |

The arguments that robotdesigner sets itself cannot appear in `env_kwargs`: the robot, controller, gripper, control
frequency, horizon, rendering and seed come from the robot, control, sim and task configs. The list is
`robotdesigner.config.task.RESERVED_ENV_KWARGS`.

- **Only what you list is configured.** Every other argument takes robosuite's own default. The environment records the
  complete set it ran with in `env.resolved_env_kwargs`, but those defaults are **not** part of `env_hash`.
- **Success and reward** are robosuite's. `reward_shaping` and `reward_scale` are ordinary `env_kwargs` where the
  environment has them.
- Values are stored immutably: lists become tuples, and key order does not matter.
- The random-object variants `PickPlaceSingle` and `NutAssemblySingle` are refused by the backend, because their
  observations change from episode to episode; fix the object with `single_object_mode=2` and `object_type`.

```python
lift = TaskConfig(
    id="robosuite/Lift",
    horizon=200,
    success_tolerance=None,
    target_region=None,
    params=RobosuiteParams({"reward_shaping": True, "table_full_size": [0.8, 0.8, 0.05]}, reset_noise=0.0,
                           terminate_on_success=False),
)
assert lift.params.kwargs == {"reward_shaping": True, "table_full_size": (0.8, 0.8, 0.05)}
assert TaskConfig.from_dict(lift.to_dict()) == lift
assert "robots" in RESERVED_ENV_KWARGS and "reward_shaping" not in RESERVED_ENV_KWARGS
```

## Errors

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

ok = RobosuiteParams({}, 0.0, False)
assert error_of(lambda: TaskConfig("native/lift", 10, 0.02, OwnReachable(), ReachParams(0.0, False))).path == "id"
assert error_of(lambda: TaskConfig("native/reach", 0, 0.02, OwnReachable(), ReachParams(0.0, False))).path == "horizon"
assert error_of(lambda: TaskConfig("native/reach", 10, None, OwnReachable(), ReachParams(0.0, False))).path == "success_tolerance"
assert error_of(lambda: TaskConfig("robosuite/Lift", 10, 0.02, None, ok)).path == "success_tolerance"   # robosuite defines success
assert error_of(lambda: TaskConfig("robosuite/Lift", 10, None, OwnReachable(), ok)).path == "target_region"
assert error_of(lambda: TaskConfig("robosuite/", 10, None, None, ok)).path == "id"                       # needs an environment name
assert error_of(lambda: FixedBox((1.0, 0.0, 0.0), (0.0, 1.0, 1.0))).path == "low[0]"
assert error_of(lambda: ReachParams(-1.0, False)).path == "reset_noise"
assert error_of(lambda: RobosuiteParams({"robots": "Panda"}, 0.0, False)).path == "env_kwargs"
assert error_of(lambda: RobosuiteParams({"x": object()}, 0.0, False)).path == "env_kwargs.x"
assert error_of(lambda: TaskConfig("robosuite/Lift", 10, None, None, ReachParams(0.0, False))).path == "params"
```

## Adding a task id

A task id needs a parameter class with `to_dict` / `from_dict`, registered in `robotdesigner.config.task.TASK_PARAMS`
(an exact id) or `TASK_PREFIX_PARAMS` (a prefix such as `robosuite/`). A parameter class with the class attribute
`uses_goal = False` takes `None` for `success_tolerance` and `target_region`. A backend then lists the id in its
capabilities and implements it in its environment.
