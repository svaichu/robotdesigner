# robosuite scenes

[robosuite](https://robosuite.ai) is a **source of task assets** here, not a simulator. `robosuite/<EnvName>` loads the
arena, the objects and the placement rule of the robosuite environment `<EnvName>` into the simulation named by
`sim.backend` (today: `mujoco`). That simulation steps everything: your robot, your control law and `SimConfig`'s time
step; robosuite's own physics options, controllers, robots and rewards are not used.

```python
from robotdesigner.config import TaskConfig
from robotdesigner.tasks.robosuite import LiftObject, ReachObject, RobosuiteParams
from robotdesigner.tasks.robosuite.params import RESERVED_ENV_KWARGS
```

Related pages: [TaskConfig](../configs/task.md), [SimConfig](../configs/sim.md), [backends](../backends.md). Needs the `robosuite` extra to build an environment; the config side below needs only numpy.

## `RobosuiteParams(env_kwargs, objective, reset_noise, terminate_on_success)`

| field | type | rule |
|---|---|---|
| `env_kwargs` | mapping | keyword arguments for the robosuite environment constructor that shape the scene (`table_full_size`, `single_object_mode`, ...). Keys must be argument names; values must be `bool`, `int`, `float`, `str`, `None`, lists or mappings of those. Names the environment does not accept are reported when the environment is built |
| `objective` | `LiftObject` or `ReachObject` | what counts as success, and which object the reward measures |
| `reset_noise` | float | `>= 0`; uniform noise on the initial joint positions, in radians |
| `terminate_on_success` | bool | `terminated` when the objective is met |

`TaskConfig` takes `success_tolerance=None` and `target_region=None` for these tasks: the objective defines success.

| objective | success | reward |
|---|---|---|
| `LiftObject(object, height)` | the object's centre is at least `height` metres above the floor plane (world z) | `-‖ee − object‖` |
| `ReachObject(object, tolerance)` | the end effector is closer than `tolerance` metres to the object's centre | `-‖ee − object‖` |

Reward shaping is deliberately basic and the same for both objectives; it is the part you will most likely want to
replace (add an objective, see [adding a task provider](../configs/task.md#adding-a-task-provider)).

```python
lift = TaskConfig(
    id="robosuite/Lift",
    horizon=200,
    success_tolerance=None,
    target_region=None,
    params=RobosuiteParams({"table_full_size": [0.8, 0.8, 0.05]}, LiftObject("cube", 0.84),
                           reset_noise=0.0, terminate_on_success=False),
)
assert lift.params.kwargs == {"table_full_size": (0.8, 0.8, 0.05)}
assert lift.to_dict()["params"]["objective"] == {"kind": "lift_object", "object": "cube", "height": 0.84}
assert TaskConfig.from_dict(lift.to_dict()) == lift
assert "robots" in RESERVED_ENV_KWARGS and "table_full_size" not in RESERVED_ENV_KWARGS
```

The arguments that robotdesigner sets itself cannot appear in `env_kwargs`: the robot, gripper, controller, control
frequency, horizon, rendering and seed come from the other config sections (`RESERVED_ENV_KWARGS`). Everything not
listed takes robosuite's own default. Values are stored immutably; key order does not matter.

## What is taken from robosuite, and what is not

| from robosuite | from robotdesigner |
|---|---|
| the arena (table, floor, walls, cameras), the objects with their geoms, masses, friction and textures | the robot, with its own links, joints, actuators and `home_qpos` |
| the rule that places the objects at reset (robosuite's own sampler: ranges, rotation, collision-free placement) | the time step, integrator, device and seed (`SimConfig`); `MujocoOptions` |
| the position where robosuite mounts a stock arm in that scene (`default_mount`) | control modes and gains (`ControlConfig`), all of the backend's modes |
| | success, reward and observations |

Not used: robosuite's controllers, its robot models and grippers (`robot.tool` is ignored by the MuJoCo backend), its
reward functions, its `<option>`, `<size>` and `<visual>` sections, and its IK target bodies. Contact physics therefore
follow MuJoCo's defaults, not the settings robosuite's worlds ship with (for example its elliptic friction cone).

## Building an environment

```python no-run
from robotdesigner import make_env
from robotdesigner.config import (ControlConfig, EnvConfig, FixedGains, MujocoOptions, SimConfig, TaskConfig, chain)
from robotdesigner.tasks.robosuite import LiftObject, RobosuiteParams, default_mount

arm = chain(4, "zyyy", total_length=0.9, total_mass=6.0, radius=0.04, damping=1.0, armature=0.05, frictionloss=0.0,
            effort_limit=80.0, range=(-2.9, 2.9), base_pose=default_mount("Lift"))   # where robosuite mounts a stock arm
cfg = EnvConfig(
    robot=arm,
    control=ControlConfig("joint_pos_abs", FixedGains(kp=60.0, kd=6.0), action_scale=None),
    task=TaskConfig("robosuite/Lift", 200, None, None,
                    RobosuiteParams({}, LiftObject("cube", 0.84), reset_noise=0.0, terminate_on_success=False)),
    sim=SimConfig("mujoco", num_envs=4, device="cpu", dt=0.002, integrator="euler", control_freq=50.0, seed=0,
                  scene="empty"),
    backend_options=MujocoOptions(self_collision=False),
)
env = make_env(cfg)
```

- **Robot placement.** `robot.base_pose` is a world pose, as for every task; the arena origin is the world origin, and
  the table top of `Lift` is at z = 0.8. `default_mount(env_name, **env_kwargs)` returns where robosuite would mount a
  stock arm in that scene (for `Lift`: `(-0.56, 0, 0.912)`); use it, or any pose, as the base of your arm.
- **Scene.** `sim.scene` must be `"empty"`: the arena brings its own floor.
- **Validation.** `make_env` reports every problem at once: a wrong scene, an unknown environment, a multi-arm
  environment, `env_kwargs` the environment does not accept, an objective naming an object that is not in the scene
  (the message lists the objects that are).
- **Observations.** `task/ee_pos` and, for every physical object of the scene, `task/<name>_pos` `[N, 3]` and
  `task/<name>_quat` `[N, 4]` (w x y z), all in the robot's base frame. Visual-only markers (such as the target bins of
  `PickPlace`) are in the scene but not observed.
- **Randomness.** The scene is built once, with robosuite's own draws (for example the size of the `Lift` cube) seeded from
  `sim.seed`. Objects with a free joint are placed by robosuite's sampler at every reset, each env with its own stream
  derived from the task's random stream, so `env.set_seed` reproduces episodes. That stream does not depend on the
  robot: arms with different numbers of joints get the same placements for the same seed and `num_envs`. Objects without a free joint (the
  door of `Door`, the target bins of `PickPlace`) are fixtures: all envs share one model, so they are placed once, at build time,
  and stay put. Objects the sampler does not place keep their pose in the model.
- **Cameras.** The arena's cameras are in the model, with the prefix `rs_`: `env.render(camera="rs_agentview")`.
- **Versions.** robosuite 1.5.x as released fails on MuJoCo 3.10 and later while an environment is built.
  `robotdesigner.tasks.robosuite.apply_compat()` patches the two affected code paths in the running process, applies
  only what the installed MuJoCo needs, and is called for you. Call it yourself in worker processes started with `spawn`.
  Verified with robosuite 1.5.1; the placement code has a path for 1.5.2's own generators that has not been run.

## Which environments work

Tried on robosuite 1.5.1 with a three-link arm, a `ReachObject` objective on the first physical object, 10 random steps
and `check_env_specs`:

| environment | physical objects | result |
|---|---|---|
| `Lift` | `cube` | works |
| `Stack` | `cubeA`, `cubeB` | works |
| `NutAssembly`, `NutAssemblySquare`, `NutAssemblyRound` | `SquareNut`, `RoundNut` | work |
| `PickPlace` (with `single_object_mode=2, object_type="can"`), `PickPlaceBread`, `PickPlaceCan`, `PickPlaceCereal`, `PickPlaceMilk` | `Milk`, `Bread`, `Cereal`, `Can` | work |
| `Door` | `Door` | works; the door is a fixture |
| `ToolHang` | `stand`, `frame`, `tool` | works |
| `NutAssemblySingle`, `PickPlaceSingle` | | refused: the object changes every episode, so the observation keys would; fix it with `single_object_mode=2` and `object_type`, or use `NutAssemblySquare` / `NutAssemblyRound` |
| `Wipe` | none | no object for either objective; needs an objective of its own |
| two-arm environments | | refused: the robot is one arm |

## Errors

```python
def error_of(build):
    try:
        build()
    except Exception as e:
        return e
    raise AssertionError("no error")

objective = ReachObject("cube", 0.05)
ok = RobosuiteParams({}, objective, 0.0, False)
assert error_of(lambda: TaskConfig("robosuite/", 10, None, None, ok)).path == "id"                   # needs an environment name
assert error_of(lambda: TaskConfig("robosuite/Lift", 10, 0.02, None, ok)).path == "success_tolerance"  # the objective defines success
assert error_of(lambda: TaskConfig("robosuite/Lift", 10, None, None, objective)).path == "params"
assert error_of(lambda: RobosuiteParams({"robots": "Panda"}, objective, 0.0, False)).path == "env_kwargs"
assert error_of(lambda: RobosuiteParams({"x": object()}, objective, 0.0, False)).path == "env_kwargs.x"
assert error_of(lambda: RobosuiteParams({}, "lift", 0.0, False)).path == "objective"
assert error_of(lambda: ReachObject("cube", 0.0)).path == "tolerance"
```
