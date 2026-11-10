# robosuite scenes: `robosuite/<EnvName>`

[robosuite](https://robosuite.ai) is a **source of task assets** here, never a simulator. `robosuite/<EnvName>` takes the arena, the objects and the
placement rule of the robosuite environment `<EnvName>` and puts them into the simulation named by `sim.simulator` (`mujoco` or `warp`). That simulator steps everything with
**your** robot, **your** control law and `SimConfig`'s time step. robosuite's controllers, robot models, grippers, reward functions, physics options
(`<option>`, `<size>`, `<visual>`) and IK target bodies are not used, so contact physics follow MuJoCo's defaults rather than the settings robosuite's worlds ship with.

Needs the `robosuite` extra to build an environment (`pip install -e ".[env,robosuite]"`, robosuite 1.5.x). The config side on this page needs only numpy.

```python
from robotdesigner.config import ConfigError, Mount, Pose, TaskConfig
from robotdesigner.tasks.robosuite import LiftObject, ReachObject, RobosuiteParams
from robotdesigner.tasks.robosuite.params import RESERVED_ENV_KWARGS
```

## `RobosuiteParams(name, env_kwargs, objective, reset_noise, terminate_on_success)`

The class names the provider; `name` is the robosuite environment (`"Lift"`), so `TaskConfig.id` is `"robosuite/Lift"`.

| field | rule |
|---|---|
| `name` | non-empty; whether such an environment exists is checked by `make_env` |
| `env_kwargs` | constructor arguments of the robosuite environment that shape the scene (`table_full_size`, `single_object_mode`, ...). Keys are argument names; values are `bool`, `int`, `float`, `str`, `None`, or lists / mappings of those. Stored immutably, key order does not matter. Anything not listed takes robosuite's default |
| `objective` | `LiftObject` or `ReachObject`: what counts as success and what the reward measures |
| `reset_noise` | `>= 0`; uniform noise (rad) on the initial joint positions |
| `terminate_on_success` | set `terminated` when the objective is met |

The arguments robotdesigner sets itself cannot appear in `env_kwargs` (`RESERVED_ENV_KWARGS`): robots, grippers, controller, control frequency, horizon, seed and everything about rendering.
`TaskConfig` takes `success_tolerance=None` and `target_region=None`: the objective defines success.

| objective | success | reward |
|---|---|---|
| `LiftObject(object, height)` | the object's centre is at least `height` metres above the world floor plane (z) | `-‖ee − object‖` |
| `ReachObject(object, tolerance)` | the end effector is closer than `tolerance` metres to the object's centre | `-‖ee − object‖` |

The reward is deliberately basic and the same for both objectives. It is the part you will most likely replace: add an objective, or write your own provider
([writing a provider](writing-a-provider.md)).

```python
lift = TaskConfig(
    params=RobosuiteParams(name="Lift", env_kwargs={"table_full_size": [0.8, 0.8, 0.05]},
                           objective=LiftObject("cube", 0.84), reset_noise=0.0, terminate_on_success=False),
    max_episode_steps=200, success_tolerance=None, target_region=None,
    mount=Mount(),                                            # snapped onto the scene's mount frame (below)
)
assert lift.id == "robosuite/Lift"
assert lift.params.kwargs == {"table_full_size": (0.8, 0.8, 0.05)}
assert lift.to_dict()["params"]["objective"] == {"kind": "lift_object", "object": "cube", "height": 0.84}
assert TaskConfig.from_dict(lift.to_dict()) == lift
assert "robots" in RESERVED_ENV_KWARGS and "table_full_size" not in RESERVED_ENV_KWARGS
```

## Building an environment

```python no-run
from robotdesigner import make_env
from robotdesigner.config import BaseEnvConfig, Chain
from robotdesigner.tasks.robosuite import LiftObject, RobosuiteParams

cfg = (
    BaseEnvConfig()
    .robot(Chain(n=4, axes="zyyy", total_length=0.9, total_mass=6.0))
    .task(RobosuiteParams("Lift", {}, LiftObject("cube", 0.84), reset_noise=0.0, terminate_on_success=False))   # the mount defaults to robosuite's
    .sim(num_envs=4)
)
env = cfg.build()
```

- **Placement.** The arena origin is the world origin (the table top of `Lift` is at z = 0.8). The scene's **mount frame** is where robosuite mounts a stock arm in that scene
  (for `Lift`: `(-0.56, 0, 0.912)`, identity orientation). It is the full pose of the stock Panda's root body, a property of that reference robot and not of the robot you attach, read from
  robosuite's own `base_pos` and `base_ori`. `PROVIDER.mount_frame(params)` returns it, follows `env_kwargs` (a longer table moves it by half the change in length) and not `sim.seed`, and is
  memoised per scene, so a scene costs at most one extra robosuite build. The compiled model carries it as the site `rs_mount` (`TaskImpl.mount_site()`), and the compile snaps the robot base onto
  it. `task.mount` is relative to it: `Mount()` snaps exactly ([task](../configs/task.md#mount)). Resolving a config does not read the frame, so it needs no robosuite.
  The stock robot's pedestal and controller box are part of its model and are **not** recreated: the base is a welded anchor and renders show it floating. That is a modelling choice, so that
  every embodiment sees the same scene. Only single-arm environments are accepted, as before.
- **Scene.** `sim.scene` must be `"empty"` (the default): the arena brings its own floor.
- **Validation.** `make_env` reports every problem at once: a wrong scene, an unknown environment, a multi-arm environment, `env_kwargs` the environment does not accept,
  an objective that names an object not in the scene (the message lists the objects that are), and environments that pick a different object each episode (below).
- **Observations** (the `task` group, all in the robot's **base frame**): `ee_pos` `[N, 3]`, and for every physical object `<name>_pos` `[N, 3]` and `<name>_quat` `[N, 4]` (w x y z).
  Visual-only markers (the target bins of `PickPlace`) are in the scene but not observed.
- **Randomness.** The scene is built once, with robosuite's own construction-time draws (the size of the `Lift` cube) seeded from `sim.seed`. Objects with a free joint are placed by robosuite's
  sampler at every reset, from the task's own stream, so the **same seed and `num_envs` give every embodiment the same placements**, whatever its number of joints, and `env.set_seed` reproduces episodes.
  Objects without a free joint (the door of `Door`, the bins of `PickPlace`) are *fixtures*: all envs share one model, so they are placed once when the environment is built and then stay put.
- **A robot from a library.** A Menagerie arm works as it is: `BaseEnvConfig().robot(PROVIDER.describe("franka_emika_panda")).task(RobosuiteParams("Lift", ...), mount=Mount())` builds, steps and places
  the cube exactly as for a `ParametricArm` at the same mount and seed. `ee_pos` is the robot's end effector. The CPU simulator builds no gripper action for an external robot, so the fingers are held
  and `Lift` cannot be solved; it runs. A robot with `end_effector=None` (a quadruped) is refused with a `Problem` at `robot.end_effector`. See [the robot provider](../robots/menagerie.md).
- **Cameras.** The arena's cameras are in the model with the prefix `rs_`: `env.render(camera="rs_agentview")`.
- **Names.** Everything attached from the scene starts with `rs_` in the compiled model.

## Versions

robosuite 1.5.x as released fails on MuJoCo >= 3.10 while an environment is built (a joint-address comparison and the mass-matrix call).
`robotdesigner.tasks.robosuite.apply_compat()` patches the affected code in the running process, only as far as the installed MuJoCo needs, and the provider calls it for you.
Call it yourself in worker processes started with `spawn`. It returns a `CompatReport` saying what it patched.
Dynamics differ slightly between MuJoCo versions (a 60-step trajectory differed by up to 4e-3 rad between 3.9 and 3.17), so **fix the MuJoCo version of a study**.
The placement code was verified with robosuite 1.5.1; its path for the sampler generators of 1.5.2 exists but has not been exercised here.

## Which environments work

Recorded results, from a three-link arm with a `ReachObject` objective on the first physical object, 10 random steps and `check_env_specs`. The `mujoco` column was tried with
robosuite 1.5.1. The `warp` column was tried with robosuite 1.5.2 on the Warp library's CPU device; a CUDA run of the same check is outstanding.
Nothing is claimed for an environment that is not in the table. `pytest -m robosuite` is the check to rerun after upgrading robosuite or MuJoCo.

| environment | physical objects | `mujoco` | `warp` |
|---|---|---|---|
| `Lift` | `cube` | works | works |
| `Stack` | `cubeA`, `cubeB` | works | works |
| `NutAssembly`, `NutAssemblySquare`, `NutAssemblyRound` | `SquareNut`, `RoundNut` | works | works with `WarpOptions(njmax=512, nconmax=128, cuda_graph=True)`; the default capacities overflow and the env raises `CapacityError` |
| `PickPlace` (with `single_object_mode=2, object_type="can"`), `PickPlaceBread`, `PickPlaceCan`, `PickPlaceCereal`, `PickPlaceMilk` | `Milk`, `Bread`, `Cereal`, `Can` | works | works |
| `Door` | `Door` | works; the door is a fixture | works; the door is a fixture |
| `ToolHang` | `stand`, `frame`, `tool` | works | works |
| `NutAssemblySingle`, `PickPlaceSingle` | | refused: the object changes every episode, and so would the observation keys. Fix it with `single_object_mode=2` and `object_type`, or use `NutAssemblySquare` / `NutAssemblyRound` | same |
| `Wipe` | none | no object for either objective; needs an objective of its own | same |
| two-arm environments | | refused: the robot is one arm | same |

## Errors

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

objective = ReachObject("cube", 0.05)
ok = RobosuiteParams("Lift", {}, objective, 0.0, False)
assert error_of(lambda: RobosuiteParams("", {}, objective, 0.0, False)).path == "name"
assert error_of(lambda: TaskConfig(ok, 10, 0.02, None, Mount())).path == "success_tolerance"      # the objective defines success
assert error_of(lambda: RobosuiteParams("Lift", {"robots": "Panda"}, objective, 0.0, False)).path == "env_kwargs"
assert error_of(lambda: RobosuiteParams("Lift", {"x": object()}, objective, 0.0, False)).path == "env_kwargs.x"
assert error_of(lambda: RobosuiteParams("Lift", {}, "lift", 0.0, False)).path == "objective"
assert error_of(lambda: ReachObject("cube", 0.0)).path == "tolerance"
```
