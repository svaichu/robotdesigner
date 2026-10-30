# Writing a task provider

A **task provider** brings tasks to the framework: it owns a namespace (`robosuite/...`, `mine/...`), names its tasks, gives each a typed params class, and builds
the task for a simulator. The package names only the placeholder task `empty/idle`, so a new task source is a provider. This page builds one end to end, a goal task `mine/reach`
that moves the end effector to a target point. `tests/toy_provider/` is the same thing in full; the engine tests use it.

Related pages: [TaskConfig](../configs/task.md), [simulators](../simulators.md), [robosuite scenes](robosuite.md) (a provider that brings assets).

A provider has two halves, kept apart so that the config layer never imports an engine:

| half | needs | holds |
|---|---|---|
| **config side** | numpy | the params class of each task and the provider object (`validate` has to run without an engine) |
| **engine side** | torch, torchrl, the simulator's library | a `TaskImpl`: scene assets, per-env state, observations, success, reward |

## 1. The params class (config side)

One frozen dataclass per task, with `to_dict()` and `from_dict()`, validated in `__post_init__`. **The params object is the task**: the class attribute
`provider` is the namespace and the field `name` says which task of the provider it is, so `TaskConfig(params=ReachParams(name="reach", ...))` needs no
second place to name the provider (there is no `provider=` argument and no string id). Both are part of `to_dict()` (`provider` is written by `TaskConfig`,
`name` by the params). The class attribute `uses_goal` says whether the task is defined by a target region and a tolerance (`True`: `TaskConfig` then
requires `success_tolerance` and `target_region`; `False`: it forbids them).

```python
from dataclasses import dataclass
from typing import ClassVar

from robotdesigner.config import (
    ConfigError, FixedBox, Pose, Problem, TaskConfig, list_task_providers, register_task_provider, unregister_task_provider,
)


@dataclass(frozen=True)
class ReachParams:
    provider: ClassVar[str] = "mine"  # the namespace: this class belongs to the provider registered as "mine"
    uses_goal: ClassVar[bool] = True  # a goal task: the config carries a target region and a tolerance

    name: str                        # which task of the provider: "reach"
    reset_noise: float               # radians of uniform noise on the initial joint positions (the environment applies it)
    terminate_on_success: bool       # `terminated` when the task is solved (the environment applies it)

    def __post_init__(self):
        if self.reset_noise < 0:
            raise ConfigError("reset_noise", "must be >= 0")      # the path says which field is at fault

    def to_dict(self):
        return {"name": self.name, "reset_noise": self.reset_noise, "terminate_on_success": self.terminate_on_success}

    @classmethod
    def from_dict(cls, d):
        return cls(**d)
```

`reset_noise` and `terminate_on_success` are the two params the environment itself looks for; everything else in the class is yours. For free-form
mappings (keyword arguments of a library) use `robotdesigner.config._canon.freeze` and `thaw`, which make them immutable and hashable.

## 2. The provider object

Five methods, no engine import at module level. `default_mount(name)` says where the task expects the robot base (the origin when the scene has no opinion); the user's `TaskConfig.mount` is what counts.

```python
class MyTasks:
    SIMULATORS = frozenset({"mujoco"})

    def task_names(self):                      # names or patterns: they are listed when an id is unknown
        return ("reach",)

    def params_class(self, name):              # None means "no such task"
        return ReachParams if name == "reach" else None

    def default_mount(self, name):             # where the robot base goes if the user does not say: a world Pose
        return Pose.identity()

    def validate(self, cfg, simulator):        # every reason this task cannot run on that simulator
        if simulator not in self.SIMULATORS:
            return [Problem("sim.simulator", f"task {cfg.task.id!r} runs on {sorted(self.SIMULATORS)}, not on {simulator!r}")]
        return []

    def build(self, cfg, simulator):           # the engine side; import the heavy modules here, not above
        from my_package.reach import ReachTask

        return ReachTask(cfg)


PROVIDER = MyTasks()
```

`validate` returns a list of `Problem(path, message)`; `make_env` collects them with the simulator's own problems and raises one error that lists all of them.

## 3. Register it

Three ways: a subpackage of `robotdesigner.tasks` that exposes `PROVIDER` (the package name is the namespace); an entry point in another package,
which is how a project brings its own tasks; and `register_task_provider`, which is also what tests use and which takes precedence over the other two.

```toml
# pyproject.toml of your package
[project.entry-points."robotdesigner.task_providers"]
mine = "my_package:PROVIDER"
```

```python
register_task_provider("mine", PROVIDER, replace=True)       # a lazy "my_package:PROVIDER" string works as well
assert "mine" in list_task_providers()

task = TaskConfig(
    params=ReachParams(name="reach", reset_noise=0.0, terminate_on_success=False),
    max_episode_steps=200, success_tolerance=0.02,
    target_region=FixedBox(low=(0.2, -0.3, -0.2), high=(0.6, 0.3, 0.2)),
    mount=PROVIDER.default_mount("reach"),                        # the robot base in the world; the target region is relative to it
)
assert task.id == "mine/reach"                                # derived from the class and the name
assert task.to_dict()["provider"] == "mine"
assert TaskConfig.from_dict(task.to_dict()) == task           # provider and params.name pick the params class on the way back in

for wrong in (lambda: TaskConfig(ReachParams("lift", 0.0, False), 200, None, None, Pose.identity()),          # no such task
              lambda: TaskConfig(ReachParams("reach", 0.0, False), 200, None, None, Pose.identity()),         # a goal task needs its tolerance
              lambda: ReachParams("reach", -1.0, False)):                                     # the params validate themselves
    try:
        wrong()
    except ConfigError as e:
        assert e.path in ("params.name", "success_tolerance", "reset_noise")
    else:
        raise AssertionError("no error")

unregister_task_provider("mine")
```

From here the config side is complete: configs round-trip, hash and compare, without torch or MuJoCo installed.

## 4. The engine side

Subclass `robotdesigner.tasks.base.BaseTask` (the optional hooks have defaults) and state only what the task adds: the robot, the control law, batching,
masked reset, episode counters and rendering belong to the environment. This is a trimmed `tests/toy_provider/reach.py`.

```python no-run
import numpy as np
import torch
from torchrl.data import Composite, Unbounded

from robotdesigner.tasks.base import BaseTask


class ReachTask(BaseTask):
    def __init__(self, cfg):                       # cfg is the whole EnvConfig
        self._region, self._tol = cfg.task.target_region, cfg.task.success_tolerance

    def bind(self, sim):                           # once, after the model is compiled: per-env state for sim.num_envs envs
        self._target = torch.zeros(sim.num_envs, 3)

    def observation_spec(self, num_envs):          # the "task" observation group
        f32 = torch.float32
        return Composite(ee_pos=Unbounded(shape=(num_envs, 3), dtype=f32),
                         target_pos=Unbounded(shape=(num_envs, 3), dtype=f32), shape=(num_envs,))

    def reset(self, sim, mask, rand, injected):    # place the task for the envs selected by mask (bool (N,))
        low, high = (torch.tensor(v, dtype=torch.float64) for v in (self._region.low, self._region.high))
        target = (low + rand(sim.num_envs, 3) * (high - low)).to(torch.float32)   # draw for the WHOLE batch, whatever the mask
        if injected("target") is not None:         # a value the caller put in the reset TensorDict
            target = injected("target").to(torch.float32)
        m = torch.from_numpy(np.asarray(mask, dtype=bool))
        self._target[m] = target[m]

    def observe(self, sim):                        # float32 tensors with leading dimension N, positions in the base frame
        return {"ee_pos": torch.from_numpy(sim.ee_pos_base()).to(torch.float32), "target_pos": self._target.clone()}

    def success(self, sim, obs):                   # bool (N, 1)
        return torch.linalg.norm(obs["ee_pos"] - obs["target_pos"], dim=-1, keepdim=True) < self._tol

    def reward(self, sim, obs):                    # float32 (N, 1)
        return -torch.linalg.norm(obs["ee_pos"] - obs["target_pos"], dim=-1, keepdim=True)
```

- **Draws.** `rand(*shape)` is the task's own seeded stream: it does not depend on the robot, so the same seed gives every embodiment the same draws. Always
  draw for the whole batch and then apply the mask, so the stream does not depend on which envs are reset.
- **Scene assets.** `attach(spec)` adds bodies and geoms to the `MjSpec` before it is compiled (robosuite's provider attaches an arena and objects under a
  prefix); `bind(sim)` then resolves names to ids. Objects without a free joint are shared by every env of the batch.
- **Overlays.** `overlays(scene, sim, data, env_index)` draws state that is not in the model (a goal marker) into a render scene; a task that keeps everything in
  the model draws none.
- **Injected state.** `injected(key)` returns a tensor the caller put in the reset TensorDict, or `None`; document the keys you accept.

## 5. Test it

Copy `tests/toy_provider/` and `tests/conftest.py` as a starting point: the conftest registers the provider lazily with
`register_task_provider("toy", "tests.toy_provider:PROVIDER", replace=True)`. A provider needs at least:

1. `check_env_specs(env)` on an environment built with your task;
2. a masked-reset test: reset some rows and check that only those rows changed;
3. a seeding test: the same seed gives the same draws, a different seed different ones, and two robots with different joint counts get the same draws;
4. its config tests: round trip through `to_dict` / `from_dict`, and the errors above.

The engine tests of the package are a worked example of each (`tests/test_mujoco_env.py`).
