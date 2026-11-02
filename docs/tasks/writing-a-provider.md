# Writing a task provider

A **task provider** brings tasks to the framework. It owns a namespace (`mine/...`), names its tasks, gives each a typed params class and builds the task for a simulator.
The package names only the placeholder `empty/idle`, so every real task source, yours included, is a provider. This page builds one end to end: `mine/reach`, a goal task
that moves the end effector to a target point. The test-only provider in `tests/toy_provider/` is the same thing in full, and the engine tests run on it.

A provider has two halves, kept apart so the config layer never imports an engine:

| half | needs | holds |
|---|---|---|
| **config side** | numpy | the params class of each task and the provider object (`validate` must run without an engine) |
| **engine side** | torch, torchrl, the simulator's library | a `TaskImpl`: scene assets, per-env state, observations, success, reward |

Related: [TaskConfig](../configs/task.md), [simulators](../simulators.md), [robosuite](robosuite.md) (a provider that brings assets).

## 1. The params class (config side)

One frozen dataclass per task, with `to_dict()` / `from_dict()`, validated in `__post_init__`. **The params object is the task**: the class attribute `provider` is the namespace and
the field `name` says which task of the provider it is, so there is no second place to name the provider. `uses_goal` says whether the task is defined by a target region and a tolerance
(`True`: `TaskConfig` requires `success_tolerance` and `target_region`; `False`: it forbids them).

```python
from dataclasses import dataclass
from typing import ClassVar

from robotdesigner.config import (
    ConfigError, FixedBox, Pose, Problem, TaskConfig, list_task_providers, register_task_provider, unregister_task_provider,
)


@dataclass(frozen=True)
class ReachParams:
    provider: ClassVar[str] = "mine"   # the namespace this class belongs to
    uses_goal: ClassVar[bool] = True   # a goal task: the config carries a target region and a tolerance

    name: str                          # which task of the provider: "reach"
    reset_noise: float                 # rad of uniform noise on the initial joint positions (the environment applies it)
    terminate_on_success: bool         # `terminated` when solved (the environment applies it)

    def __post_init__(self):
        if self.reset_noise < 0:
            raise ConfigError("reset_noise", "must be >= 0")     # the path names the field at fault

    def to_dict(self):
        return {"name": self.name, "reset_noise": self.reset_noise, "terminate_on_success": self.terminate_on_success}

    @classmethod
    def from_dict(cls, d):
        return cls(**d)
```

`reset_noise` and `terminate_on_success` are the two params the environment looks for (`getattr`, so they are optional); everything else in the class is yours.
For free-form mappings (keyword arguments of a library) use `robotdesigner._canon.freeze` / `thaw`, which make them immutable and hashable.

## 2. The provider object

Five methods; no engine import at module level.

```python
class MyTasks:
    SIMULATORS = frozenset({"mujoco", "warp"})   # the simulators whose SimView the engine side was written and tested for

    def task_names(self):                        # names or patterns; listed in the error for an unknown id
        return ("reach",)

    def params_class(self, name):                # None means "no such task"
        return ReachParams if name == "reach" else None

    def default_mount(self, name):               # where the robot base goes if the user does not say: a world Pose
        return Pose.identity()

    def validate(self, cfg, simulator):          # every reason this task cannot run on that simulator
        if simulator not in self.SIMULATORS:
            return [Problem("sim.simulator", f"task {cfg.task.id!r} runs on {sorted(self.SIMULATORS)}, not on {simulator!r}")]
        return []

    def build(self, cfg, simulator):             # the engine side; import the heavy modules here, not above
        from my_package.reach import ReachTask

        return ReachTask(cfg)


PROVIDER = MyTasks()
```

`validate` returns `Problem(path, message)` items; `make_env` collects them with the simulator's own and raises one error listing everything.

## 3. Register it

Three ways ([sources and precedence](../configs/task.md#where-tasks-come-from)): a subpackage of `robotdesigner.tasks` exposing `PROVIDER`; an entry point of another package,
which is how a *project* brings its own tasks; and `register_task_provider`, which tests use and which wins over the other two.

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
    target_region=FixedBox(low=(0.2, -0.3, -0.2), high=(0.6, 0.3, 0.2)),    # in the robot's base frame; it follows the mount
    mount=PROVIDER.default_mount("reach"),
)
assert task.id == "mine/reach" and task.to_dict()["provider"] == "mine"
assert TaskConfig.from_dict(task.to_dict()) == task

for wrong, path in (
    (lambda: TaskConfig(ReachParams("lift", 0.0, False), 200, 0.02, FixedBox((0, 0, 0), (1, 1, 1)), Pose.identity()), "params.name"),   # no such task
    (lambda: TaskConfig(ReachParams("reach", 0.0, False), 200, None, None, Pose.identity()), "success_tolerance"),   # a goal task needs its tolerance
    (lambda: ReachParams("reach", -1.0, False), "reset_noise"),                                                      # the params validate themselves
):
    try:
        wrong()
    except ConfigError as e:
        assert e.path == path, e.path
    else:
        raise AssertionError("no error")

unregister_task_provider("mine")
```

At this point the config side is complete: configs round-trip, hash and compare with neither torch nor MuJoCo installed.
In a worker process (a cluster job, a `spawn`ed child) the registry starts empty: register the provider again, or declare the entry point so it is discovered ([cluster](../cluster.md)).

## 4. The engine side

Subclass `robotdesigner.tasks.base.BaseTask` (the optional hooks have defaults) and state only what the task adds. The robot, the control law, batching, masked reset, episode
counters and rendering belong to the environment. This is a trimmed `tests/toy_provider/reach.py`.

```python no-run
import torch
from torchrl.data import Composite, Unbounded

from robotdesigner.tasks.base import BaseTask


class ReachTask(BaseTask):
    def __init__(self, cfg):                       # cfg is the whole EnvConfig
        self._region, self._tol = cfg.task.target_region, cfg.task.success_tolerance

    def bind(self, sim):                           # once, after compilation: per-env state for sim.num_envs envs
        self._target = torch.zeros(sim.num_envs, 3, device=sim.device)

    def observation_spec(self, num_envs):          # the "task" observation group
        f32 = torch.float32
        return Composite(ee_pos=Unbounded(shape=(num_envs, 3), dtype=f32),
                         target_pos=Unbounded(shape=(num_envs, 3), dtype=f32), shape=(num_envs,))

    def reset(self, sim, mask, rand, injected):    # place the task for the envs selected by mask (bool (N,), on sim.device)
        low, high = (torch.tensor(v, dtype=torch.float64) for v in (self._region.low, self._region.high))
        target = (low + rand(sim.num_envs, 3) * (high - low)).to(torch.float32)    # draw for the WHOLE batch, whatever the mask
        if injected("target") is not None:         # a value the caller put in the reset TensorDict
            target = injected("target").to(torch.float32)
        self._target[mask] = target.to(sim.device)[mask]

    def observe(self, sim):                        # float32 tensors on sim.device, leading dimension N, in the base frame
        return {"ee_pos": sim.site_pos(sim.iface.ee_site_id, "base"), "target_pos": self._target.clone()}

    def success(self, sim, obs):                   # bool (N, 1)
        return torch.linalg.norm(obs["ee_pos"] - obs["target_pos"], dim=-1, keepdim=True) < self._tol

    def reward(self, sim, obs):                    # float32 (N, 1)
        return -torch.linalg.norm(obs["ee_pos"] - obs["target_pos"], dim=-1, keepdim=True)
```

### The hooks

| hook | when | what you do |
|---|---|---|
| `attach(spec)` | before the model compiles | add bodies and geoms to the `mujoco.MjSpec` (robosuite's provider attaches an arena and objects under a prefix) |
| `bind(sim)` | once, after compilation | resolve names to ids, allocate per-env state; the **last** point at which the host model `sim.model` may be edited |
| `observation_spec(num_envs)` | at construction | the TorchRL `Composite` of the `task` group, batch shape `(num_envs,)` |
| `reset(sim, mask, rand, injected)` | after the robot is reset | set the task state of the masked envs |
| `observe(sim)` | every step and reset | dict of float32 tensors on `sim.device` |
| `success(sim, obs)`, `reward(sim, obs)` | every step | `(N, 1)` bool / float32 |
| `overlays(scene, sim, data, env_index)` | rendering only | draw state that is not in the model (a goal marker) into an `mjvScene` |

### Rules that keep embodiments comparable

- **Draws.** `rand(*shape)` is the task's own seeded stream. It does not depend on the robot, so the same seed gives every embodiment the same draws. Always draw for the whole batch and then
  apply the mask, so the stream does not depend on which envs are reset.
- **The simulation view.** Hooks receive a `SimView` ([`tasks/base.py`](../../src/robotdesigner/tasks/base.py)), not a simulator's internals: `num_envs`, `device`, `iface` (joint
  ranges, `ee_site_id`, the mount), `body_pose(body, frame)` and `site_pos(site, frame)` with `frame` either `"world"` or `"base"` (float32 on `device`), `write_qpos(adr, values, mask)`
  to place a free body, `base_frame()` and `host_data(env_index)`. List in `SIMULATORS` every simulator you ran the task on.
- **Fixtures.** All envs of a batch share one model, so a scene object without a free joint is placed once, at `bind`; per-env placement needs a free joint.
- **Injected state.** `injected(key)` returns a tensor the caller put in the reset TensorDict (a replayed `target`), or `None`. Document the keys you accept.
- **Frames.** Observe positions in the robot's base frame (`"base"`), as the other providers do, so a task reads the same for every mount.

## 5. Test it

Copy `tests/toy_provider/` and `tests/conftest.py` as a start: the conftest registers the provider lazily with `register_task_provider("toy", "tests.toy_provider:PROVIDER", replace=True)`.
A provider needs at least:

1. `check_env_specs(env)` on an environment built with your task;
2. a masked-reset test: reset some rows and check that only those rows changed;
3. a seeding test: the same seed gives the same draws, a different seed different ones, and two robots with different joint counts get the same draws;
4. its config tests: round trip through `to_dict` / `from_dict` and the errors above.

`tests/test_mujoco_env.py` has a worked example of each.
