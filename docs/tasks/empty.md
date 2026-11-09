# `empty/idle`: the placeholder task

The package names exactly one task of its own, `empty/idle`, so that a default environment can run without any provider installed: the robot stands in the
simulator's scene and nothing is asked of it. It is the task of `BaseEnvConfig()`. Any real task comes from another provider
([robosuite](robosuite.md), [writing a provider](writing-a-provider.md)); `tests/test_no_hardcoded_tasks.py` keeps the rest of the core free of task names.

```python
from robotdesigner.config import BaseEnvConfig, TaskConfig, get_task_provider
from robotdesigner.config import Pose
from robotdesigner.tasks.empty import EmptyParams

task = TaskConfig(EmptyParams(name="idle"), max_episode_steps=100, success_tolerance=None, target_region=None, mount=Pose.identity())
assert task.id == "empty/idle"
assert get_task_provider("empty").default_mount("idle") == Pose.identity()
assert BaseEnvConfig().resolve().task.id == "empty/idle"
```

| | |
|---|---|
| params | `EmptyParams(name)`; `name` must be `"idle"`. No other fields, so no `reset_noise` and no `terminate_on_success` (the environment reads them when a params class has them) |
| goal | none: `success_tolerance` and `target_region` must be `None` |
| mount | the origin |
| simulators | `mujoco`, `warp` |
| `task` observations | none (the group is empty); the `proprio`, `success` and `episode_step` entries are present as for every task |
| success, reward | success is always `False`, reward always `0.0` |
| scene objects | none; draws nothing from the task's random stream |

Use it to check an embodiment on its own: how it falls under gravity, how a control mode and gains respond to a step in `joint_pos_abs`, how the cost per step scales with `n`.
The `empty` *provider* is unrelated to the `empty` *scene* of `sim.scene` (a preset of the simulator, see [sim](../configs/sim.md#scenes)).
