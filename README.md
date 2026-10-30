# robotdesigner

Describe robot arms as validated, hashable data and run them in batched [TorchRL](https://github.com/pytorch/rl)
environments, so the same control task can be compared across embodiments.

## Install

```bash
pip install -e .              # config layer only (numpy)
pip install -e ".[env]"       # + torch, torchrl, tensordict, mujoco: the CPU MuJoCo simulator
pip install -e ".[env,robosuite]"   # + robosuite: its arenas and objects as tasks
pip install -e ".[env,dev]"   # + pytest, ruff
```

Python >= 3.10. Other extras: `robosuite` (the task provider: robosuite arenas and objects as tasks; the quick start needs it),
`viz` (Pillow, imageio-ffmpeg: GIF and MP4 output) and `warp` (a later milestone; declared only).
`import robotdesigner.config` works with numpy alone and never imports an engine.

## The idea

- **The robot is data.** A `ParametricArm` is frozen, validated and serializable. It has a stable
  `config_hash` (12 characters, SHA-256 prefix) that ignores labels and float noise.
- **Everything that changes behaviour is in `EnvConfig`**: robot, control, task and sim (which carries the options of the simulator). There
  are no hidden defaults. `env_hash` identifies a run.
- **`env = make_env(cfg)`** is the one step from config to environment, for every task.
- **Simulators and tasks are separate.** `sim.simulator = "mujoco"` names the simulator (the CPU reference simulator). A
  task id is `"<namespace>/<name>"` and comes from a **task provider**: the package names one placeholder task
  (`empty/idle`, [docs](docs/tasks/empty.md)) and ships one real provider, `robosuite`; others register themselves (see [TaskConfig](docs/configs/task.md#where-tasks-come-from)). An unsupported combination
  (control mode, device, task on this simulator, ...) is rejected up front with every problem listed, never
  approximated.
- **One env contract.** Every environment is a TorchRL `EnvBase` with `batch_size = [num_envs]`. The env never
  autoresets; reset rows with the `_reset` mask.

## Documentation

Reference pages in [`docs/`](docs/README.md): [RobotConfig](docs/configs/robot.md), [ControlConfig](docs/configs/control.md),
[TaskConfig](docs/configs/task.md), [SimConfig and simulator options](docs/configs/sim.md), [EnvConfig](docs/configs/env.md), [BaseEnvConfig](docs/configs/base-env.md),
[simulators and the environment contract](docs/simulators.md), [robosuite scenes](docs/tasks/robosuite.md),
[writing a task provider](docs/tasks/writing-a-provider.md), [recording videos](docs/viz.md) and the
[design record](docs/spec_robotconfig_env.md). The words the docs use (embodiment, simulator, task provider, scene, ...) are
defined in the [glossary](docs/README.md#words-used-in-these-docs). The Python examples in the pages are executed by the tests.

## Quick start

A three-link arm in the robosuite `Lift` scene (a table and a cube): robosuite supplies the assets, the simulator named
in `sim.simulator` steps everything.

```python
from robotdesigner import make_env
from robotdesigner.config import (
    ControlConfig, EnvConfig, FixedGains, MujocoOptions, SimConfig, TaskConfig, chain,
)
from robotdesigner.tasks.robosuite import LiftObject, RobosuiteParams, default_mount   # a task comes from the provider that owns it

arm = chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0, radius=0.03,
            damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0, range=(-2.5, 2.5))

cfg = EnvConfig(
    robot=arm,
    control=ControlConfig("joint_pos_abs", FixedGains(kp=60.0, kd=6.0), action_scale=None),
    task=TaskConfig(params=RobosuiteParams(name="Lift", env_kwargs={}, objective=LiftObject("cube", height=0.84),
                                           reset_noise=0.0, terminate_on_success=False),
                    horizon=200, success_tolerance=None, target_region=None,
                    mount=default_mount("Lift")),   # where robosuite mounts a stock arm in this scene
    sim=SimConfig(simulator="mujoco", num_envs=8, device="cpu", dt=0.002, integrator="euler",
                  control_freq=50, seed=0, scene="empty",      # the arena brings its own floor
                  options=MujocoOptions(self_collision=False)),
)

env = make_env(cfg)
print(env.robot_hash, env.env_hash)
traj = env.rollout(50)          # random policy, TorchRL's own rollout
print(traj.shape)               # torch.Size([8, 50])
```

The full script is `examples/robosuite_scene.py` (headless; `python examples/robosuite_scene.py`). A walkthrough notebook is
`examples/scene_walkthrough.ipynb` (embodiment comparison, control modes, rendering). Recording, saving and showing
videos: `examples/scene_viz.ipynb`.

The same environment, written as chained refinements of a complete default (`BaseEnvConfig`, see [its page](docs/configs/base-env.md); the
result is the frozen `EnvConfig` above, `.resolve()`, and `.build()` is `make_env(self.resolve())`):

```python no-run
from robotdesigner.config import BaseEnvConfig, Chain, FixedGains

cfg = (
    BaseEnvConfig()
    .robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0))   # written fields change, the rest carries from the base
    .control(gains=FixedGains(kp=60.0, kd=6.0))
    .sim(num_envs=8)
)
env = cfg.build()
five = cfg.robot(Chain(n=5, axes="zyyyy"))    # a new config; cfg is unchanged
```

Step by hand:

```python
td = env.reset()
td["action"] = env.action_spec.rand()          # [8, n] in [-1, 1]
td = env.step(td)                              # outputs under ("next", ...)

# or, with driver-side autoreset (the env itself never resets):
data, td = env.step_and_maybe_reset(td)
```

## Tasks, control and rendering

- **Tasks come from providers.** The package ships one, [robosuite scenes](docs/tasks/robosuite.md) (`robosuite/<EnvName>`): robosuite supplies
  the arena, the objects and their placement rule, our simulator steps everything with your robot and control law, and the task's `objective`
  (`LiftObject`, `ReachObject`) says what success and reward are. 12 of the 15 single-arm environments work (the table is on the page). Your own
  tasks: [writing a task provider](docs/tasks/writing-a-provider.md). robosuite 1.5.x does not run on MuJoCo >= 3.10 as released; `apply_compat()`
  patches that in the running process and is called for you.
- **Control.** Four joint-space modes (`joint_pos_abs`, `joint_pos_delta`, `joint_vel`, `joint_torque`) with `FixedGains` or `InertiaScaledGains`:
  [ControlConfig](docs/configs/control.md). A reset can inject `qpos`, `qvel` and whatever the task documents.
- **Rendering and video** need an OpenGL context (headless: set `MUJOCO_GL=egl` before the first import of mujoco) and, for files, the `viz` extra
  (`pip install -e ".[env,viz]"`: Pillow writes GIFs, imageio-ffmpeg writes MP4s).

```python
from robotdesigner.viz import record_rollout

traj, clip = record_rollout(env, 150, env_index=0, camera="rs_sideview")   # renders the live state while the rollout runs
clip                      # last expression of a notebook cell: plays inline
clip.save("run.mp4")      # the format follows the suffix: .mp4 or .gif
```

`env.render(env_index, camera=..., size=...)` returns one `uint8 (H, W, 3)` frame of the live state, so any step loop can collect its own frames;
rendering never changes the simulation, and only the CPU MuJoCo simulator renders so far. To compare embodiments give them one camera
(`auto_camera(cfg_a, cfg_b)`, or a fixed arena camera such as `rs_sideview`): with the same seed they get the same scene. `examples/record_scene.py`
records a four-env rollout to `scene.mp4` and `scene.gif`; the full guide is [docs/viz.md](docs/viz.md).

## Layout

```
src/robotdesigner/
    config/      numpy only: configs, chain() generator, validation, hashing, kinematics oracle
    simulators/  simulators: protocol, capabilities and lazy registry; mujoco/ is the CPU simulator and its generic env
    tasks/       task providers, one subpackage per namespace (robosuite/), and the task contract (base.py)
    envs/        make_env(cfg)
    viz/         Clip (save GIF / MP4, show inline), frame recorder, camera framing; numpy only
```

## Tests

```bash
pytest -m "not gpu and not robosuite and not render"
```

Markers: `gpu`, `robosuite` (needs the robosuite extra: `pytest -m robosuite`), `render` (skipped when no OpenGL context
is available). `tests/test_docs.py` runs the Python
examples in `docs/` and checks that every link and anchor resolves; it needs no simulator.
