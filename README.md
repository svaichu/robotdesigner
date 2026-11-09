# robotdesigner

Load different robot configurations into one batched RL environment, so the same control task can be compared across embodiments.

A robot arm is **validated, hashable data**. An environment is one complete config (robot, control, task, simulation) with its own hash, and `make_env(cfg)` turns it into a batched
[TorchRL](https://github.com/pytorch/rl) environment on the CPU (MuJoCo) or the GPU (MuJoCo Warp). Tasks come from providers (robosuite arenas and objects ship with the package; you can add your own),
and the same seed gives every embodiment the same scene.

```python
from robotdesigner.config import BaseEnvConfig, Chain

cfg = (
    BaseEnvConfig()                                                          # a complete default: one-link arm, empty/idle task, CPU MuJoCo
    .robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0))         # only what you write changes
    .sim(num_envs=8, seed=0)
)
env = cfg.build()                         # make_env(cfg.resolve())
traj = env.rollout(50)                    # random policy; traj.shape == [8, 50]
print(env.robot_hash, env.env_hash)       # identify the embodiment and the run
```

## Install

```bash
pip install -e .                        # config layer only (numpy): write, validate, hash and sweep configs anywhere
pip install -e ".[env]"                 # + torch, TorchRL, MuJoCo: the CPU simulator
pip install -e ".[env,robosuite]"       # + robosuite scenes as tasks
pip install -e ".[env,warp]"            # + MuJoCo Warp: the GPU simulator (an NVIDIA GPU)
pip install -e ".[menagerie]"           # + robots from MuJoCo Menagerie (Python >= 3.10.12)
pip install -e ".[env,viz]"             # + GIF / MP4 output
pip install -e ".[env,robosuite,viz,dev]"   # a development install; "yaml" adds YAML config files
```

Python >= 3.10. `import robotdesigner.config` never imports an engine.

## Ideas that shape the API

- **The robot is data.** A `ParametricArm` is frozen, validated and serializable; `config_hash` ignores labels and float noise. The arm has no pose: the task's `mount` places it.
- **Everything that changes behaviour is in `EnvConfig`.** No hidden defaults in the core; defaults live in the builder (`BaseEnvConfig`), versioned. `env_hash` identifies a run.
- **Reject, never approximate.** A control mode, device, task or scene a simulator cannot run is rejected up front, with every problem listed and the config path to change.
- **Comparable by construction.** `chain(total_length=, total_mass=)` holds size fixed while `n` varies, gains are explicit (`InertiaScaledGains` tunes joints to one bandwidth), and the task's random stream does not depend on the robot.
- **One environment contract.** Every environment is a TorchRL `EnvBase` with `batch_size=[num_envs]`. It never autoresets; reset rows with the `_reset` mask.

## A robot from a library

A model that already exists, such as a Panda or a Go1 from MuJoCo Menagerie, is an `ExternalRobot`: a reference with the numbers the config needs, driven by the model's own actuators.

```python
from robotdesigner.config import BaseEnvConfig
from robotdesigner.robot.providers.menagerie import PROVIDER     # pip install -e ".[menagerie]"; models are fetched by the library, never copied here

panda = PROVIDER.describe("franka_emika_panda")
env = BaseEnvConfig().robot(panda).sim(num_envs=4, scene="plain_floor").build()
```

[Robots from Menagerie](docs/robots/menagerie.md) · [writing a robot provider](docs/robots/writing-a-robot-provider.md).

## Compare embodiments

```python
family = cfg.sweep(paired={"robot.n": [2, 3, 4, 5], "robot.axes": ["zy", "zyy", "zyyy", "zyyyy"]}, grid={"sim.seed": range(3)})   # 12 configs
for env_cfg in family.configs():          # resolved, frozen, each with its own env_hash; store env_cfg.to_dict() with the run
    ...
```

`sweep` expands; it launches nothing. [Running a family on a cluster](docs/cluster.md).

## Documentation

Start with [`docs/README.md`](docs/README.md). The main pages:

| | |
|---|---|
| [Getting started](docs/getting-started.md) | install, first robot, first environment, sweep, GPU, video |
| [Robot](docs/configs/robot.md), [Control](docs/configs/control.md), [Task](docs/configs/task.md), [Simulation](docs/configs/sim.md), [Environment](docs/configs/env.md), [Builder](docs/configs/base-env.md) | the config reference |
| [Simulators and the environment contract](docs/simulators.md) | specs, reset, seeding, CPU and GPU |
| [robosuite scenes](docs/tasks/robosuite.md), [writing a task provider](docs/tasks/writing-a-provider.md) | tasks |
| [Rendering and video](docs/viz.md), [Running on a cluster](docs/cluster.md) | using it |
| [Design](docs/design.md), [Development](docs/development.md) | why, and how to work on it |

Notebooks (`examples/`): `quickstart_cpu_mujoco`, `quickstart_gpu_warp`, `scene_walkthrough`, `scene_viz`, `embodiment_sweep`.

## Tests

```bash
pytest -m "not gpu and not robosuite and not render"
```

Markers: `gpu` (a CUDA device), `robosuite` (the robosuite extra), `render` (an OpenGL context; `MUJOCO_GL=egl` when headless). `tests/test_docs.py` runs the Python examples in the docs and checks every link and anchor; it needs no simulator.

## License

See [LICENSE](LICENSE).
