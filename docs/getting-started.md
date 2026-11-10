# Getting started

## Install

```bash
pip install -e .                          # the config layer only (numpy)
pip install -e ".[env]"                   # + torch, TorchRL, tensordict, MuJoCo: the CPU simulator
pip install -e ".[env,robosuite]"         # + robosuite: its arenas and objects as tasks
pip install -e ".[env,warp]"              # + mujoco-warp, warp-lang: the GPU simulator (an NVIDIA GPU)
pip install -e ".[env,viz]"               # + Pillow, imageio-ffmpeg: GIF and MP4 output
pip install -e ".[env,dev]"               # + pytest, ruff, pyyaml
```

Python >= 3.10. `import robotdesigner.config` works with numpy alone and never imports an engine, so you can write, validate, hash and sweep configs anywhere.
For `warp`, `mujoco` and `mujoco-warp` must be of one release. YAML config files need the `yaml` extra.

## 1. Describe a robot

The robot is data: every physical number is stated, and the result has a stable hash.

```python
from robotdesigner.config import chain

arm = chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0, radius=0.03,
            damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0, range=(-2.5, 2.5))
print(arm.n, arm.config_hash)
assert arm.n == 3 and len(arm.config_hash) == 12
```

[Robot reference](configs/robot.md).

## 2. Describe the environment

An environment config has four parts: robot, control, task, simulation. The builder `BaseEnvConfig` starts from a complete default and changes only what you write:

```python
from robotdesigner.config import BaseEnvConfig, Chain, FixedGains

cfg = (
    BaseEnvConfig()
    .robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0))
    .control(mode="joint_pos_abs", gains=FixedGains(kp=60.0, kd=6.0))
    .sim(num_envs=8, seed=0)
)
print(cfg.explain())                       # every field, its value, and whether you set it or it is a default
env_cfg = cfg.resolve()                    # the frozen, validated EnvConfig; raises one error listing every problem
print(env_cfg.env_hash)
assert env_cfg.sim.num_envs == 8
```

[Builder](configs/base-env.md), [EnvConfig](configs/env.md).

## 3. Build and run

`make_env` is the single step from a config to an environment. This needs the `env` extra:

```python no-run
env = cfg.build()                          # = make_env(cfg.resolve())
print(env.robot_hash, env.env_hash)

td = env.reset()                           # a TensorDict, batch size [8]
traj = env.rollout(50)                     # random policy; traj.shape == [8, 50]
print(env.action_space)                    # torch.Size([3]): one env's action, one command per joint
print(env.observation_space)               # nested dict of per-env shapes

td["action"] = env.action_spec.rand()      # [8, 3] in [-1, 1]
td = env.step(td)                          # outputs under ("next", ...)
```

The environment never autoresets: reset rows with the `_reset` mask, or use `step_and_maybe_reset`. The full contract: [simulators](simulators.md).
The default task, `empty/idle`, asks nothing of the arm. For a real task, bring a provider.

## 4. Use a task: a robosuite scene

With the `robosuite` extra, robosuite supplies the arena and objects, and the simulator named in `sim.simulator` steps everything with your robot:

```python no-run
from robotdesigner.tasks.robosuite import LiftObject, RobosuiteParams

cfg = (
    BaseEnvConfig()
    .robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0))
    .task(RobosuiteParams(name="Lift", env_kwargs={}, objective=LiftObject("cube", height=0.84),
                          reset_noise=0.0, terminate_on_success=False), max_episode_steps=200)
    .sim(num_envs=8)
)
env = cfg.build()
```

The robot stands where robosuite mounts a stock arm in that scene (the scene's mount frame; the default `Mount()` snaps onto it). [robosuite scenes](tasks/robosuite.md); your own tasks: [writing a provider](tasks/writing-a-provider.md).
The notebook `examples/quickstart_cpu_mujoco.ipynb` is this, executed.

## 5. Compare embodiments

```python
family = cfg.sweep(paired={"robot.n": [2, 3, 4, 5], "robot.axes": ["zy", "zyy", "zyyy", "zyyyy"]}, grid={"sim.seed": range(3)})
print(len(family), "configs")             # 12: arms of 2 to 5 joints, each with the same total length and mass, three seeds
for row in family.table()[:3]:
    print(row)                             # index, the overrides, env_hash
assert len(family) == 12
```

Every item is a full config with its own `env_hash`; run them with the same seed and `num_envs` and they share the task's draws. [Sweeps](configs/base-env.md#families-of-embodiments-sweep),
[on a cluster](cluster.md). `examples/embodiment_sweep.ipynb` is a worked case.

## 6. GPU

On a CUDA machine (`warp` extra) the same environment is three fields of `sim`; the robot is compiled exactly as for the CPU and the tensors live on the device:

```python no-run
env = cfg.sim(simulator="warp", device="cuda:0", num_envs=4096).build()
```

[Simulators](simulators.md#gpu-mujoco-warp-specifics); `examples/quickstart_gpu_warp.ipynb`.

## 7. See it

```python no-run
from robotdesigner.viz import record_rollout

traj, clip = record_rollout(env, 150, env_index=0, camera="iso")
clip.save("run.mp4")
```

Headless machines need `MUJOCO_GL=egl`. [Rendering and video](viz.md); `examples/scene_viz.ipynb`.

## Notebooks

| notebook | shows |
|---|---|
| `examples/quickstart_cpu_mujoco.ipynb` | a three-link arm in the robosuite `Lift` scene, CPU |
| `examples/quickstart_gpu_warp.ipynb` | the same on the Warp GPU simulator |
| `examples/scene_walkthrough.ipynb` | embodiment comparison, control modes, rendering, step by step |
| `examples/scene_viz.ipynb` | recording rollouts, tiling, saving MP4 and GIF |
| `examples/embodiment_sweep.ipynb` | a family of arms with `sweep`, one task and seed |
