# robotdesigner

Describe robot arms as validated, hashable data and run them in batched [TorchRL](https://github.com/pytorch/rl)
environments, so the same control task can be compared across embodiments.

## Install

```bash
pip install -e .              # config layer only (numpy)
pip install -e ".[env]"       # + torch, torchrl, tensordict, mujoco: the CPU MuJoCo backend
pip install -e ".[env,dev]"   # + pytest, ruff
```

Python >= 3.10. Other extras: `viz` (imageio), `warp` and `robosuite` (later milestones; declared only).
`import robotdesigner.config` works with numpy alone and never imports an engine.

## The idea

- **The robot is data.** A `ParametricArm` is frozen, validated and serializable. It has a stable
  `config_hash` (12 characters, SHA-256 prefix) that ignores labels and float noise.
- **Everything that changes behaviour is in `EnvConfig`**: robot, control, task, sim and backend options. There
  are no hidden defaults. `env_hash` identifies a run.
- **Backends realize configs.** `sim.backend = "mujoco"` is the CPU reference backend. An unsupported combination
  (control mode, device, task, ...) is rejected up front with every problem listed, never approximated.
- **One env contract.** Every backend returns a TorchRL `EnvBase` with `batch_size = [num_envs]`. The env never
  autoresets; reset rows with the `_reset` mask.

## Quick start

```python
from robotdesigner.config import (
    ControlConfig, EnvConfig, FixedBox, FixedGains, MujocoOptions, Pose, ReachParams,
    SimConfig, TaskConfig, chain,
)
from robotdesigner.envs import make_env

arm = chain(n=3, axes="zyz", total_length=0.9, total_mass=3.0, radius=0.03,
            damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0, range=(-2.5, 2.5),
            base_pose=Pose((0.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0)))

cfg = EnvConfig(
    robot=arm,
    control=ControlConfig("joint_pos_abs", FixedGains(kp=60.0, kd=6.0), action_scale=None),
    task=TaskConfig("native/reach", horizon=200, success_tolerance=0.02,
                    target_region=FixedBox(low=(0.2, -0.3, -0.2), high=(0.6, 0.3, 0.2)),
                    params=ReachParams(reset_noise=0.0, terminate_on_success=False)),
    sim=SimConfig(backend="mujoco", num_envs=8, device="cpu", dt=0.002, integrator="euler",
                  control_freq=50, seed=0, scene="plain_floor"),
    backend_options=MujocoOptions(self_collision=False),
)

env = make_env(cfg)
print(env.robot_hash, env.env_hash)
traj = env.rollout(50)          # random policy, TorchRL's own rollout
print(traj.shape)               # torch.Size([8, 50])
```

The full script is `examples/reach_cpu.py` (headless; `python examples/reach_cpu.py`).

Step by hand:

```python
td = env.reset()
td["action"] = env.action_spec.rand()          # [8, n] in [-1, 1]
td = env.step(td)                              # outputs under ("next", ...)

# or, with driver-side autoreset (the env itself never resets):
data, td = env.step_and_maybe_reset(td)
```

## Control modes (CPU backend)

| mode | action `a in [-1, 1]` |
|---|---|
| `joint_pos_abs` | finite range mapped affinely onto `[lo, hi]`; unlimited joints map to `a * pi` |
| `joint_pos_delta` | `q_target += a * action_scale`, clipped to the joint range |
| `joint_vel` | velocity target `a * action_scale` (rad/s) |
| `joint_torque` | `a * effort_limit` |

Gains are `FixedGains(kp, kd)` or `InertiaScaledGains(bandwidth, damping_ratio)`: `kp_i = bandwidth^2 * M_ii`,
`kd_i = max(2 * damping_ratio * sqrt(kp_i * M_ii) - damping_i, 0)`, with `M` the joint-space mass matrix at
`home_qpos` (an approximation, since `M` changes with configuration). The effective gains are stored on the
compiled model's `RobotInterface` (`kp`, `kd`).

## Targets

`FixedBox(low, high)` samples targets uniformly in the base frame. `OwnReachable()` uses end-effector positions of
uniformly sampled joint configurations, so targets are always reachable. Reset can inject `target`, `qpos` and
`qvel` through the reset TensorDict.

## Rendering (optional)

```python
from robotdesigner.backends.mujoco.render import render_rgb, launch_viewer

frame = render_rgb(env, env_index=0, size=(240, 320))   # uint8 (H, W, 3); needs an OpenGL context
launch_viewer(env, env_index=0)                          # passive viewer; needs a display
```

On headless machines set `MUJOCO_GL=egl` before importing mujoco. No other code path needs rendering.

## Layout

```
src/robotdesigner/
    config/      numpy only: configs, chain() generator, validation, hashing, kinematics oracle
    backends/    protocol, capabilities and lazy registry; mujoco/ is the CPU backend
    envs/        make_env(cfg)
```

The design spec is in `docs/spec_robotconfig_env.md`.

## Tests

```bash
pytest -m "not gpu and not robosuite and not render"
```

Markers: `gpu`, `robosuite`, `render` (skipped when no OpenGL context is available).
