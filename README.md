# robotdesigner

Programmatically generate N-DOF robot arm models and register them as simulation environments.

- **Composer** — build a `mujoco.MjSpec` for any N-DOF arm with position or torque control
- **Gymnasium env** — wrap the spec as a standard `gym.Env` for CPU-based RL
- **MJX env** — wrap the spec as a JAX-native env for GPU/TPU-accelerated simulation (jit, vmap)

## Requirements

- Python ≥ 3.13
- MuJoCo ≥ 3.6

## Installation

```bash
pip install robotdesigner
```

Or with [uv](https://github.com/astral-sh/uv):

```bash
uv add robotdesigner
```

## Build a robot arm spec

```python
from robotdesigner.mujoco.composer import build_arm, save_arm

# 3-DOF arm, position control, default z/y alternating joint orientations
spec = build_arm(ndof=3, control="pos")

# Custom joint orientations (cycles if shorter than ndof)
spec = build_arm(ndof=3, control="torque", joint_orientation=["z", "y", "z"])

# Save to assets/ as XML
save_arm(spec, ndof=3, control="pos", joint_orientation=["z", "y", "z"])
# -> assets/three_arm_pos_zyz.xml
```

### `build_arm` parameters

| Parameter | Type | Description |
|---|---|---|
| `ndof` | `int` | Number of joints (≥ 1) |
| `control` | `str` | `"pos"` (PD position) or `"torque"` |
| `joint_orientation` | `list[str]` | Per-joint axis: `"z"` (yaw) or `"y"` (pitch). Cycles if shorter than `ndof`. Defaults to alternating `z/y`. |

**Assumptions:** fixed base, each link has length = 1 m and mass = 1 kg.

### CLI

```bash
python -m robotdesigner.mujoco.composer --ndof 3 --control pos --joint-orientation z y z
python -m robotdesigner.mujoco.composer --ndof 5 --control torque --output my_arm.xml
```

## Gymnasium environment

```python
import gymnasium as gym
from robotdesigner.mujoco.composer import build_arm

spec = build_arm(ndof=3, control="pos")

gym.register(
    id="AbstractArm-v0",
    entry_point="robotdesigner.envs.abstract_mujoco_env:make_env",
)

env = gym.make("AbstractArm-v0", spec=spec, render_mode="rgb_array", max_episode_steps=200)
obs, _ = env.reset()

for _ in range(100):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        break
```

### RLlib training

```python
import ray
from ray.rllib.algorithms.ppo import PPOConfig

ray.init()

algo = (
    PPOConfig()
    .environment(
        env="AbstractArm-v0",
        env_config={"xml": spec.to_xml(), "render_mode": "rgb_array"},
    )
    .env_runners(num_env_runners=2)
    .build()
)

result = algo.train()
print(result["env_runners"]["episode_reward_mean"])
```

## MJX environment (JAX)

```python
import jax
from robotdesigner.mujoco.composer import build_arm
from robotdesigner.envs import load_mjx_environment, list_mjx_environments

spec = build_arm(ndof=3, control="pos")
env = load_mjx_environment("AbstractArmMjx", spec=spec)

print(list_mjx_environments())   # ('AbstractArmMjx',)
print(env.action_size)
print(env.observation_size)

rng = jax.random.PRNGKey(0)
state = env.reset(rng)

# Single step
state = env.step(state, jax.random.uniform(rng, (env.action_size,), minval=-1, maxval=1))

# JIT-compiled rollout
jit_step = jax.jit(env.step)

# Batched rollout with vmap
batch_size = 8
rngs = jax.random.split(rng, batch_size)
states = jax.vmap(env.reset)(rngs)
states = jax.vmap(env.step)(states, jax.random.uniform(rng, (batch_size, env.action_size)))
```

## Interactive notebooks (marimo)

The `example/` directory contains [marimo](https://marimo.io) notebooks:

| File | Description |
|---|---|
| `example_GenMjSpec_mo.py` | Build and view a spec in the MuJoCo viewer |
| `example_env_mujoco_mo.py` | Gymnasium rollout + RLlib training |
| `example_env_mjx_mo.py` | MJX rollout, JIT, vmap, render |

Run with:

```bash
marimo run example/example_env_mujoco_mo.py
```

## License

See [LICENSE](LICENSE).
