# Environment: `EnvConfig`

`EnvConfig(robot, control, task, sim)` is the complete, declarative description of one environment. Everything that changes behaviour is in it, and nothing is
filled in behind your back: every field of every part is required. Its hash, `env_hash`, identifies a run.

```python
from robotdesigner.config import (
    ConfigError, ControlConfig, EnvConfig, FixedGains, MujocoOptions, Pose, SimConfig, TaskConfig, chain,
)
from robotdesigner.tasks.empty import EmptyParams

arm = chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0, radius=0.03, damping=0.5, armature=0.01,
            frictionloss=0.0, effort_limit=50.0, range=(-2.5, 2.5))
cfg = EnvConfig(
    robot=arm,
    control=ControlConfig("joint_pos_abs", FixedGains(kp=60.0, kd=6.0), action_scale=None),
    task=TaskConfig(EmptyParams("idle"), max_episode_steps=200, success_tolerance=None, target_region=None, mount=Pose.identity()),
    sim=SimConfig("mujoco", num_envs=8, device="cpu", dt=0.002, integrator="euler", control_freq=50.0, seed=0,
                  scene="plain_floor", options=MujocoOptions(self_collision=False)),
)
assert len(cfg.env_hash) == 12
```

The four parts: [robot](robot.md), [control](control.md), [task](task.md), [sim](sim.md). To build the environment from a config:
`env = make_env(cfg)` ([simulators](../simulators.md)). To write a config with defaults and refine it step by step: [`BaseEnvConfig`](base-env.md).

## Hashes

| hash | covers | use |
|---|---|---|
| `robot.config_hash` (12 hex) | the embodiment, label excluded | which arm; also `env.robot_hash` |
| `cfg.env_hash` (12 hex) | the robot's full hash, control, task (including mount) and sim (including options) | which run; also `env.env_hash` |

`full_env_hash` and `robot.full_hash` are the untruncated SHA-256 digests. The input is canonical JSON with floats rounded to 12 significant digits.

```python
same = EnvConfig.from_dict(cfg.to_dict())
assert same == cfg and same.env_hash == cfg.env_hash

import dataclasses
more_envs = dataclasses.replace(cfg, sim=dataclasses.replace(cfg.sim, num_envs=16))
assert more_envs.env_hash != cfg.env_hash                      # num_envs is part of the sim, so of the hash
assert more_envs.robot.config_hash == cfg.robot.config_hash    # the embodiment did not change
```

Anything in the config changes the hash, including `seed` and `num_envs`. If two runs should be comparable *because* they share a hash,
that is what you want; if you need "same everything except seed", compare the dicts or key on `robot_hash` plus the task.
Defaults exist only in `BaseEnvConfig`. An `EnvConfig` holds every value, so its hash cannot move when the package's defaults change; what moves is the
`EnvConfig` that an unchanged builder script resolves to. That is why you store `resolve().to_dict()` with a run
([base-env](base-env.md#files-and-reproducibility)).

## Serialization

`to_dict()` returns plain JSON-able data (lists, numbers, strings, `None`), and `EnvConfig.from_dict(d)` is its exact inverse; parsing validates every nested part and
reports errors with the full path. Task params are rebuilt by the provider named in the dict, so a custom provider must be registered
in the process that parses it.

## Cross-part checks

`EnvConfig` checks what no single part can: per-joint `kp` / `kd` sequences must have as many entries as the robot has joints, and the control must fit the robot kind.
`NativeGains` needs an [`ExternalRobot`](robot.md#externalrobot-a-model-that-lives-elsewhere) (error at `control.gains` otherwise); an `ExternalRobot` takes any of the three kinds of gains. With `NativeGains` its `control.mode` must fit its `actuator_kind` (error at `control.mode` listing the modes that fit, see [control](control.md#gains)); with `FixedGains` or `InertiaScaledGains` every mode fits, because the actuators are ours.
`robot` is a `ParametricArm` or an `ExternalRobot`; `from_dict` reads either through `robot_from_dict`.

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

wrong_gains = ControlConfig("joint_pos_abs", FixedGains(kp=(60.0, 40.0), kd=6.0), action_scale=None)
from robotdesigner.config import NativeGains
native = ControlConfig("joint_pos_abs", NativeGains(), action_scale=None)
assert error_of(lambda: dataclasses.replace(cfg, control=native)).path == "control.gains"   # a parametric arm has no actuators of its own
assert error_of(lambda: dataclasses.replace(cfg, control=wrong_gains)).path == "control.gains.kp"
assert error_of(lambda: EnvConfig(arm, cfg.control, cfg.task, "sim")).path == "sim"
assert "unknown keys: extra" in str(error_of(lambda: EnvConfig.from_dict({**cfg.to_dict(), "extra": 1})))   # a typo is an error, not ignored
```

What a config *cannot* tell you, because it depends on the installed engine, is whether it can run: the simulator supports the control mode, the device exists, the task's
assets can be built. That is `make_env`'s job and it reports every such problem at once.

`EnvConfig.resolve()` returns the config itself. It exists so that code can treat `EnvConfig` and `BaseEnvConfig` alike (`BaseEnvConfig.resolve()` is where the work is).
