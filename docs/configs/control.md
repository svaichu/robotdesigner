# ControlConfig

How the policy's action reaches the joints: the control **mode**, the controller **gains** and the **action scale**.
Hardware limits (`effort_limit`) belong to the robot; this config only describes the control interface. Importing it
needs numpy only.

```python
from robotdesigner.config import ConfigError, ControlConfig, FixedGains, InertiaScaledGains
```

Related pages: [RobotConfig](robot.md), [TaskConfig](task.md), [SimConfig](sim.md), [EnvConfig](env.md),
[backends](../backends.md).

## `ControlConfig(mode, gains, action_scale)`

| field | type | rule |
|---|---|---|
| `mode` | str | one of the modes below |
| `gains` | `FixedGains` or `InertiaScaledGains` | |
| `action_scale` | float or `None` | `> 0` and required for `joint_pos_delta` and `joint_vel`; must be `None` for every other mode |

Every environment takes actions in `[-1, 1]`, one per joint. The mode decides what an action means.

### Modes

| mode | meaning of an action `a` in `[-1, 1]` (MuJoCo backend) | `action_scale` | gains used |
|---|---|---|---|
| `joint_pos_abs` | target position: `a = -1` and `a = +1` are the two ends of the joint range (`±pi` for an unlimited joint) | `None` | `kp`, `kd` |
| `joint_pos_delta` | `q_target += a * action_scale` each step (rad per step), clipped to the joint range; the target restarts at the measured position on reset | rad per step | `kp`, `kd` |
| `joint_vel` | target velocity `a * action_scale` (rad/s) | rad/s | `kd` |
| `joint_torque` | torque `a * effort_limit` | `None` | none |
| `ee_pos_delta`, `ee_pose_delta` | reserved names for end-effector control | | no backend supports them yet |

A backend that does not support a mode rejects the config before building anything; it never approximates
(see [backends](../backends.md#capabilities)).

## Gains

### `FixedGains(kp, kd)`

A scalar applies to every joint; a sequence gives one value per joint (its length must equal `n`, which
[`EnvConfig`](env.md) checks). Both must be `>= 0`.

- Position modes: torque `= kp * (q_target - q) - kd * qdot` (N·m/rad and N·m·s/rad).
- `joint_vel`: torque `= kd * (qdot_target - qdot)`.

### `InertiaScaledGains(bandwidth, damping_ratio)`

Gains derived from the inertia each joint sees, so one setting suits arms of different size and mass. Both must be `> 0`.

```
kp_i = bandwidth² · M_ii
kd_i = max(2 · damping_ratio · sqrt(kp_i · M_ii) − damping_i, 0)
```

`M` is the joint-space mass matrix of the compiled model evaluated at `home_qpos` (armature included), and `damping_i`
is the joint's own damping, which is subtracted so the total damping is what the formula says. It is an approximation:
`M` changes with the arm's configuration. `bandwidth` is the closed-loop frequency in rad/s and `damping_ratio` is the
dimensionless damping (1 is critical).

```python
# the law, for one joint with M_ii = 0.5 kg·m² and 0.5 N·m·s/rad of joint damping
bandwidth, zeta, M, joint_damping = 10.0, 1.0, 0.5, 0.5
kp = bandwidth**2 * M
kd = max(2 * zeta * (kp * M) ** 0.5 - joint_damping, 0.0)
assert abs(kp - 50.0) < 1e-12 and abs(kd - 9.5) < 1e-12
```

The MuJoCo backend evaluates `M` once at `home_qpos`.

## Examples

```python
delta = ControlConfig("joint_pos_delta", InertiaScaledGains(bandwidth=7.0, damping_ratio=1.0), action_scale=0.05)
absolute = ControlConfig("joint_pos_abs", FixedGains(kp=(60.0, 40.0), kd=6.0), action_scale=None)

assert delta.to_dict() == {
    "mode": "joint_pos_delta",
    "gains": {"kind": "inertia_scaled", "bandwidth": 7.0, "damping_ratio": 1.0},
    "action_scale": 0.05,
}
assert absolute.to_dict()["gains"] == {"kind": "fixed", "kp": [60.0, 40.0], "kd": 6.0}
assert ControlConfig.from_dict(delta.to_dict()) == delta
assert ControlConfig.from_dict(absolute.to_dict()) == absolute
```

## Errors

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

assert error_of(lambda: ControlConfig("joint_pos_delta", FixedGains(1, 1), None)).path == "action_scale"   # required
assert error_of(lambda: ControlConfig("joint_torque", FixedGains(1, 1), 0.1)).path == "action_scale"       # must be None
assert error_of(lambda: ControlConfig("teleport", FixedGains(1, 1), None)).path == "mode"
assert error_of(lambda: FixedGains(-1.0, 1.0)).path == "kp"
assert error_of(lambda: InertiaScaledGains(0.0, 1.0)).path == "bandwidth"
```

## In the hash

The whole config, the gain kind included, is part of `env_hash`. See [EnvConfig](env.md#hashing).
