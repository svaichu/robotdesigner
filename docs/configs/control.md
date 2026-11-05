# Control: `ControlConfig`

`ControlConfig(mode, gains, action_scale)` says how an action in `[-1, 1]` becomes a command to the joints. It is explicit data, so a difference between two
embodiments is never partly a hidden difference in how they were driven.

```python
from robotdesigner.config import CONTROL_MODES, ConfigError, ControlConfig, FixedGains, InertiaScaledGains, NativeGains

pos = ControlConfig("joint_pos_abs", FixedGains(kp=60.0, kd=6.0), action_scale=None)
delta = ControlConfig("joint_pos_delta", FixedGains(kp=60.0, kd=6.0), action_scale=0.05)
assert ControlConfig.from_dict(delta.to_dict()) == delta
assert "joint_torque" in CONTROL_MODES
```

## Modes

The action has one component per joint (plus one for the gripper opening if the arm has a gripper, see [robot](robot.md#gripper-and-tool)). The clamp to `[-1, 1]`
is applied first. Both simulators implement the same four modes with the same arithmetic.

| mode | command | `action_scale` | actuator |
|---|---|---|---|
| `joint_pos_abs` | target position = action mapped linearly onto the joint range (`±π` for an unlimited joint) | must be `None` | position servo with `kp`, `kd` |
| `joint_pos_delta` | target += `action * action_scale` (rad), clipped to the joint range; the target starts at the reset posture | required, `> 0` | position servo with `kp`, `kd` |
| `joint_vel` | velocity target = `action * action_scale` (rad/s) | required, `> 0` | velocity servo: force = `kd * (target - qvel)`; `kp` is not used |
| `joint_torque` | torque = `action * effort_limit` of each joint | must be `None` | direct; `gains` are carried but unused |

`CONTROL_MODES` also lists `ee_pos_delta` and `ee_pose_delta`. They are reserved names: **no simulator supports them yet**, so `make_env` rejects a config that uses them
(see [simulators](../simulators.md#capabilities-and-validation)). Constructing the `ControlConfig` is allowed, so a config can be written, hashed and stored before a simulator learns the mode.

Whatever the mode, the force of each joint is limited to its `effort_limit` (an actuator force range), so a large gain does not buy unlimited torque.

## Gains

`FixedGains(kp, kd)` gives scalars (every joint) or one value per joint. A per-joint sequence must have `n` entries; this is checked when the
[`EnvConfig`](env.md) is built, because `ControlConfig` does not know `n`.

`InertiaScaledGains(bandwidth, damping_ratio)` derives gains from the inertia each joint sees:

```
kp_i = bandwidth² · M_ii
kd_i = max(2 · damping_ratio · sqrt(kp_i · M_ii) − damping_i, 0)
```

`M` is the joint-space mass matrix of the compiled model at `home_qpos`, armature included. It is an approximation, since `M` changes with posture, but it removes
the largest cause of a spurious embodiment difference under equal gains: a long, heavy arm and a short, light one respond alike when tuned to the same bandwidth.
The simulator evaluates it when it compiles the robot, so the numbers it produces are not in the config; the config stores only `bandwidth` and `damping_ratio`.

```python
scaled = ControlConfig("joint_pos_abs", InertiaScaledGains(bandwidth=20.0, damping_ratio=1.0), action_scale=None)
assert scaled.to_dict()["gains"] == {"kind": "inertia_scaled", "bandwidth": 20.0, "damping_ratio": 1.0}
per_joint = FixedGains(kp=(60.0, 40.0, 20.0), kd=6.0)          # kd is a scalar: it applies to every joint
assert per_joint.kp == (60.0, 40.0, 20.0)
```

`NativeGains()` has no fields: it says "drive the model with its own actuators and gains", and nothing is replaced. It belongs to an
[`ExternalRobot`](robot.md#externalrobot-a-model-that-lives-elsewhere), and an `ExternalRobot` takes nothing else for now (`FixedGains` and `InertiaScaledGains` are
rejected at `control.gains` until driving it with gains of this package, re-actuation, exists). The mode must fit the model's actuators, `robot.actuator_kind`:

| `actuator_kind` | modes that fit |
|---|---|
| `position` | `joint_pos_abs`, `joint_pos_delta` |
| `torque` | `joint_torque` |
| `velocity` | `joint_vel` |
| `mixed` | none: restrict `robot.joints` to joints with one kind of actuator |

```python
native = ControlConfig("joint_pos_abs", NativeGains(), action_scale=None)
assert native.to_dict()["gains"] == {"kind": "native"}
assert ControlConfig.from_dict(native.to_dict()) == native
```

The checks that need the robot (`NativeGains` with a `ParametricArm`; the mode against `actuator_kind`) are made by [`EnvConfig`](env.md#cross-part-checks).

## Errors

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

g = FixedGains(60.0, 6.0)
assert error_of(lambda: ControlConfig("joint_pos_delta", g, None)).path == "action_scale"        # delta needs a scale
assert error_of(lambda: ControlConfig("joint_pos_abs", g, 0.1)).path == "action_scale"           # abs takes none
assert error_of(lambda: ControlConfig("joint_pos_delta", g, 0.0)).path == "action_scale"
assert error_of(lambda: ControlConfig("joint_acc", g, None)).path == "mode"
assert error_of(lambda: FixedGains(-1.0, 1.0)).path == "kp"
assert error_of(lambda: FixedGains((1.0, -1.0), 1.0)).path == "kp[1]"
assert error_of(lambda: InertiaScaledGains(0.0, 1.0)).path == "bandwidth"
```
