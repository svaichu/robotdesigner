# RobotConfig

A `RobotConfig` describes one embodiment as plain data. It is frozen, validated on construction, serializable to JSON and
hashable, and importing it never imports a simulator or torch.

Today there is one kind, **`ParametricArm`**: a serial chain of hinge joints and capsule links on a fixed base. A second
kind, `ExternalRobot` (a reference to an existing model, for baselines such as a stock Panda), is designed but not
implemented. Everything below is about `ParametricArm`; "robot config" and `ParametricArm` mean the same thing in the code.

```python
from robotdesigner.config import ParametricArm, Segment, JointSpec, LinkSpec, Pose, Capsule, ToolSpec, chain
```

Related pages: [ControlConfig](control.md), [TaskConfig](task.md), [SimConfig](sim.md), [EnvConfig](env.md),
[simulators](../simulators.md).

## Conventions

- **Units:** SI. Lengths in metres, masses in kilograms, angles in radians, torques in N·m.
- **Quaternions** are `(w, x, y, z)`. They are normalized on construction, and the sign is fixed so that the first
  non-zero component is positive (`q` and `-q` are the same rotation, so they hash the same).
- **Everything dynamic is explicit.** Damping, armature, friction loss, effort limit, mass and radius have no defaults;
  a simulator never fills one in. Only `label`, and the optional arguments of `chain()` listed below, have defaults.
- **Frames.** The chain starts at the *base frame*. The robot has no pose of its own: where the base stands in the world is a property of the scene, `TaskConfig.mount`, so the same arm has the same `config_hash` in every scene. Segment `i` has a *joint frame*
  (the pose of the joint relative to the previous link's end frame; the base frame for `i = 0`), a *link frame* (the
  joint frame rotated by the joint angle) and an *end frame* (the link frame translated by `tip_offset`; same
  orientation). The end effector is the last end frame composed with `end_effector`.

In formulas (this is exactly what `robotdesigner.config.kinematics` computes):

```
T_0 = base                          the base frame (the task's mount; the identity if you work in the base frame)
F_i = T_i · joint_frame_i          joint frame of segment i
L_i = F_i · Rot(axis_i, q_i)       link frame
T_{i+1} = L_i · Pose(tip_offset_i) end frame (becomes the next base)
ee = T_n · end_effector
```

At `q = 0` every link of a `chain()` arm points along +x of the base frame.

## Classes

All fields are required unless a default is shown. Numbers must be finite; booleans and strings are rejected where a
number is expected.

### `Pose(pos, quat)`

| field | type | rule |
|---|---|---|
| `pos` | 3 floats | metres |
| `quat` | 4 floats `(w, x, y, z)` | non-zero; normalized; sign canonicalized |

`Pose.identity()` is the zero translation and the identity rotation.

### `Capsule(radius)`

The only geometry in schema version 1. A capsule runs from the link origin to `tip_offset`.

| field | type | rule |
|---|---|---|
| `radius` | float | `> 0` |

### `JointSpec(axis, range, damping, armature, frictionloss, effort_limit)`

A hinge joint.

| field | type | rule |
|---|---|---|
| `axis` | 3 floats | non-zero; normalized on construction; expressed in the joint frame |
| `range` | `(lo, hi)` or `None` | `lo < hi`, radians; `None` means unlimited |
| `damping` | float | `>= 0`, N·m·s/rad |
| `armature` | float | `>= 0`, kg·m² (rotor inertia added to the joint) |
| `frictionloss` | float | `>= 0`, N·m (dry friction) |
| `effort_limit` | float | `> 0`, N·m; the actuator's force and control limit |

### `LinkSpec(joint_frame, tip_offset, mass, geometry, inertia)`

| field | type | rule |
|---|---|---|
| `joint_frame` | `Pose` | pose of the joint frame relative to the previous end frame |
| `tip_offset` | 3 floats | position of the end frame in the link frame, metres |
| `mass` | float | `> 0`, kg |
| `geometry` | `Capsule` | |
| `inertia` | 3 floats or `None` | `None`: derived from the capsule and the mass. Otherwise the three principal moments (kg·m², each `> 0`) about the centre of mass, with the axes of the link frame; the centre of mass is the capsule midpoint, `tip_offset / 2` |

### `Segment(joint, link)`

One joint and the link it moves. A `ParametricArm` is a tuple of segments in kinematic order.

### `ToolSpec(gripper, mount_pos, mount_quat)`

A gripper mounted on the end effector.

| field | type | rule |
|---|---|---|
| `gripper` | str | non-empty; a gripper name such as `"PandaGripper"` |
| `mount_pos` | 3 floats | offset of the gripper's first body, metres |
| `mount_quat` | 4 floats | orientation of the gripper's first body, normalized like any quaternion |

It is stored and hashed. The MuJoCo simulator does not build grippers yet and ignores it, so it changes `env_hash` but not the physics.

### `ParametricArm(schema_version, segments, home_qpos, end_effector, tool, label="")`

| field | type | rule |
|---|---|---|
| `schema_version` | int | must be `1` |
| `segments` | tuple of `Segment` | at least one; `n = len(segments)` |
| `home_qpos` | `n` floats | each inside its joint's `range` |
| `end_effector` | `Pose` | relative to the last end frame |
| `tool` | `ToolSpec` or `None` | |
| `label` | str | metadata only; not part of the hash |

`arm.n` is the number of joints. The number of joints is a property of the arm, never a field of any other config.

## Building arms

### `chain(n, axes, ...)`

Builds an `n`-joint arm that is straight along +x at `q = 0`, with identity joint frames, capsule links and derived
inertia. `n` is an argument of the generator, not a stored field, so a study can sweep it.

| argument | meaning |
|---|---|
| `n` | integer `>= 1` |
| `axes` | a string of `x`/`y`/`z` letters of length `n` (`"zyy"`), or a sequence of `n` axis vectors. No cycling |
| `link_length` **or** `total_length` | exactly one. A scalar or `n` per-link values; a total is split evenly over the links |
| `mass` **or** `total_mass` | exactly one, same rules |
| `radius`, `damping`, `armature`, `frictionloss`, `effort_limit` | required; a scalar (all joints) or `n` per-joint values |
| `range` | `None` (unlimited), one `(lo, hi)` for all joints, or `n` entries each `(lo, hi)` or `None` |
| `home_qpos` | default: all zeros |
| `end_effector` | default: `Pose.identity()` |
| `tool` | default: `None` |
| `label` | default: `""` |

Holding `total_length` or `total_mass` fixed while `n` changes is how a family of arms with comparable size is made.

```python
import json
from dataclasses import replace

from robotdesigner.config import (
    Capsule, ConfigError, JointSpec, LinkSpec, ParametricArm, Pose, Segment, ToolSpec, chain, kinematics,
)

def close(a, b, tol=1e-9):
    return abs(a - b) < tol

common = dict(radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0)

arm = chain(3, "zyy", total_length=0.9, total_mass=3.0, range=(-2.5, 2.5), **common)
assert arm.n == 3
assert close(sum(s.link.tip_offset[0] for s in arm.segments), 0.9)   # total_length split over 3 links
assert close(sum(s.link.mass for s in arm.segments), 3.0)
assert arm.home_qpos == (0.0, 0.0, 0.0) and arm.tool is None

# the same arm with more joints and the same total length and mass
arm5 = chain(5, "zyyyy", total_length=0.9, total_mass=3.0, range=(-2.5, 2.5), **common)
assert arm5.n == 5 and close(sum(s.link.mass for s in arm5.segments), 3.0)

# per-joint values: the first joint unlimited, the others limited
mixed = chain(3, "zyy", link_length=[0.4, 0.3, 0.2], mass=[1.0, 0.8, 0.5], range=[None, (-2.0, 2.0), (-1.0, 1.0)], **common)
assert [s.joint.range for s in mixed.segments] == [None, (-2.0, 2.0), (-1.0, 1.0)]
```

### Building a `ParametricArm` directly

Use the classes when the chain is not straight along +x: offset joint frames, perpendicular links, explicit inertia.

```python
seg = Segment(
    joint=JointSpec(axis=(0, 0, 1), range=None, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0),
    link=LinkSpec(
        joint_frame=Pose((0.0, 0.0, 0.1), (1.0, 0.0, 0.0, 0.0)),   # the joint sits 0.1 m above the previous end frame
        tip_offset=(0.0, 0.0, 0.4),                                  # this link points up
        mass=1.5,
        geometry=Capsule(radius=0.04),
        inertia=(0.02, 0.02, 0.003),                                 # explicit principal moments
    ),
)
tower = ParametricArm(
    schema_version=1,
    segments=(seg, seg),
    home_qpos=(0.0, 0.0),
    end_effector=Pose.identity(),
    tool=None,
    label="two vertical links",
)
ee = kinematics.forward(tower, [0.0, 0.0]).ee
assert close(ee[2, 3], 0.1 + 0.4 + 0.1 + 0.4)   # two links, each 0.1 above the previous end plus 0.4 long
```

### The end effector, a gripper and the mount

```python
placed = chain(
    3, "zyy", total_length=0.9, total_mass=3.0, **common,
    end_effector=Pose((0.0, 0.0, 0.05), (0.7071068, 0.0, 0.7071068, 0.0)),   # tool frame: 5 cm along z, rotated 90 deg about y
    tool=ToolSpec("PandaGripper", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
)
assert placed.tool.gripper == "PandaGripper"
mount = Pose((0.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0))       # where the arm stands: given to the kinematics (and to `TaskConfig.mount`), not to the arm
p = kinematics.forward(placed, [0.0, 0.0, 0.0], mount).ee[:3, 3]
assert close(p[0], 0.9) and close(p[1], 0.0) and close(p[2], 0.5 + 0.05)   # along +x at height 0.5, plus the 5 cm tool offset
q = kinematics.forward(placed, [0.0, 0.0, 0.0]).ee[:3, 3]                 # without a mount: in the base frame
assert close(q[2], 0.05)
```

## Serialization and the hash

```python
d = arm.to_dict()                     # plain JSON-able dict; tuples become lists
assert d["kind"] == "parametric_arm" and d["schema_version"] == 1 and "label" in d
assert ParametricArm.from_dict(json.loads(json.dumps(d))) == arm
```

`from_dict` is strict: unknown keys, missing keys and a wrong `kind` or `schema_version` raise `ConfigError`. Only
`label` may be omitted.

The hash is the SHA-256 of a canonical JSON form (sorted keys, no whitespace) of the dict **without `label`**, with every
float rounded to 12 significant digits and `-0.0` written as `0.0`. Because axes and quaternions are normalized on
construction, equal arms written differently hash identically.

| property | value |
|---|---|
| `arm.config_hash` | the first 12 hex characters; use it to name and compare robots |
| `arm.full_hash` | all 64 hex characters; what `EnvConfig` hashes |
| `arm.canonical_json()` | the exact text that is hashed |

```python
assert len(arm.config_hash) == 12 and arm.full_hash.startswith(arm.config_hash)
assert replace(arm, label="renamed").config_hash == arm.config_hash          # the label is metadata
assert chain(1, [(0, 0, 5)], link_length=0.3, mass=1.0, **common).config_hash == \
       chain(1, "z", link_length=0.3, mass=1.0, **common).config_hash       # axes are normalized
assert chain(1, "z", link_length=0.3, mass=1.0, **common).config_hash != \
       chain(1, "z", link_length=0.31, mass=1.0, **common).config_hash      # any physical change changes it
```

## Errors

Every validation error is a `ConfigError` (a `ValueError`). Its `.path` names the offending field, counted from the
object you were building, and the message starts with that path.

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

e = error_of(lambda: chain(2, "zy", link_length=0.3, mass=1.0, range=(-1.0, 1.0), home_qpos=(0.0, 2.0), **common))
assert e.path == "home_qpos[1]" and "outside the joint range" in str(e)

e = error_of(lambda: chain(2, "zy", link_length=0.3, total_length=0.6, mass=1.0, **common))
assert e.path == "link_length/total_length"                                  # give exactly one of the pair

bad = arm.to_dict()
bad["segments"][1]["joint"]["axis"] = [0, 0, 0]
assert error_of(lambda: ParametricArm.from_dict(bad)).path == "segments[1].joint.axis"
```

## Kinematics oracle

`robotdesigner.config.kinematics` computes forward kinematics in pure numpy, independent of any simulator. The tests
use it to check every compiled model.

| function | returns |
|---|---|
| `forward(arm, q, base=None)` | `FK` with `link_frames` `(n, 4, 4)`, `tip_frames` `(n, 4, 4)` and `ee` `(4, 4)`, all poses in the frame `base` is given in: world poses for `base = task.mount`, base-frame poses for `None` |
| `forward_ee_batch(arm, q_batch, base=None)` | end-effector positions `(B, 3)`: in the **world** frame with `base = task.mount`, in the robot's base frame with `None` |
| `sample_joint_positions(arm, num, rng)` | `(num, n)` samples, uniform inside each joint range and in `[-pi, pi]` for unlimited joints |

```python
import numpy as np

pos = kinematics.forward_ee_batch(arm, np.zeros((2, 3)))
assert pos.shape == (2, 3) and close(pos[0, 0], 0.9)    # straight along +x
q = kinematics.sample_joint_positions(arm, 10, np.random.default_rng(0))
assert q.shape == (10, 3) and np.all(np.abs(q) <= 2.5)
```

## How the MuJoCo simulator uses the robot

| aspect | MuJoCo simulator |
|---|---|
| names | `base`, `link_{i}`, `joint_{i}`, `geom_{i}`, `ee` (site), `act_{i}` |
| actuators | position servo, velocity servo or torque, by control mode; force limited to `±effort_limit` |
| `ToolSpec` | ignored |
| mount | the `base` body sits at `TaskConfig.mount`, for every task |
| collisions | robot geoms collide with the scene; with each other only if `MujocoOptions.self_collision` |

The chain is built by one function (`simulators/mujoco/arm_spec.py`), so the physical robot is the same in every scene.
