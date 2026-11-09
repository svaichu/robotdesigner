# Robot: `ParametricArm` and `ExternalRobot`

A robot is a **frozen, validated, hashable value**. It has no pose and knows nothing about a simulator: where the base stands is
`TaskConfig.mount`, and everything that changes how it is driven is in [`ControlConfig`](control.md) and [`SimConfig`](sim.md).
Two consequences: the same arm object can be dropped into any scene, and `robot_hash` identifies the embodiment alone.

Everything on this page is numpy only (`robotdesigner.robot`, re-exported by `robotdesigner.config`).

```python
import numpy as np

from robotdesigner.config import (
    Capsule, ConfigError, EndEffectorRef, ExternalJoint, ExternalRobot, JointSpec, LinkSpec, ParametricArm, Pose, Segment,
    TwoFingerGripper, chain, kinematics as kin, register_robot_provider, robot_from_dict, unregister_robot_provider,
)
```

## `chain()`: the usual way to get an arm

`chain` builds a serial arm of `n` hinge joints with capsule links. At `q = 0` the arm lies straight along +x of the base frame.

```python
arm = chain(
    n=3, axes="zyy",                       # one letter (x, y, z) or one vector per joint; no cycling
    total_length=0.9, total_mass=3.0,      # split evenly over the links ...
    radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0,
    range=(-2.5, 2.5),                     # rad; None = unlimited, or one (lo, hi) / None per joint
)
assert arm.n == 3
assert [s.link.tip_offset[0] for s in arm.segments] == [0.3, 0.3, 0.3]
assert abs(sum(s.link.mass for s in arm.segments) - 3.0) < 1e-12
```

| argument | meaning |
|---|---|
| `n`, `axes` | number of joints; the axis of each (`"zyy"` or a sequence of 3-vectors; the length must be `n`) |
| `link_length` **or** `total_length` | exactly one: metres per link (scalar or per link) or the total, divided by `n` |
| `mass` **or** `total_mass` | exactly one, the same way (kg) |
| `radius`, `damping`, `armature`, `frictionloss`, `effort_limit` | scalar (every joint) or one value per joint; all required |
| `range` | `None`, one `(lo, hi)` for every joint, or `n` entries (each `None` or `(lo, hi)`) |
| `home_qpos` | start posture, default zeros; must lie inside the ranges |
| `end_effector` | a `Pose` relative to the last link's end frame, default identity |
| `gripper`, `tool` | optional, at most one; see below |
| `label` | free text, **not** hashed |

There are no defaults for the physical parameters: you state every number. (The builder layer, [`BaseEnvConfig`](base-env.md), is where defaults live.)

Holding `total_length` and `total_mass` fixed while `n` changes keeps reach and mass the same across a family of embodiments, so what differs is the
distribution of joints, not the size of the arm.

## The data model

`ParametricArm` is a tuple of `Segment(joint, link)` plus `home_qpos`, `end_effector`, `tool`, `gripper`, `label` and `schema_version` (only `1` exists).

| class | fields | rules |
|---|---|---|
| `JointSpec` | `axis`, `range`, `damping`, `armature`, `frictionloss`, `effort_limit` | axis is normalised and non-zero; `range` is `None` or `lo < hi`; damping, armature, friction loss `>= 0`; effort limit `> 0` |
| `LinkSpec` | `joint_frame`, `tip_offset`, `mass`, `geometry`, `inertia` | `joint_frame` is a `Pose` relative to the previous link's end frame; `tip_offset` is where this link's end frame sits in the link frame; mass `> 0`; `inertia` is `None` (derived from the capsule and the mass) or three principal moments `> 0` |
| `Capsule` | `radius` | `> 0`; the capsule runs from the link origin to `tip_offset` |
| `Pose` | `pos`, `quat` | quaternion `(w, x, y, z)`, normalised; `q` and `-q` are stored the same way |

Joints are hinges only, links are capsules only. Build an arm by hand when `chain` is too regular (a different `joint_frame` per link, explicit inertia):

```python
seg = Segment(
    joint=JointSpec(axis=(0, 0, 1), range=(-1.0, 1.0), damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=40.0),
    link=LinkSpec(joint_frame=Pose.identity(), tip_offset=(0.4, 0.0, 0.0), mass=1.2, geometry=Capsule(0.03), inertia=None),
)
one = ParametricArm(schema_version=1, segments=(seg,), home_qpos=(0.0,), end_effector=Pose.identity(), tool=None)
assert one.n == 1
```

## Gripper and tool

`TwoFingerGripper` is a parametric parallel gripper with one degree of freedom, the opening `w` in `[0, 2 * stroke]`. It is part of the arm's identity
(hashed) and is **built by the CPU MuJoCo simulator**, which adds one action dimension (the opening, `[-1, 1]` mapped to `[0, max_opening]` in every control mode) and a `proprio/gripper`
observation. The Warp simulator does not build grippers yet and reports `robot.gripper` as a problem.

```python
g = TwoFingerGripper.default(stroke=0.05)           # keyword arguments replace fields; see the class for the defaults
gripped = chain(3, "zyy", total_length=0.9, total_mass=3.0, radius=0.03, damping=0.5, armature=0.01,
                frictionloss=0.0, effort_limit=50.0, gripper=g)
assert g.max_opening == 0.1 and g.tcp_offset == (0.09, 0.0, 0.0)
assert gripped.config_hash != arm.config_hash
```

`ToolSpec(gripper, mount_pos, mount_quat)` names a gripper by string. It is stored and hashed but the simulators ignore it. Giving both a `gripper` and a `tool` is an error.

## Identity: serialization and hashing

```python
d = arm.to_dict()
assert ParametricArm.from_dict(d) == arm                      # exact round trip (JSON-able: lists, numbers, None)

relabeled = chain(3, "zyy", total_length=0.9, total_mass=3.0, radius=0.03, damping=0.5, armature=0.01,
                  frictionloss=0.0, effort_limit=50.0, range=(-2.5, 2.5), label="my arm")
assert relabeled.config_hash == arm.config_hash                # the label is metadata
assert len(arm.config_hash) == 12 and arm.full_hash.startswith(arm.config_hash)
```

`config_hash` is the first 12 hex characters of the SHA-256 of the canonical JSON (sorted keys, floats rounded to 12 significant digits, `-0.0` as `0.0`, label
left out). The same embodiment therefore hashes the same across machines and float noise below 1e-12. Changing any physical number, the home posture, the gripper or the tool changes it.
`env.robot_hash` of a built environment is this value.

## Errors

Every constructor validates and raises `ConfigError`, a `ValueError` whose `.path` names the offending field and `.reason` says why.

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

base = dict(radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0)
assert error_of(lambda: chain(0, "", total_length=1.0, total_mass=1.0, **base)).path == "n"
assert error_of(lambda: chain(2, "zy", total_length=1.0, link_length=0.5, total_mass=1.0, **base)).path == "link_length/total_length"
assert error_of(lambda: chain(2, "zq", total_length=1.0, total_mass=1.0, **base)).path == "axes[1]"
assert error_of(lambda: chain(2, "zy", total_length=1.0, total_mass=1.0, home_qpos=(0.0, 9.0), range=(-1.0, 1.0), **base)).path == "home_qpos[1]"
assert error_of(lambda: JointSpec((0, 0, 0), None, 0.0, 0.0, 0.0, 1.0)).path == "axis"
```

Errors raised while a larger config is being parsed carry the full dotted path (`robot.segments[1].joint.range`).

## Forward kinematics

`robotdesigner.config.kinematics` is a pure-numpy oracle, independent of any simulator: the tests compare every simulator against it.

```python
fk = kin.forward(arm, np.zeros(3))
assert np.allclose(fk.ee[:3, 3], [0.9, 0.0, 0.0])               # straight along +x
base = Pose((0.5, 0.0, 1.0), (1.0, 0.0, 0.0, 0.0))
assert np.allclose(kin.forward(arm, np.zeros(3), base).ee[:3, 3], [1.4, 0.0, 1.0])   # base = the task's mount
q = kin.sample_joint_positions(arm, 5, np.random.default_rng(0))                     # uniform in the ranges
assert kin.forward_ee_batch(arm, q).shape == (5, 3)
```

Convention: `T_0 = base`; for each segment `F_i = T_i · joint_frame_i`, `L_i = F_i · Rot(axis_i, q_i)`, `tip_i = L_i · Pose(tip_offset_i)`, `T_{i+1} = tip_i`;
`ee = T_n · end_effector`. Without `base` the poses are in the robot's base frame.

## `ExternalRobot`: a model that lives elsewhere

A `ParametricArm` is generated from numbers. An `ExternalRobot` **refers** to a model that already exists, for baselines such as a stock Panda
from MuJoCo Menagerie. It holds no model, only what the config layer must know without opening one, so it stays numpy only, frozen, validated and hashable
like the arm. Its dict has `"kind": "external_robot"`; `robot_from_dict` reads either kind.

| field | meaning |
|---|---|
| `provider`, `name`, `entry` | the [robot provider](#robot-providers) (a namespace such as `menagerie`), the robot within it, the model file within the robot |
| `source` | a content id the provider vouches for (Menagerie: the git tree id of the model directory). **Hashed**: a changed model is a different robot |
| `joints` | the joints a `ControlConfig` drives, in order, each an `ExternalJoint(name, range, effort_limit)`; `n = len(joints)`. A joint the model has but `joints` omits (a finger) is not controlled |
| `home_qpos` | one value per joint, inside its range |
| `home_root` | for a model with a free root: the root body at home in the model's world frame; else `None` |
| `keyframe` | the model keyframe that sets the state of the joints that are not driven, or `None` |
| `end_effector` | an `EndEffectorRef`: an existing `site`, or a `body` plus an `offset`; exactly one of the two. `None` for a robot without one |
| `actuator_kind` | the model's own actuators on `joints`: `position`, `torque`, `velocity` or `mixed` |
| `label` | free text, **not** hashed |

A provider's `describe` reads the model and returns these numbers; a config can also be written by hand, as here (the provider below is a stand-in, which
is all the config side needs).

```python
from types import SimpleNamespace

# a provider that only names a robot: enough for the config side (the engine-side methods are not called)
stand_in = SimpleNamespace(names=lambda: ("arm",), describe=None, spec=None, validate=lambda robot, simulator: [], kinematics=None)
register_robot_provider("demo", stand_in, replace=True)

panda_like = ExternalRobot(
    provider="demo", name="arm", entry="arm.xml", source="3f9a2c",
    joints=(ExternalJoint("j1", (-2.9, 2.9), 87.0), ExternalJoint("j2", (-1.7, 1.7), 87.0)),
    home_qpos=(0.0, -0.5), home_root=None, keyframe="home",
    end_effector=EndEffectorRef(site="ee", body=None, offset=Pose.identity()), actuator_kind="position",
)
assert panda_like.n == 2
assert robot_from_dict(panda_like.to_dict()) == panda_like                  # the kind picks the class
assert ExternalRobot.from_dict(panda_like.to_dict()).full_hash == panda_like.full_hash
changed = ExternalRobot.from_dict({**panda_like.to_dict(), "source": "7be104"})
assert changed.config_hash != panda_like.config_hash                         # a changed model is another robot
assert ExternalRobot.from_dict({**panda_like.to_dict(), "label": "baseline"}).config_hash == panda_like.config_hash
```

Validation raises `ConfigError` with a path, as for the arm: `provider` must be registered, `joints` non-empty with unique names, `home_qpos` inside the ranges,
`actuator_kind` one of the four, `EndEffectorRef` with exactly one of `site` and `body` (and an identity `offset` for a site).

```python
def error_of(build):
    try:
        build()
    except ConfigError as e:
        return e
    raise AssertionError("no error")

assert error_of(lambda: ExternalRobot.from_dict({**panda_like.to_dict(), "provider": "nowhere"})).path == "provider"
assert error_of(lambda: ExternalRobot.from_dict({**panda_like.to_dict(), "home_qpos": [0.0, 9.0]})).path == "home_qpos[1]"
assert error_of(lambda: ExternalRobot.from_dict({**panda_like.to_dict(), "actuator_kind": "magic"})).path == "actuator_kind"
assert error_of(lambda: EndEffectorRef(site="ee", body="hand", offset=Pose.identity())).reason == "give exactly one of site and body"
assert error_of(lambda: robot_from_dict({"kind": "mystery"})).path == "kind"
```

What an `ExternalRobot` can do depends on its parts. By default it is driven with its own actuators and gains (`NativeGains`); `FixedGains` or `InertiaScaledGains` replace the actuators of the driven joints with ours (see [control](control.md#gains)). A simulator must list the kind `external` among its capabilities: the CPU MuJoCo simulator does ([how it runs one](../simulators.md#external-robots-on-the-cpu-simulator)); Warp does not yet, so `make_env` rejects it there.

### Robot providers

A robot provider owns a namespace, as a [task provider](../tasks/writing-a-provider.md) does. It is an object with five methods: `names()`; `describe(name, **options)`,
which reads the model and returns an `ExternalRobot`; `spec(robot)`, which returns an editable `mujoco.MjSpec` after checking the model against `robot.source`;
`validate(robot, simulator)`; and `kinematics(robot)`, which returns an oracle with `forward_ee_batch(q, base)` and `sample_joint_positions(num, rng)`. The last
three and `describe` are engine side: importing the provider imports no engine. Providers are found lazily, in this order of precedence:
`register_robot_provider(namespace, target)`, the entry-point group `robotdesigner.robot_providers`, then the subpackages of `robotdesigner.robot.provider`.
`list_robot_providers()` loads nothing; `get_robot_provider(namespace)` loads one. The core names no provider. The one shipped provider is [`menagerie`](../robots/menagerie.md).

`config.kinematics.forward_ee_batch` and `sample_joint_positions` accept an `ExternalRobot` and ask its provider for the oracle; without the engine they raise a
`ConfigError` that says what to install. `forward` (link frames) is for the `ParametricArm` only.

```python
unregister_robot_provider("demo")                                           # the stand-in was for this page
assert error_of(lambda: ExternalRobot.from_dict(panda_like.to_dict())).path == "provider"
```

## What the CPU MuJoCo simulator builds from it

The names are stable, so tasks and your own code can look things up: bodies `base` (at the mount), `link_{i}`; hinges `joint_{i}`; capsule geoms `geom_{i}`; actuators
`act_{i}`; the end effector is the site `ee`. With a gripper: joints `finger_joint_{0,1}`, actuator `act_gripper`, site `tcp`. Robot geoms collide with the scene; they collide with
each other only if `MujocoOptions.self_collision` is true ([SimConfig](sim.md)). Mass and inertia come from the capsule unless the link gives explicit `inertia`.
