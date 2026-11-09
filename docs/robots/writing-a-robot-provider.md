# Writing a robot provider

A robot provider turns a source of robot models into [`ExternalRobot`](../configs/robot.md#externalrobot-a-model-that-lives-elsewhere)s and opens them for the simulator. It is the counterpart of a
[task provider](../tasks/writing-a-provider.md) and is found the same way. The shipped example is [`menagerie`](menagerie.md); `tests/toy_robot_provider/` is a complete small one with its models written inline (no
download), and the engine tests run on it.

A provider has a **config side**, numpy only, importable anywhere, and an **engine side** that needs MuJoCo.

## 1. The config side: names and the registry

The config side is a namespace and the data an `ExternalRobot` carries: `provider` (the namespace), `name`, `entry` (the model file), `source` (a content id you vouch for: a hash of the model, a git tree id; it is
**hashed**, so a changed model is a different robot), the driven `joints` with their ranges and effort limits, `home_qpos`, `home_root`, `keyframe`, `end_effector` and `actuator_kind`. A config that names your
provider validates as soon as the namespace is registered, without loading the provider:

```python
from types import SimpleNamespace

from robotdesigner.config import (
    ConfigError, EndEffectorRef, ExternalJoint, ExternalRobot, Pose, get_robot_provider, list_robot_providers,
    register_robot_provider, unregister_robot_provider,
)

def not_here(*args, **kwargs):
    raise NotImplementedError("engine side: see the next section")


mine = SimpleNamespace(                      # the five methods of the protocol; the engine ones are not called here
    names=lambda: ("hand",), describe=not_here, spec=not_here, validate=lambda robot, simulator: [], kinematics=not_here,
)
register_robot_provider("mine", mine, replace=True)
assert "mine" in list_robot_providers() and get_robot_provider("mine") is mine

robot = ExternalRobot(
    provider="mine", name="hand", entry="hand.xml", source="1f2e3d",
    joints=(ExternalJoint("knuckle", (0.0, 1.5), 5.0),), home_qpos=(0.2,), home_root=None, keyframe=None,
    end_effector=EndEffectorRef(site=None, body="tip", offset=Pose((0.0, 0.0, 0.03), (1.0, 0.0, 0.0, 0.0))),
    actuator_kind="position",
)
assert robot.n == 1 and robot.config_hash != ExternalRobot.from_dict({**robot.to_dict(), "source": "other"}).config_hash

unregister_robot_provider("mine")
try:
    ExternalRobot.from_dict(robot.to_dict())
except ConfigError as e:
    assert e.path == "provider"              # an unregistered namespace is an error at the field
```

## 2. The provider object

Five methods; no engine import at module level (the test `tests/test_packaging.py` imports every provider package with the engines blocked).

| method | what it does | side |
|---|---|---|
| `names()` | the robot names, or patterns; shown when a name is unknown | config |
| `describe(name, **options)` | open the model, read the numbers, return an `ExternalRobot` | engine |
| `spec(robot)` | an editable `mujoco.MjSpec` of the model, **after checking `robot.source`**; a mismatch is a `ConfigError` at `robot.source` ("the model changed since this config was written"), never a silent load | engine |
| `validate(robot, simulator)` | every reason this robot cannot run on that simulator, as `Problem(path, message)`; `make_env` collects them with the simulator's own | may import |
| `kinematics(robot)` | an oracle with `forward_ee_batch(q, base)` and `sample_joint_positions(num, rng)`; for a MuJoCo-based provider, `robotdesigner.simulators.mujoco.external.MujocoKinematics(robot, self.spec(robot))` is one | engine |

```python no-run
from robotdesigner.errors import ConfigError, Problem
from robotdesigner.robot import EndEffectorRef, ExternalJoint, ExternalRobot, Pose


class MyRobots:
    def names(self):
        return ("hand",)

    def describe(self, name, **options):
        import mujoco                                   # the engine, only when called

        spec = self._open(name)
        model = spec.compile()
        ...                                             # joints, ranges, effort limits, home posture, end effector
        return ExternalRobot(provider="mine", name=name, entry="hand.xml", source=self._source(name), ...)

    def spec(self, robot):
        if self._source(robot.name) != robot.source:
            raise ConfigError("robot.source", "the model changed since this config was written")
        return self._open(robot.name)

    def validate(self, robot, simulator):
        return [] if simulator in ("mujoco", "warp") else [Problem("sim.simulator", "...")]

    def kinematics(self, robot):
        from robotdesigner.simulators.mujoco.external import MujocoKinematics

        return MujocoKinematics(robot, self.spec(robot))


PROVIDER = MyRobots()
```

What the simulator does with the model is fixed by the compile, not by the provider: it attaches your `spec` into a frame at the mount with the prefix `ext_`, drives the `joints` with the model's own actuators (one with a joint
transmission each) or replaces them, and holds the others at the keyframe's control ([simulators](../simulators.md#external-robots-on-the-cpu-simulator)). So `describe` has to give joint names as they are in the model, and
a model whose driven joints have no actuator, or two, is reported as a problem by the simulator.

## 3. Register it

Three ways, in order of precedence: `register_robot_provider(namespace, target)` (an object, a class, or a lazy `"module:attr"` string); the entry-point group `robotdesigner.robot_providers` of your package; or a subpackage of
`robotdesigner.robot.providers` for a provider that ships with this package.

```toml
# pyproject.toml of your package
[project.entry-points."robotdesigner.robot_providers"]
mine = "my_package.robots:PROVIDER"
```

The core names no provider and no robot; `tests/test_no_hardcoded_robots.py` fails if it does.

## 4. Test it

A checklist, from the tests of the toy provider and of `menagerie`:

1. **Importing your package imports no engine** (copy `test_menagerie_provider_imports_without_an_engine_or_the_library` of `tests/test_packaging.py`).
2. **`describe` against the model**: the numbers (joint count, ranges, effort limits, home posture, root pose) are the model's; a joint that is not an actuated hinge or slide is refused with a path.
3. **`spec` refuses a changed `source`**, with the message above.
4. **`validate` returns `Problem`s with config paths** for each case it knows: an unknown robot or entry, a stale source, a library that is not installed (naming its extra).
5. **`make_env` builds and steps** the robot on the CPU simulator, with the `empty/idle` task, `plain_floor`, a few steps of random actions, finite observations; a fixed base at a rotated mount, a free root at several poses.
6. **A coverage file** (as `tests/menagerie_expected.json`) if the source has many models: the status of every robot, a test that fails when one flips, and a docs table checked against it.
7. **Licences**: do not copy model files into the repository; fetch them through a library or a cache, and report each licence (a `license(name)` method is shown by `BaseEnvConfig.explain()`).
