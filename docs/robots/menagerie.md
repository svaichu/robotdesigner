# Robot provider: `menagerie`

The robot provider `menagerie` turns a model of [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) into an [`ExternalRobot`](../configs/robot.md#externalrobot-a-model-that-lives-elsewhere),
through the `mujoco-menagerie` library. Install it with `pip install 'robotdesigner[menagerie]'` (Python 3.10.12 or later).

**No model is copied into this repository.** The library resolves a model on first use and caches it outside the repository tree (`MENAGERIE_CACHE_DIR` sets the cache directory;
`MENAGERIE_ROOT` uses a checkout instead). Licences differ per model, so each robot's licence is in the table below and `BaseEnvConfig.explain()` shows it as `robot.license`.

The config side needs neither the library nor an engine, so a config naming this provider can be written, hashed and stored anywhere:

```python
from robotdesigner.config import EndEffectorRef, ExternalJoint, ExternalRobot, Pose, list_robot_providers
from robotdesigner.robot_providers.menagerie import cards

assert "menagerie" in list_robot_providers()
ur5e_like = ExternalRobot(
    provider="menagerie", name="universal_robots_ur5e", entry="ur5e", source="a92a363f",
    joints=(ExternalJoint("shoulder_pan_joint", (-6.2832, 6.2832), 150.0),),
    home_qpos=(-1.5708,), home_root=None, keyframe="home",
    end_effector=EndEffectorRef(site="attachment_site", body=None, offset=Pose.identity()), actuator_kind="position",
)
assert ur5e_like.n == 1 and ur5e_like.provider == "menagerie"
assert "universal_robots_ur5e" in cards() and cards()["franka_emika_panda"]["end_effector"]["body"] == "hand"   # data, not code
```

With the library installed, `describe` opens the model and fills these numbers in:

```python no-run
from robotdesigner.config import BaseEnvConfig
from robotdesigner.robot_providers.menagerie import PROVIDER

panda = PROVIDER.describe("franka_emika_panda")        # 7 driven joints, position servos, end effector on the hand body
env = BaseEnvConfig().robot(panda).sim(num_envs=4, scene="plain_floor").build()
go2 = PROVIDER.describe("unitree_go2")                  # 12 joints, plain motors, a free root with a home pose
PROVIDER.describe("kuka_iiwa_14", effort_limit=200.0)   # a model without force limits needs one
```

## What `describe` does

`describe(name, entry=None, *, controlled=None, end_effector=None, keyframe=..., effort_limit=None)`:

- **entry**: the card's, else the library's default model. A scene entry is refused: the compile and the scene presets bring floor and lights.
- **controlled**: the card's, else every joint with an actuator that has a joint transmission, in actuator order. Free joints are never driven; a joint a model drives through a tendon is not driven.
- **range / effort_limit**: the joint range when limited, else `None`. The effort limit is the force range of the actuator, else the actuator-force range of the joint, else, for a plain motor, its
  `ctrlrange` times gain and gear. Otherwise `describe` raises a `ConfigError` at `effort_limit` that names the joint and the argument to pass.
- **keyframe / home**: the card's keyframe, else the model's `home`, else none (then `qpos0`). `home_qpos` is that state at the driven joints; `home_root` is the free joint's pose. A home value
  outside its range is clipped, with a warning.
- **end_effector**: the card's, else a site with a conventional name (`attachment_site`, `tcp_site`, `pinch_site`, `gripperframe`, `pinch`, `ee_gripper_link`, `link_tcp`), else none. A task that
  needs an end effector then refuses the robot.
- **actuator_kind**: from the gain and bias of the driven actuators: a position servo, a velocity servo, a plain motor, or `mixed`.

A **card** (`robot_providers/menagerie/cards.json`) holds what cannot be derived: the driven joints where a finger is a joint actuator, an end effector where the model has no suitable site (the
offsets of body end effectors approximate the gripper mounting point, they are not calibrated), a keyframe that is not called `home`. Every arm and quadruped has one; the test suite checks each against its model.

`spec(robot)` returns the model's `MjSpec` after checking `robot.source` (the library's content id of the model directory) and fails with `robot.source: the model changed since this config was written` otherwise.
`validate` reports the library missing (naming the extra), an unknown robot (with the library's near match), an unknown or scene entry and a stale source, each with its config path.

## Which robots run

`tests/menagerie_expected.json` records the status of every robot of the categories `arm`, `quadruped` and `mobile_manipulator` when described with no arguments and run with the `empty/idle`
task on the `plain_floor` scene (2 envs, 10 zero steps and 10 random steps, finite observations). The suite fails when a robot flips, and the table below is checked against that file. Today 32 run
and 9 are refused; `refused: needs effort_limit` means the model has no force limit and `describe(..., effort_limit=...)` supplies one; `refused: mixed actuators` means the driven joints have more than
one kind of actuator, so a smaller `controlled` tuple is needed.

| robot | category | licence | status |
|---|---|---|---|
| `agilex_piper` | arm | MIT | ok |
| `arx_l5` | arm | BSD-3-Clause | ok |
| `dynamixel_2r` | arm | MIT | refused: needs effort_limit |
| `fanuc_crx5ia` | arm | Apache-2.0 | ok |
| `flexiv_rizon4` | arm | Apache-2.0 | ok |
| `flexiv_rizon4s` | arm | Apache-2.0 | ok |
| `franka_emika_panda` | arm | Apache-2.0 | ok |
| `franka_fr3` | arm | Apache-2.0 | ok |
| `franka_fr3_v2` | arm | Apache-2.0 | ok |
| `i2rt_yam` | arm | MIT | ok |
| `kinova_gen3` | arm | BSD-3-Clause | ok |
| `kuka_iiwa_14` | arm | BSD-3-Clause | refused: needs effort_limit |
| `low_cost_robot_arm` | arm | Apache-2.0 | ok |
| `rethink_robotics_sawyer` | arm | Apache-2.0 | ok |
| `robotis_omy_3m` | arm | Apache-2.0 | ok |
| `robotstudio_so101` | arm | Apache-2.0 | ok |
| `seeed_rebot_devarm` | arm | MIT | ok |
| `trossen_vx300s` | arm | BSD-3-Clause | ok |
| `trossen_wx250s` | arm | BSD-3-Clause | ok |
| `trossen_wxai` | arm | BSD-3-Clause | ok |
| `trs_so_arm100` | arm | Apache-2.0 | ok |
| `ufactory_lite6` | arm | BSD-3-Clause | ok |
| `ufactory_xarm7` | arm | BSD-3-Clause | ok |
| `unitree_z1` | arm | BSD-3-Clause | ok |
| `universal_robots_ur10e` | arm | BSD-3-Clause | ok |
| `universal_robots_ur5e` | arm | BSD-3-Clause | ok |
| `anybotics_anymal_b` | quadruped | BSD-3-Clause | ok |
| `anybotics_anymal_c` | quadruped | BSD-3-Clause | ok |
| `boston_dynamics_spot` | quadruped | BSD-3-Clause | ok |
| `google_barkour_v0` | quadruped | Apache-2.0 | refused: needs effort_limit |
| `google_barkour_vb` | quadruped | Apache-2.0 | ok |
| `unitree_a1` | quadruped | BSD-3-Clause | ok |
| `unitree_go1` | quadruped | BSD-3-Clause | ok |
| `unitree_go2` | quadruped | BSD-3-Clause | ok |
| `google_robot` | mobile manipulator | Apache-2.0 | ok |
| `hello_robot_stretch` | mobile manipulator | BSD-3-Clause-Clear | refused: needs effort_limit |
| `hello_robot_stretch_3` | mobile manipulator | Apache-2.0 | refused: needs effort_limit |
| `pal_tiago` | mobile manipulator | Apache-2.0 | refused: mixed actuators |
| `pal_tiago_dual` | mobile manipulator | Apache-2.0 | refused: mixed actuators |
| `rainbow_robotics_rby1` | mobile manipulator | Apache-2.0 | refused: needs effort_limit |
| `stanford_tidybot` | mobile manipulator | MIT | refused: needs effort_limit |
