# Design

Why the package looks the way it does. Reference for each part is in the pages it links to; this page is the reasoning and the boundaries.

## Purpose

A framework that loads **different robot configurations into one RL environment**, so that other projects can study how embodiment relates to control: the same task, the same
control interface, the same seeds, with only the robot changed. The first family is an `n`-joint arm with `n` as a parameter, kept separate from the environment.

## Principles

1. **The robot is data.** A `ParametricArm` is frozen, validated and serializable, with a hash that ignores labels and float noise. A rollout can always be traced to an embodiment, and two
   embodiments can be compared, stored and rebuilt on another machine from a dict.
2. **No hidden defaults in the core.** `EnvConfig` requires every field. Defaults exist in exactly one place, the builder (`BaseEnvConfig`), are listed, and are versioned. An environment is
   identified by `env_hash` of a complete config.
3. **Configs are separate from engines.** `robotdesigner.config` and `.robot` need numpy only and never import a simulator, torch or robosuite. Configs can be written, validated, hashed and swept on a
   machine that cannot run a simulation. Engine code loads when an environment is built.
4. **Reject, never approximate.** A combination the chosen simulator cannot run (control mode, device, task, scene) is rejected up front with every problem listed and the config path to
   change. Nothing is silently substituted.
5. **One construction path.** `make_env(cfg)` for every task and every simulator. One environment contract (a batched TorchRL `EnvBase`), which the simulators implement identically.
6. **The task is a first-class object that does not know the robot.** It never mentions `n` or a link length. Its random draws come from their own stream that does not depend on the robot,
   so the same seed gives every embodiment the same scene.
7. **The core names no task.** Tasks come from providers. The single exception is the placeholder `empty/idle`, so that a default environment can run. A test enforces it.

## What is compared, and what is held fixed

Comparisons across embodiments are easily confounded, so the framework makes the controls explicit:

| confound | how it is controlled |
|---|---|
| `n` changes reach, mass and inertia together | `chain(total_length=, total_mass=)` splits fixed totals over `n` links |
| a fixed target is unreachable for small `n` and trivial for large `n` | goal tasks draw targets from the arm's own reachable set (`OwnReachable`) or from a box in the base frame |
| equal gains on every joint make control differences partly gain mismatch | gains are an explicit, recorded choice; `InertiaScaledGains` tunes each joint to a bandwidth |
| different scene randomness per robot | the task's own stream, seeded by `sim.seed`, independent of the robot |
| different simulators give different numbers | one reference simulator (CPU MuJoCo), a GPU twin held to it by parity tests, and the MuJoCo version recorded per study |

## Boundaries (not built, on purpose)

- **No cross-embodiment view.** Observation and action sizes vary with `n`; padding, masks or per-embodiment heads belong to the project that studies it, not here.
- **No trainer.** No RL library shapes the design; the builder borrows the ergonomics of RLlib-style config objects but imports nothing from them. The environment is a TorchRL `EnvBase`, which any
  loop can drive.
- **No job launcher.** `sweep` expands, you launch ([cluster](cluster.md)).
- **robosuite is a source of assets, not a simulator.** Success, reward, control and physics settings are ours, so every task runs through the same simulation path.
- **Only hinge joints and capsule links** in a `ParametricArm`. A robot that exists as a model elsewhere is an `ExternalRobot`, a reference with the numbers the config layer needs. Robot kinds are a capability of a simulator, so a new kind is added without touching tasks.
- **Not yet:** end-effector control modes (`ee_pos_delta`, `ee_pose_delta` are reserved names that are rejected), grippers on the Warp simulator, a task that defines its own reward beyond the
  simple distance reward of the robosuite objectives.

## Vocabulary

The words are used with one meaning throughout; old commit messages and issues may say "backend" for what is now **simulator**.

| word | meaning |
|---|---|
| **embodiment** | a robot configuration (`ParametricArm`) |
| **simulator** | a physics engine realised as code (`robotdesigner.simulators.<name>`), named by `sim.simulator`: `mujoco`, `warp` |
| **task** | `"<namespace>/<name>"`; defined by a params object of a provider |
| **task provider** | the owner of a namespace; builds tasks for simulators |
| **robot provider** | the owner of a robot namespace (`config.robot_providers`); reads a model, opens it as a spec and gives a kinematics oracle |
| **external robot** | `ExternalRobot`: a reference to a model of a robot provider, plus joints, home posture and end effector as numbers; driven with `NativeGains` |
| **scene** | `sim.scene`, a preset of the simulator (`empty`, `plain_floor`). A *robosuite scene* is an arena plus objects brought by a task |
| **mount frame** | the scene's place for a robot, a world pose the task provider names from the task's params |
| **mount** | what the task adds to the frame: `Mount(offset)` for a fixed-base arm, `MobileMount(spawn, heading)` for a floating-base robot; the world pose is `world_mount(cfg)` |
| **options** | the settings of one simulator, in `sim.options` |
| **config side / engine side** | numpy-only classes versus code that needs torch and MuJoCo |
| **resolve** | turn a `BaseEnvConfig` into the frozen `EnvConfig` with every value explicit |

## Decisions that shaped the API

Dated in the project history; here in outline.

- The GPU path is MuJoCo Warp driven from torch; the environment contract is TorchRL's `EnvBase`. A JAX path was shelved.
- robosuite 1.5.x does not run on MuJoCo >= 3.10 as released. The provider patches the incompatibility at run time (`apply_compat`) instead of capping the MuJoCo version.
- The params object *is* the task: its class names the provider, its `name` names the task, and the id is derived. There is no second place to state a provider.
- The robot has no pose. The scene owns the mount frame and the mount lives in `TaskConfig` as a relative placement, because scenes disagree about where an arm belongs, and `robot_hash` then identifies the embodiment alone.
- `horizon` became `max_episode_steps` (Gymnasium's name for a time limit); the old key is still read, with a warning.
- `env.observation_space` and `env.action_space` are per-env shapes derived from the TorchRL specs, not Gymnasium spaces.
- `BaseEnvConfig` holds the defaults; changing one raises `DEFAULTS_VERSION` because it changes hashes.
- The golden hashes in `tests/golden_hashes.json` pin `robot_hash` and `env_hash` of a fixed set of configs. A change that moves one is intended or a bug; if intended, regenerate the file with `python -m tests.test_golden_hashes` in the same pull request and add a row here.
- A robot that exists as a model elsewhere is an `ExternalRobot` (a reference, hashed with the provider's content id), found through robot providers that mirror task providers. By default it is driven by the model's own actuators (`NativeGains`); `FixedGains` and `InertiaScaledGains` replace the actuators on the driven joints with the ones a `ParametricArm` gets (re-actuation, a go on the spike of #52), so an external arm and a parametric one can be compared under one control law. The model's damping, armature and friction stay; only actuators are replaced.
- The CPU simulator runs an `ExternalRobot` by attaching the provider's `MjSpec` into a frame at the mount, next to an empty welded `base`, with the prefix `ext_`. Native control maps the action to the actuator's `ctrlrange`; the scene keeps `dt` and the integrator.
- Robots of MuJoCo Menagerie come through the robot provider `menagerie`: `describe` derives an `ExternalRobot` from the model, a small data table of cards holds what cannot be derived, nothing of a model is copied into the repository, and `tests/menagerie_expected.json` records which robots run.
- 2026-11-08, go on the re-actuation spike (#52): the native settings stay the default of an `ExternalRobot` and `FixedGains` / `InertiaScaledGains` are accepted with it. Measured: with the model's own numbers the Panda reproduces the native trajectory exactly and the UR5e does too for equal control targets; with equal *actions* the two differ because native control maps the action onto the actuator's `ctrlrange` and re-actuation onto the joint range, which the library rounds differently for the UR5e (6.2831 against 6.28319). Kept as specified.
- The Warp simulator runs an `ExternalRobot` with the CPU compile and a reset that writes the same state. Parity with the CPU simulator is measured per robot and mode (docs/simulators.md); where it is outside the tolerances the page says why, and no tolerance was loosened. The CUDA path is not verified for external robots.
- 2026-11-08, the contact and solver options of an attached model (cone, impratio, solver, iterations) are **not** carried over: the scene's win, so that every embodiment sees one physics for a task. Measured on Go1 and A1 (benchmarks/menagerie_options.py): both stand either way, carrying changes tilt by 0.1 to 0.2 degrees. `external.CARRY_MODEL_OPTIONS` keeps the hook; no field on `ExternalRobot`.
- 2026-11-09, the scene owns the mount frame (#42, #43): `TaskConfig.mount` is `Mount(offset)` for a fixed-base arm or `MobileMount(spawn, heading)` for a floating-base robot, relative to `TaskProvider.mount_frame(params)`, and `world_mount(cfg)` is the only place that composes them (`T_world_base = T_frame @ T_mount`). `default_mount(name)` is removed, with no alias; a `Pose` at `task.mount` is an error that shows the replacement. A `Mount` on a floating-base robot and a `MobileMount` on a fixed-base one are errors at `task.mount`; a floating base is an `ExternalRobot` with a `home_root` (a planar base built from slide joints, as on TidyBot, is not detected and takes a `Mount`). `heading` is a yaw in radians, `spawn` may carry a `z` that is added to the home height. `BaseEnvConfig` has no mount special case any more and `resolve()` calls no provider. `robot_hash` is unchanged; every `env_hash` changed once (the shape of `task.mount`), and the golden hashes were regenerated. `DEFAULTS_VERSION` is not bumped: it holds numbers, and a default config resolves to the same world base pose as before for default `env_kwargs`.
- 2026-11-09, the compile snaps the base onto the scene's mount marker (#44): a task adds a marker site in `attach` and names it with `TaskImpl.mount_site()` (`None` for the presets and `empty/idle`). The compile reads the marker's world pose from a compile of the scene alone plus `mj_forward` (chosen over reading the spec: it works whatever the nesting and prefix of the marker; the scene is static), builds `base` at `marker @ mount`, and raises, naming both poses, when the pose disagrees with `mount_frame(params)` by more than 1e-6. A task without a marker uses the provider's frame. `RobotInterface.mount` and `env.world_mount` hold the compiled pose. Attaching the arm's own spec to the marker site stays deferred.
