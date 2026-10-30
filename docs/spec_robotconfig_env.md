# RobotConfig, EnvConfig and simulators: design record (v7, as built)

> **Status.** Implemented and tested: the config layer, the CPU MuJoCo simulator with one generic environment, task providers
> (`robosuite`, a source of arenas and objects, is the one provider in the package). Not implemented: the `warp` GPU simulator, `ExternalRobot`, family-reachable target
> regions and end-effector control modes. Written 2026-10-27 as a proposal, brought in line with the code on 2026-10-29, and revised the same day for task
> providers (section 5a; Appendix A is rewritten), and on 2026-10-30 again for the removal of the in-package reach task (section 5b), the simulator naming and `sim.options` (5c).
> Field-level detail lives in the reference pages ([docs/README.md](README.md)), which the tests check against the code;
> this document keeps the purpose, the principles, the decisions and their reasons. Companion note:
> `.agent/quotient_space.md`. Working notes: `.agent/memory.md`.

## 1. Purpose and scope

"Quotient" is the downstream research project that studies relationships between embodiments performing the **same
control task**. `robotdesigner` provides:

1. an embodiment described as validated, hashable data (`RobotConfig`);
2. batched RL environments that instantiate it on several simulators;
3. task descriptions that do not depend on the embodiment;
4. traceability: every run maps to a configuration hash.

From `.agent/quotient_space.md`: an n-DoF arm in an RL environment with `n` as a parameter, and the robot configuration
kept separate from the env.

**Non-goals (v1):** training code or trainer-specific glue; JAX / MJX and Playground simulators (shelved); multi-robot,
mobile or legged robots (two-arm robosuite environments are refused); autodiff through the simulator; non-MuJoCo
simulators; a Gymnasium-native contract (an adapter may follow).

## 2. Principles

1. **The robot is data.** `RobotConfig` is frozen, validated, hashable and serializable, and imports no engine.
2. **Simulators realize configs; envs consume them.** Dependencies point from simulators to the config layer, never back.
3. **One batched env contract:** every simulator returns a TorchRL `EnvBase` over TensorDicts, with `batch_size = [num_envs]`.
4. **The task is embodiment-independent and recorded.**
5. **Everything that changes behaviour is in `EnvConfig` and its hash.** No hidden defaults. The one exception is
   deliberate: robosuite's own defaults for task arguments you do not set are recorded on the environment
   (`resolved_env_kwargs`) but are not hashed.
6. **A simulator refuses what it cannot run**, with every problem listed at once; it never approximates.

## 3. Architecture and packaging

```
robotdesigner.config               numpy only: configs, generators, validation, hashing, kinematics oracle
robotdesigner.make_env             make_env(cfg), the one step from an EnvConfig to an environment (robotdesigner.envs.make_env)
robotdesigner.viz                  clips, frame recorder, camera framing (saving needs the viz extra); see viz.md
robotdesigner.simulators             simulators: protocol, capabilities, lazy registry
robotdesigner.simulators.mujoco      CPU reference simulator and its generic env; arm_spec.add_arm builds the chain
robotdesigner.tasks                task contract (base.py); one subpackage per provider namespace, found by config.providers
robotdesigner.tasks.robosuite      robosuite arenas and objects as a task; the compatibility layer
(planned) simulators.warp            GPU training
```

- `config` imports no engine, no torch and no robosuite (a test checks it). Simulators are imported lazily by name
  (`sim.simulator`); a simulator imports its engine inside `validate` and `make_env`.
- `config.kinematics` is a pure-numpy forward-kinematics implementation of the robot config: the engine-independent
  oracle against which compiled models are tested.

**Packaging** (versions read from PyPI on 2026-10-27):

```
base        numpy>=1.26                                     # robotdesigner.config
[env]       torch>=2.7, tensordict>=0.14.3,<0.15, torchrl>=0.14.1,<0.15, mujoco>=3.13
[warp]      [env] + mujoco-warp, warp-lang>=1.15            # declared; simulator not implemented
[robosuite] [env] + robosuite>=1.5,<1.6
[viz]       pillow, imageio, imageio-ffmpeg
[dev]       pytest, ruff
```

- `requires-python = ">=3.10"`; CI covers 3.10, 3.12 and 3.13, plus a separate job for the `robosuite` marker (added, not
  yet run on GitHub); development default 3.12. uv with a committed lockfile, src layout.
- torchrl 0.14.1 pins tensordict below 0.15, and tensordict 0.15.3 needs torch 2.11 or newer, so the 0.14 line serves
  torch 2.7 to 2.8 environments. Widen the pins only after CI passes on a newer line.
- numpy 1.26 is the floor because older downstream environments pin it on Python 3.10; mujoco 3.13 is the floor because
  mujoco-warp requires it.
- robosuite 1.5.x declares `mujoco>=3.3.0` but does not run on MuJoCo 3.10 or later as released. Rather than capping
  MuJoCo (which would conflict with the Warp floor), `tasks/robosuite/_compat.py` patches the two broken code paths in
  the running process (section 5).

## 4. Configuration objects

```
EnvConfig(robot, control, task, sim)
```

| section | class | reference |
|---|---|---|
| robot | `ParametricArm` (a `RobotConfig`) | [robot.md](configs/robot.md) |
| control | `ControlConfig` with `FixedGains` or `InertiaScaledGains` | [control.md](configs/control.md) |
| task | `TaskConfig` with the params class of the task's provider | [task.md](configs/task.md), [robosuite.md](tasks/robosuite.md) |
| sim | `SimConfig` (names a simulator and carries its options) | [sim.md](configs/sim.md) |
| simulator options (inside `sim.options`) | `MujocoOptions` | [sim.md](configs/sim.md#simulator-options) |

As built, and where it differs from the first proposal:

- **RobotConfig.** One kind exists, `ParametricArm`: segments of a hinge joint and a capsule link, in kinematic order, with
  `n = len(segments)` and `chain()` as the generator. Every dynamic parameter is required. `ExternalRobot` (a reference to
  an existing model, for baselines such as the stock Panda) is designed but not implemented. The hash is the SHA-256 of the
  canonical JSON without the label; quaternion signs are canonicalized so the first non-zero component is positive.
- **ControlConfig.** Six mode names are declared; the MuJoCo simulator implements four (`joint_pos_abs`, `joint_pos_delta`,
  `joint_vel`, `joint_torque`), and no simulator implements the end-effector modes. Hardware limits stay in the robot (`effort_limit`).
- **TaskConfig.** An id is `<namespace>/<name>`; the namespace selects a task provider (section 5a) that owns the params
  class, so the core knows no task. A target region is `FixedBox` or `OwnReachable()` (no margin); a family-reachable
  region is deferred. Tasks that define their own goal take `None` for `target_region` and `success_tolerance`.
- **SimConfig.** `simulator, num_envs, device, dt, integrator, control_freq, seed, scene, options`, all required, no defaults (the
  proposal planned defaults for `dt` and `integrator` and a `render` field; neither exists). `simulator` names a simulator. `1 / (control_freq * dt)`
  must be a positive integer. Rendering is not a config field: `env.render()` on the MuJoCo env and `robotdesigner.viz`
  produce frames and clips ([viz.md](viz.md)).
- **EnvConfig.** `resolve()` is the identity today (nothing needs resolving until family-reachable regions exist);
  `env_hash` covers the robot's full hash, control, task and sim (the simulator's options are inside `sim`), so `seed` and `num_envs` are part of it.

## 5. Simulators

A simulator implements `capabilities()`, `validate(cfg)` (a list of problems, before anything is built) and `make_env(cfg)`.

| | `mujoco` | `warp` (planned) |
|---|---|---|
| Role | reference, debugging | training |
| Device | CPU | CUDA |
| Robot | ours, compiled to `MjSpec` | ours |
| Scene, task | a scene preset plus whatever the task's provider attaches | the same |
| Control | ours: actuators and the env layer | ours |
| Stepping | N `MjData` stepped in a loop | MuJoCo Warp, `nworld = num_envs` |

- **`mujoco`.** `ParametricArm` compiles, with a scene preset and the task's assets attached to the same `MjSpec`, to one
  `MjModel`; the compile also produces the `RobotInterface`. `MjSpec` reads angles as degrees unless
  `compiler.degree = False`; the compile sets it (an early version did not, and joint limits came out 57 times too small).
- **`warp`.** Not started. The plan: the same `MjModel` goes through `put_model` and `make_data(nworld, ...)`, state is
  exposed as torch tensors, parity is tested against the CPU simulator from injected state, and per-world model fields may
  let same-topology embodiments share one simulation (different `n` needs a separate simulation per topology group).
- **Shared robot construction.** `simulators/mujoco/arm_spec.add_arm` builds the links, joints and capsules for every
  MuJoCo-based simulator, so the physical robot is identical across them.

### 5a. Task providers (2026-10-29)

The first version had the backend choose the task (simulators were then called backends, see 5c): `MujocoBackend` built the
reach environment, `RobosuiteBackend` ran robosuite's own simulation, and each listed task ids in its capabilities. Two
things were wrong with that. A simulator should be able to run any task, and a task source (robosuite) is not a simulator,
yet it was a value of `sim.backend`, with its own controller and physics, so embodiments could not be compared across tasks
on one control path.

- **`env = make_env(cfg)`** is the only construction path. `EnvConfig` stays data (no `make()`, so `config` imports no engine).
- **A task provider owns a namespace.** Config side: params classes (numpy only). Engine side: a `TaskImpl` that adds scene
  assets to the model spec, places things at reset, observes, scores. `validate(cfg, simulator)` collects the provider's
  problems with the simulator's into one `ConfigError`; `Capabilities` has no task ids.
- **One generic environment per simulator.** `MujocoEnv` owns the robot, control law, batching, masked reset, episode
  counters and rendering; a task states only what it adds.
- **No task is named in the core.** Providers are the subpackages of `robotdesigner.tasks`, the entry-point group
  `robotdesigner.task_providers`, or `register_task_provider`; `tests/test_no_hardcoded_tasks.py` reads the names from the
  registry and fails if config, simulators, envs, viz or the task contract mention one.
- **robosuite is a source of assets** (Appendix A). Reward and success are ours.
- Moving the reach task out of the core changed nothing it computes: rollouts of four configs (absolute, delta with a masked
  reset, torque with self-collision, velocity) were bit-identical to those of the previous version, and so were the `env_hash`es.

### 5b. No task ships except robosuite's (2026-10-30)

The package no longer contains any task of its own. The reach task, its `reach_task` helper and its page are gone: the
package is a framework for loading embodiments into an RL env, tasks are what a provider brings, and the one provider in
the package is `robosuite` (arenas and objects). Consequences:

- The engine tests (batched env, control modes, masked reset, seeding, rendering overlays, goal regions) run on a test-only
  provider, `tests/toy_provider` (namespace `toy`, registered by `tests/conftest.py`); it is also the smallest example of a
  provider. The reach `env_hash`es of section 5a no longer exist in the library.
- The goal machinery (`FixedBox`, `OwnReachable`, `uses_goal`, `success_tolerance`, `target_region`) stays in the config
  layer: it is the contract goal tasks of any provider use.
- **The task has its own random stream.** The environment keeps two generators seeded from `sim.seed`: one for the reset noise of
  the robot (its shape depends on `n`) and one for the task (`rand` in `TaskImpl.reset`). Before this change a task's draws
  followed the robot's, so two embodiments with the same seed got different placements; now they get the same ones, which is
  what comparing embodiments on one task needs. Reset noise and task draws changed relative to the previous commit for a
  given seed; the config and its hash did not.

### 5c. Simulator naming and `sim.options` (2026-10-30)

Two changes, one commit, no compatibility aliases (nothing outside this repository depended on the old names):

- **Names.** "Backend" is a code and dependency-management word: a library that is loaded and makes functions and classes
  available. What `SimConfig` names is the **simulator** (a physics engine such as MuJoCo; later MuJoCo Warp). So the field is
  `sim.simulator`, the package is `robotdesigner.simulators` (registry: `register_simulator`, `get_simulator`,
  `list_simulators`; protocol `Simulator`; `MujocoSimulator`), the options registry is `SIMULATOR_OPTIONS` /
  `register_simulator_options`, and a task provider's `validate` / `build` take the simulator name. Text of this record from before
  this date, and `.agent/memory.md`, use the old word; read "backend" there as "simulator". `build-backend` in `pyproject.toml`
  is packaging terminology and stays.
- **Options live in `SimConfig`.** `EnvConfig` had a fifth field, `backend_options`, the options class of the simulator named in
  `sim`. A separate field meant two fields that had to agree and a cross-check in `EnvConfig`. Now `SimConfig(simulator, ..., scene,
  options)` carries them and checks `options` against the class registered for `simulator` (error paths `options` and `simulator`);
  `EnvConfig` has four fields. Serialization nests them (`sim.options`), so every `env_hash` changed; `robot_hash` did not.
  Old config dicts do not load.

## 6. Env contract (TorchRL)

```
env.batch_size = [num_envs]       # batch-locked
env.reset(td=None) -> td          # td["_reset"], bool [num_envs, 1], selects envs; absent = reset all
env.step(td)       -> td          # reads td["action"]; outputs live under ("next", ...)
env.step_and_maybe_reset(td)      # driver-side autoreset: reads the done keys, writes "_reset"
env.rollout(max_steps, policy)    # TorchRL's own; this package ships no rollout helper
env.set_seed(seed)
env.robot_hash, env.env_hash
```

Subclasses implement `_reset`, `_step` and `_set_seed` and declare the specs; they never override `reset` or `step`.

- **Observation keys are declared per simulator.** `qpos` and `qvel` are required in `proprio`; `effort` exists on the MuJoCo
  simulator only; the `task` group depends on the task. Analyses that must run everywhere use the required subset. Details:
  [simulators.md](simulators.md#the-environment-contract).
- **No autoreset inside the env.** The driver reads the done keys and writes `_reset`.
- **Partial reset** honours the `_reset` mask. TorchRL does not verify that an env implements it, so each simulator has its
  own test.
- **Injected state:** optional entries in the reset TensorDict (`qpos`, `qvel`, and whatever the task accepts, such as reach's `target`) override random
  initialization, for parity tests and for replaying a world on another simulator.
- **Seeding:** the same simulator, device and seed give the same results; random resets differ across simulators, so parity uses
  injected state.
- **Not implemented:** flat-observation views and padding to a maximum `n`. Heterogeneous families (different `n`) have
  different specs; combining them with lazy stacking is untested.
- **Conformance:** `check_env_specs(env)` passes for every task the tests build.

## 7. Verification

- **Config:** validation errors; hash stability (key order, `-0.0`, scaled axes); generator invariants; `config` imports
  without torch or any engine.
- **Documentation:** the Python examples in `docs/configs/*.md` and `docs/README.md` run, and every relative link and
  anchor resolves (`tests/test_docs.py`).
- **Kinematics oracle:** for several `n`, axis patterns and parameter sets, the end-effector pose of the compiled model
  matches `config.kinematics`; the same comparison holds for the robot inside robosuite.
- **MuJoCo env** (with a test-only provider, `tests/toy_provider`, that has a reach task): `check_env_specs`; masked reset; seeding determinism; injected state; control modes; joint limits are in
  radians.
- **robosuite scenes** (marker `robosuite`): several environments build and step; control holds and tracks; seeding and
  masked reset are independent per sub-environment; validation lists every problem. Run on robosuite 1.5.1 and 1.5.2 with
  MuJoCo 3.17; the compatibility layer is also tested across MuJoCo 3.3.1 to 3.17.
- **Parity CPU against Warp:** not applicable until the Warp simulator exists.

## 8. Status

Run and checked:

- the CPU MuJoCo env on the test-only reach task; the whole suite passes;
- robosuite scenes as tasks: 13 of the 15 single-arm robosuite environments build, reset and step with a generated arm and pass
  `check_env_specs`; the hand pose in robosuite equals the numpy kinematics; hold and tracking behave as configured;
- the compatibility layer on MuJoCo 3.3.1 to 3.17.

Not done or not verified:

- the Warp simulator, its masked reset, per-world model fields and CPU-versus-Warp parity;
- policy performance on any task; robosuite throughput with many sub-environments (they are stepped one after another);
- robots other than generated arms (no `ExternalRobot`, so no stock-Panda baseline through our contract);
- the `robosuite` CI job, which has been added but not run on GitHub;
- lazy stacking of heterogeneous families.

## 9. Decisions

**Adopted:**

1. Segment-based `ParametricArm` with generators; `ExternalRobot` only for baselines.
2. Control mode lives in `ControlConfig`, not in the robot.
3. `config` is numpy-only; the env layer requires torch, tensordict and torchrl.
4. The env contract is TorchRL `EnvBase`; the env never autoresets; nested observations are primary.
5. GPU simulator: MuJoCo Warp driven from torch. JAX is shelved. CPU MuJoCo is the reference and debugging simulator.
6. Frozen dataclasses with hand-written validation; packaging as in section 3.
7. robosuite is a source of task assets, generic over environments (`robosuite/<EnvName>`): its arena, objects and placement
   rule are loaded into our MuJoCo simulation; we supply the robot, the control, the physics settings, success and
   reward. (Replaces the first design, in which robosuite ran its own simulation and controller as a simulator.)
8. Control means the same for every task because there is one control path. (The first design drove robosuite's joint
   controller with our accumulated target to approximate that.)
9. robosuite's incompatibility with current MuJoCo is patched at run time, only where the installed MuJoCo needs it,
   instead of capping MuJoCo.
10. Tasks that define their own goal take `None` for `target_region` and `success_tolerance`; passing a value is an error.
11. Environments whose observation keys change between episodes (`PickPlaceSingle`, `NutAssemblySingle`) are refused, and
    two-arm environments are refused.
12. `.agent/` is committed, with no machine-specific strings.
13. `sim.simulator` names a simulator only; a task source is never a simulator.
14. The core names no task, and a test enforces it. Providers register themselves.
15. Objects without a free joint (fixtures) are placed once at build time, because the envs of a batch share one model.

**Open:**

- Family batching on the GPU: separate simulation per topology group, lazy stacking, or padding.
- The smallest `n` that can physically solve a manipulation task such as `Lift`; infeasibility must be separable from
  failures of learning.
- Whether to run robosuite environments in worker processes for throughput.
- An `ExternalRobot` baseline (the stock Panda) through the same contract.

## 10. Milestones

1. **Config layer and CPU simulator.** Done. `dt` and `integrator` stayed explicit (no defaults).
2. **CPU MuJoCo env (TorchRL).** Done: `check_env_specs`, masked `_reset`, seeding, injected state. It was first built around
   a reach task; that task is no longer in the package and survives as a test-only provider.
3. **`warp` simulator.** Not started.
4. **robosuite.** Done as a source of assets (Appendix A) for generated arms: scenes of any single-arm environment that has
   a physical object, with objectives of our own. The stock-Panda baseline needs `ExternalRobot` and has not been done.
5. **Later:** Playground and JAX simulators.

## Appendix A. robosuite as a task provider, as built

Package `robotdesigner.tasks.robosuite`. Reference: [tasks/robosuite.md](tasks/robosuite.md).

- **Loading.** `scene.load_scene` builds the robosuite environment once, with a stock Panda and without rendering, seeded
  from `sim.seed` (robosuite draws object sizes while it builds). It keeps the arena and objects (as one MJCF document
  made by `ManipulationTask` without robots), the placement sampler and the position where robosuite mounts a stock arm,
  then closes the environment. Its simulation is never stepped.
- **Attaching.** `clean_xml` drops the `<option>`, `<size>` and `<visual>` sections (the simulator settings are ours) and
  the IK mocap bodies; `MjSpec.attach` merges the rest under the prefix `rs_`. The arena brings its own floor, so
  `sim.scene` must be `empty`. Physics settings follow `SimConfig` and MuJoCo's defaults.
- **Placement.** At every reset, per env, robosuite's own sampler draws poses for the objects that have a free joint, with
  the global numpy stream (robosuite 1.5.1) or the sampler's generator (1.5.2, not exercised) seeded from the task's
  random stream, and restored afterwards. Objects without a free joint are placed once when the task is bound and written
  into the shared model.
- **Objective.** `LiftObject(object, height)` and `ReachObject(object, tolerance)`; reward is the negative distance from the
  end effector to the object. Observations are `ee_pos` and `<object>_pos` / `<object>_quat` for physical objects, in the
  robot's base frame.
- **Versions.** `_compat.apply_compat` patches two robosuite 1.5.x code paths that break on MuJoCo >= 3.10 (joint-address
  enum comparison, the mass-matrix call), each only if its probe fails. It is needed because building a robosuite environment
  runs its controller setup.
- **Not carried over from the first design:** the generated robot class and gripper (`arm.py`), robosuite's controller,
  `RobosuiteOptions.base_placement`, the per-sub-environment robosuite instances and their random streams, the fixed
  `dt = 0.002`. `default_mount(env_name)` replaces `env_default` placement: it returns the pose, you use it (or not) as the
  base of your arm.
