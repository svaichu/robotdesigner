# RobotConfig, EnvConfig and backends: design record (v7, as built)

> **Status.** Implemented and tested: the config layer, the native CPU MuJoCo backend (`native/reach`) and the robosuite
> backend (any single-arm environment). Not implemented: the `warp` GPU backend, `ExternalRobot`, family-reachable target
> regions and end-effector control modes. Written 2026-10-27 as a proposal, brought in line with the code on 2026-10-29.
> Field-level detail lives in the reference pages ([docs/README.md](README.md)), which the tests check against the code;
> this document keeps the purpose, the principles, the decisions and their reasons. Companion note:
> `.agent/quotient_space.md`. Working notes: `.agent/memory.md`.

## 1. Purpose and scope

"Quotient" is the downstream research project that studies relationships between embodiments performing the **same
control task**. `robotdesigner` provides:

1. an embodiment described as validated, hashable data (`RobotConfig`);
2. batched RL environments that instantiate it on several simulation backends;
3. task descriptions that do not depend on the embodiment;
4. traceability: every run maps to a configuration hash.

From `.agent/quotient_space.md`: an n-DoF arm in an RL environment with `n` as a parameter, and the robot configuration
kept separate from the env.

**Non-goals (v1):** training code or trainer-specific glue; JAX / MJX and Playground backends (shelved); multi-robot,
mobile or legged robots (two-arm robosuite environments are refused); autodiff through the simulator; non-MuJoCo
simulators; a Gymnasium-native contract (an adapter may follow).

## 2. Principles

1. **The robot is data.** `RobotConfig` is frozen, validated, hashable and serializable, and imports no engine.
2. **Backends realize configs; envs consume them.** Dependencies point from backends to the config layer, never back.
3. **One batched env contract:** every backend returns a TorchRL `EnvBase` over TensorDicts, with `batch_size = [num_envs]`.
4. **The task is embodiment-independent and recorded.**
5. **Everything that changes behaviour is in `EnvConfig` and its hash.** No hidden defaults. The one exception is
   deliberate: robosuite's own defaults for task arguments you do not set are recorded on the environment
   (`resolved_env_kwargs`) but are not hashed.
6. **A backend refuses what it cannot run**, with every problem listed at once; it never approximates.

## 3. Architecture and packaging

```
robotdesigner.config               numpy only: configs, generators, validation, hashing, kinematics oracle
robotdesigner.envs                 make_env(cfg)
robotdesigner.viz                  clips, frame recorder, camera framing (saving needs the viz extra); see viz.md
robotdesigner.backends             protocol, capabilities, lazy registry
robotdesigner.backends.mujoco      CPU reference backend; arm_spec.add_arm builds the chain for every MuJoCo-based backend
robotdesigner.backends.robosuite   robosuite tasks with our robot; the compatibility layer
(planned) backends.warp            GPU training
```

- `config` imports no engine, no torch and no robosuite (a test checks it). Backends are imported lazily by name
  (`sim.backend`); a backend imports its engine inside `validate` and `make_env`.
- `config.kinematics` is a pure-numpy forward-kinematics implementation of the robot config: the engine-independent
  oracle against which compiled models are tested.

**Packaging** (versions read from PyPI on 2026-10-27):

```
base        numpy>=1.26                                     # robotdesigner.config
[env]       torch>=2.7, tensordict>=0.14.3,<0.15, torchrl>=0.14.1,<0.15, mujoco>=3.13
[warp]      [env] + mujoco-warp, warp-lang>=1.15            # declared; backend not implemented
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
  MuJoCo (which would conflict with the Warp floor), `backends/robosuite/_compat.py` patches the two broken code paths in
  the running process (section 5).

## 4. Configuration objects

```
EnvConfig(robot, control, task, sim, backend_options)
```

| section | class | reference |
|---|---|---|
| robot | `ParametricArm` (a `RobotConfig`) | [robot.md](configs/robot.md) |
| control | `ControlConfig` with `FixedGains` or `InertiaScaledGains` | [control.md](configs/control.md) |
| task | `TaskConfig` with `ReachParams` or `RobosuiteParams` | [task.md](configs/task.md) |
| sim | `SimConfig` | [sim.md](configs/sim.md) |
| backend options | `MujocoOptions`, `RobosuiteOptions` | [sim.md](configs/sim.md#backend-options) |

As built, and where it differs from the first proposal:

- **RobotConfig.** One kind exists, `ParametricArm`: segments of a hinge joint and a capsule link, in kinematic order, with
  `n = len(segments)` and `chain()` as the generator. Every dynamic parameter is required. `ExternalRobot` (a reference to
  an existing model, for baselines such as the stock Panda) is designed but not implemented. The hash is the SHA-256 of the
  canonical JSON without the label; quaternion signs are canonicalized so the first non-zero component is positive.
- **ControlConfig.** Six mode names are declared; the native backend implements four (`joint_pos_abs`, `joint_pos_delta`,
  `joint_vel`, `joint_torque`), robosuite one (`joint_pos_delta` with `InertiaScaledGains`), and no backend implements the
  end-effector modes. Hardware limits stay in the robot (`effort_limit`).
- **TaskConfig.** Task ids are `native/reach` and `robosuite/<EnvName>`. A target region is `FixedBox` or `OwnReachable()`
  (no margin); a family-reachable region is deferred. Tasks that define their own goal (every robosuite task) take `None`
  for `target_region` and `success_tolerance`.
- **SimConfig.** `backend, num_envs, device, dt, integrator, control_freq, seed, scene`, all required, no defaults (the
  proposal planned defaults for `dt` and `integrator` and a `render` field; neither exists). `1 / (control_freq * dt)`
  must be a positive integer. Rendering is not a config field: `env.render()` on the native env and `robotdesigner.viz`
  produce frames and clips ([viz.md](viz.md)).
- **EnvConfig.** `resolve()` is the identity today (nothing needs resolving until family-reachable regions exist);
  `env_hash` covers the robot's full hash, control, task, sim and options, so `seed` and `num_envs` are part of it.

## 5. Backends

A backend implements `capabilities()`, `validate(cfg)` (a list of problems, before anything is built) and `make_env(cfg)`.

| | `mujoco` | `robosuite` | `warp` (planned) |
|---|---|---|---|
| Role | reference, debugging | library tasks | training |
| Device | CPU | CPU | CUDA |
| Robot | ours, compiled to `MjSpec` | ours, generated into a robosuite robot class | ours |
| Scene, task | ours (`native/reach`) | the library's | ours |
| Control | ours: actuators and the env layer | the library's joint controller, driven with our accumulated target | ours |
| Stepping | N `MjData` stepped in a loop | N robosuite environments stepped in a loop | MuJoCo Warp, `nworld = num_envs` |

- **`mujoco`.** `ParametricArm` compiles to a robot-only `MjSpec`, is attached to a scene preset and compiled to one
  `MjModel`; the compile also produces the `RobotInterface`. `MjSpec` reads angles as degrees unless
  `compiler.degree = False`; the compile sets it (an early version did not, and joint limits came out 57 times too small).
- **`robosuite`.** Task ids `robosuite/<EnvName>` for every registered single-arm manipulation environment. See Appendix A.
- **`warp`.** Not started. The plan: the same `MjModel` goes through `put_model` and `make_data(nworld, ...)`, state is
  exposed as torch tensors, parity is tested against the CPU backend from injected state, and per-world model fields may
  let same-topology embodiments share one simulation (different `n` needs a separate simulation per topology group).
- **Shared robot construction.** `backends/mujoco/arm_spec.add_arm` builds the links, joints and capsules for every
  MuJoCo-based backend, so the physical robot is identical across them.

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

- **Observation keys are declared per backend.** `qpos` and `qvel` are required in `proprio`; `effort` exists on the native
  backend only; the `task` group depends on the task. Analyses that must run everywhere use the required subset. Details:
  [backends.md](backends.md#the-environment-contract).
- **No autoreset inside the env.** The driver reads the done keys and writes `_reset`.
- **Partial reset** honours the `_reset` mask. TorchRL does not verify that an env implements it, so each backend has its
  own test.
- **Injected state:** optional entries in the reset TensorDict (`qpos`, `qvel`; `target` on `native/reach`) override random
  initialization, for parity tests and for replaying a world on another backend.
- **Seeding:** the same backend, device and seed give the same results; random resets differ across backends, so parity uses
  injected state.
- **Not implemented:** flat-observation views and padding to a maximum `n`. Heterogeneous families (different `n`) have
  different specs; combining them with lazy stacking is untested.
- **Conformance:** `check_env_specs(env)` passes for both backends.

## 7. Verification

- **Config:** validation errors; hash stability (key order, `-0.0`, scaled axes); generator invariants; `config` imports
  without torch or any engine.
- **Documentation:** the Python examples in `docs/configs/*.md` and `docs/README.md` run, and every relative link and
  anchor resolves (`tests/test_docs.py`).
- **Kinematics oracle:** for several `n`, axis patterns and parameter sets, the end-effector pose of the compiled model
  matches `config.kinematics`; the same comparison holds for the robot inside robosuite.
- **Native env:** `check_env_specs`; masked reset; seeding determinism; injected state; control modes; joint limits are in
  radians.
- **robosuite backend** (marker `robosuite`): several environments build and step; control holds and tracks; seeding and
  masked reset are independent per sub-environment; validation lists every problem. Run on robosuite 1.5.1 and 1.5.2 with
  MuJoCo 3.17; the compatibility layer is also tested across MuJoCo 3.3.1 to 3.17.
- **Parity CPU against Warp:** not applicable until the Warp backend exists.

## 8. Status

Run and checked:

- native reach on CPU; the whole suite passes;
- the robosuite backend: 13 of the 15 single-arm robosuite environments build, reset and step with a generated arm and pass
  `check_env_specs`; the hand pose in robosuite equals the numpy kinematics; hold and tracking behave as configured;
- the compatibility layer on MuJoCo 3.3.1 to 3.17.

Not done or not verified:

- the Warp backend, its masked reset, per-world model fields and CPU-versus-Warp parity;
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
5. GPU backend: MuJoCo Warp driven from torch. JAX is shelved. CPU MuJoCo is the reference and debugging backend.
6. Frozen dataclasses with hand-written validation; packaging as in section 3.
7. The robosuite backend is generic over environments (`robosuite/<EnvName>`), not tied to one task. robosuite keeps the
   task, scene, gripper and reward; we supply the robot and the control.
8. robosuite's joint controller is driven in absolute mode with the accumulated target `q_target += action * scale`, so one
   `ControlConfig` means the same on both backends (robosuite's own delta input adds to the current position and the arm
   settles far below the commanded rate).
9. robosuite's incompatibility with current MuJoCo is patched at run time, only where the installed MuJoCo needs it,
   instead of capping MuJoCo.
10. Tasks that define their own goal take `None` for `target_region` and `success_tolerance`; passing a value is an error.
11. Environments whose observation keys change between episodes (`PickPlaceSingle`, `NutAssemblySingle`) are refused, and
    two-arm environments are refused.
12. `.agent/` is committed, with no machine-specific strings.

**Open:**

- Family batching on the GPU: separate simulation per topology group, lazy stacking, or padding.
- The smallest `n` that can physically solve a manipulation task such as `Lift`; infeasibility must be separable from
  failures of learning.
- Whether to run robosuite environments in worker processes for throughput.
- An `ExternalRobot` baseline (the stock Panda) through the same contract.

## 10. Milestones

1. **Config layer and CPU backend.** Done. `dt` and `integrator` stayed explicit (no defaults).
2. **Native Reach on CPU (TorchRL).** Done: `check_env_specs`, masked `_reset`, seeding, injected state.
3. **`warp` backend.** Not started.
4. **robosuite.** Done for generated arms in any single-arm environment, in a more general form than the planned `Lift`
   only. The stock-Panda baseline needs `ExternalRobot` and has not been done.
5. **Later:** Playground and JAX backends.

## Appendix A. The robosuite backend as built

Entry points: `RobosuiteBackend` (capabilities, `validate`, `make_env`), `RobosuiteEnv`, `arm.py` (robot class and
controller config), `_introspect.py` (reads robosuite's environment classes), `_compat.py` (the compatibility layer).

- **Environment names.** Read from robosuite's registry: single-arm manipulation environments only. `validate` checks that
  the name exists, that the class is single-arm, that it defines `_check_success`, that `env_kwargs` are arguments it
  accepts (following `**kwargs` forwarding up the class chain), and that it does not pick a random object per episode.
- **Robot.** `arm_xml` builds a robot-only MJCF from the `ParametricArm` (root body `base`, mount body `link0` carrying
  `base_pose` and the `right_center` site that robosuite's controller uses as the arm's base frame, the shared chain,
  a `right_hand` body at the end-effector pose, one torque motor per joint limited to `effort_limit`). A `ManipulatorModel`
  subclass named from the config hash reads that file. robosuite's registries are global to the process, so the class is
  registered once and reused. `ToolSpec.gripper` and its mount offsets select and place the gripper.
- **Placement.** `RobosuiteOptions.base_placement`: `env_default` mounts the arm where robosuite mounts a stock arm (the rule
  is read from robosuite's own Panda) and treats `base_pose` as relative to that point; `explicit` puts the root at the
  arena origin.
- **Control.** A composite controller with the arm `JOINT_POSITION` controller in absolute mode, `kp = bandwidth²`, the given
  damping ratio, and the `GRIP` gripper controller. The action layout is `[arm (n), gripper dimensions]`. robosuite adds
  gravity compensation and re-evaluates the inertia every step.
- **Observations.** `task` is every non-robot observation under robosuite's own names; `proprio` is `qpos`, `qvel` and the
  robot's other observations without the `robot0_` prefix; aggregate `*-state` vectors are dropped.
- **Reward and done.** The reward is robosuite's, as `[N, 1]`; `success` is the environment's success check; `truncated` is
  the horizon; `terminated` is success only if `terminate_on_success`.
- **Reset and seeding.** `hard_reset` is off. The robot starts from `robot.init_qpos`, which is set to `home_qpos` plus our
  noise or to the injected `qpos`; robosuite's own noise is zeroed. Each sub-environment has its own numpy random stream
  seeded with `seed + i`, swapped in around every call into robosuite; robosuite 1.5.2 is also given `seed` and its
  generator is reseeded in place.
- **Physics.** robosuite owns the time step (0.002) and the integrator (MuJoCo's default, Euler), so the backend rejects
  other values of `sim.dt` and `sim.integrator`.
- **Compatibility layer.** Two patches, each applied only if its probe fails: robosuite's joint-address lookup compares a
  numpy integer with an enum (False from MuJoCo 3.12), and its controller reads `MjData.qM` with the old `mj_fullM`
  signature (changed in 3.10, `qM` removed in 3.11). Upstream reports: robosuite issue 831 and pull request 832.

**Constraints on an embodiment family.** Scale: the task layouts are built around Panda-sized arms, so `chain` with
`total_length` keeps families comparable. Degrees of freedom: whether a 3-DoF arm can grasp a 4 cm cube is itself a study
result. Gains: robosuite's default joint gains were tuned for Panda inertias; `InertiaScaledGains` makes them follow the
arm. Compute: the environments run on the CPU, one after another.
