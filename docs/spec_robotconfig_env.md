# robotdesigner: design record

Why the package is built the way it is, what is decided, what is done and what is open. Field-level detail lives in the reference pages
([docs/README.md](README.md)), which the tests check against the code. Working notes for agents: `.agent/memory.md`. Last revised 2026-10-30.

**Status.** Built and tested: the config layer, the CPU MuJoCo simulator with one generic environment, and task providers (`robosuite`, arenas and
objects as task assets, is the one real provider in the package, next to the `empty` placeholder). Not built: the `warp` GPU simulator, `ExternalRobot`, end-effector control modes, family-reachable
target regions (section 8).

## 1. Purpose and scope

"Quotient" is the downstream research project that studies relationships between embodiments performing the **same control task**.
`robotdesigner` provides (1) an embodiment as validated, hashable data (`RobotConfig`); (2) batched RL environments that instantiate it on a
simulator; (3) task descriptions that do not depend on the embodiment; (4) traceability: every run maps to a configuration hash. From
`.agent/quotient_space.md`: an n-DoF arm in an RL environment with `n` as a parameter, the robot configuration kept separate from the env.

**Non-goals (v1):** training code or trainer-specific glue; JAX / MJX and Playground simulators (shelved); multi-robot, mobile or legged robots
(two-arm robosuite environments are refused); autodiff through the simulator; a Gymnasium-native contract (an adapter may follow).

## 2. Principles

1. **The robot is data:** frozen, validated, hashable, serializable, no engine import.
2. **Simulators realize configs; envs consume them.** Dependencies point from simulators to the config layer, never back.
3. **One batched env contract:** every simulator returns a TorchRL `EnvBase` over TensorDicts, `batch_size = [num_envs]`.
4. **The task is embodiment-independent and recorded.** It never mentions `n` or link lengths.
5. **Everything that changes behaviour is in `EnvConfig` and its hash.** No hidden defaults. One deliberate exception: robosuite's own defaults for
   task arguments you do not set are recorded on the environment (`resolved_env_kwargs`) but not hashed.
6. **A simulator refuses what it cannot run**, with every problem listed at once; it never approximates.
7. **The core names no task, except the placeholder.** A provider owns a namespace and its task ids; a test enforces it, exempting the `empty` namespace only.
8. **A task draws from its own seeded stream:** the same seed gives every embodiment the same scene.

## 3. Architecture and packaging

```
robotdesigner.config               numpy only: configs, generators, validation, hashing, kinematics oracle
robotdesigner.make_env             make_env(cfg), the one step from an EnvConfig to an environment (robotdesigner.envs.make_env)
robotdesigner.viz                  clips, frame recorder, camera framing (saving needs the viz extra); see viz.md
robotdesigner.simulators           simulators: protocol, capabilities, lazy registry
robotdesigner.simulators.mujoco    CPU reference simulator and its generic env; arm_spec.add_arm builds the chain
robotdesigner.tasks                task contract (base.py); one subpackage per provider namespace, found by config.providers
robotdesigner.tasks.robosuite      robosuite arenas and objects as a task; the compatibility layer
(planned) simulators.warp          GPU training
```

`config` imports no engine, no torch and no robosuite (a test checks it); "engine" means those heavy libraries. Simulators are imported lazily by name
(`sim.simulator`) and import their engine inside `validate` and `make_env`. `config.kinematics` is a pure-numpy forward-kinematics implementation: the
engine-independent oracle compiled models are tested against.

```
base        numpy>=1.26                                     # robotdesigner.config
[env]       torch>=2.7, tensordict>=0.14.3,<0.15, torchrl>=0.14.1,<0.15, mujoco>=3.13
[warp]      [env] + mujoco-warp, warp-lang>=1.15            # declared; simulator not implemented
[robosuite] [env] + robosuite>=1.5,<1.6
[viz]       pillow, imageio, imageio-ffmpeg
[dev]       pytest, ruff
```

`requires-python = ">=3.10"`; CI covers 3.10, 3.12, 3.13 and a job for the `robosuite` marker; uv with a committed lockfile, src layout. The floors have
reasons: torchrl 0.14.1 pins tensordict below 0.15 and tensordict 0.15.3 needs torch 2.11, so the 0.14 line serves torch 2.7 to 2.8 (widen only after CI
passes on a newer line); older downstream environments pin numpy 1.26 on Python 3.10; mujoco-warp requires mujoco 3.13. robosuite 1.5.x does not run on
MuJoCo 3.10 or later as released; instead of capping MuJoCo (a conflict with the Warp floor), `tasks/robosuite/_compat.py` patches the two broken code
paths in the running process (Appendix A).

## 4. Configuration objects

```
EnvConfig(robot, control, task, sim)
```

`BaseEnvConfig` ([base-env.md](configs/base-env.md)) is the builder layer on top: a complete default refined by accumulating `robot` / `control` / `task` / `sim` calls and resolved into the `EnvConfig`. The defaults live there only; the frozen configs below require every field.

| section | class | answers | reference |
|---|---|---|---|
| robot | `ParametricArm` (a `RobotConfig`) | what the body is; the thing a study varies, with its own hash | [robot.md](configs/robot.md) |
| control | `ControlConfig` with `FixedGains` or `InertiaScaledGains` | how an action drives the joints | [control.md](configs/control.md) |
| task | `TaskConfig` with the params class of the task's provider | what to do, independent of the embodiment | [task.md](configs/task.md), [robosuite.md](tasks/robosuite.md) |
| sim | `SimConfig`, with the simulator's `options` inside | which simulator, how it runs, its own settings | [sim.md](configs/sim.md) |

- **RobotConfig.** `ParametricArm`: segments of a hinge joint and a capsule link, `n = len(segments)`, `chain()` as the generator; every dynamic
  parameter required. `ExternalRobot` (a reference to an existing model, for baselines such as the stock Panda) is designed, not built. The hash is the
  SHA-256 of the canonical JSON without the label, quaternion signs canonicalized.
- **ControlConfig.** Six mode names are declared; the MuJoCo simulator implements four (`joint_pos_abs`, `joint_pos_delta`, `joint_vel`,
  `joint_torque`), none implements the end-effector modes. Hardware limits stay in the robot (`effort_limit`).
- **TaskConfig.** `TaskConfig(params, max_episode_steps, success_tolerance, target_region, mount)`: the params object is the task; `mount` is the robot base in the world (a `Pose`, required; `TaskProvider.default_mount(name)` says where the task expects it). The robot has no pose, so `robot_hash` identifies the embodiment alone; target regions stay in the robot's base frame and follow the mount. Its class names the provider (class attribute `provider`) and its `name` field the task within it; `TaskConfig.id` (`<provider>/<name>`) is derived and read-only, and no string id or `provider=` argument exists. `to_dict()` writes `provider` next to `params`; `from_dict()` finds the params class through the registry (errors at `provider` and `params.name`). A goal task takes `FixedBox` or `OwnReachable()` and a tolerance (family-reachable
  is deferred); a task that defines its own success takes `None` for both, and passing a value is an error.
- **SimConfig.** `simulator, num_envs, device, dt, integrator, control_freq, seed, scene, options`, all required; `1 / (control_freq * dt)` must be a
  positive integer. `options` must be an instance of the class registered for `simulator`. Rendering is not a field ([viz.md](viz.md)).
- **EnvConfig.** `resolve()` is the identity until family-reachable regions exist. `env_hash` covers the robot's full hash, control, task and sim (so `seed`
  and `num_envs` count, the robot's label does not).

## 5. Simulators, tasks and providers

**Simulators.** A simulator is a physics engine realized as code: `capabilities()`, `validate(cfg)` (problems, before anything is built), `make_env(cfg)`.

| | `mujoco` | `warp` (planned) |
|---|---|---|
| Role, device | reference and debugging, CPU | training, CUDA |
| Robot, control | ours, compiled to `MjSpec`; actuators and the env layer | ours, the same model |
| Scene, task | a scene preset plus what the task's provider attaches | the same |
| Stepping | N `MjData` stepped one after another | MuJoCo Warp, `nworld = num_envs` |

`ParametricArm` compiles, with the scene preset and the task's assets on one `MjSpec`, to one `MjModel` and a `RobotInterface`; the compile sets
`compiler.degree = False` (`MjSpec` reads degrees otherwise). `arm_spec.add_arm` builds the chain for every MuJoCo-based simulator. For Warp the plan is
the same `MjModel` through `put_model` and `make_data(nworld)`, torch views, parity against the CPU simulator from injected state, and per-world model
fields to let same-topology embodiments share one simulation (a different `n` needs another simulation per topology group).

**Providers.** A task provider owns a namespace. Config side: params classes (numpy only), each with `provider` and a `name` field; providers read the task name from `cfg.task.params.name`. Engine side: a `TaskImpl` that adds scene assets to the model
spec, places things at reset, observes and scores. `validate(cfg, simulator)` collects the provider's problems with the simulator's into one `ConfigError`.
Providers are the subpackages of `robotdesigner.tasks`, the entry-point group `robotdesigner.task_providers`, or `register_task_provider` ([how to write one](tasks/writing-a-provider.md)).
`env = make_env(cfg)` is the only construction path, and `MujocoEnv` is the one generic environment: it owns the robot, control law, batching, masked
reset, episode counters and rendering, and a task states only what it adds. The package ships one placeholder task (`empty/idle`, nothing to do) and no real one of its own: the engine tests run on a test-only
provider, `tests/toy_provider` (namespace `toy`), the smallest example of a provider. The goal machinery (`FixedBox`, `OwnReachable`, `uses_goal`,
`success_tolerance`, `target_region`) stays in the config layer for goal tasks of any provider.

**Randomness.** The environment keeps two generators seeded from `sim.seed`: one for the robot's reset noise (its shape depends on `n`), one for the task
(`rand` in `TaskImpl.reset`). Embodiments with different joint counts therefore get the same task draws, the same cube placement for example.

## 6. Env contract (TorchRL)

```
env.batch_size = [num_envs]       # batch-locked
env.reset(td=None) -> td          # td["_reset"], bool [num_envs, 1], selects envs; absent = reset all
env.step(td)       -> td          # reads td["action"]; outputs live under ("next", ...)
env.step_and_maybe_reset(td)      # driver-side autoreset: reads the done keys, writes "_reset"
env.rollout(max_steps, policy)    # TorchRL's own; this package ships no rollout helper
env.set_seed(seed); env.robot_hash, env.env_hash
```

Subclasses implement `_reset`, `_step` and `_set_seed` and declare the specs; they never override `reset` or `step`.

- **Observation keys are declared per simulator.** `qpos` and `qvel` are required in `proprio`; `effort` exists on the MuJoCo simulator only; the `task`
  group is the task's. Analyses that must run everywhere use the required subset ([simulators.md](simulators.md#the-environment-contract)).
- **No autoreset in the env; partial reset honours the mask.** TorchRL does not verify that an env implements it, so each simulator has its own test.
- **Injected state:** optional reset entries (`qpos`, `qvel`, and what a task documents) override random initialization, for parity tests and replays.
- **Seeding:** the same simulator, device and seed give the same results; random resets differ across simulators, so parity uses injected state.
- **Not implemented:** flat-observation views and padding to a maximum `n`; combining heterogeneous families (different `n`) by lazy stacking is untested.
- **Conformance:** `check_env_specs(env)` passes for every task the tests build.

## 7. Verification

- **Config:** validation errors and paths; hash stability (key order, `-0.0`, scaled axes); generator invariants; `config` imports without torch or an engine.
- **Docs:** every Python example in `docs/configs`, `docs/tasks` and `docs/README.md` runs; every relative link and anchor resolves (`tests/test_docs.py`).
- **Kinematics oracle:** the end-effector pose of the compiled model matches `config.kinematics` for several `n`, axis patterns and parameter sets.
- **MuJoCo env** (on the test-only provider): `check_env_specs`, masked reset, seeding, injected state, control modes, radians, and that task draws do not
  depend on the robot.
- **robosuite provider** (marker `robosuite`): validation lists every problem; the environments in [robosuite.md](tasks/robosuite.md) build, step and pass
  `check_env_specs`; placement is seeded; a masked reset replaces only the masked envs; fixtures are placed once; the compatibility layer holds on MuJoCo
  3.3.1 to 3.17. **Rendering** (marker `render`, needs OpenGL): shapes, overlays, cameras, the recorder, rendering leaves the simulation bit-identical.

## 8. Status and open items

At the last revision 163 tests without the `render` marker and 12 render tests pass, 12 of the 15 single-arm robosuite environments build and step with a
generated arm, and both example notebooks run every cell. Done: config layer and CPU simulator, the CPU TorchRL env, robosuite as a source of assets.
Not started: the `warp` simulator. Shelved: Playground and JAX.

Not done or not verified: Warp (masked reset, per-world fields, CPU parity); policy performance on any task; the throughput of the CPU simulator with many
envs; the robosuite 1.5.2 placement path (written, never run) and the `robosuite` CI job (added, not checked); an `ExternalRobot` baseline; a gripper (the
MuJoCo simulator does not build `robot.tool`, so grasping tasks cannot be solved); lazy stacking of heterogeneous families.

Open decisions: family batching on the GPU (a simulation per topology group, lazy stacking, or padding); the smallest `n` that can physically solve a task
such as `Lift`, with infeasibility separable from failures of learning; whether to run robosuite environments in worker processes.

## 9. Decision log

| date | decision | reason |
|---|---|---|
| 2026-10-27 | `ParametricArm` built from segments by generators; `ExternalRobot` only for baselines; control mode in `ControlConfig`, hardware limits in the robot | comparing and sampling embodiments needs a parameter space, which `MjSpec` does not expose; one arm is run under several controls |
| 2026-10-27 | `config` is numpy-only; frozen dataclasses with hand-written validation; no inherited defaults (`dt`, `integrator`, gains, masses explicit) | configs and hashes must work in any analysis environment; every behaviour-changing value must be in `env_hash` |
| 2026-10-27 | env contract = TorchRL `EnvBase`, never autoreset, nested observations primary (user) | TorchRL collectors and `rollout` read the done keys and the `_reset` mask; a flat view is a transform |
| 2026-10-27 | GPU simulator = MuJoCo Warp driven from torch; JAX shelved (user) | no JAX-to-torch bridge to maintain; it consumes the same `MjSpec` |
| 2026-10-28 | robosuite's MuJoCo incompatibility is patched at run time, only where a probe fails | capping MuJoCo would conflict with the Warp floor |
| 2026-10-29 | `env = make_env(cfg)` is the only construction path; `EnvConfig` has no `make()` (user) | one step from config to environment; the config layer imports no engine |
| 2026-10-29 | robosuite is a source of task assets, `robosuite/<EnvName>`, never a simulator; success and reward are ours (user) | as a simulator it brought its own controller and physics, so one robot was not controlled the same way across tasks |
| 2026-10-29 | a provider owns a namespace; the core names no task and a test enforces it (user) | no hardcoded envs |
| 2026-10-29 | a task with its own success takes `None` for `target_region` and `success_tolerance`; `*Single` and two-arm environments are refused; fixtures (no free joint) are placed once | no value is silently ignored; specs must be fixed; the envs of a batch share one model |
| 2026-10-29 | `.agent/` is committed, with no machine-specific strings | the repository is public |
| 2026-10-30 | the `native` provider and its reach task are removed; tests keep `tests/toy_provider` (user) | the package loads embodiments into an RL env, tasks are what providers bring, more will come |
| 2026-10-30 | the task draws from its own random stream | with one stream the joint count changed the cube placement, which defeats comparing embodiments |
| 2026-10-30 | the params object is the task: `TaskConfig.id` is removed (derived, read-only), params classes declare `provider` and have `name`, no `provider=` argument, no string id (user, roadmap #32, issue #33) | one place says which task runs, and a second argument could contradict the class. The `task` part of `to_dict()` changes shape, so **every `env_hash` changes once**: hashes recorded earlier no longer match |
| 2026-10-30 | `TaskConfig.horizon` is renamed `max_episode_steps` (Gymnasium's name for the time limit); loaders still read `horizon` with a `DeprecationWarning`, both keys together are an error (issue #40) | the repo already emits Gymnasium's `terminated` / `truncated`, and the old name hid that `env.rollout(500)` stops at the limit. **Every `env_hash` changes once** (the key of `task.to_dict()` changed); hashes recorded earlier no longer match. The `DEFAULTS_VERSION` stays 1 (no default value changed). Release note: this rename and the hash change belong in the notes of the next version |
| 2026-10-30 | `env.action_space` and `env.observation_space` are plain per-env shapes (`torch.Size`, and a nested dict of them), derived from the specs by `envs.spaces.SpacesMixin` (issue #41) | sizing a policy or a buffer for an embodiment needed the nested `Composite` specs with the batch dimension in front. Proposals of the issue taken: per-env without the batch dimension, `torch.Size`, Gymnasium's names kept with the difference documented |
| 2026-10-30 | `BaseEnvConfig.sweep(paired=, grid=)` returns a `Family` (iterable of configs, `configs()`, `table()`); paired axes zip, grid axes multiply, repeated `env_hash` items are dropped with a note, invalid items raise at expansion naming the index (user, roadmap #32, issue #38) | embodiment studies need one config per embodiment with its own hash. A plain expansion: jobs (Slurm or other) are the downstream project's, and workers rebuild from the resolved dict because the registries are process-global. Random or Optuna-style sampling is later |
| 2026-10-30 | `with_overrides` by dotted path (mapping or CLI `path=value` strings, values as JSON literals), `to_dict` / `from_dict` with `defaults_version` (a mismatch warns), `dump` / `load` as JSON and, with the optional extra `yaml`, YAML (user, roadmap #32, issue #37) | overrides are generic: a section is converted to a dict, edited and rebuilt with its own `from_dict`, so providers need no extra code. The base install stays numpy only: YAML is an extra, and its absence raises an error that names it. The record to keep with a run remains `resolve().to_dict()` |
| 2026-10-30 | `BaseEnvConfig` (roadmap #32, issue #36): a complete runnable base refined with accumulating section calls (`robot`, `control`, `task`, `sim`), each returning a new config; `Chain` is the partial robot spec; `NOT_SET` marks "not passed"; `resolve()` returns the `EnvConfig` and lists every problem; `explain()`, `diff()`, `validate()`; `build()` stays a one-line delegate to `make_env(self.resolve())` (user) | RLlib-like ergonomics for embodiment studies with no `ray` and no trainer coupling. **Defaults live in the builder layer only** (`config/defaults.py`, `DEFAULTS_VERSION`, `MujocoOptions.default()`): the frozen configs keep requiring every field, so the 2026-10-27 rule holds for them. Calls accumulate, so a second `.sim(dt=...)` keeps `num_envs`. A change of a default changes the `env_hash` of configs that relied on it: store `resolve().to_dict()` with every run. The mount is the task's default until pinned |
| 2026-10-30 | the package ships one placeholder task, `empty/idle` (user, roadmap #32, issue #35); `tests/test_no_hardcoded_tasks.py` exempts the `empty` namespace | supersedes "the core names no task" (2026-10-29) and "the package ships no task of its own" (2026-10-30) for this one task: a default environment must run, so it needs a task. Every other namespace, task id and params class stays enforced; the `empty` scene preset of the simulator is unrelated |
| 2026-10-30 | the robot mount moves from `ParametricArm.base_pose` to `TaskConfig.mount` (user, roadmap #32, issue #34); `TaskProvider.default_mount(name)`; the kinematics oracle takes the base as an argument | where an arm stands is a property of the scene: the same arm had a different `robot_hash` in every scene. **Every `robot_hash` and `env_hash` changes again** (the robot loses `base_pose`, the task gains `mount`); hashes from before this entry no longer match |
| 2026-10-30 | "backend" becomes "simulator" everywhere, no aliases; options move into `SimConfig`, so `EnvConfig` has four fields (user) | "backend" is dependency vocabulary and `SimConfig` names a physics engine; two fields that must agree need a cross-check. Every `env_hash` changed, `robot_hash` did not |

## Appendix A. robosuite as a task provider

Package `robotdesigner.tasks.robosuite`. Reference: [tasks/robosuite.md](tasks/robosuite.md).

- **Loading.** `scene.load_scene` builds the robosuite environment once, with a stock Panda and no rendering, seeded from `sim.seed` (robosuite draws
  object sizes while it builds). It keeps the arena and objects (one MJCF document from `ManipulationTask` without robots), the placement sampler and the
  mount of a stock arm (`default_mount(env_name)`; use it, or any pose, as your arm's base), then closes the environment. Its simulation is never stepped.
- **Attaching.** `clean_xml` drops `<option>`, `<size>`, `<visual>` and the IK mocap bodies; `MjSpec.attach` merges the rest under the prefix `rs_`. The
  arena brings its own floor, so `sim.scene` must be `empty`. Physics settings follow `SimConfig` and MuJoCo's defaults.
- **Placement.** At every reset, per env, robosuite's own sampler places the objects that have a free joint, with the global numpy stream (1.5.1) or the
  sampler's generator (1.5.2, not exercised), seeded from the task's stream and restored afterwards. Objects without a free joint are placed once at bind.
- **Objective.** `LiftObject(object, height)` and `ReachObject(object, tolerance)`; reward is the negative distance from the end effector to the object.
  Observations: `ee_pos` and `<object>_pos` / `<object>_quat` for physical objects, in the robot's base frame.
- **Versions.** `_compat.apply_compat` patches two robosuite 1.5.x code paths that break on MuJoCo >= 3.10 (joint-address enum comparison, mass-matrix
  call), each only if its probe fails; building a robosuite environment runs its controller setup, which needs both.
