# robotdesigner: agent memory

Start with `AGENTS.md` (what the package is), `docs/README.md` (reference index) and `docs/spec_robotconfig_env.md` (design record and dated
decision log, with the reasons); `docs/tasks/writing-a-provider.md` shows how to add a task source. This file keeps what those do not: working rules, environment facts, pitfalls, open decisions.
Last updated: 2026-10-30. Work happens on `devel` (merged locally; the user pushes). The older history of this file is in git.

## Goal
A framework to load different robot configs into an RL env, reused in other projects to study relationships between embodiments in
control (downstream project: "Quotient"). The user's own note is `.agent/quotient_space.md`: an n-DoF arm in an RL env with `n` as a
parameter, the robot config kept separate from the env. Do not overwrite the user's notes (`quotient_space.md`, `future_ideas.md`).

## Working rules
- Commit and merge only when the user asks. Never push. Cut work on a branch in its own worktree (`<repo>-wt/<name>`) and leave the user's
  checkout alone. Fast-forward `devel` only when it has not moved and the user's checkout has no uncommitted changes; if it has, ask what
  to do with them (stash is reversible, discarding needs an explicit yes, and a yes covers that one case only).
- The repo is public: no cluster paths, user names, job ids, host names or tool / model names in tracked files, issues, notebook outputs
  or commit messages (stored notebook output has the venv path replaced by `<site-packages>`).
- Do not delete repo files without asking. No trainer (RLlib or other) shapes the design.
- Do not hardcode a task or env in the core unless the user says so (the one exception is the `empty` placeholder, decided 2026-10-30): `tests/test_no_hardcoded_tasks.py` enforces it.
- Docs are tested (`tests/test_docs.py`): every ```python block in `docs/configs`, `docs/tasks` and `docs/README.md` is executed (numpy only,
  no simulator; fence a block as ```python no-run to skip it) and every relative link and anchor must resolve. Change a config's fields,
  rules or hashing and you change its page.
- The user works in the repo concurrently: check `git status` before editing shared files. Tell the user what a long step is doing before
  it starts; they ask "is it done?" when a run is silent.
- After a rename or removal, grep for leftovers in code, docs, images (`docs/architecture.png`), notebook outputs and this file.

## Terminology (the user's, 2026-10-30)
- **simulator**: a physics engine realized as code, `robotdesigner.simulators.<name>`, named by `sim.simulator` (`mujoco`; later `warp`).
  Git history, closed issues and old text say "backend": read it as "simulator". `build-backend` in `pyproject.toml` is packaging vocabulary.
- **task provider**: owner of a task-id namespace (`robosuite`); **task**: `<namespace>/<name>`; **options**: the settings of one simulator,
  in `sim.options`. **config side / engine side**: numpy-only classes vs the code that needs torch and MuJoCo ("engine" = those libraries).
- **embodiment**: a robot config (`ParametricArm`). **scene**: `sim.scene` is a preset of the simulator (`empty`, `plain_floor`); a
  robosuite scene is an arena plus objects brought by a task.

## Why the project looks like this
- Same control task, varying embodiment: the task is a first-class object that does not mention `n` or link lengths. A fixed target is
  unreachable for small `n` and trivial for large `n`, so targets are drawn per arm (`OwnReachable`) or from a box in the base frame.
- `n` confounds reach, mass and inertia: hold total length and mass fixed and split them over `n` (`chain(total_length=, total_mass=)`).
- Equal gains on every joint make control differences partly gain mismatch: gains are an explicit recorded choice, `InertiaScaledGains`.
- Every rollout must trace to an embodiment: configs are frozen, validated, hashable (`robot_hash`, `env_hash`); a task draws from its own
  seeded stream, so the same seed gives every embodiment the same scene.
- Observation and action dimensions vary with `n`; a cross-embodiment view (padding, masks, per-embodiment heads) is not built.

## Decisions (dated; reasons in the spec decision log)
- 2026-10-27 user: JAX shelved, GPU = MuJoCo Warp driven from torch; env contract = TorchRL `EnvBase`. Agent, under "do what is right": python
  >= 3.10, extras `env`, `warp`, `robosuite`, `viz`, `dev`, nested observations.
- 2026-10-28 agent, accepted: robosuite's MuJoCo incompatibility is patched at run time (`apply_compat`), not by capping MuJoCo.
- 2026-10-29 user: `env = make_env(cfg)`; robosuite is a source of assets, never a simulator (success and reward are ours); no hardcoded tasks.
- 2026-10-30 user: the `native` provider and its reach task are purged (the tests keep a test-only provider, `tests/toy_provider`, namespace
  `toy`); "backend" becomes "simulator" everywhere and the options move into `SimConfig`. Agent: the task gets its own random stream.
- 2026-10-30 user (roadmap #32, issue #33 done): the params object is the task (`TaskConfig(params, horizon, success_tolerance, target_region)`; params classes have `provider` and `name`; `task.id` is derived); every `env_hash` changed once. `provider_for` is removed (`get_task_provider(cfg.task.params.provider)`). Issue #34 done: `base_pose` left the robot, `TaskConfig.mount` (required `Pose`) and `TaskProvider.default_mount(name)` replace it; `kin.forward(arm, q, base=None)`; hashes changed again. Issue #35 done: `robotdesigner.tasks.empty` (`empty/idle`, `EmptyParams`), the one task the core may name; the no-hardcoded test exempts namespace `empty`. Issue #36 done: `BaseEnvConfig` (`config/base_env.py`), `Chain`/`NOT_SET` (`specs.py`), `DEFAULTS` (`defaults.py`, version 1); numbers chosen there (one z joint 0.5 m 1 kg, kp 60 kd 6, dt 0.002, 50 Hz, 1 env, scene `empty`, horizon 200), for the user's review; defaults change hashes, store `resolve().to_dict()`. Issue #37 done: `with_overrides`, `to_dict`/`from_dict`/`dump`/`load` (JSON; YAML via extra `yaml`, also in `dev`). Issue #38 done: `sweep`/`Family` (`config/sweep.py`), `examples/embodiment_sweep.ipynb`. Roadmap #32 is complete (#33 to #38), merged as PR #39. Issue #40: `horizon` -> `max_episode_steps` (loaders read `horizon` with a DeprecationWarning; hashes changed again). Issue #41: `env.action_space` / `env.observation_space` as shapes (`envs/spaces.py`) in order, one issue at a time, wait for the user between issues.
- GitHub issues: roadmap #10; M1 #11 to #14 and #16; M2 #15 and #17 to #20 (all closed, implemented).

## Environment and workflow
- HPC: CPU jobs on partition `c23ms` (8 CPUs), GPU on `c23g`; not the `devel` partition. The user's venv is the one to use (do not build another):
  Python 3.13, mujoco 3.17.0, robosuite 1.5.1, torch 2.14.1, torchrl 0.14.1, tensordict 0.14.3, numpy 2.5.4, pytest 9.1.1, imageio and ffmpeg, Pillow,
  ipykernel and jupyter_client (no nbclient / nbformat). It is an editable install of the main checkout; to run a worktree: `unset PYTHONPATH`,
  then `PYTHONPATH=<worktree>/src`.
- Tests: `pytest -m "not render"` (163 tests, about 1 min on c23ms); `-m render` (12 tests) needs EGL: `MUJOCO_GL=egl`, and on a multi-GPU login
  node pin an idle device with `MUJOCO_EGL_DEVICE_ID` (osmesa and glfw did not work). Run heavy work as jobs, not on the login node.
- Slurm submission can fail with "Socket timed out" (nothing is queued): retry. SSH access once failed for hours: ask the user to restore it.
- A pytest run from the repo root writes `MUJOCO_LOG.TXT` (git-ignored now). Read repo files with `git show` or `cat`.
- Moving files to the cluster: one tar bundle of the changed files is far faster than file by file.
- Notebooks (`examples/*.ipynb`) are stored with outputs, executed in a fresh kernel with `jupyter_client` on a login node; rerun them when an
  API they use changes. Quiet robosuite's per-scene INFO line with `logging.getLogger("robosuite_logs").setLevel(logging.WARNING)`.
- Only the tip of a branch is tested; intermediate commits are topic splits and may not pass alone.

## robosuite facts and pitfalls (1.5.1 unless stated)
- 1.5.x as released fails on MuJoCo >= 3.10 (joint-address enum comparison; mass-matrix call). `apply_compat()` probes and patches only what fails;
  tested on MuJoCo 3.3.1 to 3.17. Absolute dynamics differ across MuJoCo versions (a 60-step trajectory differs by up to 4e-3 rad between 3.9 and
  3.17): fix the version per study.
- The provider builds the environment once with a stock Panda, keeps `ManipulationTask(arena, [], objects).get_xml()`, strips `<option>`, `<size>`,
  `<visual>` and the IK mocap bodies `*_eef_target`, and attaches it with `MjSpec.attach` under the prefix `rs_` (must be non-empty). `sim.scene`
  must be `empty`: the arena brings its floor.
- Object sizes (the `Lift` cube) and the placement samplers draw from the GLOBAL numpy stream: seed and restore it. 1.5.2 uses sampler generators
  (path written, unrun). 1.5.1 `Lift` has no `seed` argument. Do not hand robosuite the env's own generator; derive an integer seed.
- Objects without a free joint (the `Door` door, the `PickPlace` bins) are fixtures: baked once, since the envs of a batch share one model.
  `obj_type == "visual"` objects are not observed. 12 of 15 single-arm environments build; `*Single` refused (observation keys change per
  episode); `Wipe` has no object; two-arm refused. Introspecting a constructor must follow `**kwargs` forwarding.
- `Lift`: table top z = 0.8; stock Panda mount `(-0.56, 0, 0.912)`; cube half-size about 0.021, placed within +-3 cm of the table centre.
- Pitfalls: `MjSpec` reads angles as degrees unless `compiler.degree = False` (the compile sets it; a test guards it); `xpos` is stale until the
  next forward; robosuite's own delta input settles at about 17 % of the commanded rate, so delta mode accumulates the target in our env.

## MuJoCo Warp (next milestone; nothing built)
mujoco-warp 3.17.0 and warp-lang 1.18.0 are installed. Source-read API: `put_model(mjm, batch_sizes)`, `make_data(mjm, nworld, ...)`, `put_data`,
`get_data_into(result, mjm, d, world_id)` (copy one world to a host `MjData` to replay or view it), `reset_data(m, d, reset)` (a mask: confirm it is
a masked partial reset), `step`, integrators euler / implicit / rk4. Per-world model fields can batch same-topology embodiments; different `n`
needs one simulation per topology group (open). Warp is float32 and CPU MuJoCo float64: parity needs tolerances and short, contact-free
horizons first. Warp on a CPU device is unverified; NVIDIA only, no autodiff. Position and torque actuators run in the physics, delta and
end-effector modes are ours.

## TorchRL conventions
Subclass `_reset`, `_step`, `_set_seed` and the specs; never override `reset` / `step`. `td["_reset"]` is bool `[B, 1]` (absent = reset all); the env
never autoresets; done, terminated, truncated and reward carry a trailing dim of 1; `check_env_specs` is the conformance check. torchrl 0.14.1 pins
tensordict < 0.15, tensordict 0.15.3 needs torch >= 2.11: widen the pins only after CI passes on the newer line.

## Open decisions
1. Family batching on the GPU: one simulation per topology group, lazy stacking, or padding.
2. The smallest `n` that can physically solve `Lift`; infeasibility must be separable from failures of learning. The MuJoCo simulator builds no
   gripper (`robot.tool` is ignored), so grasping tasks are out of reach.
3. Reward shaping of the robosuite objectives is basic (negative distance); `Wipe` needs an objective of its own.
4. An `ExternalRobot` baseline (the stock Panda) through the same contract.
5. robosuite 1.5.2 placement path, robosuite throughput with many envs, and the `robosuite` CI job are unchecked.
6. The user's idea list item "molvospaces": with the terms above that is a task provider (assets) or a simulator (physics): decide which.

## Lessons
- A number or claim in the docs goes stale silently (the spec said 13 of 15 environments after the design changed to 12): recheck claims
  against code and tests whenever a design changes.
- After the reach task was purged a diagram still listed it: images and notebook outputs need the same grep as code.
- A user's uncommitted change in their checkout is theirs: ask, and prefer a reversible step.
