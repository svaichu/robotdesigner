This package provides robot embodiments as validated, hashable data and batched TorchRL environments:

1. `robotdesigner.config`: numpy-only configs (`ParametricArm`, `ControlConfig`, `TaskConfig`, `SimConfig`, `EnvConfig`, ...), the `chain()` generator and a kinematics oracle.
2. `robotdesigner.simulators` / `robotdesigner.envs`: simulator registry and `make_env(cfg)`, the one way from an `EnvConfig` to an environment. The CPU MuJoCo simulator (`simulators/mujoco`) has one generic env (`MujocoEnv`) that runs any task.
3. `robotdesigner.tasks`: task providers, one subpackage per namespace (`<namespace>/<name>` ids), each with a numpy-only config side (params classes) and an engine side (a `TaskImpl`, contract in `tasks/base.py`). The core names no task: providers are discovered from the registry (`config/providers.py`), and `tests/test_no_hardcoded_tasks.py` enforces it. robosuite is a provider that supplies assets (arena, objects); it is never a simulator.
4. `robotdesigner.viz`: clips, frame recorder and camera framing for rendered rollouts (guide: `docs/viz.md`).

Documentation: `docs/README.md` indexes the reference pages (`docs/configs/*.md`, `docs/simulators.md`); `docs/spec_robotconfig_env.md` is the design record. The Python examples in `docs/configs/*.md`, `docs/tasks/*.md` and `docs/README.md` are executed by `tests/test_docs.py`, and links are checked: when you change a config's fields, validation rules, defaults or hashing, update its page. Working notes: `.agent/memory.md`.
