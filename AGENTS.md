This package provides robot embodiments as validated, hashable data and batched TorchRL environments:

1. `robotdesigner.config`: numpy-only configs (`ParametricArm`, `ControlConfig`, `TaskConfig`, `SimConfig`, `EnvConfig`, ...), the `chain()` generator and a kinematics oracle.
2. `robotdesigner.backends` / `robotdesigner.envs`: backend registry and `make_env(cfg)`. The CPU MuJoCo backend is in `backends/mujoco`, the robosuite backend in `backends/robosuite`; both build the robot with `backends/mujoco/arm_spec.add_arm`.
3. `robotdesigner.viz`: clips, frame recorder and camera framing for rendered rollouts (guide: `docs/viz.md`).

Documentation: `docs/README.md` indexes the reference pages (`docs/configs/*.md`, `docs/backends.md`); `docs/spec_robotconfig_env.md` is the design record. The Python examples in `docs/configs/*.md` and `docs/README.md` are executed by `tests/test_docs.py`, and links are checked: when you change a config's fields, validation rules, defaults or hashing, update its page. Working notes: `.agent/memory.md`.
