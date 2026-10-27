This package provides robot embodiments as validated, hashable data and batched TorchRL environments:

1. `robotdesigner.config`: numpy-only configs (`ParametricArm`, `EnvConfig`, ...), the `chain()` generator and a kinematics oracle.
2. `robotdesigner.backends` / `robotdesigner.envs`: backend registry and `make_env(cfg)`; the CPU MuJoCo backend lives in `backends/mujoco`.

See README.md and docs/spec_robotconfig_env.md.
