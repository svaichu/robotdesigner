# Development

## Layout

```
src/robotdesigner/
    robot/        ParametricArm, JointSpec, LinkSpec, TwoFingerGripper, Pose, chain()            numpy only
    config/       ControlConfig, SimConfig, TaskConfig, EnvConfig, BaseEnvConfig, Chain, sweep,    numpy only
                  the task-provider registry (providers.py), kinematics oracle, defaults.py
    simulators/   registry, Capabilities, Simulator protocol
        mujoco/   CPU simulator: model compiler, batched sim, MujocoEnv, renderer, scene presets
        warp/     GPU simulator: WarpBatchedSim, WarpEnv
    tasks/        the task contract (base.py) and one subpackage per provider: empty/, robosuite/
    envs/         make_env(cfg), observation_space / action_space
    viz/          Clip, FrameRecorder, record_rollout, camera framing                           numpy only
    _canon.py     canonical JSON, hashing, field coercion;  errors.py: ConfigError, Problem
tests/            one file per area; toy_provider/ is a test-only task provider (namespace "toy")
examples/         notebooks only
benchmarks/       bench_warp.py
docs/             this documentation
pizza_robot/      a standalone MuJoCo scene (an ElRobot arm on a table, with a viewer script); not wired into the package
```

The two halves of the dependency rule: everything marked *numpy only* must import without torch, MuJoCo or robosuite (`import robotdesigner.config` is the test), and the engine side is
imported lazily, by `make_env` and by the provider's `build`.

## Install for development

```bash
pip install -e ".[env,robosuite,viz,dev]"      # + ".[warp]" on a machine with an NVIDIA GPU
```

Python >= 3.10. Extras: `env` (torch, TorchRL, tensordict, MuJoCo), `robosuite`, `warp`, `viz`, `yaml`, `dev` (pytest, ruff, pyyaml). The lock file `uv.lock` pins one resolution for `uv run`.

## Tests

```bash
pytest -m "not gpu and not robosuite and not render"      # what the main CI job runs
pytest -m robosuite                                       # needs the robosuite extra
pytest -m "gpu"                                           # needs a CUDA device
MUJOCO_GL=egl pytest -m render                            # needs an OpenGL context
```

| marker | needs |
|---|---|
| `gpu` | a CUDA device (and the `warp` extra) |
| `robosuite` | the `robosuite` extra |
| `render` | an OpenGL context; skipped when there is none |

CI (`.github/workflows/ci.yml`) runs the unmarked tests on Python 3.10, 3.12 and 3.13, and the robosuite tests (without GPU and render) in a separate job, because robosuite 1.5.x needs the compat layer on current MuJoCo
and those tests notice when either side changes. Only the tip of a branch is expected to pass.

Beyond unit tests there are three kinds of check worth knowing:

- **Parity** (`tests/parity.py`, `tests/test_warp_parity.py`): the GPU simulator against the CPU one, and both against the numpy kinematics oracle. A capability is claimed only after it matches.
- **Notebook smoke tests** (`tests/test_notebooks.py`, `tests/nbrun.py`): the code cells of every notebook run in-process.
- **`tests/test_no_hardcoded_tasks.py`**: the core must not name a task (the `empty` namespace is exempt).

## Documentation is tested

`tests/test_docs.py` makes the docs a checked artifact:

- every ```` ```python ```` block in the documentation pages is **executed**, in order, in one namespace per page, so a block can use what an earlier block of the page defined. They use the config side only (numpy):
  no simulator, torch or robosuite. A block that needs an engine is fenced as ```` ```python no-run ```` and is not executed.
- every relative link and every `#anchor` in `README.md` and `docs/**/*.md` must resolve (anchors follow GitHub's heading slugs; headings inside code blocks do not count).

So: when you change a config's fields, rules, defaults or hashing, change the page that documents it, and run `pytest tests/test_docs.py`. Pages without an executable example are listed in the test.

## Examples

Examples are **notebooks only** (`examples/*.ipynb`), written with the builder pattern, stored with outputs and executed in a fresh kernel. Rerun them when an API they use changes. Keep machine-specific
paths out of stored outputs: the repository is public.

## Adding a simulator

1. Write a class with `name`, `capabilities()` (a `Capabilities` of control modes, devices, robot kinds, integrators: list only what you have shown to match the reference), `validate(cfg)` (engine-specific
   `Problem`s) and `make_env(cfg)` that returns a batched TorchRL `EnvBase` honouring the [contract](simulators.md#the-environment).
2. Register it: `register_simulator("name", "my_package.module:MySimulator")` (a lazy string keeps the import out of the config layer).
3. Declare its options: `register_simulator_options("name", MyOptions)`, a frozen dataclass with `to_dict`, `from_dict` and optionally `default()`.
4. Implement `SimView` (`tasks/base.py`) in your batched simulation so existing tasks run on it unchanged; draw the task's random numbers from the generators of `MujocoEnv`
   (`task_seed(cfg.sim.seed)`) so a seed means the same on every simulator.
5. Add parity tests against `mujoco` and against `config.kinematics`.

## Adding a task provider

[Writing a task provider](tasks/writing-a-provider.md).

## Conventions

- Errors in configs are `ConfigError(path, message)` with the dotted path of the field to change; collect all problems before raising when you can (`Problem`).
- A new config class is frozen, validated in `__post_init__`, has `to_dict` / `from_dict`, rejects unknown keys, and is hashed through canonical JSON. Anything that changes behaviour belongs in the hash.
- The repository is public: no cluster paths, user names, host names, job ids or tool names in tracked files, issues or commit messages.
- After renaming or removing something, grep the code, the docs, the notebooks' stored outputs and any images for leftovers.
