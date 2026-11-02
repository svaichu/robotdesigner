# Running on a cluster

robotdesigner does not launch jobs. It gives you the pieces that make a launch safe: configs that are plain data, a hash per run, a `sweep` that expands a family, and environments that build on
a worker from a dict. This page is how to put them together on a shared machine (Slurm or anything else). It uses no scheduler-specific code.

## The pattern: one resolved dict per job

```python
import json
import os
import tempfile

from robotdesigner.config import BaseEnvConfig, Chain, EnvConfig

base = BaseEnvConfig().robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0)).sim(num_envs=64)
family = base.sweep(
    paired={"robot.n": [2, 3, 4, 5], "robot.axes": ["zy", "zyy", "zyyy", "zyyyy"]},
    grid={"sim.seed": range(3)},
)

# launcher: one file per job, named by the run's identity
with tempfile.TemporaryDirectory() as out:
    for env_cfg in family.configs():                                  # the resolved EnvConfigs
        with open(os.path.join(out, f"{env_cfg.env_hash}.json"), "w") as f:
            json.dump(env_cfg.to_dict(), f)
    files = sorted(os.listdir(out))
    assert len(files) == len(family) == 12

    # worker: rebuild from the file alone; env = make_env(cfg) would follow
    with open(os.path.join(out, files[0])) as f:
        cfg = EnvConfig.from_dict(json.load(f))
    assert f"{cfg.env_hash}.json" == files[0]                         # the same hash on the other side
```

Why a *resolved* dict and not the builder script:

- It holds every value, so a later change of a default cannot silently change a queued job ([base-env](configs/base-env.md#files-and-reproducibility)).
- The hash in the file name is checked by the worker for free.
- A worker needs no state from the launcher. The task and simulator registries are **per process**: a worker that uses a custom task provider has to register it
  (`register_task_provider(...)` at the top of the job script, or an entry point in an installed package), and a `spawn`ed robosuite worker has to call `apply_compat()`.

A job array is then "job *i* reads file *i*": the index maps to a row of `family.table()`, which you keep with the results.

## Choosing the simulator for the job

| workload | use |
|---|---|
| debugging, one or a few envs, exact reference numbers | `mujoco` on the CPU. The worlds of one environment run serially in one process, so give a job one core per environment process and parallelise across jobs |
| thousands of envs of one embodiment | `warp` on one GPU: `num_envs` in the thousands, `WarpOptions` capacities sized for the scene ([sim](configs/sim.md#warpoptionsnjmax-nconmax-cuda_graph)) |
| comparing embodiments | one job per embodiment (a different `n` is a different model, hence a different environment), same `sim.seed` and `num_envs` so they share the task draws |

Request the GPU in the scheduler and use `sim.device="cuda:0"`: a job normally sees only the devices it was given, renumbered from 0. Choose the CPU count of a job from what it runs; do not over-subscribe a node
with MuJoCo environment processes.

## Environment checklist

- **One MuJoCo version per study**, recorded with the results; dynamics differ slightly between versions. For `warp`, `mujoco` and `mujoco-warp` must be one release (`make_env` says so if not).
- **Same Python environment on login and compute nodes**: build it once on shared storage and activate it in the job. The package is a normal editable install (`pip install -e ".[env,robosuite,warp]"`).
- **Rendering on a node without a display**: `MUJOCO_GL=egl` before MuJoCo is imported; on a multi-GPU node pin an idle device with `MUJOCO_EGL_DEVICE_ID` ([viz](viz.md#headless-machines)).
- **Do not run heavy work on a login node.** Tests, rollouts and notebooks that step many envs belong in a job. A short interactive allocation is fine for notebooks.
- **Caches**: Warp compiles kernels on first use; point its cache at node-local or shared storage you own if the home directory is small or read-only.
- **Record**: `env.env_hash`, `env.robot_hash`, the resolved dict, the package version (`robotdesigner.__version__`) plus its git commit, and the MuJoCo, torch, TorchRL and (if used) robosuite and Warp versions.

## Moving code between machines

When developing on your own machine and running on the cluster, ship the repository as one archive (or `git` push/pull) rather than file by file, and keep the environment on the cluster
as an editable install of the checkout, so `git pull` is the whole update. Do not commit cluster paths, user names, host names or job ids: the repository is public, and the resolved dict contains everything a rerun needs.
