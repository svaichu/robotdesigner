"""Throughput and capacity benchmark of the warp simulator. A plain script, not part of the wheel.

    python benchmarks/bench_warp.py --task robosuite/Lift --set robot.n=3 \\
           --sweep sim.num_envs=256,1024,4096,16384 sim.options.cuda_graph=true,false \\
           --steps 200 --out bench_lift.json
    python benchmarks/bench_warp.py --task empty/idle --sweep robot.n=3,7 sim.num_envs=1024,4096 --out bench_idle.json

Every setting is a config built with the config layer (dotted paths, as ``BaseEnvConfig.with_overrides``), so a record carries
its ``env_hash``. Per setting one JSON record is written, and the same setting on the CPU simulator is run as a baseline for
``num_envs <= --baseline-max-envs``. ``--smoke`` runs the warp simulator on the Warp library's CPU device, to check the script
and to measure capacities (contact and constraint counts do not depend on the device); its timings mean nothing.

Time what a user gets: steady-state ``env.step`` after warm-up, reported apart from the first call (kernel compilation, graph
capture) and from reset cost; the device is synchronised before a clock is read.
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
import time
from typing import Any, Callable

RECORD_FIELDS = (
    "env_hash", "overrides", "task", "n", "simulator", "device", "env_steps_per_s", "step_ms", "first_call_s",
    "reset_ms", "step_and_reset_steps_per_s", "reset_rate", "peak_gpu_mem_mb", "max_nefc", "max_nacon", "max_contacts_per_world",
    "gpu", "versions", "failed",
)


def parse_value(text: str) -> Any:
    """A JSON literal, or the string itself (the same rule as ``BaseEnvConfig.with_overrides``)."""
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return text


def parse_assignments(items: list[str]) -> dict[str, Any]:
    out = {}
    for item in items or []:
        path, sep, value = item.partition("=")
        if not sep or not path:
            raise SystemExit(f"expected path=value, got {item!r}")
        out[path] = parse_value(value)
    return out


def parse_sweep(items: list[str]) -> dict[str, list[Any]]:
    axes = {}
    for item in items or []:
        path, sep, values = item.partition("=")
        if not sep or not path or not values:
            raise SystemExit(f"expected path=v1,v2,..., got {item!r}")
        axes[path] = [parse_value(v) for v in values.split(",")]
    return axes


def settings(fixed: dict[str, Any], axes: dict[str, list[Any]]) -> list[dict[str, Any]]:
    """The grid of overrides: ``fixed`` plus every combination of the sweep axes (the last axis varies fastest)."""
    if not axes:
        return [dict(fixed)]
    keys = list(axes)
    return [{**fixed, **dict(zip(keys, combo))} for combo in itertools.product(*(axes[k] for k in keys))]


def base_config(task: str):
    """A ``BaseEnvConfig`` for ``empty/idle`` or ``robosuite/<Name>`` (Lift: the cube height; others: reach the first object)."""
    from robotdesigner.config import BaseEnvConfig, Chain

    cfg = BaseEnvConfig().robot(Chain(n=3, axes="zyy", total_length=0.9, total_mass=3.0, range=(-2.5, 2.5)))
    namespace, _, name = task.partition("/")
    if (namespace, name) == ("empty", "idle"):
        return cfg
    if namespace == "robosuite":
        from robotdesigner.tasks.robosuite import LiftObject, ReachObject, RobosuiteParams, default_mount
        from robotdesigner.tasks.robosuite.scene import load_scene

        if name == "Lift":
            objective = LiftObject("cube", 0.84)
        else:
            first = next(o.name for o in load_scene(name, {}, 0).objects if not o.visual)
            objective = ReachObject(first, 0.05)
        params = RobosuiteParams(name, {}, objective, 0.0, False)
        return cfg.task(params, mount=default_mount(name)).sim(scene="empty")
    raise SystemExit(f"unknown task {task!r}: use empty/idle or robosuite/<EnvName>")


def failure(record: dict, err: BaseException) -> dict:
    record["failed"] = f"{type(err).__name__}: {err}".splitlines()[0][:300]
    return record


def run_setting(cfg, overrides: dict, args, simulator: str) -> dict:
    """Measure one resolved config. ``cfg`` is a ``BaseEnvConfig``; the simulator and device are set here."""
    record = {k: None for k in RECORD_FIELDS}
    record.update(overrides=overrides, task=args.task, simulator=simulator, reset_rate=args.reset_rate)
    try:
        import torch

        device = "cpu" if simulator == "mujoco" or args.smoke else args.device
        full = cfg.with_overrides({**overrides, "sim.simulator": simulator, "sim.device": device if simulator == "warp" else "cpu"})
        resolved = full.resolve()
        record.update(env_hash=resolved.env_hash, n=resolved.robot.n, device=resolved.sim.device)
        env = _build(resolved, simulator, args.smoke)
        record.update(_measure(env, args, torch))
        record["gpu"] = torch.cuda.get_device_name(0) if torch.cuda.is_available() else None
        record["versions"] = versions()
        env.close()
    except Exception as err:  # a failed setting (out of memory, overflow) is recorded, not skipped
        failure(record, err)
    return record


def _build(resolved, simulator: str, smoke: bool):
    from robotdesigner.config.providers import get_task_provider
    from robotdesigner.envs import make_env

    if simulator == "warp" and smoke:  # make_env refuses the cpu device; this is the tests' route as well
        from robotdesigner.simulators.warp.env import WarpEnv

        return WarpEnv(resolved, get_task_provider(resolved.task.params.provider).build(resolved, "warp"))
    return make_env(resolved)


def sync(device) -> None:
    import torch

    if torch.device(device).type == "cuda":
        torch.cuda.synchronize(device)


def _measure(env, args, torch) -> dict:
    N, n = env.batch_size[0], env._n
    gen = torch.Generator().manual_seed(0)

    def actions():
        return (torch.rand(N, n, generator=gen) * 2 - 1).to(env.device)

    mem0 = torch.cuda.mem_get_info()[0] if env.device.type == "cuda" else None
    t0 = time.perf_counter()
    td = env.reset()
    td["action"] = actions()
    td = env.step(td)["next"]
    sync(env.device)
    first_call = time.perf_counter() - t0
    peak = None if mem0 is None else (mem0 - torch.cuda.mem_get_info()[0]) / 2**20

    for _ in range(args.warmup):
        td["action"] = actions()
        td = env.step(td)["next"]
    max_nefc = max_contacts = max_nacon = None
    sim = env._sim
    sync(env.device)
    t0 = time.perf_counter()
    for _ in range(args.steps):
        td["action"] = actions()
        td = env.step(td)["next"]
    sync(env.device)
    elapsed = time.perf_counter() - t0
    if hasattr(sim, "capacity_usage"):  # a separate pass: reading counters synchronises and must not be timed
        max_nefc = max_contacts = max_nacon = 0
        for _ in range(min(args.steps, 50)):
            td["action"] = actions()
            td = env.step(td)["next"]
            u = sim.capacity_usage()
            max_nefc, max_contacts, max_nacon = max(max_nefc, u["max_nefc"]), max(max_contacts, u["max_contacts_per_world"]), max(max_nacon, u["nacon"])

    reset_ms, mixed = _reset_costs(env, td, args, torch, actions, gen)
    return {
        "env_steps_per_s": N * args.steps / elapsed, "step_ms": 1e3 * elapsed / args.steps, "first_call_s": first_call,
        "peak_gpu_mem_mb": peak, "max_nefc": max_nefc, "max_nacon": max_nacon,
        "max_contacts_per_world": max_contacts, "reset_ms": reset_ms, "step_and_reset_steps_per_s": mixed,
    }


def _reset_costs(env, td, args, torch, actions, gen):
    """The cost of one masked reset (with the host-side placement sampler) and the throughput of step + reset at a stated rate."""
    N = env.batch_size[0]
    reps = max(1, min(args.steps, 20))
    sync(env.device)
    t0 = time.perf_counter()
    for _ in range(reps):
        td["_reset"] = (torch.rand(N, 1, generator=gen) < args.reset_rate).to(env.device)
        td = env.reset(td)
    sync(env.device)
    reset_ms = 1e3 * (time.perf_counter() - t0) / reps
    t0 = time.perf_counter()
    for _ in range(reps):
        td["action"] = actions()
        td["_reset"] = (torch.rand(N, 1, generator=gen) < args.reset_rate).to(env.device)
        td = env.step(td)["next"]
        td = env.reset(td)
    sync(env.device)
    return reset_ms, N * reps / (time.perf_counter() - t0)


def versions() -> dict[str, str | None]:
    from importlib import metadata

    out = {}
    for key, dist in (("mujoco", "mujoco"), ("mujoco_warp", "mujoco-warp"), ("warp", "warp-lang"), ("torch", "torch")):
        try:
            out[key] = metadata.version(dist)
        except metadata.PackageNotFoundError:
            out[key] = None
    return out


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--task", default="empty/idle", help="empty/idle or robosuite/<EnvName>")
    p.add_argument("--set", nargs="*", default=[], metavar="PATH=VALUE", help="fixed overrides, dotted config paths")
    p.add_argument("--sweep", nargs="*", default=[], metavar="PATH=V1,V2", help="axes of a grid of overrides")
    p.add_argument("--steps", type=int, default=200, help="timed steps per setting")
    p.add_argument("--warmup", type=int, default=20, help="untimed steps before timing")
    p.add_argument("--reset-rate", type=float, default=0.02, help="fraction of envs reset per step in the reset measurements")
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--baseline-max-envs", type=int, default=64, help="also run the CPU simulator for num_envs up to this")
    p.add_argument("--smoke", action="store_true", help="run warp on the Warp library's CPU device (timings are meaningless)")
    p.add_argument("--out", default="bench.json")
    return p


def main(argv: list[str] | None = None, runner: Callable | None = None) -> list[dict]:
    args = build_parser().parse_args(argv)
    runner = runner or run_setting
    fixed, axes = parse_assignments(args.set), parse_sweep(args.sweep)
    cfg = base_config(args.task)
    records = []
    for overrides in settings(fixed, axes):
        sims = ["warp"]
        if overrides.get("sim.num_envs", cfg.resolve().sim.num_envs) <= args.baseline_max_envs:
            sims.append("mujoco")
        for simulator in sims:
            rec = runner(cfg, overrides, args, simulator)
            records.append(rec)
            status = rec["failed"] or f"{rec['env_steps_per_s']:.0f} env-steps/s, step {rec['step_ms']:.2f} ms"
            print(f"{simulator:6s} {json.dumps(overrides)} -> {status}", flush=True)
    with open(args.out, "w") as f:
        json.dump(records, f, indent=2)
    return records


if __name__ == "__main__":
    main(sys.argv[1:])
    sys.exit(0)
