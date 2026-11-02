"""The benchmark script's argument handling and JSON schema, with a fake runner (numpy only: no GPU, no engine)."""

import importlib.util
import json
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location("bench_warp", Path(__file__).parent.parent / "benchmarks" / "bench_warp.py")
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


def test_values_assignments_and_sweeps_parse_like_overrides():
    assert bench.parse_value("64") == 64 and bench.parse_value("true") is True and bench.parse_value("zyy") == "zyy"
    assert bench.parse_assignments(["robot.n=3", "sim.integrator=rk4"]) == {"robot.n": 3, "sim.integrator": "rk4"}
    assert bench.parse_sweep(["sim.num_envs=256,1024", "sim.options.cuda_graph=true,false"]) == {
        "sim.num_envs": [256, 1024], "sim.options.cuda_graph": [True, False]}
    for bad in (["no_equals"], ["=1"]):
        with pytest.raises(SystemExit):
            bench.parse_assignments(bad)
    with pytest.raises(SystemExit):
        bench.parse_sweep(["sim.num_envs="])


def test_the_grid_varies_the_last_axis_fastest():
    grid = bench.settings({"robot.n": 3}, {"sim.num_envs": [1, 2], "sim.options.cuda_graph": [True, False]})
    assert grid == [
        {"robot.n": 3, "sim.num_envs": 1, "sim.options.cuda_graph": True},
        {"robot.n": 3, "sim.num_envs": 1, "sim.options.cuda_graph": False},
        {"robot.n": 3, "sim.num_envs": 2, "sim.options.cuda_graph": True},
        {"robot.n": 3, "sim.num_envs": 2, "sim.options.cuda_graph": False},
    ]
    assert bench.settings({"robot.n": 3}, {}) == [{"robot.n": 3}]


def fake_runner(cfg, overrides, args, simulator):
    record = {k: None for k in bench.RECORD_FIELDS}
    record.update(overrides=overrides, task=args.task, simulator=simulator, env_steps_per_s=1.0, step_ms=1.0)
    if overrides.get("sim.num_envs") == 99:
        bench.failure(record, MemoryError("out of memory"))
    return record


def test_main_writes_one_record_per_setting_with_a_cpu_baseline_for_small_batches(tmp_path, capsys):
    out = tmp_path / "bench.json"
    records = bench.main(["--task", "empty/idle", "--set", "robot.n=3", "--sweep", "sim.num_envs=8,128,99",
                          "--out", str(out)], runner=fake_runner)
    assert [(r["overrides"]["sim.num_envs"], r["simulator"]) for r in records] == [
        (8, "warp"), (8, "mujoco"), (128, "warp"), (99, "warp")]
    saved = json.loads(out.read_text())
    assert saved == records and all(set(r) == set(bench.RECORD_FIELDS) for r in saved)
    assert [r["failed"] for r in saved if r["failed"]] == ["MemoryError: out of memory"]  # recorded, not skipped
    assert "warp" in capsys.readouterr().out


def test_the_base_config_builds_for_the_empty_task_and_rejects_unknown_tasks():
    assert bench.base_config("empty/idle").resolve().task.id == "empty/idle"
    with pytest.raises(SystemExit):
        bench.base_config("nope/task")
