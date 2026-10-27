import importlib.util
import pathlib

import pytest

pytest.importorskip("torchrl")
pytest.importorskip("mujoco")


def test_example_smoke(capsys):
    path = pathlib.Path(__file__).parent.parent / "examples" / "reach_cpu.py"
    spec = importlib.util.spec_from_file_location("reach_cpu", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.main(num_envs=2, steps=4)
    out = capsys.readouterr().out
    assert "rollout batch shape: (2, 4)" in out and "robot_hash=" in out and "env_hash=" in out
