import subprocess
from dataclasses import replace
import sys

import pytest

from robotdesigner.config import ConfigError, EnvConfig, MujocoOptions, SimConfig, WarpOptions
from robotdesigner.simulators import get_simulator, list_simulators
from tests.test_env_config import make_cfg

SIM = dict(simulator="warp", num_envs=4, device="cuda:0", dt=0.002, integrator="euler", control_freq=50.0, seed=0,
           scene="empty")


def sim(**kw):
    return SimConfig(**{**SIM, "options": WarpOptions(njmax=64, nconmax=16, cuda_graph=True), **kw})


def test_options_round_trip_and_validation():
    o = WarpOptions(njmax=64, nconmax=16, cuda_graph=False)
    assert WarpOptions.from_dict(o.to_dict()) == o
    assert WarpOptions.default().njmax >= 1
    for bad, path in [
        (dict(njmax=0, nconmax=1, cuda_graph=True), "njmax"),
        (dict(njmax=True, nconmax=1, cuda_graph=True), "njmax"),
        (dict(njmax=1, nconmax=0, cuda_graph=True), "nconmax"),
        (dict(njmax=1, nconmax=1, cuda_graph=1), "cuda_graph"),
    ]:
        with pytest.raises(ConfigError) as e:
            WarpOptions(**bad)
        assert e.value.path == path
    with pytest.raises(ConfigError):
        WarpOptions.from_dict({"njmax": 1, "nconmax": 1})


def test_simulator_config_and_hash():
    s = sim()
    assert SimConfig.from_dict(s.to_dict()) == s
    base = make_cfg()
    hashes = {
        replace(base, sim=sim(options=WarpOptions(**{**s.options.to_dict(), **change}))).env_hash
        for change in [{}, {"njmax": 65}, {"nconmax": 17}, {"cuda_graph": False}]
    }
    assert len(hashes) == 4


def test_wrong_options_class_both_directions():
    with pytest.raises(ConfigError) as e:
        sim(options=MujocoOptions(self_collision=False))
    assert e.value.path == "options"
    with pytest.raises(ConfigError) as e:
        SimConfig(**{**SIM, "simulator": "mujoco", "device": "cpu", "options": WarpOptions.default()})
    assert e.value.path == "options"


def test_registered_and_capabilities_without_heavy_imports():
    assert "warp" in list_simulators()
    code = (
        "import sys\n"
        "for m in ['torch', 'mujoco', 'mujoco_warp', 'warp']: sys.modules[m] = None\n"
        "from robotdesigner.simulators import get_simulator\n"
        "c = get_simulator('warp').capabilities()\n"
        "assert c.devices == {'cuda'}\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_validate_reports_every_problem_without_cuda():
    cfg = replace(make_cfg(), sim=sim(scene="plain_floor"))
    paths = {p.path for p in get_simulator("warp").validate(cfg)}
    try:
        import torch

        has_cuda = torch.cuda.is_available()
    except ImportError:
        has_cuda = False
    assert ("sim.device" in paths) != has_cuda
