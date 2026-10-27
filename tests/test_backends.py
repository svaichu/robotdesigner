import subprocess
import sys
from dataclasses import dataclass

import pytest

from robotdesigner.backends import (
    Capabilities,
    Problem,
    get_backend,
    list_backends,
    register_backend,
)
from robotdesigner.config import ConfigError, register_backend_options
from robotdesigner.envs import make_env
from tests.test_env_config import make_cfg


@dataclass(frozen=True)
class DummyOptions:
    def to_dict(self):
        return {}

    @classmethod
    def from_dict(cls, d):
        return cls()


class DummyBackend:
    name = "dummy"

    def __init__(self, extra=()):
        self.extra = list(extra)

    def capabilities(self):
        return Capabilities(
            control_modes=frozenset({"joint_torque"}),
            task_ids=frozenset({"native/reach"}),
            devices=frozenset({"cuda"}),
            robot_kinds=frozenset({"parametric"}),
            integrators=frozenset({"euler"}),
        )

    def validate(self, cfg):
        return self.extra

    def make_env(self, cfg):
        return ("env", cfg.env_hash)


def dummy_cfg(**sim):
    from dataclasses import replace

    base = make_cfg()
    return type(base)(base.robot, base.control, base.task, replace(base.sim, backend="dummy", **sim), DummyOptions())


@pytest.fixture
def dummy():
    register_backend_options("dummy", DummyOptions)
    register_backend("dummy", DummyBackend(extra=[Problem("sim.scene", "scene not available")]))
    yield
    from robotdesigner.backends import _REGISTRY

    _REGISTRY.pop("dummy", None)


def test_unknown_backend_lists_registered_names():
    with pytest.raises(ConfigError) as e:
        get_backend("nope")
    assert "nope" in str(e.value) and "mujoco" in str(e.value)
    assert "mujoco" in list_backends()


def test_dummy_backend_reports_all_problems(dummy):
    cfg = dummy_cfg()
    with pytest.raises(ConfigError) as e:
        make_env(cfg)
    msg = str(e.value)
    assert "control.mode" in msg and "sim.device" in msg and "sim.scene" in msg
    assert "'dummy'" in msg and "3 problems" in msg


def test_dummy_backend_runs_when_supported(dummy):
    from dataclasses import replace

    register_backend("dummy", DummyBackend())
    cfg = dummy_cfg(device="cuda:0")
    cfg = type(cfg)(cfg.robot, replace(cfg.control, mode="joint_torque", gains=cfg.control.gains), cfg.task, cfg.sim, cfg.backend_options)
    assert make_env(cfg) == ("env", cfg.env_hash)


def test_mujoco_backend_rejects_unsupported_config():
    cfg = make_cfg()
    from dataclasses import replace

    cuda = type(cfg)(cfg.robot, cfg.control, cfg.task, replace(cfg.sim, device="cuda:0"), cfg.backend_options)
    with pytest.raises(ConfigError, match="sim.device"):
        make_env(cuda)


def test_lazy_import_keeps_engines_out():
    code = (
        "import sys\n"
        "import robotdesigner.backends, robotdesigner.envs\n"
        "bad = [m for m in ('mujoco', 'torch', 'torchrl') if m in sys.modules]\n"
        "assert not bad, bad\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
