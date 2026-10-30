import subprocess
import sys
from dataclasses import dataclass

import pytest

from robotdesigner.simulators import (
    Capabilities,
    Problem,
    get_simulator,
    list_simulators,
    register_simulator,
)

from robotdesigner.config import ConfigError, Pose, TaskConfig, register_simulator_options, register_task_provider
from robotdesigner.envs import make_env
from tests.test_env_config import make_cfg


@dataclass(frozen=True)
class DummyOptions:
    def to_dict(self):
        return {}

    @classmethod
    def from_dict(cls, d):
        return cls()


@dataclass(frozen=True)
class DummyParams:
    provider = "dummy"
    uses_goal = False
    name: str = "noop"

    def to_dict(self):
        return {"name": self.name}

    @classmethod
    def from_dict(cls, d):
        return cls(**d)


class DummyTasks:
    """A provider for tests: one task, no engine."""

    def __init__(self, problems=()):
        self.problems = list(problems)

    def task_names(self):
        return ("noop",)

    def params_class(self, name):
        return DummyParams if name == "noop" else None

    def default_mount(self, name):
        return Pose.identity()

    def validate(self, cfg, simulator):
        return self.problems

    def build(self, cfg, simulator):
        return ("task", simulator)


class DummySimulator:
    name = "dummy"

    def __init__(self, extra=()):
        self.extra = list(extra)

    def capabilities(self):
        return Capabilities(
            control_modes=frozenset({"joint_torque"}),
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
    task = TaskConfig(DummyParams(), 10, None, None, Pose.identity())
    return type(base)(base.robot, base.control, task, replace(base.sim, simulator="dummy", options=DummyOptions(), **sim))


@pytest.fixture
def dummy():
    from robotdesigner.config import providers

    register_simulator_options("dummy", DummyOptions)
    register_simulator("dummy", DummySimulator(extra=[Problem("sim.scene", "scene not available")]))
    register_task_provider("dummy", DummyTasks(), replace=True)
    yield
    from robotdesigner.config.sim import SIMULATOR_OPTIONS
    from robotdesigner.simulators import _REGISTRY

    _REGISTRY.pop("dummy", None)
    SIMULATOR_OPTIONS.pop("dummy", None)
    providers._REGISTRY.pop("dummy", None)
    providers._EXPLICIT.discard("dummy")


def test_unknown_simulator_lists_registered_names():
    with pytest.raises(ConfigError) as e:
        get_simulator("nope")
    assert "nope" in str(e.value) and "mujoco" in str(e.value)
    assert "mujoco" in list_simulators()


def test_dummy_simulator_reports_all_problems(dummy):
    cfg = dummy_cfg()
    with pytest.raises(ConfigError) as e:
        make_env(cfg)
    msg = str(e.value)
    assert "control.mode" in msg and "sim.device" in msg and "sim.scene" in msg
    assert "'dummy'" in msg and "3 problems" in msg


def test_provider_problems_are_collected_with_the_simulator_problems(dummy):
    register_task_provider("dummy", DummyTasks([Problem("task.params", "needs a gripper")]), replace=True)
    with pytest.raises(ConfigError) as e:
        make_env(dummy_cfg())
    msg = str(e.value)
    assert "4 problems" in msg and "task.params: needs a gripper" in msg and "control.mode" in msg


def test_a_task_of_one_provider_is_checked_against_the_simulator(dummy):
    # a task provider runs only on the simulators it names; the dummy simulator is not one of them
    from dataclasses import replace

    cfg = dummy_cfg(device="cuda:0")
    cfg = type(cfg)(cfg.robot, replace(cfg.control, mode="joint_torque", gains=cfg.control.gains),
                    make_cfg().task, cfg.sim)
    register_simulator("dummy", DummySimulator())
    with pytest.raises(ConfigError, match="task 'toy/reach' runs on"):
        make_env(cfg)


def test_dummy_simulator_runs_when_supported(dummy):
    from dataclasses import replace

    register_simulator("dummy", DummySimulator())
    cfg = dummy_cfg(device="cuda:0")
    cfg = type(cfg)(cfg.robot, replace(cfg.control, mode="joint_torque", gains=cfg.control.gains), cfg.task, cfg.sim)
    assert make_env(cfg) == ("env", cfg.env_hash)


def test_mujoco_simulator_rejects_unsupported_config():
    cfg = make_cfg()
    from dataclasses import replace

    cuda = type(cfg)(cfg.robot, cfg.control, cfg.task, replace(cfg.sim, device="cuda:0"))
    with pytest.raises(ConfigError, match="sim.device"):
        make_env(cuda)


def test_lazy_import_keeps_engines_out():
    code = (
        "import sys\n"
        "import robotdesigner.simulators, robotdesigner.envs\n"
        "bad = [m for m in ('mujoco', 'torch', 'torchrl') if m in sys.modules]\n"
        "assert not bad, bad\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
