import importlib.metadata
import pkgutil
import subprocess
import sys
from dataclasses import dataclass

import pytest

import robotdesigner.tasks
from robotdesigner.config import ConfigError, Problem, TaskConfig, list_task_providers, register_task_provider
from robotdesigner.config import providers
from robotdesigner.config.providers import ENTRY_POINT_GROUP, known_task_ids, params_class, split_task_id

BLOCKED = ["torch", "torchrl", "tensordict", "mujoco", "mujoco_warp", "robosuite"]


@dataclass(frozen=True)
class ExtParams:
    provider = "ext"
    uses_goal = False
    name: str = "thing"

    def to_dict(self):
        return {"name": self.name}

    @classmethod
    def from_dict(cls, d):
        return cls(**d)


class ExtTasks:
    def task_names(self):
        return ("thing",)

    def params_class(self, name):
        return ExtParams if name == "thing" else None

    def validate(self, cfg, simulator):
        return []

    def build(self, cfg, simulator):
        return None


EXT = ExtTasks()


@pytest.fixture
def clean_registry():
    saved = (dict(providers._REGISTRY), set(providers._EXPLICIT), dict(providers._STATE))
    yield
    providers._REGISTRY.clear()
    providers._REGISTRY.update(saved[0])
    providers._EXPLICIT.clear()
    providers._EXPLICIT.update(saved[1])
    providers._STATE.update(saved[2])


def test_split_task_id():
    assert split_task_id("ns/name") == ("ns", "name")
    assert split_task_id("ns/a/b") == ("ns", "a/b")
    for bad in ("", "ns", "ns/", "/name", "NS/name", "1ns/name", None, 3):
        assert split_task_id(bad) is None


def test_every_subpackage_of_tasks_is_a_registered_provider():
    scanned = {m.name for m in pkgutil.iter_modules(robotdesigner.tasks.__path__) if m.ispkg}
    assert scanned and scanned <= set(list_task_providers())
    ids = known_task_ids()
    assert all(any(i.startswith(f"{ns}/") for i in ids) for ns in scanned)


def test_config_side_of_every_provider_needs_no_engine():
    code = (
        "import sys\n"
        f"for m in {BLOCKED!r}: sys.modules[m] = None\n"
        "from robotdesigner.config import list_task_providers\n"
        "from robotdesigner.config.providers import known_task_ids, params_class\n"
        "ids = known_task_ids()\n"
        "assert ids and not any('could not be loaded' in i for i in ids), ids\n"
        "for i in ids:\n"
        "    if '<' not in i:\n"
        "        assert params_class(i) is not None, i\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_explicit_registration_and_duplicates(clean_registry):
    register_task_provider("ext", EXT)
    assert "ext" in list_task_providers() and "ext/thing" in known_task_ids()
    assert params_class("ext/thing") is ExtParams and params_class("ext/other") is None
    register_task_provider("ext", EXT)  # the same target again is fine
    with pytest.raises(ConfigError, match="already registered"):
        register_task_provider("ext", ExtTasks())
    register_task_provider("ext", ExtTasks, replace=True)  # a class is instantiated on first use
    assert params_class("ext/thing") is ExtParams
    cfg = TaskConfig(ExtParams(), 5, None, None)
    assert TaskConfig.from_dict(cfg.to_dict()) == cfg


def test_unregister(clean_registry):
    from robotdesigner.config import unregister_task_provider

    register_task_provider("ext", EXT)
    unregister_task_provider("ext")
    unregister_task_provider("ext")  # no error the second time
    assert "ext" not in list_task_providers() and params_class("ext/thing") is None
    register_task_provider("ext", ExtTasks())  # free again, no replace needed


@pytest.mark.parametrize("bad", ["", "Ext", "1ext", "ext-x", "a b", None])
def test_namespace_must_be_an_identifier(bad):
    with pytest.raises(ConfigError, match="namespace"):
        register_task_provider(bad, EXT)


def test_entry_point_group_adds_providers(clean_registry, monkeypatch):
    ep = importlib.metadata.EntryPoint(name="viaep", value="tests.test_task_registry:EXT", group=ENTRY_POINT_GROUP)
    monkeypatch.setattr(
        importlib.metadata, "entry_points", lambda **kw: [ep] if kw.get("group") == ENTRY_POINT_GROUP else []
    )
    providers._STATE["discovered"] = False
    assert "viaep" in list_task_providers()
    assert params_class("viaep/thing") is ExtParams


def test_explicit_registration_wins_over_discovery(clean_registry):
    providers._STATE["discovered"] = False
    first = list_task_providers()[0]
    register_task_provider(first, EXT, replace=True)
    assert providers._load(first) is EXT


def test_provider_that_cannot_load_is_reported_not_fatal(clean_registry):
    register_task_provider("broken", "no_such_module_anywhere:PROVIDER")
    with pytest.raises(ConfigError, match="'broken'.*could not be loaded"):
        params_class("broken/x")
    assert any(i.startswith("broken/?") for i in known_task_ids())
    assert any(not i.startswith("broken/") for i in known_task_ids())  # the others are still listed


def test_object_without_the_protocol_is_rejected(clean_registry):
    register_task_provider("odd", object())
    with pytest.raises(ConfigError, match="TaskProvider protocol"):
        params_class("odd/x")


def test_unknown_provider_and_name_errors_carry_paths_and_list_what_is_known(clean_registry):
    register_task_provider("ext", EXT)
    good = TaskConfig(ExtParams(), 5, None, None).to_dict()
    with pytest.raises(ConfigError) as e:
        TaskConfig.from_dict({**good, "provider": "nope"})
    assert str(e.value).startswith("provider:") and all(ns in str(e.value) for ns in list_task_providers())
    with pytest.raises(ConfigError) as e:
        TaskConfig.from_dict({**good, "params": {"name": "nothing"}})
    assert str(e.value).startswith("params.name:") and all(i in str(e.value) for i in known_task_ids())
    with pytest.raises(ConfigError, match="^params.name: missing"):
        TaskConfig.from_dict({**good, "params": {}})
