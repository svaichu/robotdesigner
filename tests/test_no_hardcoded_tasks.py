"""The core names no task.

Task ids, provider namespaces and params classes belong to the providers (``robotdesigner/tasks/<namespace>``). The code
that every task goes through (configs, backends, ``make_env``, viz, the task contract) must not mention them. The
names to look for are read from the registry, so this test does not hold a list of tasks either.
"""

import pathlib
import re

from robotdesigner.config import get_task_provider, list_task_providers

SRC = pathlib.Path(__file__).resolve().parents[1] / "src" / "robotdesigner"
CORE = [
    *sorted((SRC / "config").rglob("*.py")),
    *sorted((SRC / "backends").rglob("*.py")),
    *sorted((SRC / "envs").rglob("*.py")),
    *sorted((SRC / "viz").rglob("*.py")),
    SRC / "__init__.py",
    SRC / "tasks" / "__init__.py",
    SRC / "tasks" / "base.py",
]


def needles():
    out = []
    for ns in list_task_providers():
        out.append((f"task id prefix {ns}/", re.compile(rf"(?<![\w.]){re.escape(ns)}/")))
        out.append((f"namespace literal {ns!r}", re.compile(rf"""["']{re.escape(ns)}["']""")))
        provider = get_task_provider(ns)
        for name in provider.task_names():
            cls = provider.params_class(name)
            if cls is not None:
                out.append((f"params class {cls.__name__}", re.compile(rf"\b{cls.__name__}\b")))
    return out


def test_registry_is_not_empty_so_the_check_means_something():
    assert len(needles()) >= 4 and len(CORE) > 10


def test_core_code_names_no_task():
    found = []
    for path in CORE:
        for lineno, line in enumerate(path.read_text().splitlines(), 1):
            for what, pattern in needles():
                if pattern.search(line):
                    found.append(f"{path.relative_to(SRC.parent)}:{lineno}: {what}: {line.strip()[:90]}")
    assert not found, "the core must not name tasks:\n" + "\n".join(found)


def test_capabilities_have_no_task_ids():
    from robotdesigner.backends import Capabilities

    assert "task_ids" not in Capabilities.__dataclass_fields__
