"""The core names no robot provider and no robot.

Robot namespaces and robots belong to the providers (``robotdesigner/robot_providers/<namespace>``). The code every robot
goes through (configs, simulators, ``make_env``, viz) must not mention them. The names to look for are read from the
registry and the provider cards, so this test holds no list of robots either.
"""

import json
import pathlib
import re

from robotdesigner.config import list_robot_providers

SRC = pathlib.Path(__file__).resolve().parents[1] / "src" / "robotdesigner"
PROVIDERS = SRC / "robot_providers"
CORE = [
    *sorted((SRC / "config").rglob("*.py")),
    *sorted((SRC / "simulators").rglob("*.py")),
    *sorted((SRC / "envs").rglob("*.py")),
    *sorted((SRC / "viz").rglob("*.py")),
    *sorted((SRC / "robot").rglob("*.py")),
    *sorted((SRC / "tasks").rglob("*.py")),
    SRC / "__init__.py",
]


def needles():
    out = []
    for ns in list_robot_providers():
        if ns in {"toyrobot", "demo", "scratch", "broken"}:  # registered by tests, not shipped
            continue
        out.append((f"namespace literal {ns!r}", re.compile(rf"""["']{re.escape(ns)}["']""")))
        out.append((f"namespace {ns}", re.compile(rf"(?<![\w.]){re.escape(ns)}(?![\w])(?=[/.:])")))
        for card in sorted((PROVIDERS / ns).glob("cards*.json")):
            for name in json.loads(card.read_text()):
                if not name.startswith("_"):
                    out.append((f"robot {name}", re.compile(rf"\b{re.escape(name)}\b")))
    return out


def test_registry_is_not_empty_so_the_check_means_something():
    assert "menagerie" in list_robot_providers() and len(needles()) > 20 and len(CORE) > 10


def test_core_code_names_no_robot_provider_or_robot():
    found = []
    for path in CORE:
        for lineno, line in enumerate(path.read_text().splitlines(), 1):
            for what, pattern in needles():
                if pattern.search(line):
                    found.append(f"{path.relative_to(SRC)}:{lineno}: {what}: {line.strip()}")
    assert not found, "the core must not name a robot provider or a robot:\n" + "\n".join(found)
