"""Task providers: who owns which task ids. Imports no simulation engine, torch or robosuite.

A task id has the form ``<namespace>/<name>``; it is derived: ``TaskConfig.params`` holds the namespace (the class
attribute ``provider`` of the params class) and the name (its ``name`` field). The namespace selects a *provider*. A provider

- names its tasks and gives the typed ``params`` class of each (the config side, numpy only),
- checks a config against a simulator (``validate``), and
- builds the engine side of a task for that simulator (``build``): scene assets, resets, observations, reward.

robotdesigner's own code knows no namespace and no task. Providers are found lazily, from

1. the subpackages of ``robotdesigner.tasks``: the subpackage name is the namespace and it exposes ``PROVIDER``;
2. the entry-point group ``robotdesigner.task_providers``: the entry-point name is the namespace, the value
   ``"module:attr"`` (this is how another project adds tasks);
3. :func:`register_task_provider`, which takes precedence over both.
"""

from __future__ import annotations

import importlib
import importlib.metadata
import pkgutil
import re
from typing import Any, Protocol, runtime_checkable

from .errors import ConfigError, Problem

__all__ = [
    "ENTRY_POINT_GROUP",
    "TaskProvider",
    "get_task_provider",
    "known_task_ids",
    "list_task_providers",
    "params_class",
    "register_task_provider",
    "split_task_id",
    "unregister_task_provider",
]

ENTRY_POINT_GROUP = "robotdesigner.task_providers"
_NAMESPACE = re.compile(r"[a-z][a-z0-9_]*")


@runtime_checkable
class TaskProvider(Protocol):
    """What a provider offers. Importing the module that defines it must import no engine."""

    def task_names(self) -> tuple[str, ...]:
        """Task names, or patterns such as ``"<EnvName>"``; shown when an id is unknown."""
        ...

    def params_class(self, name: str) -> type | None:
        """The ``params`` class of task ``name`` (``to_dict`` / ``from_dict``), or ``None`` if there is no such task."""
        ...

    def validate(self, cfg: Any, simulator: str) -> list[Problem]:
        """Every reason ``cfg`` cannot run this task on ``simulator``. May import engines."""
        ...

    def build(self, cfg: Any, simulator: str) -> Any:
        """The engine side of the task (a ``tasks.base.TaskImpl``) for ``simulator``."""
        ...


# namespace -> provider object, or a lazy "module:attr" string resolved on first use
_REGISTRY: dict[str, Any] = {}
_EXPLICIT: set[str] = set()
_STATE = {"discovered": False}


def register_task_provider(namespace: str, target: Any, *, replace: bool = False) -> None:
    """Register a provider under ``namespace``. ``target`` is a provider, a provider class (instantiated without
    arguments) or a lazy ``"module:attr"`` string. Registering a different target under a taken namespace is an
    error unless ``replace`` is true."""
    if not isinstance(namespace, str) or not _NAMESPACE.fullmatch(namespace):
        raise ConfigError("", f"task namespace must match [a-z][a-z0-9_]*, got {namespace!r}")
    if namespace in _EXPLICIT and _REGISTRY[namespace] is not target and not replace:
        raise ConfigError("", f"task namespace {namespace!r} is already registered; pass replace=True to override it")
    _REGISTRY[namespace] = target
    _EXPLICIT.add(namespace)


def unregister_task_provider(namespace: str) -> None:
    """Forget ``namespace``. Ids of that namespace become unknown until it is registered or discovered again."""
    _REGISTRY.pop(namespace, None)
    _EXPLICIT.discard(namespace)


def _discover() -> None:
    if _STATE["discovered"]:
        return
    _STATE["discovered"] = True
    tasks = importlib.import_module("robotdesigner.tasks")
    for info in pkgutil.iter_modules(tasks.__path__):
        if info.ispkg and _NAMESPACE.fullmatch(info.name):
            _REGISTRY.setdefault(info.name, f"robotdesigner.tasks.{info.name}:PROVIDER")
    for ep in importlib.metadata.entry_points(group=ENTRY_POINT_GROUP):
        if _NAMESPACE.fullmatch(ep.name):
            _REGISTRY.setdefault(ep.name, ep.value)


def _load(namespace: str) -> TaskProvider:
    target = _REGISTRY[namespace]
    if isinstance(target, str):
        module, _, attr = target.partition(":")
        try:
            obj = getattr(importlib.import_module(module), attr or "PROVIDER")
        except Exception as e:
            raise ConfigError(
                "task.provider", f"task provider {namespace!r} ({target}) could not be loaded: {type(e).__name__}: {e}"
            ) from e
        target = obj() if isinstance(obj, type) else obj
        _REGISTRY[namespace] = target
    elif isinstance(target, type):
        target = _REGISTRY[namespace] = target()
    if not isinstance(target, TaskProvider):
        raise ConfigError("task.provider", f"task provider {namespace!r} does not implement the TaskProvider protocol")
    return target


def list_task_providers() -> tuple[str, ...]:
    """Namespaces of every registered provider, sorted. Loads none of them."""
    _discover()
    return tuple(sorted(_REGISTRY))


def get_task_provider(namespace: str) -> TaskProvider:
    _discover()
    if namespace not in _REGISTRY:
        raise ConfigError(
            "task.provider", f"unknown task provider {namespace!r}; registered: {list(list_task_providers())}"
        )
    return _load(namespace)


def split_task_id(task_id: Any) -> tuple[str, str] | None:
    """``(namespace, name)`` of ``"<namespace>/<name>"``, or ``None`` when ``task_id`` has no such form."""
    if not isinstance(task_id, str):
        return None
    namespace, sep, name = task_id.partition("/")
    if not sep or not name or not _NAMESPACE.fullmatch(namespace):
        return None
    return namespace, name


def params_class(task_id: Any) -> type | None:
    """The ``params`` class of ``task_id``, or ``None`` when no registered provider has that task."""
    parts = split_task_id(task_id)
    if parts is None:
        return None
    _discover()
    if parts[0] not in _REGISTRY:
        return None
    return _load(parts[0]).params_class(parts[1])


def known_task_ids() -> list[str]:
    """Every id of every registered provider, for error messages. A provider that fails to load is listed as such."""
    out: list[str] = []
    for namespace in list_task_providers():
        try:
            out += [f"{namespace}/{name}" for name in _load(namespace).task_names()]
        except ConfigError:
            out.append(f"{namespace}/? (provider could not be loaded)")
    return out
