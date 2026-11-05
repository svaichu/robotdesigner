"""Robot providers: who owns which robot namespace. Imports no simulation engine and no torch.

An :class:`~robotdesigner.robot.ExternalRobot` refers to a model that lives elsewhere. Its ``provider`` field is a
namespace (``"menagerie"``), and the namespace selects a *provider*, which

- lists its robots (``names``),
- reads a model and returns the numbers a config needs (``describe``, engine side),
- opens the model as an editable ``mujoco.MjSpec`` after checking it against ``robot.source`` (``spec``, engine side),
- checks a robot against a simulator (``validate``), and
- gives a numpy oracle of the robot's kinematics (``kinematics``, engine side).

This mirrors task providers (``config/providers.py``). The core knows no namespace. Providers are found lazily, from

1. the subpackages of ``robotdesigner.robot_providers``: the subpackage name is the namespace and it exposes ``PROVIDER``;
2. the entry-point group ``robotdesigner.robot_providers``: the entry-point name is the namespace, the value ``"module:attr"``;
3. :func:`register_robot_provider`, which takes precedence over both.

Answering ``list_robot_providers`` loads nothing; a provider module is imported when the provider is first used.
"""

from __future__ import annotations

import importlib
import importlib.metadata
import pkgutil
import re
from typing import Any, Protocol, runtime_checkable

from ..errors import ConfigError, Problem

__all__ = [
    "ENTRY_POINT_GROUP",
    "RobotProvider",
    "get_robot_provider",
    "list_robot_providers",
    "register_robot_provider",
    "unregister_robot_provider",
]

ENTRY_POINT_GROUP = "robotdesigner.robot_providers"
_NAMESPACE = re.compile(r"[a-z][a-z0-9_]*")


@runtime_checkable
class RobotProvider(Protocol):
    """What a robot provider offers. Importing the module that defines it must import no engine."""

    def names(self) -> tuple[str, ...]:
        """Robot names, or patterns; shown when a name is unknown."""
        ...

    def describe(self, name: str, **options: Any) -> Any:
        """Read the model of robot ``name`` and return an ``ExternalRobot``. Engine side."""
        ...

    def spec(self, robot: Any) -> Any:
        """An editable ``mujoco.MjSpec`` of ``robot``, checked against ``robot.source``. Engine side."""
        ...

    def validate(self, robot: Any, simulator: str) -> list[Problem]:
        """Every reason ``robot`` cannot run on ``simulator``. May import engines."""
        ...

    def kinematics(self, robot: Any) -> Any:
        """An object with ``forward_ee_batch(q, base)`` and ``sample_joint_positions(num, rng)``. Engine side."""
        ...


# namespace -> provider object, or a lazy "module:attr" string resolved on first use
_REGISTRY: dict[str, Any] = {}
_EXPLICIT: set[str] = set()
_STATE = {"discovered": False}


def register_robot_provider(namespace: str, target: Any, *, replace: bool = False) -> None:
    """Register a provider under ``namespace``. ``target`` is a provider, a provider class (instantiated without
    arguments) or a lazy ``"module:attr"`` string. Registering a different target under a taken namespace is an
    error unless ``replace`` is true."""
    if not isinstance(namespace, str) or not _NAMESPACE.fullmatch(namespace):
        raise ConfigError("", f"robot namespace must match [a-z][a-z0-9_]*, got {namespace!r}")
    if namespace in _EXPLICIT and _REGISTRY[namespace] is not target and not replace:
        raise ConfigError("", f"robot namespace {namespace!r} is already registered; pass replace=True to override it")
    _REGISTRY[namespace] = target
    _EXPLICIT.add(namespace)


def unregister_robot_provider(namespace: str) -> None:
    """Forget ``namespace``. Robots of that namespace become unknown until it is registered or discovered again."""
    _REGISTRY.pop(namespace, None)
    _EXPLICIT.discard(namespace)


def _discover() -> None:
    if _STATE["discovered"]:
        return
    _STATE["discovered"] = True
    package = importlib.import_module("robotdesigner.robot_providers")
    for info in pkgutil.iter_modules(package.__path__):
        if info.ispkg and _NAMESPACE.fullmatch(info.name):
            _REGISTRY.setdefault(info.name, f"robotdesigner.robot_providers.{info.name}:PROVIDER")
    for ep in importlib.metadata.entry_points(group=ENTRY_POINT_GROUP):
        if _NAMESPACE.fullmatch(ep.name):
            _REGISTRY.setdefault(ep.name, ep.value)


def _load(namespace: str) -> RobotProvider:
    target = _REGISTRY[namespace]
    if isinstance(target, str):
        module, _, attr = target.partition(":")
        try:
            obj = getattr(importlib.import_module(module), attr or "PROVIDER")
        except Exception as e:
            raise ConfigError(
                "robot.provider",
                f"robot provider {namespace!r} ({target}) could not be loaded: {type(e).__name__}: {e}",
            ) from e
        target = obj() if isinstance(obj, type) else obj
        _REGISTRY[namespace] = target
    elif isinstance(target, type):
        target = _REGISTRY[namespace] = target()
    if not isinstance(target, RobotProvider):
        raise ConfigError(
            "robot.provider", f"robot provider {namespace!r} does not implement the RobotProvider protocol"
        )
    return target


def list_robot_providers() -> tuple[str, ...]:
    """Namespaces of every registered robot provider, sorted. Loads none of them."""
    _discover()
    return tuple(sorted(_REGISTRY))


def get_robot_provider(namespace: str) -> RobotProvider:
    _discover()
    if namespace not in _REGISTRY:
        raise ConfigError(
            "robot.provider", f"unknown robot provider {namespace!r}; registered: {list(list_robot_providers())}"
        )
    return _load(namespace)
