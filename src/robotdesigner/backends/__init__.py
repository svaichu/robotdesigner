"""Backend registry. Importing this module never imports mujoco, torch or torchrl."""

from __future__ import annotations

import importlib
from typing import Any

from ..config import ConfigError
from .base import Backend, Capabilities, Problem, check_capabilities

__all__ = [
    "Backend",
    "Capabilities",
    "Problem",
    "check_capabilities",
    "get_backend",
    "list_backends",
    "register_backend",
]

# name -> backend object, or a lazy "module:attr" string resolved on first use
_REGISTRY: dict[str, Any] = {}


def register_backend(name: str, target: Any) -> None:
    """Register a backend by name. ``target`` is a backend instance or a lazy ``"module:attr"`` entry.

    The attribute may be a backend class (instantiated without arguments) or an instance.
    """
    _REGISTRY[name] = target


def list_backends() -> tuple[str, ...]:
    return tuple(sorted(_REGISTRY))


def get_backend(name: str) -> Backend:
    if name not in _REGISTRY:
        raise ConfigError("sim.backend", f"unknown backend {name!r}; registered: {list(list_backends())}")
    target = _REGISTRY[name]
    if isinstance(target, str):
        module, _, attr = target.partition(":")
        target = getattr(importlib.import_module(module), attr)
        if isinstance(target, type):
            target = target()
        _REGISTRY[name] = target
    return target


register_backend("mujoco", "robotdesigner.backends.mujoco:MujocoBackend")
