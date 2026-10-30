"""Simulator registry. Importing this module never imports mujoco, torch or torchrl."""

from __future__ import annotations

import importlib
from typing import Any

from ..config import ConfigError
from .base import Simulator, Capabilities, Problem, check_capabilities

__all__ = [
    "Simulator",
    "Capabilities",
    "Problem",
    "check_capabilities",
    "get_simulator",
    "list_simulators",
    "register_simulator",
]

# name -> simulator object, or a lazy "module:attr" string resolved on first use
_REGISTRY: dict[str, Any] = {}


def register_simulator(name: str, target: Any) -> None:
    """Register a simulator by name. ``target`` is a simulator instance or a lazy ``"module:attr"`` entry.

    The attribute may be a simulator class (instantiated without arguments) or an instance.
    """
    _REGISTRY[name] = target


def list_simulators() -> tuple[str, ...]:
    return tuple(sorted(_REGISTRY))


def get_simulator(name: str) -> Simulator:
    if name not in _REGISTRY:
        raise ConfigError("sim.simulator", f"unknown simulator {name!r}; registered: {list(list_simulators())}")
    target = _REGISTRY[name]
    if isinstance(target, str):
        module, _, attr = target.partition(":")
        target = getattr(importlib.import_module(module), attr)
        if isinstance(target, type):
            target = target()
        _REGISTRY[name] = target
    return target


register_simulator("mujoco", "robotdesigner.simulators.mujoco:MujocoSimulator")
register_simulator("warp", "robotdesigner.simulators.warp:WarpSimulator")
