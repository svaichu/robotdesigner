"""Partial specs for the builder layer: ``NOT_SET`` and ``Chain``. numpy only.

A spec says only what the caller wrote. ``BaseEnvConfig`` merges specs into a complete description; the frozen configs
(``ParametricArm``, ``SimConfig``, ...) never see a missing value.
"""

from __future__ import annotations

from dataclasses import dataclass, fields, replace
from typing import Any

from .errors import ConfigError
from .generators import chain
from .robot import ParametricArm

__all__ = ["NOT_SET", "Chain"]


class _NotSet:
    """The marker of "argument not passed". ``None`` is a meaningful value in some configs, so it cannot be used."""

    _instance: "_NotSet | None" = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __repr__(self) -> str:
        return "NOT_SET"

    def __bool__(self) -> bool:
        return False

    def __copy__(self):
        return self

    def __deepcopy__(self, memo):
        return self

    def __reduce__(self):
        return (_NotSet, ())


NOT_SET: Any = _NotSet()

_PAIRS = (("link_length", "total_length"), ("mass", "total_mass"))
_REQUIRED = ("n", "axes", "radius", "damping", "armature", "frictionloss", "effort_limit")


@dataclass(frozen=True, slots=True)
class Chain:
    """The arguments of :func:`chain` (without a base pose: the mount belongs to the task), each optional.

    Used as a partial description, ``BaseEnvConfig().robot(Chain(n=5, axes="zyyyy"))`` changes what is written and
    keeps the rest. Writing one of ``link_length`` / ``total_length`` (or ``mass`` / ``total_mass``) clears the other.
    ``arm()`` returns the :class:`ParametricArm`, and needs every required value to be set.
    """

    n: Any = NOT_SET
    axes: Any = NOT_SET
    link_length: Any = NOT_SET
    total_length: Any = NOT_SET
    mass: Any = NOT_SET
    total_mass: Any = NOT_SET
    radius: Any = NOT_SET
    damping: Any = NOT_SET
    armature: Any = NOT_SET
    frictionloss: Any = NOT_SET
    effort_limit: Any = NOT_SET
    range: Any = NOT_SET
    home_qpos: Any = NOT_SET
    end_effector: Any = NOT_SET
    tool: Any = NOT_SET
    label: Any = NOT_SET

    def written(self) -> tuple[str, ...]:
        """The names of the fields that were given."""
        return tuple(f.name for f in fields(self) if getattr(self, f.name) is not NOT_SET)

    def merge(self, other: "Chain") -> "Chain":
        """``self`` with the fields written in ``other`` on top; the partner of an exclusive pair is cleared."""
        if not isinstance(other, Chain):
            raise ConfigError("robot", f"expected a Chain, got {type(other).__name__}")
        changes = {name: getattr(other, name) for name in other.written()}
        for a, b in _PAIRS:
            if a in changes and b not in changes:
                changes[b] = NOT_SET
            elif b in changes and a not in changes:
                changes[a] = NOT_SET
        return replace(self, **changes)

    def arm(self) -> ParametricArm:
        """Build the arm. Raises ``ConfigError`` (path: the argument of ``chain``) when it cannot be built."""
        for name in _REQUIRED:
            if getattr(self, name) is NOT_SET:
                raise ConfigError(name, "not set")

        def get(name, fallback=None):
            v = getattr(self, name)
            return fallback if v is NOT_SET else v

        return chain(
            self.n, self.axes,
            link_length=get("link_length"), total_length=get("total_length"),
            mass=get("mass"), total_mass=get("total_mass"),
            radius=self.radius, damping=self.damping, armature=self.armature,
            frictionloss=self.frictionloss, effort_limit=self.effort_limit,
            range=get("range"), home_qpos=get("home_qpos"), end_effector=get("end_effector"),
            tool=get("tool"), label=get("label", ""),
        )
