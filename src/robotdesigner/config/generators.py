"""Generators that build ``ParametricArm`` objects. ``n`` is an argument here, not a config field."""

from __future__ import annotations

import numbers
from typing import Any, Sequence

from ._canon import as_float
from .errors import ConfigError
from .robot import Capsule, JointSpec, LinkSpec, ParametricArm, Pose, Segment, ToolSpec

_AXIS_LETTERS = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}


def _is_number(x: Any) -> bool:
    return isinstance(x, numbers.Real) and not isinstance(x, bool)


def _per_link(value: Any, n: int, name: str) -> list[float]:
    """A scalar is repeated ``n`` times; a sequence must have length ``n``."""
    if _is_number(value):
        return [as_float(value, name)] * n
    try:
        items = list(value)
    except TypeError:
        raise ConfigError(name, "expected a number or a sequence of numbers") from None
    if len(items) != n:
        raise ConfigError(name, f"expected {n} values, got {len(items)}")
    return [as_float(v, f"{name}[{i}]") for i, v in enumerate(items)]


def _split(single: Any, single_name: str, total: Any, total_name: str, n: int) -> list[float]:
    """Resolve an exactly-one-of (per-link, total) pair into per-link values."""
    if (single is None) == (total is None):
        raise ConfigError(f"{single_name}/{total_name}", f"give exactly one of {single_name} and {total_name}")
    if single is not None:
        return _per_link(single, n, single_name)
    return [as_float(total, total_name) / n] * n


def _axes(axes: Any, n: int) -> list[tuple[float, ...]]:
    items = list(axes)
    if len(items) != n:
        raise ConfigError("axes", f"expected {n} axes, got {len(items)}")
    out = []
    for i, a in enumerate(items):
        if isinstance(a, str):
            if a not in _AXIS_LETTERS:
                raise ConfigError(f"axes[{i}]", f"unknown axis letter {a!r}; use x, y or z")
            out.append(_AXIS_LETTERS[a])
        else:
            out.append(tuple(a))
    return out


def _ranges(range_: Any, n: int) -> list[tuple[float, float] | None]:
    if range_ is None:
        return [None] * n
    items = list(range_)
    if len(items) == 2 and all(_is_number(v) for v in items):
        return [tuple(items)] * n
    if len(items) != n:
        raise ConfigError("range", f"expected None, (lo, hi) or {n} entries, got {len(items)}")
    return [None if r is None else tuple(r) for r in items]


def chain(
    n: int,
    axes: str | Sequence[Any],
    *,
    link_length: float | Sequence[float] | None = None,
    total_length: float | None = None,
    mass: float | Sequence[float] | None = None,
    total_mass: float | None = None,
    radius: float | Sequence[float],
    damping: float | Sequence[float],
    armature: float | Sequence[float],
    frictionloss: float | Sequence[float],
    effort_limit: float | Sequence[float],
    range: Any = None,
    home_qpos: Sequence[float] | None = None,
    end_effector: Pose | None = None,
    tool: ToolSpec | None = None,
    label: str = "",
) -> ParametricArm:
    """An ``n``-joint serial arm, straight along +x of the base frame at ``q = 0``.

    ``axes`` is a string of x/y/z letters or a sequence of vectors, one per joint (no cycling).
    Exactly one of ``link_length`` / ``total_length`` and one of ``mass`` / ``total_mass`` is given;
    totals are split uniformly. The remaining physical parameters are scalars or per-joint sequences.
    """
    if isinstance(n, bool) or not isinstance(n, numbers.Integral) or n < 1:
        raise ConfigError("n", "must be an integer >= 1")
    n = int(n)
    axis_list = _axes(axes, n)
    lengths = _split(link_length, "link_length", total_length, "total_length", n)
    masses = _split(mass, "mass", total_mass, "total_mass", n)
    for i, L in enumerate(lengths):
        if L <= 0.0:
            raise ConfigError(f"link_length[{i}]", "must be > 0")
    radii = _per_link(radius, n, "radius")
    dampings = _per_link(damping, n, "damping")
    armatures = _per_link(armature, n, "armature")
    frictions = _per_link(frictionloss, n, "frictionloss")
    efforts = _per_link(effort_limit, n, "effort_limit")
    ranges = _ranges(range, n)

    segments = tuple(
        Segment(
            joint=JointSpec(
                axis=axis_list[i],
                range=ranges[i],
                damping=dampings[i],
                armature=armatures[i],
                frictionloss=frictions[i],
                effort_limit=efforts[i],
            ),
            link=LinkSpec(
                joint_frame=Pose.identity(),
                tip_offset=(lengths[i], 0.0, 0.0),
                mass=masses[i],
                geometry=Capsule(radius=radii[i]),
                inertia=None,
            ),
        )
        for i, _ in enumerate(axis_list)
    )
    return ParametricArm(
        schema_version=1,
        segments=segments,
        home_qpos=(0.0,) * n if home_qpos is None else tuple(home_qpos),
        end_effector=Pose.identity() if end_effector is None else end_effector,
        tool=tool,
        label=label,
    )
