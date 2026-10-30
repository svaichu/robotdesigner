"""``BaseEnvConfig.sweep``: a family of environment configs, one per combination of overrides. numpy only.

A plain expansion: launching the jobs (Slurm or other) is left to the project that uses it. Give each job the resolved dict
(``config.resolve().to_dict()``) and rebuild with ``EnvConfig.from_dict`` in the worker: the task and robot registries are
process-global, so a worker must not rely on state of the launching process.
"""

from __future__ import annotations

import itertools
import warnings
from typing import TYPE_CHECKING, Any, Iterator, Mapping

from .env import EnvConfig
from .errors import ConfigError

if TYPE_CHECKING:
    from .base_env import BaseEnvConfig

__all__ = ["Family"]


def _axes(kind: str, spec: Mapping[str, Any] | None) -> dict[str, list]:
    out: dict[str, list] = {}
    for key, values in (spec or {}).items():
        if isinstance(values, (str, bytes)) or not hasattr(values, "__iter__"):
            raise ConfigError(f"sweep.{kind}.{key}", f"expected a sequence of values, got {type(values).__name__}")
        values = list(values)
        if not values:
            raise ConfigError(f"sweep.{kind}.{key}", "has no values")
        out[key] = values
    return out


class Family:
    """The configs of a sweep, in order: ``len``, iteration and indexing give ``BaseEnvConfig`` items. ``notes`` lists
    the items dropped because their resolved ``env_hash`` repeated."""

    def __init__(self, items: list[tuple[dict[str, Any], "BaseEnvConfig", EnvConfig]], notes: list[str]):
        self._items = items
        self.notes = notes

    def __len__(self) -> int:
        return len(self._items)

    def __iter__(self) -> Iterator["BaseEnvConfig"]:
        return (cfg for _, cfg, _ in self._items)

    def __getitem__(self, i: int) -> "BaseEnvConfig":
        return self._items[i][1]

    def configs(self) -> list[EnvConfig]:
        """The resolved ``EnvConfig`` of every item."""
        return [env for _, _, env in self._items]

    def overrides(self) -> list[dict[str, Any]]:
        """The overrides that produced every item."""
        return [dict(o) for o, _, _ in self._items]

    def table(self) -> list[dict[str, Any]]:
        """One row per item: ``index``, the override values by dotted path, and ``env_hash``."""
        return [{"index": i, **o, "env_hash": env.env_hash} for i, (o, _, env) in enumerate(self._items)]

    def __repr__(self) -> str:
        return f"Family({len(self)} configs)"


def expand(base: "BaseEnvConfig", paired: Mapping[str, Any] | None, grid: Mapping[str, Any] | None) -> Family:
    paired_axes, grid_axes = _axes("paired", paired), _axes("grid", grid)
    both = sorted(set(paired_axes) & set(grid_axes))
    if both:
        raise ConfigError(f"sweep.{both[0]}", "is in both paired and grid")
    lengths = {k: len(v) for k, v in paired_axes.items()}
    if len(set(lengths.values())) > 1:
        raise ConfigError("sweep.paired", f"axes must have equal lengths, got {lengths}")
    groups = ([dict(zip(paired_axes, vals)) for vals in zip(*paired_axes.values())] if paired_axes else [{}])
    combos = [dict(zip(grid_axes, vals)) for vals in itertools.product(*grid_axes.values())]
    items: list[tuple[dict[str, Any], "BaseEnvConfig", EnvConfig]] = []
    seen: dict[str, int] = {}
    notes: list[str] = []
    index = -1
    for group in groups:
        for combo in combos:
            index += 1
            overrides = {**group, **combo}
            try:
                cfg = base.with_overrides(overrides)
                env = cfg.resolve()
            except ConfigError as e:
                err = ConfigError(e.path, f"{e.reason} (sweep item {index}: {overrides})")
                err.problems = getattr(e, "problems", ())
                raise err from None
            if env.env_hash in seen:
                notes.append(f"item {index} {overrides} repeats item {seen[env.env_hash]} (env_hash {env.env_hash}); dropped")
                continue
            seen[env.env_hash] = index
            items.append((overrides, cfg, env))
    if notes:
        warnings.warn("; ".join(notes), stacklevel=3)
    return Family(items, notes)
