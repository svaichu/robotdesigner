"""Rigid poses: position (m) and quaternion ``(w, x, y, z)``."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

from .._canon import as_vector, check_keys
from ..errors import ConfigError


def _normalize_quat(q: tuple[float, ...], path: str) -> tuple[float, ...]:
    norm = math.sqrt(sum(c * c for c in q))
    if norm == 0.0:
        raise ConfigError(path, "must be non-zero")
    q = tuple(c / norm for c in q)
    # q and -q are the same rotation: make the first non-zero component positive.
    for c in q:
        if c != 0.0:
            if c < 0.0:
                q = tuple(-v for v in q)
            break
    return q


@dataclass(frozen=True, slots=True)
class Pose:
    """Position (m) and orientation as a quaternion ``(w, x, y, z)``."""

    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]

    def __post_init__(self):
        object.__setattr__(self, "pos", as_vector(self.pos, 3, "pos"))
        quat = as_vector(self.quat, 4, "quat")
        object.__setattr__(self, "quat", _normalize_quat(quat, "quat"))

    @classmethod
    def identity(cls) -> "Pose":
        return cls(pos=(0.0, 0.0, 0.0), quat=(1.0, 0.0, 0.0, 0.0))

    def to_dict(self) -> dict[str, Any]:
        return {"pos": list(self.pos), "quat": list(self.quat)}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "Pose":
        check_keys(d, {"pos", "quat"}, "")
        return cls(pos=d["pos"], quat=d["quat"])
