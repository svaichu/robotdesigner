"""Robot embodiments as frozen, validated, hashable data (numpy only; no simulator is imported)."""

from __future__ import annotations

from typing import Any, Mapping

from ..errors import ConfigError
from .arm import Capsule, JointSpec, LinkSpec, ParametricArm, Segment, ToolSpec
from .external import ACTUATOR_KINDS, EndEffectorRef, ExternalJoint, ExternalRobot
from .generators import chain
from .gripper import TwoFingerGripper
from .pose import Pose

__all__ = [
    "ACTUATOR_KINDS", "Capsule", "EndEffectorRef", "ExternalJoint", "ExternalRobot", "JointSpec", "LinkSpec",
    "ParametricArm", "Pose", "Segment", "ToolSpec", "TwoFingerGripper", "chain", "robot_from_dict",
]

_KINDS = {"parametric_arm": ParametricArm, "external_robot": ExternalRobot}


def robot_from_dict(d: Mapping[str, Any]) -> ParametricArm | ExternalRobot:
    """The robot ``d`` describes, dispatching on its ``kind`` (``parametric_arm`` or ``external_robot``)."""
    if not isinstance(d, Mapping):
        raise ConfigError("", f"expected a mapping, got {type(d).__name__}")
    if "kind" not in d:
        raise ConfigError("kind", f"missing; expected one of {sorted(_KINDS)}")
    if d["kind"] not in _KINDS:
        raise ConfigError("kind", f"unknown robot kind {d['kind']!r}; expected one of {sorted(_KINDS)}")
    return _KINDS[d["kind"]].from_dict(d)
