"""Configuration layer: numpy only. Imports no simulation engine and no torch."""

from . import kinematics
from .errors import ConfigError
from .generators import chain
from .robot import (
    Capsule,
    JointSpec,
    LinkSpec,
    ParametricArm,
    Pose,
    Segment,
    ToolSpec,
)

__all__ = [
    "Capsule",
    "ConfigError",
    "JointSpec",
    "LinkSpec",
    "ParametricArm",
    "Pose",
    "Segment",
    "ToolSpec",
    "chain",
    "kinematics",
]
