"""Configuration layer: numpy only. Imports no simulation engine and no torch."""

from .errors import ConfigError
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
]
