"""Configuration layer: numpy only. Imports no simulation engine and no torch."""

from . import kinematics
from .control import CONTROL_MODES, ControlConfig, FixedGains, InertiaScaledGains
from .env import EnvConfig
from .errors import ConfigError
from .generators import chain
from .sim import INTEGRATORS, MujocoOptions, SimConfig, register_backend_options
from .task import FixedBox, OwnReachable, ReachParams, TaskConfig
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
    "CONTROL_MODES",
    "ControlConfig",
    "EnvConfig",
    "FixedBox",
    "FixedGains",
    "INTEGRATORS",
    "InertiaScaledGains",
    "MujocoOptions",
    "OwnReachable",
    "ReachParams",
    "SimConfig",
    "TaskConfig",
    "register_backend_options",
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
