"""Configuration layer: numpy only. Imports no simulation engine and no torch."""

from . import kinematics
from .base_env import BaseEnvConfig
from .control import CONTROL_MODES, ControlConfig, FixedGains, InertiaScaledGains
from .env import EnvConfig
from .errors import ConfigError, Problem
from .generators import chain
from .providers import (
    TaskProvider,
    get_task_provider,
    list_task_providers,
    register_task_provider,
    unregister_task_provider,
)
from .specs import NOT_SET, Chain
from .sim import INTEGRATORS, MujocoOptions, SimConfig, register_simulator_options
from .task import FixedBox, OwnReachable, TaskConfig
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
    "BaseEnvConfig",
    "Chain",
    "NOT_SET",
    "CONTROL_MODES",
    "ControlConfig",
    "EnvConfig",
    "FixedBox",
    "FixedGains",
    "INTEGRATORS",
    "InertiaScaledGains",
    "MujocoOptions",
    "OwnReachable",
    "Problem",
    "SimConfig",
    "TaskConfig",
    "TaskProvider",
    "get_task_provider",
    "list_task_providers",
    "register_simulator_options",
    "register_task_provider",
    "unregister_task_provider",
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
