"""Parameters of ``robosuite/<EnvName>`` tasks. numpy only; robosuite is not imported here.

robosuite supplies the assets of the task: the arena, the objects and the rule that places them at reset. Success and
reward are defined on our side, by the ``objective``, so every task runs through the same simulation and control path.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any, ClassVar, Mapping

from ...config._canon import as_bool, as_float, as_str, canonicalize, check_keys, freeze, thaw, within
from ...config.errors import ConfigError

# Constructor arguments of a robosuite environment that robotdesigner sets itself (the environment is built only to
# read its assets, with a stock robot and no rendering) or that have no meaning for assets. They must not appear in
# ``RobosuiteParams.env_kwargs``.
RESERVED_ENV_KWARGS = frozenset(
    {
        "robots", "controller_configs", "gripper_types", "base_types", "mount_types", "initial_qpos",
        "initialization_noise", "control_freq", "horizon", "hard_reset", "ignore_done", "seed", "lite_physics",
        "has_renderer", "has_offscreen_renderer", "use_camera_obs", "camera_names", "camera_heights",
        "camera_widths", "camera_depths", "camera_segmentations", "renderer", "renderer_config", "render_camera",
        "render_collision_mesh", "render_visual_mesh", "render_gpu_device_id",
    }
)  # fmt: skip


@dataclass(frozen=True, slots=True)
class LiftObject:
    """Success when the centre of ``object`` is at least ``height`` metres above the world floor plane (z)."""

    object: str
    height: float

    def __post_init__(self):
        as_str(self.object, "object")
        object.__setattr__(self, "height", as_float(self.height, "height"))

    def to_dict(self) -> dict[str, Any]:
        return {"kind": "lift_object", "object": self.object, "height": self.height}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "LiftObject":
        check_keys(d, {"kind", "object", "height"}, "")
        return cls(object=d["object"], height=d["height"])


@dataclass(frozen=True, slots=True)
class ReachObject:
    """Success when the end effector is closer than ``tolerance`` metres to the centre of ``object``."""

    object: str
    tolerance: float

    def __post_init__(self):
        as_str(self.object, "object")
        tol = as_float(self.tolerance, "tolerance")
        if tol <= 0.0:
            raise ConfigError("tolerance", "must be > 0")
        object.__setattr__(self, "tolerance", tol)

    def to_dict(self) -> dict[str, Any]:
        return {"kind": "reach_object", "object": self.object, "tolerance": self.tolerance}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "ReachObject":
        check_keys(d, {"kind", "object", "tolerance"}, "")
        return cls(object=d["object"], tolerance=d["tolerance"])


Objective = LiftObject | ReachObject
_OBJECTIVES = {"lift_object": LiftObject, "reach_object": ReachObject}


def objective_from_dict(d: Mapping[str, Any]) -> Objective:
    if not isinstance(d, Mapping) or "kind" not in d:
        raise ConfigError("", "expected a mapping with a 'kind' key")
    cls = _OBJECTIVES.get(d["kind"])
    if cls is None:
        raise ConfigError("kind", f"unknown objective {d['kind']!r}; expected one of {sorted(_OBJECTIVES)}")
    return cls.from_dict(d)


@dataclass(frozen=True, slots=True)
class RobosuiteParams:
    """Parameters of the task ``robosuite/<EnvName>``; ``name`` is ``<EnvName>``.

    ``env_kwargs`` are passed to the robosuite environment when its assets are loaded (``table_full_size``,
    ``single_object_mode``, ...); everything not listed takes robosuite's own default. ``objective`` says what counts
    as success and what the reward measures. The robot, its gripper, the controller, rendering and seeding come from
    the other config sections and cannot be given here.
    """

    provider: ClassVar[str] = "robosuite"
    uses_goal: ClassVar[bool] = False  # the objective defines success; TaskConfig takes no target region

    name: str
    env_kwargs: Any
    objective: Any
    reset_noise: float
    terminate_on_success: bool

    def __post_init__(self):
        if not as_str(self.name, "name"):
            raise ConfigError("name", "must name a robosuite environment, for example 'Lift'")
        raw = self.env_kwargs
        if isinstance(raw, tuple) and raw and raw[0] == "__map__":
            raw = thaw(raw)
        if not isinstance(raw, Mapping):
            raise ConfigError("env_kwargs", f"expected a mapping, got {type(raw).__name__}")
        reserved = sorted(set(raw) & RESERVED_ENV_KWARGS)
        if reserved:
            raise ConfigError(
                "env_kwargs", f"{reserved} are set from the robot, control, sim and task config; remove them"
            )
        for key in raw:
            if not isinstance(key, str) or not key.isidentifier():
                raise ConfigError("env_kwargs", f"keys must be argument names, got {key!r}")
        object.__setattr__(self, "env_kwargs", freeze(dict(raw), "env_kwargs"))
        if not isinstance(self.objective, (LiftObject, ReachObject)):
            raise ConfigError("objective", "expected LiftObject or ReachObject")
        noise = as_float(self.reset_noise, "reset_noise")
        if noise < 0.0:
            raise ConfigError("reset_noise", "must be >= 0")
        object.__setattr__(self, "reset_noise", noise)
        as_bool(self.terminate_on_success, "terminate_on_success")

    @property
    def kwargs(self) -> dict[str, Any]:
        """The environment keyword arguments as a plain dict."""
        return thaw(self.env_kwargs)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "env_kwargs": canonicalize(self.kwargs),
            "objective": self.objective.to_dict(),
            "reset_noise": self.reset_noise,
            "terminate_on_success": self.terminate_on_success,
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "RobosuiteParams":
        check_keys(d, {f.name for f in fields(cls)}, "")
        return cls(
            name=d["name"],
            env_kwargs=d["env_kwargs"],
            objective=within("objective", objective_from_dict, d["objective"]),
            reset_noise=d["reset_noise"],
            terminate_on_success=d["terminate_on_success"],
        )
