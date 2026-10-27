"""Backend protocol and capability checks. No heavy imports."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable


@dataclass(frozen=True)
class Capabilities:
    control_modes: frozenset[str]
    task_ids: frozenset[str]  # exact ids, for example {"native/reach"}
    devices: frozenset[str]  # {"cpu"}; "cuda" matches any "cuda:N"
    robot_kinds: frozenset[str]  # {"parametric"}
    integrators: frozenset[str]


@dataclass(frozen=True)
class Problem:
    path: str  # e.g. "control.mode"
    message: str

    def __str__(self) -> str:
        return f"{self.path}: {self.message}"


@runtime_checkable
class Backend(Protocol):
    name: str

    def capabilities(self) -> Capabilities: ...

    def validate(self, cfg: Any) -> list[Problem]:
        """Engine-specific checks beyond the capabilities."""
        ...

    def make_env(self, cfg: Any) -> Any:
        """Return a torchrl ``EnvBase``."""
        ...


def _robot_kind(robot: Any) -> str:
    # Only the parametric arm exists so far; ExternalRobot will add "external".
    return "parametric" if type(robot).__name__ == "ParametricArm" else type(robot).__name__


def check_capabilities(cfg: Any, caps: Capabilities, backend: str = "") -> list[Problem]:
    """Every way ``cfg`` falls outside ``caps``, as problems naming the config path."""
    who = f"backend {backend!r}" if backend else "this backend"
    problems: list[Problem] = []

    def check(path: str, value: str, allowed: frozenset[str], what: str) -> None:
        if value not in allowed:
            problems.append(Problem(path, f"{who} does not support {what} {value!r}; supported: {sorted(allowed)}"))

    check("control.mode", cfg.control.mode, caps.control_modes, "control mode")
    check("task.id", cfg.task.id, caps.task_ids, "task")
    check("sim.integrator", cfg.sim.integrator, caps.integrators, "integrator")
    check("robot", _robot_kind(cfg.robot), caps.robot_kinds, "robot kind")

    device = cfg.sim.device
    family = "cuda" if device.startswith("cuda") else device
    if family not in caps.devices:
        problems.append(
            Problem("sim.device", f"{who} does not support device {device!r}; supported: {sorted(caps.devices)}")
        )
    return problems
