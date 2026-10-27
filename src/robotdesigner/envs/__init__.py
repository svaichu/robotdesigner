"""Environment construction. Importing this module never imports mujoco, torch or torchrl."""

from __future__ import annotations

from typing import Any

from ..backends import check_capabilities, get_backend
from ..config import ConfigError, EnvConfig


def make_env(cfg: EnvConfig) -> Any:
    """Build the environment for ``cfg`` on the backend named by ``cfg.sim.backend``.

    Every incompatibility is collected before raising, so one error lists all of them.
    """
    backend = get_backend(cfg.sim.backend)
    problems = check_capabilities(cfg, backend.capabilities(), backend.name)
    problems += backend.validate(cfg)
    if problems:
        lines = "\n".join(f"  - {p}" for p in problems)
        raise ConfigError(
            "",
            f"backend {backend.name!r} cannot run this config ({len(problems)} problem"
            f"{'s' if len(problems) != 1 else ''}):\n{lines}",
        )
    return backend.make_env(cfg)


__all__ = ["make_env"]
