"""Environment construction. Importing this module never imports mujoco, torch or torchrl."""

from __future__ import annotations

from typing import Any

from ..backends import check_capabilities, get_backend
from ..config import ConfigError, EnvConfig
from ..config.providers import provider_for


def make_env(cfg: EnvConfig) -> Any:
    """Build the environment of ``cfg``: the simulator named by ``cfg.sim.backend`` running the task ``cfg.task.id``.

    ``cfg`` is complete data (robot, control, task, sim), so ``env = make_env(cfg)`` is the only step. Every
    incompatibility, of the config with the backend and of the task with the backend, is collected before raising,
    so one error lists all of them.
    """
    backend = get_backend(cfg.sim.backend)
    provider, _ = provider_for(cfg.task.id)
    problems = check_capabilities(cfg, backend.capabilities(), backend.name)
    problems += backend.validate(cfg)
    problems += provider.validate(cfg, backend.name)
    if problems:
        lines = "\n".join(f"  - {p}" for p in problems)
        raise ConfigError(
            "",
            f"backend {backend.name!r} cannot run this config ({len(problems)} problem"
            f"{'s' if len(problems) != 1 else ''}):\n{lines}",
        )
    return backend.make_env(cfg)


__all__ = ["make_env"]
