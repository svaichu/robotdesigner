"""Environment construction. Importing this module never imports mujoco, torch or torchrl."""

from __future__ import annotations

from typing import Any

from ..simulators import check_capabilities, get_simulator
from ..config import ConfigError, EnvConfig
from ..config.providers import get_task_provider


def make_env(cfg: EnvConfig) -> Any:
    """Build the environment of ``cfg``: the simulator named by ``cfg.sim.simulator`` running the task ``cfg.task.id``.

    ``cfg`` is complete data (robot, control, task, sim), so ``env = make_env(cfg)`` is the only step. Every
    incompatibility, of the config with the simulator and of the task with the simulator, is collected before raising,
    so one error lists all of them.
    """
    simulator = get_simulator(cfg.sim.simulator)
    provider = get_task_provider(cfg.task.params.provider)
    problems = check_capabilities(cfg, simulator.capabilities(), simulator.name)
    problems += simulator.validate(cfg)
    problems += provider.validate(cfg, simulator.name)
    if problems:
        lines = "\n".join(f"  - {p}" for p in problems)
        raise ConfigError(
            "",
            f"simulator {simulator.name!r} cannot run this config ({len(problems)} problem"
            f"{'s' if len(problems) != 1 else ''}):\n{lines}",
        )
    return simulator.make_env(cfg)


__all__ = ["make_env"]
