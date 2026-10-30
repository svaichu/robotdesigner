"""Engine side of ``empty/idle``: no objects, no observations, reward 0, never solved. Needs torch and torchrl."""

from __future__ import annotations

import torch
from torchrl.data import Composite

from ..base import BaseTask


class IdleTask(BaseTask):
    """Adds nothing to the scene and draws nothing from the task's random stream."""

    def observation_spec(self, num_envs: int):
        return Composite(shape=(num_envs,))

    def reset(self, sim, mask, rand, injected) -> None:
        pass

    def observe(self, sim) -> dict[str, torch.Tensor]:
        return {}

    def success(self, sim, obs) -> torch.Tensor:
        return torch.zeros(sim.num_envs, 1, dtype=torch.bool)

    def reward(self, sim, obs) -> torch.Tensor:
        return torch.zeros(sim.num_envs, 1, dtype=torch.float32)
