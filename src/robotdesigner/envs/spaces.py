"""``observation_space`` and ``action_space`` for any batched TorchRL environment of this package. Needs torch.

Plain shapes, derived from the env's own ``observation_spec`` / ``action_spec`` in one place, so every simulator's
environment gets them by inheriting :class:`SpacesMixin` (no per-simulator code). Gymnasium users look for these
two names, but they return the *shape*, not a ``gymnasium.Space``; dtype and bounds stay in the specs.
"""

from __future__ import annotations

from typing import Any

import torch

__all__ = ["SpacesMixin"]


def _shapes(spec: Any, skip: int) -> Any:
    """A nested dict of ``torch.Size`` mirroring a (nested) ``Composite``, without its first ``skip`` batch dims."""
    if hasattr(spec, "items") and hasattr(spec, "keys"):
        return {key: _shapes(sub, skip) for key, sub in spec.items()}
    return torch.Size(spec.shape[skip:])


class SpacesMixin:
    """Mix in before ``EnvBase``. ``self.batch_size`` is the number of envs; the spaces are per env."""

    @property
    def action_space(self) -> torch.Size:
        """The shape of one env's action, ``torch.Size([n])``: the batched action has the shape
        ``env.batch_size + env.action_space``. For the observation nested dict see :attr:`observation_space`."""
        return torch.Size(self.action_spec.shape[len(self.batch_size):])

    @property
    def observation_space(self) -> dict[str, Any]:
        """The shapes of one env's observation, as a nested dict with the same keys as ``observation_spec``
        (``{"task": {...}, "proprio": {"qpos": torch.Size([n]), ...}, "success": torch.Size([1]), ...}``). Nested
        observations are primary, so nothing is flattened."""
        return _shapes(self.observation_spec, len(self.batch_size))
