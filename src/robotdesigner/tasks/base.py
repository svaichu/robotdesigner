"""The engine side of a task, as the generic environment of a simulator backend sees it.

A provider (``config.providers.TaskProvider``) builds one ``TaskImpl`` per environment. The environment owns the
robot, the control law, batching, resets, episode bookkeeping and rendering; the task owns everything the world adds:
scene assets, per-env state, the ``task`` observation group, success and reward. Nothing here imports an engine at
module level, so provider config sides stay importable without one. The hooks below are written for the MuJoCo
backend (``spec`` is a ``mujoco.MjSpec``, ``sim`` a ``BatchedSim``).
"""

from __future__ import annotations

from typing import Any, Callable, Protocol, runtime_checkable

__all__ = ["BaseTask", "TaskImpl"]


@runtime_checkable
class TaskImpl(Protocol):
    def attach(self, spec: Any) -> None:
        """Add scene elements (arena, objects) to the model spec before it is compiled."""
        ...

    def bind(self, sim: Any) -> None:
        """Called once after compilation: resolve names to ids and allocate per-env state for ``sim.num_envs``."""
        ...

    def observation_spec(self, num_envs: int) -> Any:
        """The torchrl ``Composite`` of the ``task`` observation group, with batch shape ``(num_envs,)``."""
        ...

    def reset(self, sim: Any, mask: Any, rand: Callable[..., Any], injected: Callable[[str], Any]) -> None:
        """Reset the task state of the envs selected by ``mask`` (bool ``(N,)``), after the robot has been reset.

        ``rand(*shape, dtype=torch.float64)`` draws from a stream that belongs to the task: it is seeded by
        ``sim.seed`` and does not depend on the robot, so the same task instance can be replayed on different
        embodiments. Always draw for the whole batch so the stream does not depend on the mask. ``injected(key)`` returns a tensor the caller put in
        the reset TensorDict (for example a replayed ``target``), or ``None``.
        """
        ...

    def observe(self, sim: Any) -> dict[str, Any]:
        """The ``task`` observation group: float32 tensors with leading dimension ``N``."""
        ...

    def success(self, sim: Any, obs: dict[str, Any]) -> Any:
        """Bool tensor ``(N, 1)`` computed from the current state."""
        ...

    def reward(self, sim: Any, obs: dict[str, Any]) -> Any:
        """Float32 tensor ``(N, 1)`` for the step just taken."""
        ...

    def overlays(self, scene: Any, sim: Any, data: Any, env_index: int) -> None:
        """Draw task state that is not part of the model into an ``mjvScene`` (rendering only)."""
        ...


class BaseTask:
    """Defaults for the optional hooks, so a task states only what it needs."""

    def attach(self, spec: Any) -> None:
        pass

    def bind(self, sim: Any) -> None:
        pass

    def overlays(self, scene: Any, sim: Any, data: Any, env_index: int) -> None:
        pass

    def observation_spec(self, num_envs: int) -> Any:
        raise NotImplementedError

    def reset(self, sim, mask, rand, injected) -> None:
        raise NotImplementedError

    def observe(self, sim) -> dict[str, Any]:
        raise NotImplementedError

    def success(self, sim, obs):
        raise NotImplementedError

    def reward(self, sim, obs):
        raise NotImplementedError
