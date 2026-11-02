"""The engine side of a task, as the generic environment of a simulator sees it.

A provider (``config.providers.TaskProvider``) builds one ``TaskImpl`` per environment. The environment owns the
robot, the control law, batching, resets, episode bookkeeping and rendering; the task owns everything the world adds:
scene assets, per-env state, the ``task`` observation group, success and reward. Nothing here imports an engine at
module level, so provider config sides stay importable without one. ``attach`` receives a ``mujoco.MjSpec``; every
other hook receives a ``SimView``, the engine-neutral, torch-valued view of the batched simulation, so a task does not
reach into the internals of one simulator.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Callable, Protocol, runtime_checkable

if TYPE_CHECKING:
    import numpy as np
    import torch

__all__ = ["BaseTask", "SimView", "TaskImpl"]


@runtime_checkable
class SimView(Protocol):
    """What a task engine side may use of the simulation. Implemented by the batched simulation of each simulator.

    Tensors returned are float32 on ``device``; a task keeps its own per-env state there too.
    """

    num_envs: int
    device: "torch.device"  # where returned tensors live
    iface: Any  # RobotInterface: joint ranges, ee site id, base pose, ...
    model: Any  # the host mujoco.MjModel; editable only inside ``TaskImpl.bind``

    def base_frame(self) -> "tuple[np.ndarray, np.ndarray]":
        """``(pos (3,), R (3, 3))`` of the robot's base in the world, float64, host. Constant data."""
        ...

    def body_pose(self, body_id: int, frame: str = "world") -> "tuple[torch.Tensor, torch.Tensor]":
        """Position ``(N, 3)`` and quaternion ``(N, 4)`` (w x y z) of a body. ``frame`` is ``"world"`` or ``"base"``;
        anything else raises ``ValueError``."""
        ...

    def site_pos(self, site_id: int, frame: str = "world") -> "torch.Tensor":
        """``(N, 3)``. The end effector is ``site_pos(sim.iface.ee_site_id, "base")``."""
        ...

    def write_qpos(self, adr: int, values: "torch.Tensor", mask: "torch.Tensor") -> None:
        """Set ``qpos[adr : adr + k]`` of the envs where ``mask`` (bool ``(N,)`` on ``device``) is true; ``values`` is
        ``(N, k)`` on any device. Derived quantities of those envs are valid before the next read."""
        ...

    def host_data(self, env_index: int) -> Any:
        """A host ``mujoco.MjData`` with the current state of one env, for rendering and overlays. Read-only."""
        ...


@runtime_checkable
class TaskImpl(Protocol):
    def attach(self, spec: Any) -> None:
        """Add scene elements (arena, objects) to the model spec before it is compiled."""
        ...

    def bind(self, sim: SimView) -> None:
        """Called once after compilation: resolve names to ids and allocate per-env state for ``sim.num_envs``.

        This is the last point at which the host model (``sim.model``) may be edited (fixture poses, for example): a
        simulator that copies the model to a device does so after ``bind``.
        """
        ...

    def observation_spec(self, num_envs: int) -> Any:
        """The torchrl ``Composite`` of the ``task`` observation group, with batch shape ``(num_envs,)``."""
        ...

    def reset(self, sim: SimView, mask: Any, rand: Callable[..., Any], injected: Callable[[str], Any]) -> None:
        """Reset the task state of the envs selected by ``mask`` (a bool tensor ``(N,)`` on
        ``sim.device``), after the robot has been reset.

        ``rand(*shape, dtype=torch.float64)`` draws from a stream that belongs to the task: it is seeded by
        ``sim.seed`` and does not depend on the robot, so the same task instance can be replayed on different
        embodiments. Always draw for the whole batch so the stream does not depend on the mask. ``injected(key)`` returns a tensor the caller put in
        the reset TensorDict (for example a replayed ``target``), or ``None``.
        """
        ...

    def observe(self, sim: SimView) -> dict[str, Any]:
        """The ``task`` observation group: float32 tensors on ``sim.device`` with leading dimension ``N``."""
        ...

    def success(self, sim: SimView, obs: dict[str, Any]) -> Any:
        """Bool tensor ``(N, 1)`` computed from the current state."""
        ...

    def reward(self, sim: SimView, obs: dict[str, Any]) -> Any:
        """Float32 tensor ``(N, 1)`` for the step just taken."""
        ...

    def overlays(self, scene: Any, sim: SimView, data: Any, env_index: int) -> None:
        """Draw task state that is not part of the model into an ``mjvScene`` (rendering only).

        ``data`` is ``sim.host_data(env_index)``; use ``sim.base_frame()`` for the base pose.
        """
        ...


class BaseTask:
    """Defaults for the optional hooks, so a task states only what it needs."""

    def attach(self, spec: Any) -> None:
        pass

    def bind(self, sim: SimView) -> None:
        pass

    def overlays(self, scene: Any, sim: SimView, data: Any, env_index: int) -> None:
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
