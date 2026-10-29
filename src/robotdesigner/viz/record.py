"""Record frames while a rollout runs (Gymnasium style: render the live state after every step).

The environment only has to provide ``render(env_index, *, camera, size, overlays) -> uint8 (H, W, 3)``.
"""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np

from .clip import Clip
from .frames import annotate, tile


def _indices(env, env_index) -> list[int]:
    n = int(env.batch_size[0])
    if isinstance(env_index, str):
        if env_index != "all":
            raise ValueError("env_index must be an int, a sequence of ints or 'all'")
        return list(range(n))
    idx = [int(env_index)] if isinstance(env_index, (int, np.integer)) else [int(i) for i in env_index]
    if not idx:
        raise ValueError("env_index is empty")
    for i in idx:
        if not 0 <= i < n:
            raise IndexError(f"env_index {i} out of range for {n} envs")
    return idx


def _control_freq(env) -> float | None:
    return getattr(getattr(getattr(env, "cfg", None), "sim", None), "control_freq", None)


class FrameRecorder:
    """Collects frames of one or several envs; usable as the ``callback`` of ``env.rollout``.

    TorchRL calls the callback after every step except the last (and not at all after an early stop), and never
    before the first step, so the first and last frames must be captured by hand: call ``capture()`` after the
    reset and ``finish(steps)`` after the rollout. ``record_rollout`` does all of this.
    """

    def __init__(
        self,
        env,
        env_index: int | Sequence[int] | str = 0,
        *,
        every: int = 1,
        camera: Any = None,
        size: tuple[int, int] = (240, 320),
        fps: float | None = None,
        label: bool | None = None,
        overlays: bool = True,
        ncols: int | None = None,
    ):
        if int(every) < 1:
            raise ValueError(f"every must be >= 1, got {every}")
        self.env = env
        self.indices = _indices(env, env_index)
        self.every = int(every)
        self.camera = camera
        self.size = (int(size[0]), int(size[1]))
        self.fps = fps
        self.label = len(self.indices) > 1 if label is None else bool(label)
        self.overlays = overlays
        self.ncols = ncols
        self.frames: list[np.ndarray] = []
        self.steps: list[int] = []  # env step of every stored frame
        self._step = 0

    def capture(self, step: int | None = None) -> None:
        """Store a frame of the live state (ignored if that step is already stored)."""
        step = self._step if step is None else int(step)
        if self.steps and self.steps[-1] == step:
            return
        views = []
        for i in self.indices:
            f = self.env.render(i, camera=self.camera, size=self.size, overlays=self.overlays)
            if self.label:
                f = annotate(f, f"env {i}  step {step}" if len(self.indices) > 1 else f"step {step}")
            views.append(f)
        self.frames.append(views[0] if len(views) == 1 else tile(views, ncols=self.ncols))
        self.steps.append(step)

    def __call__(self, *args, **kwargs) -> None:
        """The rollout callback. Its arguments are ignored: it reads the live state."""
        self._step += 1
        if self._step % self.every == 0:
            self.capture(self._step)

    def finish(self, steps: int) -> None:
        """Store the state after the last of ``steps`` steps, unless it is already stored."""
        self._step = int(steps)
        self.capture(self._step)

    def clip(self) -> Clip:
        """The frames as a ``Clip`` at ``control_freq / every`` fps unless ``fps`` was given."""
        if not self.frames:
            raise RuntimeError("no frames recorded")
        fps = self.fps
        if fps is None:
            freq = _control_freq(self.env)
            if freq is None:
                raise ValueError("the env does not expose cfg.sim.control_freq; pass fps=")
            fps = freq / self.every
        return Clip(np.stack(self.frames), fps)


def record_rollout(
    env,
    steps: int,
    policy=None,
    *,
    env_index: int | Sequence[int] | str = 0,
    every: int = 1,
    camera: Any = None,
    size: tuple[int, int] = (240, 320),
    fps: float | None = None,
    label: bool | None = None,
    overlays: bool = True,
    ncols: int | None = None,
    reset_td=None,
    **rollout_kwargs,
):
    """Reset, run ``env.rollout(steps, policy)`` and render the live state after every ``every``-th step.

    Returns ``(trajectory, clip)``. The clip holds the initial state, every ``every``-th state and the final
    state, so a full run of ``steps`` steps has ``steps + 1`` frames at ``every=1``. ``reset_td`` is passed to
    ``env.reset`` (for example to inject ``target``). Other keyword arguments go to ``env.rollout``.
    """
    for key in ("callback", "auto_reset", "tensordict"):
        if key in rollout_kwargs:
            raise TypeError(f"record_rollout sets {key!r} itself; do not pass it")
    rec = FrameRecorder(
        env, env_index, every=every, camera=camera, size=size, fps=fps, label=label, overlays=overlays, ncols=ncols
    )
    td0 = env.reset() if reset_td is None else env.reset(reset_td)
    rec.capture(0)
    traj = env.rollout(steps, policy, callback=rec, auto_reset=False, tensordict=td0, **rollout_kwargs)
    rec.finish(int(traj.shape[-1]))
    return traj, rec.clip()
