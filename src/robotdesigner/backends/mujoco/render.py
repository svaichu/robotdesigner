"""Optional rendering for debugging the CPU backend. Nothing else imports this module."""

from __future__ import annotations

import mujoco
import numpy as np


def _data(env, env_index: int) -> mujoco.MjData:
    sim = env._sim
    if not 0 <= env_index < sim.num_envs:
        raise IndexError(f"env_index {env_index} out of range for {sim.num_envs} envs")
    return sim.data[env_index]


def render_rgb(env, env_index: int = 0, camera=None, size: tuple[int, int] = (240, 320)) -> np.ndarray:
    """Render one env to a ``uint8`` array ``(H, W, 3)``. Needs an OpenGL context.

    ``camera`` is ``None`` (MuJoCo's free camera, framed on the model), a camera name or an id.
    On headless machines set ``MUJOCO_GL=egl`` before the first import of mujoco.
    """
    height, width = size
    data = _data(env, env_index)
    with mujoco.Renderer(env._sim.model, height=height, width=width) as renderer:
        if camera is None:
            camera = mujoco.MjvCamera()
            mujoco.mjv_defaultFreeCamera(env._sim.model, camera)
        renderer.update_scene(data, camera=camera)
        return renderer.render().copy()


def launch_viewer(env, env_index: int = 0) -> None:
    """Open a passive viewer on one env's data. Needs a display."""
    import mujoco.viewer

    data = _data(env, env_index)
    with mujoco.viewer.launch_passive(env._sim.model, data) as viewer:
        while viewer.is_running():
            viewer.sync()
