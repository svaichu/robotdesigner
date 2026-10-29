"""Rendering for the CPU MuJoCo backend. Nothing but ``MujocoEnv.render`` imports this module.

Needs an OpenGL context. On headless machines set ``MUJOCO_GL=egl`` before the first import of mujoco.

A frame shows everything in the MuJoCo model (floor, robot, any scene or object geometry) plus the overlays the task
draws for state that is not part of the model (``TaskImpl.overlays``, using ``add_geom`` and ``add_line`` below).
Rendering works on a scratch ``MjData`` that is synchronised from the live one, so it never changes the
simulation (no ``mj_forward`` on live data, no shared warm start).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...viz.camera import VIEWS, CameraSpec, auto_camera

_GL_HINT = (
    "could not create an offscreen OpenGL context ({err}). On headless machines set MUJOCO_GL=egl "
    "before the first import of mujoco."
)


def add_geom(scene: mujoco.MjvScene, gtype, size, pos, mat, rgba) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    mujoco.mjv_initGeom(
        scene.geoms[scene.ngeom],
        gtype,
        np.asarray(size, dtype=float),
        np.asarray(pos, dtype=float),
        np.asarray(mat, dtype=float).ravel(),
        np.asarray(rgba, dtype=np.float32),
    )
    scene.ngeom += 1


def add_line(scene: mujoco.MjvScene, a, b, rgba, width_px: float = 2.0) -> None:
    """A GL line from ``a`` to ``b``, ``width_px`` wide whatever the zoom. Lines cast no shadow, unlike solid geoms."""
    if scene.ngeom >= scene.maxgeom:
        return
    g = scene.geoms[scene.ngeom]
    gtype = mujoco.mjtGeom.mjGEOM_LINE
    mujoco.mjv_initGeom(
        g, gtype, np.array([width_px, 0.0, 0.0]), np.zeros(3), np.eye(3).ravel(), np.asarray(rgba, dtype=np.float32)
    )
    mujoco.mjv_connector(g, gtype, width_px, np.asarray(a, dtype=float), np.asarray(b, dtype=float))
    scene.ngeom += 1


class EnvRenderer:
    """One persistent offscreen renderer for a ``MujocoEnv``, recreated only when the frame size changes."""

    def __init__(self, env):
        self._env = env
        self._model = env._sim.model
        self._data = mujoco.MjData(self._model)  # scratch state, never stepped
        self._renderer: mujoco.Renderer | None = None
        self._size: tuple[int, int] | None = None
        self._cameras: dict[tuple[str, int, int], mujoco.MjvCamera] = {}

    # --- setup --------------------------------------------------------------------------------------

    def _ensure(self, height: int, width: int) -> mujoco.Renderer:
        if self._renderer is not None and self._size == (height, width):
            return self._renderer
        self.close()
        g = self._model.vis.global_  # the offscreen buffer is 640x480 unless the model says otherwise
        g.offwidth, g.offheight = max(int(g.offwidth), width), max(int(g.offheight), height)
        try:
            self._renderer = mujoco.Renderer(self._model, height, width)
        except Exception as err:
            raise RuntimeError(_GL_HINT.format(err=f"{type(err).__name__}: {err}")) from err
        self._size = (height, width)
        return self._renderer

    def _camera(self, camera, height: int, width: int):
        if camera is None:
            camera = "iso"
        if isinstance(camera, CameraSpec):
            return camera.to_mjv()
        if isinstance(camera, str) and camera in VIEWS:
            key = (camera, height, width)
            if key not in self._cameras:
                spec = auto_camera(
                    self._env.cfg, view=camera, fovy=float(self._model.vis.global_.fovy), aspect=width / height
                )
                self._cameras[key] = spec.to_mjv()
            return self._cameras[key]
        return camera  # a mujoco.MjvCamera, or the name / id of a camera in the model

    # --- state --------------------------------------------------------------------------------------

    def _sync(self, live: mujoco.MjData) -> None:
        d = self._data
        d.qpos[:] = live.qpos
        d.qvel[:] = live.qvel
        d.act[:] = live.act
        d.ctrl[:] = live.ctrl
        d.mocap_pos[:] = live.mocap_pos
        d.mocap_quat[:] = live.mocap_quat
        d.time = live.time
        mujoco.mj_forward(self._model, d)

    def _overlays(self, scene: mujoco.MjvScene, env_index: int) -> None:
        env = self._env
        env._task.overlays(scene, env._sim, self._data, env_index)

    # --- public -------------------------------------------------------------------------------------

    def render(self, env_index: int = 0, *, camera=None, size=(240, 320), overlays: bool = True) -> np.ndarray:
        n = self._env._sim.num_envs
        if not 0 <= env_index < n:
            raise IndexError(f"env_index {env_index} out of range for {n} envs")
        height, width = int(size[0]), int(size[1])
        if height < 1 or width < 1:
            raise ValueError(f"size must be positive (height, width), got {size}")
        renderer = self._ensure(height, width)
        self._sync(self._env._sim.data[env_index])
        renderer.update_scene(self._data, camera=self._camera(camera, height, width))
        if overlays:
            self._overlays(renderer.scene, env_index)
        return renderer.render().copy()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
        self._renderer, self._size = None, None


def get_renderer(env) -> EnvRenderer:
    renderer = getattr(env, "_renderer", None)
    if renderer is None:
        renderer = env._renderer = EnvRenderer(env)
    return renderer


def render_rgb(env, env_index: int = 0, camera=None, size: tuple[int, int] = (240, 320)) -> np.ndarray:
    """Render one env to a ``uint8`` array ``(H, W, 3)``; same as ``env.render(env_index, ...)``.

    ``camera`` is ``None`` (the ``"iso"`` view framed on the workspace), a view name, a ``CameraSpec``, or the
    name / id of a camera in the model.
    """
    return env.render(env_index, camera=camera, size=size)


def launch_viewer(env, env_index: int = 0) -> None:
    """Open a passive viewer on one env's data. Needs a display."""
    import mujoco.viewer

    sim = env._sim
    if not 0 <= env_index < sim.num_envs:
        raise IndexError(f"env_index {env_index} out of range for {sim.num_envs} envs")
    with mujoco.viewer.launch_passive(sim.model, sim.data[env_index]) as viewer:
        while viewer.is_running():
            viewer.sync()
