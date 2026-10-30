"""Batched simulation core: N independent ``MjData`` sharing one ``MjModel``.

Float64 throughout; conversion to float32 tensors happens in the env layer. The core owns no random
number generator: all randomness (noise, targets) lives in the env layer.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...config.kinematics import pose_to_matrix
from .compile import CompiledRobot


class BatchedSim:
    def __init__(self, compiled: CompiledRobot, num_envs: int, n_substeps: int):
        if num_envs < 1:
            raise ValueError("num_envs must be >= 1")
        if n_substeps < 1:
            raise ValueError("n_substeps must be >= 1")
        self.model = compiled.model
        self.iface = compiled.interface
        self.num_envs = num_envs
        self.n_substeps = n_substeps
        self.data: list[mujoco.MjData] = [mujoco.MjData(self.model) for _ in range(num_envs)]
        T = pose_to_matrix(self.iface.mount)
        self._base_R = T[:3, :3]
        self._base_pos = T[:3, 3]
        self.reset()

    @property
    def control_dt(self) -> float:
        return self.n_substeps * self.model.opt.timestep

    def reset(self, mask=None, qpos=None, qvel=None) -> None:
        """Reset the masked envs (all when ``mask`` is None) to ``qpos`` / ``qvel`` or home / rest.

        ``qpos`` and ``qvel`` are ``(N, n)``; only rows selected by the mask are read. Control, time and
        warm-start state are cleared for the masked envs; the others are not touched.
        """
        N, n, iface = self.num_envs, self.iface.n, self.iface
        mask = np.ones(N, dtype=bool) if mask is None else np.asarray(mask, dtype=bool)
        if mask.shape != (N,):
            raise ValueError(f"mask must have shape ({N},), got {mask.shape}")
        for name, arr in (("qpos", qpos), ("qvel", qvel)):
            if arr is not None and np.shape(arr) != (N, n):
                raise ValueError(f"{name} must have shape ({N}, {n}), got {np.shape(arr)}")
        for i in np.flatnonzero(mask):
            d = self.data[i]
            mujoco.mj_resetData(self.model, d)
            d.qpos[iface.qpos_adr] = iface.home_qpos if qpos is None else qpos[i]
            if qvel is not None:
                d.qvel[iface.qvel_adr] = qvel[i]
            mujoco.mj_forward(self.model, d)

    def set_ctrl(self, ctrl) -> None:
        ctrl = np.asarray(ctrl, dtype=np.float64)
        if ctrl.shape != (self.num_envs, self.model.nu):
            raise ValueError(f"ctrl must have shape ({self.num_envs}, {self.model.nu}), got {ctrl.shape}")
        for d, c in zip(self.data, ctrl):
            d.ctrl[:] = c

    def step(self) -> None:
        """Advance ``n_substeps`` physics steps in every env with the current control."""
        for d in self.data:
            mujoco.mj_step(self.model, d, nstep=self.n_substeps)

    @property
    def qpos(self) -> np.ndarray:
        return np.stack([d.qpos[self.iface.qpos_adr] for d in self.data])

    @property
    def qvel(self) -> np.ndarray:
        return np.stack([d.qvel[self.iface.qvel_adr] for d in self.data])

    @property
    def actuator_force(self) -> np.ndarray:
        return np.stack([d.actuator_force[self.iface.actuator_ids] for d in self.data])

    @property
    def time(self) -> np.ndarray:
        return np.array([d.time for d in self.data])

    def ee_pos_base(self) -> np.ndarray:
        """End-effector positions in the base frame, ``(N, 3)``."""
        world = np.stack([d.site_xpos[self.iface.ee_site_id] for d in self.data])
        return (world - self._base_pos) @ self._base_R
