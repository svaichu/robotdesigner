"""Engine side of ``toy/reach``: move the end effector to a target point. Needs torch, torchrl and MuJoCo."""

from __future__ import annotations

import numpy as np
import torch
from torchrl.data import Composite, Unbounded

from robotdesigner.config import FixedBox, OwnReachable
from robotdesigner.config import kinematics as kin
from robotdesigner.tasks.base import BaseTask

_TARGET = (0.95, 0.20, 0.20, 0.95)
_TARGET_HIT = (0.20, 0.80, 0.30, 0.95)
_EDGE = (0.25, 0.55, 1.00, 0.90)


class ReachTask(BaseTask):
    """Targets are drawn at every reset from ``target_region`` in the robot's base frame.

    Reward is ``-||ee - target||``; success is ``||ee - target|| < success_tolerance``.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self._region = cfg.task.target_region
        self._tol = cfg.task.success_tolerance
        self._target: torch.Tensor | None = None

    def bind(self, sim) -> None:
        self._target = torch.zeros(sim.num_envs, 3)

    def observation_spec(self, num_envs: int):
        f32 = torch.float32
        return Composite(
            ee_pos=Unbounded(shape=(num_envs, 3), dtype=f32),
            target_pos=Unbounded(shape=(num_envs, 3), dtype=f32),
            shape=(num_envs,),
        )

    # --- sampling -------------------------------------------------------------------

    def _sample_targets(self, sim, rand) -> torch.Tensor:
        N = sim.num_envs
        if isinstance(self._region, FixedBox):
            low = torch.tensor(self._region.low, dtype=torch.float64)
            high = torch.tensor(self._region.high, dtype=torch.float64)
            return (low + rand(N, 3) * (high - low)).to(torch.float32)
        assert isinstance(self._region, OwnReachable)
        rng = sim.iface.joint_range
        lo = np.where(np.isfinite(rng[:, 0]), rng[:, 0], -np.pi)
        hi = np.where(np.isfinite(rng[:, 1]), rng[:, 1], np.pi)
        q = lo + rand(N, sim.iface.n).numpy() * (hi - lo)
        world = kin.forward_ee_batch(self.cfg.robot, q)
        T = kin.pose_to_matrix(self.cfg.robot.base_pose)
        return torch.from_numpy((world - T[:3, 3]) @ T[:3, :3]).to(torch.float32)

    def reset(self, sim, mask, rand, injected) -> None:
        # Random draws always cover the whole batch so that rng use does not depend on the mask.
        target = self._sample_targets(sim, rand)
        target_in = injected("target")
        if target_in is not None:
            target = target_in.to(torch.float32)
        m = torch.from_numpy(np.asarray(mask, dtype=bool))
        self._target[m] = target[m]

    # --- observations, success, reward ----------------------------------------------

    def observe(self, sim) -> dict[str, torch.Tensor]:
        ee = torch.from_numpy(sim.ee_pos_base()).to(torch.float32)
        return {"ee_pos": ee, "target_pos": self._target.clone()}

    @staticmethod
    def _dist(obs) -> torch.Tensor:
        return torch.linalg.norm(obs["ee_pos"] - obs["target_pos"], dim=-1, keepdim=True)

    def success(self, sim, obs) -> torch.Tensor:
        return self._dist(obs) < self._tol

    def reward(self, sim, obs) -> torch.Tensor:
        return -self._dist(obs)

    # --- rendering ------------------------------------------------------------------

    def overlays(self, scene, sim, data, env_index: int) -> None:
        import mujoco

        from robotdesigner.simulators.mujoco.render import add_geom, add_line

        R, p = sim._base_R, sim._base_pos
        tol = float(self._tol)
        target = self._target[env_index].numpy().astype(float)  # base frame
        ee_base = (data.site_xpos[sim.iface.ee_site_id] - p) @ R
        hit = float(np.linalg.norm(ee_base - target)) < tol
        color = _TARGET_HIT if hit else _TARGET
        center = p + R @ target
        add_geom(scene, mujoco.mjtGeom.mjGEOM_SPHERE, [max(tol, 0.01), 0.0, 0.0], center, np.eye(3), color)
        # The sphere (radius = tolerance) sits inside the arm tip once the target is reached, so add a cross-hair
        # that sticks out past the arm: the red -> green change stays visible.
        arm_radius = max(seg.link.geometry.radius for seg in self.cfg.robot.segments)
        reach = 2.5 * max(tol, arm_radius)
        for axis in np.eye(3):
            add_line(scene, center - reach * axis, center + reach * axis, color)
        region = self._region
        if isinstance(region, FixedBox):  # a wireframe: a filled box casts a solid shadow on the floor
            lo, hi = np.asarray(region.low, dtype=float), np.asarray(region.high, dtype=float)
            corners = np.array(
                [[(hi if k & 1 else lo)[0], (hi if k & 2 else lo)[1], (hi if k & 4 else lo)[2]] for k in range(8)]
            )
            world = p + corners @ R.T
            for k in range(8):
                for bit in (1, 2, 4):
                    if not k & bit:
                        add_line(scene, world[k], world[k | bit], _EDGE)
