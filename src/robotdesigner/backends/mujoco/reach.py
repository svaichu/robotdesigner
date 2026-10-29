"""``native/reach`` as a TorchRL environment on the CPU MuJoCo backend."""

from __future__ import annotations

import numpy as np
import torch
from tensordict import TensorDict, TensorDictBase
from torchrl.data import Binary, Bounded, Composite, Unbounded
from torchrl.envs import EnvBase

from ...config import EnvConfig, FixedBox, OwnReachable
from ...config import kinematics as kin
from .compile import compile_robot
from .sim import BatchedSim


class MujocoReachEnv(EnvBase):
    """Move the end effector to a target. Never autoresets; honours ``td["_reset"]``."""

    batch_locked = True

    def __init__(self, cfg: EnvConfig):
        cfg = cfg.resolve()
        self.cfg = cfg
        self.robot_hash = cfg.robot.config_hash
        self.env_hash = cfg.env_hash
        N = cfg.sim.num_envs
        super().__init__(device=torch.device("cpu"), batch_size=torch.Size([N]))

        self._compiled = compile_robot(cfg)
        self._iface = self._compiled.interface
        self._sim = BatchedSim(self._compiled, N, cfg.sim.n_substeps)
        self._renderer = None  # created by the first render()
        n = self._iface.n
        self._n = n

        # Action [-1, 1] -> actuator control: ctrl = action * scale + offset.
        # Delta mode integrates the action into a target first (see _step); velocity mode scales it.
        self._mode = cfg.control.mode
        self._range = self._iface.joint_range
        self._q_target = np.zeros((N, n))
        if self._mode in ("joint_pos_delta", "joint_vel"):
            self._scale = np.full(n, cfg.control.action_scale)
            self._offset = np.zeros(n)
        elif self._mode == "joint_pos_abs":
            lo, hi = self._iface.joint_range[:, 0], self._iface.joint_range[:, 1]
            finite = np.isfinite(lo) & np.isfinite(hi)
            self._scale = np.where(finite, (hi - lo) / 2.0, np.pi)
            self._offset = np.where(finite, (hi + lo) / 2.0, 0.0)
        else:  # joint_torque
            self._scale = self._iface.effort_limit.copy()
            self._offset = np.zeros(n)

        task = cfg.task
        self._horizon = task.horizon
        self._tol = task.success_tolerance
        self._reset_noise = task.params.reset_noise
        self._terminate_on_success = task.params.terminate_on_success
        self._region = task.target_region

        self._gen = torch.Generator(device="cpu")
        self._gen.manual_seed(cfg.sim.seed)
        self._target = torch.zeros(N, 3)
        self._episode_step = torch.zeros(N, 1, dtype=torch.int64)

        f32 = torch.float32
        self.observation_spec = Composite(
            task=Composite(
                ee_pos=Unbounded(shape=(N, 3), dtype=f32),
                target_pos=Unbounded(shape=(N, 3), dtype=f32),
                shape=(N,),
            ),
            proprio=Composite(
                qpos=Unbounded(shape=(N, n), dtype=f32),
                qvel=Unbounded(shape=(N, n), dtype=f32),
                effort=Unbounded(shape=(N, n), dtype=f32),
                shape=(N,),
            ),
            success=Binary(shape=(N, 1), dtype=torch.bool),
            episode_step=Unbounded(shape=(N, 1), dtype=torch.int64),
            shape=(N,),
        )
        self.action_spec = Bounded(low=-1.0, high=1.0, shape=(N, n), dtype=f32)
        self.reward_spec = Unbounded(shape=(N, 1), dtype=f32)
        self.done_spec = Composite(
            done=Binary(shape=(N, 1), dtype=torch.bool),
            terminated=Binary(shape=(N, 1), dtype=torch.bool),
            truncated=Binary(shape=(N, 1), dtype=torch.bool),
            shape=(N,),
        )

    @property
    def q_target(self) -> np.ndarray:
        """Position targets, ``(N, n)``; meaningful in ``joint_pos_delta`` mode."""
        return self._q_target.copy()

    @property
    def interface(self):
        return self._iface

    # --- sampling -------------------------------------------------------------------

    def _rand(self, *shape, dtype=torch.float64) -> torch.Tensor:
        return torch.rand(*shape, generator=self._gen, dtype=dtype)

    def _sample_targets(self) -> torch.Tensor:
        N = self.batch_size[0]
        if isinstance(self._region, FixedBox):
            low = torch.tensor(self._region.low, dtype=torch.float64)
            high = torch.tensor(self._region.high, dtype=torch.float64)
            return (low + self._rand(N, 3) * (high - low)).to(torch.float32)
        assert isinstance(self._region, OwnReachable)
        rng = self._iface.joint_range
        lo = np.where(np.isfinite(rng[:, 0]), rng[:, 0], -np.pi)
        hi = np.where(np.isfinite(rng[:, 1]), rng[:, 1], np.pi)
        q = lo + self._rand(N, self._n).numpy() * (hi - lo)
        world = kin.forward_ee_batch(self.cfg.robot, q)
        T = kin.pose_to_matrix(self.cfg.robot.base_pose)
        return torch.from_numpy((world - T[:3, 3]) @ T[:3, :3]).to(torch.float32)

    # --- observations ---------------------------------------------------------------

    def _observe(self) -> TensorDict:
        s = self._sim
        ee = torch.from_numpy(s.ee_pos_base()).to(torch.float32)
        dist = torch.linalg.norm(ee - self._target, dim=-1, keepdim=True)
        return TensorDict(
            {
                "task": TensorDict(
                    {"ee_pos": ee, "target_pos": self._target.clone()}, batch_size=self.batch_size
                ),
                "proprio": TensorDict(
                    {
                        "qpos": torch.from_numpy(s.qpos).to(torch.float32),
                        "qvel": torch.from_numpy(s.qvel).to(torch.float32),
                        "effort": torch.from_numpy(s.actuator_force).to(torch.float32),
                    },
                    batch_size=self.batch_size,
                ),
                "success": dist < self._tol,
                "episode_step": self._episode_step.clone(),
            },
            batch_size=self.batch_size,
        )

    # --- EnvBase interface ----------------------------------------------------------

    def _reset(self, tensordict: TensorDictBase | None = None, **kwargs) -> TensorDictBase:
        N, n = self.batch_size[0], self._n
        mask = torch.ones(N, dtype=torch.bool)
        if tensordict is not None and tensordict.get("_reset", None) is not None:
            mask = tensordict.get("_reset").reshape(N).to(torch.bool)

        def injected(key):
            if tensordict is not None and tensordict.get(key, None) is not None:
                return tensordict.get(key)
            return None

        # Random draws always cover the whole batch so that rng use does not depend on the mask.
        noise = (self._rand(N, n) * 2.0 - 1.0) * self._reset_noise
        qpos = torch.from_numpy(self._iface.home_qpos).expand(N, n) + noise
        qpos = np.clip(qpos.numpy(), self._iface.joint_range[:, 0], self._iface.joint_range[:, 1])
        target = self._sample_targets()
        qpos_in, qvel_in, target_in = injected("qpos"), injected("qvel"), injected("target")
        if qpos_in is not None:
            qpos = qpos_in.to(torch.float64).numpy()
        qvel = None if qvel_in is None else qvel_in.to(torch.float64).numpy()
        if target_in is not None:
            target = target_in.to(torch.float32)

        self._sim.reset(mask=mask.numpy(), qpos=qpos, qvel=qvel)
        if self._mode == "joint_pos_delta":
            self._q_target[mask.numpy()] = self._sim.qpos[mask.numpy()]
        self._target[mask] = target[mask]
        self._episode_step[mask] = 0

        out = self._observe()
        for key in ("done", "terminated", "truncated"):
            prev = injected(key)
            flags = torch.zeros(N, 1, dtype=torch.bool)
            if prev is not None:
                flags = prev.to(torch.bool).clone()
                flags[mask] = False
            out.set(key, flags)
        return out

    def _step(self, tensordict: TensorDictBase) -> TensorDictBase:
        action = tensordict.get("action").to(torch.float64).clamp(-1.0, 1.0).numpy()
        if self._mode == "joint_pos_delta":
            self._q_target = np.clip(
                self._q_target + action * self._scale, self._range[:, 0], self._range[:, 1]
            )
            self._sim.set_ctrl(self._q_target)
        else:
            self._sim.set_ctrl(action * self._scale + self._offset)
        self._sim.step()
        self._episode_step += 1

        out = self._observe()
        ee, target = out["task", "ee_pos"], out["task", "target_pos"]
        dist = torch.linalg.norm(ee - target, dim=-1, keepdim=True)
        success = out["success"]
        truncated = self._episode_step >= self._horizon
        terminated = success.clone() if self._terminate_on_success else torch.zeros_like(success)
        out.set("reward", -dist)
        out.set("terminated", terminated)
        out.set("truncated", truncated)
        out.set("done", terminated | truncated)
        return out

    def _set_seed(self, seed: int | None):
        if seed is not None:
            self._gen.manual_seed(int(seed))
        return seed

    # --- rendering (needs an OpenGL context: MUJOCO_GL=egl on headless machines) ----------------------------

    def render(
        self,
        env_index: int = 0,
        *,
        camera=None,
        size: tuple[int, int] = (240, 320),
        overlays: bool = True,
    ) -> np.ndarray:
        """Render the live state of one env to ``uint8 (H, W, 3)`` (Gymnasium's ``rgb_array``).

        Shows the whole scene plus the target and the target region as overlays. ``camera`` is ``None`` (the
        ``"iso"`` view framed on the workspace), a view name (``iso``, ``front``, ``side``, ``top``), a
        ``robotdesigner.viz.CameraSpec``, a ``mujoco.MjvCamera``, or the name / id of a camera in the model.
        Rendering never changes the simulation.
        """
        from .render import get_renderer

        return get_renderer(self).render(env_index, camera=camera, size=size, overlays=overlays)

    def close(self, *, raise_if_closed: bool = True):
        renderer, self._renderer = getattr(self, "_renderer", None), None
        if renderer is not None:
            renderer.close()
        super().close(raise_if_closed=raise_if_closed)
