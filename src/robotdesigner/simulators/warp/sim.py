"""Batched simulation core on MuJoCo Warp: ``nworld = num_envs`` worlds in one device allocation.

The same compiled ``MjModel`` and ``RobotInterface`` as the CPU simulator feed it; nothing about the robot is rebuilt.
State is float32 torch tensors that alias the Warp arrays. Imports warp, mujoco_warp and torch.

Streams: Warp work runs on torch's current CUDA stream (``wp.stream_from_torch``), so torch reads right after
``step()`` need no extra synchronisation. Determinism: the same inputs give the same outputs on one device and library
release; the contact solver is not guaranteed to be bitwise deterministic, so comparisons across runs use tolerances.
"""

from __future__ import annotations

import contextlib

import mujoco
import mujoco_warp as mjw
import numpy as np
import torch
import warp as wp
from mujoco_warp._src import types as wtypes

from ...config.errors import Problem
from ...config.kinematics import pose_to_matrix
from ..mujoco.compile import CompiledRobot
from ...config.sim import WarpOptions

wp.config.quiet = True
wp.init()

__all__ = ["CapacityError", "WarpBatchedSim", "check_model_supported"]


class CapacityError(RuntimeError):
    """A contact or constraint capacity of ``WarpOptions`` was exceeded."""


def _quat_mul(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    aw, ax, ay, az = a.unbind(-1)
    bw, bx, by, bz = b.unbind(-1)
    return torch.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dim=-1,
    )


def check_model_supported(model: mujoco.MjModel) -> list[Problem]:
    """Every feature of the compiled model that the installed MuJoCo Warp does not run (what ``put_model`` refuses)."""
    problems: list[Problem] = []

    def check(path: str, what: str, values, allowed, enum) -> None:
        for v in sorted(set(np.asarray(values).ravel().tolist()) - {int(a) for a in allowed}):
            problems.append(Problem(path, f"{what} {enum(v).name} is not supported by the warp simulator"))

    check("robot", "actuator transmission", model.actuator_trntype, wtypes.TrnType, mujoco.mjtTrn)
    check("robot", "actuator dynamics", model.actuator_dyntype, wtypes.DynType, mujoco.mjtDyn)
    check("robot", "actuator gain", model.actuator_gaintype, wtypes.GainType, mujoco.mjtGain)
    check("robot", "actuator bias", model.actuator_biastype, wtypes.BiasType, mujoco.mjtBias)
    check("task", "equality constraint", model.eq_type, wtypes.EqType, mujoco.mjtEq)
    check("task", "geom type", model.geom_type, wtypes.GeomType, mujoco.mjtGeom)
    check("task", "sensor", model.sensor_type, wtypes.SensorType, mujoco.mjtSensor)
    check("task", "tendon wrap", model.wrap_type, wtypes.WrapType, mujoco.mjtWrap)
    check("sim.integrator", "integrator", [model.opt.integrator], wtypes.IntegratorType, mujoco.mjtIntegrator)
    check("task", "cone", [model.opt.cone], wtypes.ConeType, mujoco.mjtCone)
    check("task", "solver", [model.opt.solver], wtypes.SolverType, mujoco.mjtSolver)
    for flags, enum, mj_enum, label in (
        (model.opt.disableflags, wtypes.DisableBit, mujoco.mjtDisableBit, "disable flag"),
        (model.opt.enableflags, wtypes.EnableBit, mujoco.mjtEnableBit, "enable flag"),
    ):
        supported = 0
        for b in enum:
            supported |= int(b)
        for bit in mj_enum.__members__.values():
            if not bit.name.endswith("NUM") and int(flags) & int(bit) and not supported & int(bit):
                problems.append(Problem("task", f"{label} {bit.name} is not supported by the warp simulator"))
    if model.opt.noslip_iterations > 0 and model.opt.integrator == mujoco.mjtIntegrator.mjINT_DISCRETE:
        problems.append(Problem("sim.integrator", "the noslip solver is not supported with the discrete integrator"))
    if (model.body_plugin != -1).any():
        problems.append(Problem("task", "body plugins are not supported by the warp simulator"))
    return problems


class WarpBatchedSim:
    """Implements ``tasks.base.SimView`` on MuJoCo Warp. Nothing is allocated on the device until ``finalize()``:
    tasks may still edit ``self.model`` in ``TaskImpl.bind``."""

    def __init__(self, compiled: CompiledRobot, num_envs: int, n_substeps: int, device: str, options: WarpOptions):
        if num_envs < 1:
            raise ValueError("num_envs must be >= 1")
        if n_substeps < 1:
            raise ValueError("n_substeps must be >= 1")
        self.model = compiled.model
        self.iface = compiled.interface
        self.num_envs = num_envs
        self.n_substeps = n_substeps
        self.device = torch.device(device)
        self.options = options
        T = pose_to_matrix(self.iface.mount)
        self._base_pos64, self._base_R64 = T[:3, 3], T[:3, :3]
        q = np.asarray(self.iface.mount.quat, dtype=float)
        self._base_quat_inv64 = np.array([q[0], -q[1], -q[2], -q[3]]) / float(q @ q)
        self._mw = self._d = self._graph = self._host = None

    # --- setup ---------------------------------------------------------------------------

    @property
    def control_dt(self) -> float:
        return self.n_substeps * self.model.opt.timestep

    @contextlib.contextmanager
    def _scope(self):
        with wp.ScopedDevice(str(self.device)):
            if self.device.type == "cuda":
                with wp.ScopedStream(wp.stream_from_torch(torch.cuda.current_stream(self.device))):
                    yield
            else:
                yield

    def finalize(self) -> None:
        """Upload the model (edits to ``self.model`` after this are not seen), allocate the worlds, capture the CUDA
        graph if asked, and reset every world to the home pose."""
        problems = check_model_supported(self.model)
        if problems:
            raise NotImplementedError("; ".join(p.message for p in problems))
        dev, iface = self.device, self.iface
        f32 = dict(dtype=torch.float32, device=dev)
        with self._scope():
            self._mw = mjw.put_model(self.model)
            self._d = mjw.make_data(self.model, nworld=self.num_envs, njmax=self.options.njmax, nconmax=self.options.nconmax)
        self._qpos_idx = torch.as_tensor(iface.qpos_adr, dtype=torch.long, device=dev)
        self._qvel_idx = torch.as_tensor(iface.qvel_adr, dtype=torch.long, device=dev)
        self._act_idx = torch.as_tensor(iface.actuator_ids, dtype=torch.long, device=dev)
        self._home = torch.as_tensor(iface.home_qpos, **f32)
        self._bp = torch.as_tensor(self._base_pos64, **f32)
        self._bR = torch.as_tensor(self._base_R64, **f32)
        self._bqi = torch.as_tensor(self._base_quat_inv64, **f32)
        if self.options.cuda_graph and dev.type == "cuda":
            with self._scope():
                with wp.ScopedCapture() as capture:
                    for _ in range(self.n_substeps):
                        mjw.step(self._mw, self._d)
            self._graph = capture.graph
        self.reset()

    def _require(self):
        if self._d is None:
            raise RuntimeError("call finalize() before using the simulation")

    # --- reset, control, step -------------------------------------------------------------

    def _mask(self, mask) -> torch.Tensor:
        if mask is None:
            return torch.ones(self.num_envs, dtype=torch.bool, device=self.device)
        mask = torch.as_tensor(mask, dtype=torch.bool).to(self.device)
        if mask.shape != (self.num_envs,):
            raise ValueError(f"mask must have shape ({self.num_envs},), got {tuple(mask.shape)}")
        return mask

    def reset(self, mask=None, qpos=None, qvel=None) -> None:
        """Reset the masked worlds (all when ``mask`` is None) to ``qpos`` / ``qvel`` or home / rest.

        ``qpos`` / ``qvel`` are ``(N, n)`` on any device; only rows selected by the mask are read. Control, time and
        warm start of the masked worlds are cleared; the other worlds are not touched.
        """
        self._require()
        N, n = self.num_envs, self.iface.n
        mask = self._mask(mask)
        for name, arr in (("qpos", qpos), ("qvel", qvel)):
            if arr is not None and tuple(arr.shape) != (N, n):
                raise ValueError(f"{name} must have shape ({N}, {n}), got {tuple(arr.shape)}")
        with self._scope():
            mjw.reset_data(self._mw, self._d, reset=wp.from_torch(mask))
        d = self._d
        q, v = wp.to_torch(d.qpos), wp.to_torch(d.qvel)
        m = mask[:, None]
        new_q = self._home.expand(N, n) if qpos is None else qpos.to(self.device, torch.float32)
        q[:, self._qpos_idx] = torch.where(m, new_q, q[:, self._qpos_idx])
        if qvel is not None:
            v[:, self._qvel_idx] = torch.where(m, qvel.to(self.device, torch.float32), v[:, self._qvel_idx])
        with self._scope():
            mjw.forward(self._mw, d)

    def set_ctrl(self, ctrl: torch.Tensor) -> None:
        self._require()
        ctrl = torch.as_tensor(ctrl)
        if tuple(ctrl.shape) != (self.num_envs, self.model.nu):
            raise ValueError(f"ctrl must have shape ({self.num_envs}, {self.model.nu}), got {tuple(ctrl.shape)}")
        wp.to_torch(self._d.ctrl).copy_(ctrl.to(self.device, torch.float32))

    def step(self) -> None:
        """Advance ``n_substeps`` physics steps in every world with the current control."""
        self._require()
        with self._scope():
            if self._graph is not None:
                wp.capture_launch(self._graph)
            else:
                for _ in range(self.n_substeps):
                    mjw.step(self._mw, self._d)

    # --- state (device-side gathers of the robot's joints) ----------------------------------

    @property
    def qpos(self) -> torch.Tensor:
        return wp.to_torch(self._d.qpos)[:, self._qpos_idx]

    @property
    def qvel(self) -> torch.Tensor:
        return wp.to_torch(self._d.qvel)[:, self._qvel_idx]

    @property
    def actuator_force(self) -> torch.Tensor:
        return wp.to_torch(self._d.actuator_force)[:, self._act_idx]

    @property
    def time(self) -> torch.Tensor:
        return wp.to_torch(self._d.time)

    # --- SimView ---------------------------------------------------------------------------

    def base_frame(self) -> tuple[np.ndarray, np.ndarray]:
        return self._base_pos64, self._base_R64

    @staticmethod
    def _check_frame(frame: str) -> None:
        if frame not in ("world", "base"):
            raise ValueError(f"frame must be 'world' or 'base', got {frame!r}")

    def body_pose(self, body_id: int, frame: str = "world") -> tuple[torch.Tensor, torch.Tensor]:
        self._require()
        self._check_frame(frame)
        pos, quat = wp.to_torch(self._d.xpos)[:, body_id], wp.to_torch(self._d.xquat)[:, body_id]
        if frame == "base":
            pos, quat = (pos - self._bp) @ self._bR, _quat_mul(self._bqi.expand_as(quat), quat)
        return pos.clone(), quat.clone()

    def site_pos(self, site_id: int, frame: str = "world") -> torch.Tensor:
        self._require()
        self._check_frame(frame)
        pos = wp.to_torch(self._d.site_xpos)[:, site_id]
        return ((pos - self._bp) @ self._bR) if frame == "base" else pos.clone()

    def write_qpos(self, adr: int, values, mask) -> None:
        self._require()
        mask = self._mask(mask)
        values = torch.as_tensor(values).to(self.device, torch.float32)
        if values.ndim != 2 or len(values) != self.num_envs:
            raise ValueError("values must have shape (N, k)")
        q = wp.to_torch(self._d.qpos)
        k = values.shape[1]
        q[:, adr : adr + k] = torch.where(mask[:, None], values, q[:, adr : adr + k])
        with self._scope():
            mjw.forward(self._mw, self._d)

    def host_data(self, env_index: int) -> mujoco.MjData:
        """A scratch ``MjData`` refreshed from world ``env_index`` (rendering and debugging, not a per-step path)."""
        self._require()
        if self._host is None:
            self._host = mujoco.MjData(self.model)
        with self._scope():
            mjw.get_data_into(self._host, self.model, self._d, world_id=env_index)
        return self._host

    # --- capacities --------------------------------------------------------------------------

    def check_overflow(self) -> None:
        """Raise ``CapacityError`` if a capacity was exceeded. The library clips instead of failing: it counts contacts
        in ``nacon`` (all worlds, against ``naconmax``) and constraints in ``nefc`` (per world, against ``njmax``), and
        sets bits of ``overflow`` per world. Reads device counters, so it synchronises."""
        self._require()
        d, opts = self._d, self.options
        nacon, naconmax = int(wp.to_torch(d.nacon).max()), int(d.naconmax)
        if nacon > naconmax:
            raise CapacityError(
                f"the contacts ({nacon}) exceed the allocation of {naconmax} (nconmax={opts.nconmax} per world)"
            )
        flags = int(np.bitwise_or.reduce(wp.to_torch(d.overflow).flatten().cpu().numpy()))
        if flags:
            names = [b.name for b in wtypes.OverflowType if b.name != "ALL" and flags & int(b)]
            nefc = int(wp.to_torch(d.nefc).max())
            raise CapacityError(
                f"capacity exceeded: {', '.join(names)} (most constraint rows in a world: {nefc}, njmax={opts.njmax}; "
                f"contacts: {nacon}, nconmax={opts.nconmax} per world)"
            )

    def capacity_usage(self) -> dict[str, int]:
        """What the last step used: the most constraint rows in any world, the most contacts in any world, and the total
        contacts. Compare with ``njmax``, ``nconmax`` (per world). Reads device counters, so it synchronises."""
        self._require()
        d = self._d
        nacon = min(int(wp.to_torch(d.nacon).max()), int(d.naconmax))
        per_world = torch.bincount(wp.to_torch(d.contact.worldid)[:nacon].long(), minlength=self.num_envs)
        return {
            "max_nefc": int(wp.to_torch(d.nefc).max()),
            "max_contacts_per_world": int(per_world.max()) if nacon else 0,
            "nacon": nacon,
        }

    def close(self) -> None:
        self._mw = self._d = self._graph = self._host = None
