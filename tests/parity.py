"""Helpers for the CPU-versus-Warp parity tests: the same config on both simulators, the same injected state."""

from dataclasses import replace

import numpy as np
import torch

from robotdesigner.config import ControlConfig, FixedGains, MujocoOptions, SimConfig, WarpOptions
from robotdesigner.config.providers import get_task_provider
from robotdesigner.envs import make_env

WARP_OPTIONS = WarpOptions(njmax=128, nconmax=32, cuda_graph=False)


def with_sim(cfg, **changes):
    return replace(cfg, sim=replace(cfg.sim, **changes))


def twin(cfg):
    """The same config on the CPU simulator."""
    return with_sim(cfg, simulator="mujoco", device="cpu", options=MujocoOptions(self_collision=False))


def on_warp(cfg, device):
    return with_sim(cfg, simulator="warp", device=device, options=WARP_OPTIONS)


def with_control(cfg, mode, gains=None, scale=None):
    scale = {"joint_pos_abs": None, "joint_pos_delta": 0.05, "joint_vel": 2.0, "joint_torque": None}[mode] if scale is None else scale
    return replace(cfg, control=ControlConfig(mode, gains or FixedGains(60.0, 6.0), scale))


def build(cfg):
    """make_env, except that the Warp simulator on the library's CPU device (tests only) is built directly."""
    if cfg.sim.simulator == "warp" and cfg.sim.device == "cpu":
        from robotdesigner.simulators.warp.env import WarpEnv

        return WarpEnv(cfg, get_task_provider(cfg.task.params.provider).build(cfg, "warp"))
    return make_env(cfg)


def run(cfg, actions: np.ndarray, inject: dict[str, np.ndarray] | None = None) -> dict[str, np.ndarray]:
    """Reset with the injected state, apply the (T, N, n) actions, return float64 arrays (T, N, ...) per step."""
    env = build(cfg)
    td = env.reset()
    for key, value in (inject or {}).items():
        td[key] = torch.as_tensor(value, dtype=torch.float64).to(env.device)
    td = env.reset(td)
    out: dict[str, list] = {}
    for a in actions:
        td["action"] = torch.as_tensor(a, dtype=torch.float32, device=env.device)
        nxt = env.step(td)["next"]
        flat = {"/".join(k) if isinstance(k, tuple) else k: v for k, v in nxt.items(True, True)}
        for k, v in flat.items():
            if v.dtype.is_floating_point:
                out.setdefault(k, []).append(v.detach().cpu().double().numpy())
        td = nxt
    env.close()
    return {k: np.stack(v) for k, v in out.items()}


def random_case(n: int, num_envs: int, steps: int, seed: int = 0):
    rng = np.random.default_rng(seed)
    actions = rng.uniform(-1.0, 1.0, (steps, num_envs, n))
    inject = {"qpos": rng.uniform(-0.5, 0.5, (num_envs, n)), "qvel": rng.uniform(-0.5, 0.5, (num_envs, n))}
    return actions, inject


def errors(gpu: dict, cpu: dict) -> dict[str, float]:
    """Largest absolute difference per quantity (float64 arrays)."""
    return {k: float(np.abs(gpu[k] - cpu[k]).max()) for k in cpu if k in gpu}
