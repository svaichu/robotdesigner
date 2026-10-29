"""Any single-arm robosuite environment as a TorchRL environment, with a robot built from a ``ParametricArm``.

One robosuite environment per batch element, stepped in turn in this process. The environment never autoresets and
honours ``td["_reset"]``, like the native backend.

Observations: ``task`` holds every robosuite observation that does not belong to the robot (``cube_pos``, ...),
``proprio`` holds the robot's (``qpos`` and ``qvel`` first, the rest with the ``robot0_`` prefix removed). The
aggregate ``*-state`` vectors are dropped as redundant. Scalars become length-1 vectors.
"""

from __future__ import annotations

import contextlib

import numpy as np
import torch
from tensordict import TensorDict, TensorDictBase
from torchrl.data import Binary, Bounded, Composite, Unbounded
from torchrl.envs import EnvBase

from ...config import ConfigError, EnvConfig
from ._compat import apply_compat
from ._introspect import constructor_params, env_class, resolved_env_kwargs
from .arm import controller_config, register_arm

_ROBOT_PREFIX = "robot0_"
_RENAMED = {"robot0_joint_pos": "qpos", "robot0_joint_vel": "qvel"}


def _split_obs(obs: dict) -> tuple[dict, dict]:
    """(task, proprio) float32 arrays from one robosuite observation dict."""
    task, proprio = {}, {}
    for key, value in obs.items():
        if key.endswith("-state"):
            continue
        array = np.atleast_1d(np.asarray(value, dtype=np.float32))
        if key.startswith(_ROBOT_PREFIX):
            proprio[_RENAMED.get(key, key[len(_ROBOT_PREFIX) :])] = array
        else:
            task[key] = array
    return task, proprio


class RobosuiteEnv(EnvBase):
    """A robosuite task (``robosuite/<EnvName>``) driven through our contract. CPU, ``joint_pos_delta`` control."""

    batch_locked = True

    def __init__(self, cfg: EnvConfig):
        cfg = cfg.resolve()
        self.cfg = cfg
        self.robot_hash = cfg.robot.config_hash
        self.env_hash = cfg.env_hash
        N = cfg.sim.num_envs
        super().__init__(device=torch.device("cpu"), batch_size=torch.Size([N]))

        apply_compat()
        import robosuite

        self.robosuite_version = robosuite.__version__
        task, arm = cfg.task, cfg.robot
        self.env_name = task.id.split("/", 1)[1]
        cls = env_class(self.env_name)
        if cls is None:
            raise ConfigError("task.id", f"robosuite has no environment {self.env_name!r}")
        user_kwargs = task.params.kwargs
        #: every task argument the environment ran with (library defaults included); not part of ``env_hash``
        self.resolved_env_kwargs = resolved_env_kwargs(cls, user_kwargs)

        kwargs = dict(
            robots=register_arm(arm, cfg.backend_options.base_placement),
            controller_configs=controller_config(cfg.control),
            gripper_types=arm.tool.gripper,
            control_freq=cfg.sim.control_freq,
            horizon=task.horizon,
            hard_reset=False,  # reset() re-randomizes the task but keeps the simulation (and its handles) alive
            initialization_noise={"type": "gaussian", "magnitude": 0.0},  # reset noise is ours, below
            has_renderer=False,
            has_offscreen_renderer=False,
            use_camera_obs=False,
        )
        kwargs.update(user_kwargs)

        # robosuite 1.5.1 draws from the global numpy RNG (object sizes, placements, robot noise); 1.5.2 draws from a
        # Generator that it creates at construction, from a ``seed`` argument. Each sub-env gets its own stream,
        # swapped in around every call into robosuite, and is seeded through ``seed`` where the environment has one.
        self._rs = [np.random.RandomState(cfg.sim.seed + i) for i in range(N)]
        takes_seed = "seed" in constructor_params(cls)
        self._envs = []
        for i in range(N):
            with self._stream(i):
                try:
                    env = robosuite.make(self.env_name, **kwargs, **({"seed": cfg.sim.seed + i} if takes_seed else {}))
                except AssertionError as e:  # robosuite's own checks, for example Wipe's gripper
                    raise ConfigError("task", f"robosuite rejected {task.id!r} with this robot: {e}") from e
            self._reseed(env, cfg.sim.seed + i)
            self._envs.append(env)

        self._n = arm.n
        self._home = np.array(arm.home_qpos, dtype=float)
        rng = np.array([(-np.inf, np.inf) if s.joint.range is None else s.joint.range for s in arm.segments])
        self._lo, self._hi = rng[:, 0], rng[:, 1]
        self._reset_noise = task.params.reset_noise
        self._terminate_on_success = task.params.terminate_on_success
        self._horizon = task.horizon
        self._scale = cfg.control.action_scale
        self._q_target = np.zeros((N, arm.n))

        self._gen = torch.Generator(device="cpu")
        self._gen.manual_seed(cfg.sim.seed)
        self._episode_step = torch.zeros(N, 1, dtype=torch.int64)
        self._obs: list = [None] * N
        self._success = np.zeros(N, dtype=bool)
        for i in range(N):
            self._reset_one(i, self._home, None)

        task_obs, proprio_obs = _split_obs(self._obs[0])
        self._task_keys, self._proprio_keys = list(task_obs), list(proprio_obs)
        A = self._envs[0].action_dim
        f32 = torch.float32

        def unbounded(arrays):
            return Composite(**{k: Unbounded(shape=(N, *a.shape), dtype=f32) for k, a in arrays.items()}, shape=(N,))

        self.observation_spec = Composite(
            task=unbounded(task_obs),
            proprio=unbounded(proprio_obs),
            success=Binary(shape=(N, 1), dtype=torch.bool),
            episode_step=Unbounded(shape=(N, 1), dtype=torch.int64),
            shape=(N,),
        )
        self.action_spec = Bounded(low=-1.0, high=1.0, shape=(N, A), dtype=f32)
        self.reward_spec = Unbounded(shape=(N, 1), dtype=f32)
        self.done_spec = Composite(
            done=Binary(shape=(N, 1), dtype=torch.bool),
            terminated=Binary(shape=(N, 1), dtype=torch.bool),
            truncated=Binary(shape=(N, 1), dtype=torch.bool),
            shape=(N,),
        )

    # --- helpers --------------------------------------------------------------------

    @contextlib.contextmanager
    def _stream(self, i: int):
        """Run robosuite code with sub-env ``i``'s random stream as the global numpy stream."""
        saved = np.random.get_state()
        np.random.set_state(self._rs[i].get_state())
        try:
            yield
        finally:
            self._rs[i].set_state(np.random.get_state())
            np.random.set_state(saved)

    @staticmethod
    def _reseed(env, seed: int) -> None:
        # In place: robosuite hands this Generator to its placement sampler, so replacing the object would not reach it.
        rng = getattr(env, "rng", None)
        if rng is not None:
            rng.bit_generator.state = np.random.default_rng(seed).bit_generator.state

    def _rand(self, *shape, dtype=torch.float64) -> torch.Tensor:
        return torch.rand(*shape, generator=self._gen, dtype=dtype)

    @property
    def robosuite_envs(self) -> list:
        """The underlying robosuite environments. Their simulation objects stay valid across ``reset``."""
        return list(self._envs)

    def _reset_one(self, i: int, qpos: np.ndarray, qvel: np.ndarray | None) -> None:
        env = self._envs[i]
        env.robots[0].init_qpos = np.asarray(qpos, dtype=float)  # robosuite starts the robot from this
        with self._stream(i):
            obs = env.reset()
        if qvel is not None:
            env.sim.data.qvel[env.robots[0]._ref_joint_vel_indexes] = np.asarray(qvel, dtype=float)
            env.sim.forward()
            obs = env._get_observations(force_update=True)
        self._obs[i] = obs
        self._q_target[i] = np.asarray(obs["robot0_joint_pos"], dtype=float)
        self._success[i] = bool(env._check_success())

    @property
    def q_target(self) -> np.ndarray:
        """Joint position targets sent to the controller, ``(N, n)``."""
        return self._q_target.copy()

    def _stack(self, arrays: list) -> torch.Tensor:
        return torch.from_numpy(np.stack(arrays))

    def _observe(self) -> TensorDict:
        split = [_split_obs(o) for o in self._obs]
        for i, (t, pr) in enumerate(split):
            if set(t) != set(self._task_keys) or set(pr) != set(self._proprio_keys):
                raise RuntimeError(
                    f"sub-env {i} of robosuite/{self.env_name} changed its observation keys since the first reset "
                    "(the task samples a different object per episode?). A fixed observation spec cannot describe "
                    "that; fix the object through env_kwargs."
                )
        return TensorDict(
            {
                "task": TensorDict(
                    {k: self._stack([s[0][k] for s in split]) for k in self._task_keys}, batch_size=self.batch_size
                ),
                "proprio": TensorDict(
                    {k: self._stack([s[1][k] for s in split]) for k in self._proprio_keys}, batch_size=self.batch_size
                ),
                "success": torch.from_numpy(self._success.copy()).reshape(-1, 1),
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
        noise = ((self._rand(N, n) * 2.0 - 1.0) * self._reset_noise).numpy()
        qpos = np.clip(self._home + noise, self._lo, self._hi)
        qpos_in, qvel_in = injected("qpos"), injected("qvel")
        if qpos_in is not None:
            qpos = qpos_in.to(torch.float64).numpy()
        qvel = None if qvel_in is None else qvel_in.to(torch.float64).numpy()

        for i in np.flatnonzero(mask.numpy()):
            self._reset_one(int(i), qpos[i], None if qvel is None else qvel[i])
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
        reward = np.zeros(self.batch_size[0], dtype=np.float32)
        n = self._n
        self._q_target = np.clip(self._q_target + action[:, :n] * self._scale, self._lo, self._hi)
        for i, env in enumerate(self._envs):
            with self._stream(i):  # joint targets are absolute radians; the gripper command passes through
                obs, r, _, _ = env.step(np.concatenate([self._q_target[i], action[i, n:]]))
            self._obs[i] = obs
            reward[i] = r
            self._success[i] = bool(env._check_success())
        self._episode_step += 1

        out = self._observe()
        success = out["success"]
        truncated = self._episode_step >= self._horizon
        terminated = success.clone() if self._terminate_on_success else torch.zeros_like(success)
        out.set("reward", torch.from_numpy(reward).reshape(-1, 1))
        out.set("terminated", terminated)
        out.set("truncated", truncated)
        out.set("done", terminated | truncated)
        return out

    def _set_seed(self, seed: int | None):
        if seed is not None:
            seed = int(seed)
            self._gen.manual_seed(seed)
            self._rs = [np.random.RandomState(seed + i) for i in range(len(self._envs))]
            for i, env in enumerate(self._envs):
                self._reseed(env, seed + i)
        return seed

    def close(self, *, raise_if_closed: bool = True):
        for env in getattr(self, "_envs", []):
            env.close()
        self._envs = []
        return super().close(raise_if_closed=raise_if_closed)
