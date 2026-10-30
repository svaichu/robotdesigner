"""Engine side of ``robosuite/<EnvName>``: robosuite's scene in our MuJoCo simulation, our objective on top."""

from __future__ import annotations

import numpy as np
import torch
from torchrl.data import Composite, Unbounded

from ..base import BaseTask
from .params import LiftObject, ReachObject
from .scene import PREFIX, load_scene


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of quaternions (w, x, y, z), broadcasting over leading dimensions."""
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=-1,
    )


def _reseed_generators(sampler, seed: int) -> None:
    """robosuite >= 1.5.2 samples from a numpy Generator the sampler holds; reseed it in place."""
    rng = getattr(sampler, "rng", None)
    if rng is not None and hasattr(rng, "bit_generator"):
        rng.bit_generator.state = np.random.default_rng(seed).bit_generator.state
    for sub in getattr(sampler, "samplers", {}).values():
        _reseed_generators(sub, seed)


class RobosuiteTask(BaseTask):
    """Observations (the ``task`` group), all in the robot's base frame like every other task:

    ``ee_pos`` (3), and for every physical object of the scene ``<name>_pos`` (3) and ``<name>_quat`` (4, w x y z);
    visual-only objects are in the scene but not observed. Reward is ``-||ee - object||`` for the objective's
    object; success is the objective's test.

    Objects with a free joint are placed by robosuite's sampler at every reset. Objects without one (a door frame,
    a stand, a target bin) are fixtures: all envs share one model, so they are placed once, when the task is bound,
    from ``sim.seed``. Objects the sampler does not place stay at their pose in the model.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        params = cfg.task.params
        self._objective = params.objective
        self._assets = load_scene(params.name, params.kwargs, cfg.sim.seed)
        self._items: list[tuple[str, int, int | None]] = []  # observed objects: (name, body id, free-joint qpos address)
        self._by_name: dict[str, tuple[str, int, int | None]] = {}
        self.fixtures: tuple[str, ...] = ()  # objects placed once at bind time (no free joint)

    @property
    def object_names(self) -> tuple[str, ...]:
        """Names of the observed (physical) objects."""
        return tuple(o.name for o in self._assets.objects if not o.visual)

    # --- model -----------------------------------------------------------------------

    def attach(self, spec) -> None:
        import mujoco

        child = mujoco.MjSpec.from_string(self._assets.xml)
        spec.attach(child, frame=spec.worldbody.add_frame(), prefix=PREFIX)

    def bind(self, sim) -> None:
        import mujoco

        model = sim.model
        self._items, every = [], {}
        for o in self._assets.objects:
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, PREFIX + o.root_body)
            adr = None
            if o.joints:
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, PREFIX + o.joints[0])
                if jid >= 0 and model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE:
                    adr = int(model.jnt_qposadr[jid])
            every[o.name] = (body, adr)
            if not o.visual:
                self._items.append((o.name, body, adr))
        self._by_name = {it[0]: it for it in self._items}
        # Fixtures cannot be placed per env (they live in the shared model): one placement, baked in.
        fixtures = []
        for name, (pos, quat, _obj) in self._sample(int(self.cfg.sim.seed)).items():  # same seed as the scene
            body, adr = every.get(name, (-1, None))
            if body >= 0 and adr is None:
                model.body_pos[body] = np.asarray(pos, dtype=float)
                model.body_quat[body] = np.asarray(quat, dtype=float)
                fixtures.append(name)
        self.fixtures = tuple(fixtures)
        self._base_pos, self._base_R = sim._base_pos, sim._base_R
        q = np.asarray(self.cfg.robot.base_pose.quat, dtype=float)
        self._base_quat_inv = np.array([q[0], -q[1], -q[2], -q[3]]) / float(q @ q)

    def observation_spec(self, num_envs: int):
        f32 = torch.float32
        entries = {"ee_pos": Unbounded(shape=(num_envs, 3), dtype=f32)}
        for name, _, _ in self._items:
            entries[f"{name}_pos"] = Unbounded(shape=(num_envs, 3), dtype=f32)
            entries[f"{name}_quat"] = Unbounded(shape=(num_envs, 4), dtype=f32)
        return Composite(**entries, shape=(num_envs,))

    # --- reset -----------------------------------------------------------------------

    def _sample(self, seed: int) -> dict:
        sampler = self._assets.sampler
        if sampler is None:
            return {}
        state = np.random.get_state()
        np.random.seed(seed % 2**32)  # robosuite 1.5.1 samples from the global stream; put it back afterwards
        try:
            _reseed_generators(sampler, seed)
            return sampler.sample()
        finally:
            np.random.set_state(state)

    def reset(self, sim, mask, rand, injected) -> None:
        import mujoco

        draws = (rand(sim.num_envs) * 2**31).to(torch.int64).numpy()  # one per env, whatever the mask
        for i in np.flatnonzero(np.asarray(mask, dtype=bool)):
            d = sim.data[i]
            for name, (pos, quat, _obj) in self._sample(int(draws[i])).items():
                item = self._by_name.get(name)
                if item is not None and item[2] is not None:
                    d.qpos[item[2] : item[2] + 7] = np.concatenate([np.asarray(pos), np.asarray(quat)])
            mujoco.mj_forward(sim.model, d)

    # --- observations, success, reward -----------------------------------------------

    def _in_base_frame(self, sim, body: int) -> tuple[np.ndarray, np.ndarray]:
        pos = np.stack([d.xpos[body] for d in sim.data])
        quat = np.stack([d.xquat[body] for d in sim.data])
        return (pos - self._base_pos) @ self._base_R, _quat_mul(self._base_quat_inv, quat)

    def observe(self, sim) -> dict[str, torch.Tensor]:
        out = {"ee_pos": torch.from_numpy(sim.ee_pos_base()).to(torch.float32)}
        for name, body, _ in self._items:
            p, q = self._in_base_frame(sim, body)
            out[f"{name}_pos"] = torch.from_numpy(p).to(torch.float32)
            out[f"{name}_quat"] = torch.from_numpy(q).to(torch.float32)
        return out

    def _dist(self, obs) -> torch.Tensor:
        return torch.linalg.norm(obs["ee_pos"] - obs[f"{self._objective.object}_pos"], dim=-1, keepdim=True)

    def success(self, sim, obs) -> torch.Tensor:
        obj = self._objective
        if isinstance(obj, ReachObject):
            return self._dist(obs) < obj.tolerance
        assert isinstance(obj, LiftObject)
        body = self._by_name[obj.object][1]
        z = torch.from_numpy(np.array([d.xpos[body][2] for d in sim.data])).reshape(-1, 1)
        return z >= obj.height

    def reward(self, sim, obs) -> torch.Tensor:
        return -self._dist(obs)
