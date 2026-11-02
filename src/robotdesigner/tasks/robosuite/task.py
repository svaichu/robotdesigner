"""Engine side of ``robosuite/<EnvName>``: robosuite's scene in our MuJoCo simulation, our objective on top."""

from __future__ import annotations

import numpy as np
import torch
from torchrl.data import Composite, Unbounded

from ..base import BaseTask
from .params import LiftObject, ReachObject
from .scene import PREFIX, load_scene


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
        draws = (rand(sim.num_envs) * 2**31).to(torch.int64).numpy()  # one per env, whatever the mask
        values: dict[str, torch.Tensor] = {}
        placed: dict[str, torch.Tensor] = {}
        for i in torch.nonzero(mask.cpu()).flatten().tolist():
            for name, (pos, quat, _obj) in self._sample(int(draws[i])).items():
                item = self._by_name.get(name)
                if item is not None and item[2] is not None:
                    if name not in values:
                        values[name] = torch.zeros(sim.num_envs, 7, dtype=torch.float64)
                        placed[name] = torch.zeros(sim.num_envs, dtype=torch.bool)
                    values[name][i] = torch.from_numpy(np.concatenate([np.asarray(pos), np.asarray(quat)]))
                    placed[name][i] = True
        for name, v in values.items():
            sim.write_qpos(self._by_name[name][2], v, placed[name])

    # --- observations, success, reward -----------------------------------------------

    def observe(self, sim) -> dict[str, torch.Tensor]:
        out = {"ee_pos": sim.site_pos(sim.iface.ee_site_id, "base")}
        for name, body, _ in self._items:
            out[f"{name}_pos"], out[f"{name}_quat"] = sim.body_pose(body, "base")
        return out

    def _dist(self, obs) -> torch.Tensor:
        return torch.linalg.norm(obs["ee_pos"] - obs[f"{self._objective.object}_pos"], dim=-1, keepdim=True)

    def success(self, sim, obs) -> torch.Tensor:
        obj = self._objective
        if isinstance(obj, ReachObject):
            return self._dist(obs) < obj.tolerance
        assert isinstance(obj, LiftObject)
        body = self._by_name[obj.object][1]
        z = sim.body_pose(body, "world")[0][:, 2:3]
        return z >= obj.height

    def reward(self, sim, obs) -> torch.Tensor:
        return -self._dist(obs)
