"""The contact-option policy of an attached Menagerie model, decided with data (#54).

``MjSpec.attach`` keeps the scene's value where a model's own ``<option>`` differs (cone, impratio, solver, iterations, ...).
This script stands Go1 and A1 on the floor for 2 s from the reset pose (``joint_pos_delta``, zero action: the position
targets hold the reset pose), with the model's options carried over and not carried over, at ``sim.dt`` equal to the model's own
timestep and at twice it, and prints per case: the root height and its drift from the keyframe, the trunk tilt, the largest
penetration depth and the mean number of contacts.

    python benchmarks/menagerie_options.py            # needs: pip install 'robotdesigner[menagerie]'
"""

import warnings

import mujoco
import numpy as np
import torch

from robotdesigner.config import BaseEnvConfig, snapped
from robotdesigner.envs import make_env
from robotdesigner.robot.providers.menagerie import PROVIDER
from robotdesigner.simulators.mujoco import external


def stand(name: str, carry: bool, dt_factor: float, seconds: float = 2.0) -> dict:
    external.CARRY_MODEL_OPTIONS = carry
    robot = PROVIDER.describe(name)
    dt = external._open(robot).model.opt.timestep * dt_factor
    cfg = (
        BaseEnvConfig().robot(robot).task(mount=snapped(robot)).control(mode="joint_pos_delta", action_scale=0.05)
        .sim(num_envs=1, scene="plain_floor", dt=dt, control_freq=50.0).resolve()
    )
    env = make_env(cfg)
    td, d = env.reset(), env._sim.data[0]
    root = env.interface.root_qpos_adr
    z0, penetration, contacts = env.interface.home_root.pos[2], 0.0, []
    for _ in range(round(seconds / env._sim.control_dt)):
        td["action"] = torch.zeros(1, robot.n)
        td = env.step(td)["next"]
        contacts.append(d.ncon)
        penetration = max(penetration, -min([c.dist for c in d.contact] + [0.0]))
    _, x, y, _ = d.qpos[root + 3 : root + 7]
    model = env._compiled.model
    out = {
        "dt": dt, "z": float(d.qpos[root + 2]), "drift": float(d.qpos[root + 2] - z0),
        "tilt": float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0)))),
        "penetration": float(penetration), "contacts": float(np.mean(contacts)),
        "cone": mujoco.mjtCone(int(model.opt.cone)).name, "impratio": float(model.opt.impratio),
    }
    env.close()
    return out


if __name__ == "__main__":
    warnings.simplefilter("ignore")
    print(f"{'robot':12} {'carried':8} {'dt':>7} {'root z':>7} {'drift':>7} {'tilt°':>6} {'pen. m':>7} {'contacts':>8}  scene options used")
    for name in ("unitree_go1", "unitree_a1"):
        for carry in (False, True):
            for factor in (1, 2):
                r = stand(name, carry, factor)
                print(
                    f"{name:12} {carry!s:8} {r['dt']:7.4f} {r['z']:7.3f} {r['drift']:+7.3f} {r['tilt']:6.2f} {r['penetration']:7.4f} "
                    f"{r['contacts']:8.1f}  {r['cone']}, impratio {r['impratio']:g}"
                )
