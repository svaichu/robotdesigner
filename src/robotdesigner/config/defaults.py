"""The defaults of ``BaseEnvConfig``, in one table. Nothing else in the config layer has a default value.

``BaseEnvConfig()`` is a one-link arm in the ``empty`` scene doing the ``empty/idle`` task on the CPU MuJoCo simulator.
The numbers are plain choices, not recommendations: a change of any of them changes the ``env_hash`` of every config
that relied on it, so ``DEFAULTS_VERSION`` is raised with it. Store ``resolve().to_dict()`` with every run: it holds
every value and reproduces after the defaults change. The simulator's options are the ``default()`` of its options
class (see ``config.sim``).
"""

from __future__ import annotations

DEFAULTS_VERSION = 1

DEFAULTS: dict[str, dict] = {
    # the arm: one yaw joint, 0.5 m long, 1 kg (the arguments of ``chain``, see ``specs.Chain``)
    "robot": dict(
        n=1, axes="z", total_length=0.5, total_mass=1.0, radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0,
        effort_limit=50.0, range=(-2.5, 2.5),
    ),
    # the README quick start's control: absolute joint positions, kp 60, kd 6 on every joint
    "control": dict(mode="joint_pos_abs", kp=60.0, kd=6.0, action_scale=None),
    # the placeholder task of the package; ``name`` is also the task whose default mount is used
    "task": dict(provider="empty", name="idle", max_episode_steps=200),
    # 500 Hz physics, 50 Hz control (the quick start's numbers), one env, Euler
    "sim": dict(
        simulator="mujoco", num_envs=1, device="cpu", dt=0.002, integrator="euler", control_freq=50.0, seed=0,
        scene="empty",
    ),
}
