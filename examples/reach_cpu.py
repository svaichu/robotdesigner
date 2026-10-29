"""Headless random-policy rollout of the reach task of the native provider on a three-link arm (CPU MuJoCo).

Run:  python examples/reach_cpu.py
"""

from __future__ import annotations

from robotdesigner import make_env
from robotdesigner.config import (
    ControlConfig,
    EnvConfig,
    FixedBox,
    FixedGains,
    MujocoOptions,
    Pose,
    SimConfig,
    TaskConfig,
    chain,
)
from robotdesigner.tasks.native import ReachParams


def build_config(num_envs: int = 8, horizon: int = 200) -> EnvConfig:
    arm = chain(
        n=3,
        axes="zyz",
        total_length=0.9,
        total_mass=3.0,
        radius=0.03,
        damping=0.5,
        armature=0.01,
        frictionloss=0.0,
        effort_limit=50.0,
        range=(-2.5, 2.5),
        base_pose=Pose((0.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0)),
    )
    return EnvConfig(
        robot=arm,
        control=ControlConfig("joint_pos_abs", FixedGains(kp=60.0, kd=6.0), action_scale=None),
        task=TaskConfig(
            id="native/reach",
            horizon=horizon,
            success_tolerance=0.02,
            target_region=FixedBox(low=(0.2, -0.3, -0.2), high=(0.6, 0.3, 0.2)),
            params=ReachParams(reset_noise=0.0, terminate_on_success=False),
        ),
        sim=SimConfig(
            backend="mujoco", num_envs=num_envs, device="cpu", dt=0.002,
            integrator="euler", control_freq=50, seed=0, scene="plain_floor",
        ),
        backend_options=MujocoOptions(self_collision=False),
    )


def main(num_envs: int = 8, steps: int = 50) -> None:
    env = make_env(build_config(num_envs=num_envs, horizon=max(steps, 1)))
    print(f"robot_hash={env.robot_hash} env_hash={env.env_hash}")
    traj = env.rollout(steps)
    print("rollout batch shape:", tuple(traj.shape))
    print("ee_pos:", tuple(traj["next", "task", "ee_pos"].shape))
    print("reward:", tuple(traj["next", "reward"].shape))


if __name__ == "__main__":
    main()
