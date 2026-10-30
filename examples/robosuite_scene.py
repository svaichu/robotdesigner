"""A three-link arm in the robosuite Lift scene (table and cube), simulated by the CPU MuJoCo simulator.

robosuite only supplies the assets (arena, cube, placement rule); our MuJoCo simulation steps everything.
Needs the robosuite extra:  pip install -e ".[env,robosuite]"
Run:  python examples/robosuite_scene.py
"""

from __future__ import annotations

from robotdesigner import make_env
from robotdesigner.config import (
    ControlConfig,
    EnvConfig,
    FixedGains,
    MujocoOptions,
    SimConfig,
    TaskConfig,
    chain,
)
from robotdesigner.tasks.robosuite import LiftObject, RobosuiteParams, default_mount


def build_config(num_envs: int = 4, horizon: int = 100) -> EnvConfig:
    arm = chain(
        n=3,
        axes="zyy",
        total_length=0.9,
        total_mass=3.0,
        radius=0.03,
        damping=0.5,
        armature=0.01,
        frictionloss=0.0,
        effort_limit=50.0,
        range=(-2.5, 2.5),
    )
    return EnvConfig(
        robot=arm,
        control=ControlConfig("joint_pos_abs", FixedGains(kp=60.0, kd=6.0), action_scale=None),
        task=TaskConfig(
            params=RobosuiteParams(
                name="Lift", env_kwargs={}, objective=LiftObject("cube", height=0.84),
                reset_noise=0.0, terminate_on_success=False,
            ),
            horizon=horizon,
            success_tolerance=None,
            target_region=None,
            mount=default_mount("Lift"),  # where robosuite mounts a stock arm in this scene
        ),
        sim=SimConfig(
            simulator="mujoco", num_envs=num_envs, device="cpu", dt=0.002,
            integrator="euler", control_freq=50, seed=0, scene="empty", options=MujocoOptions(self_collision=False),
        ),
    )


def main(num_envs: int = 4, steps: int = 50) -> None:
    env = make_env(build_config(num_envs=num_envs, horizon=max(steps, 1)))
    print(f"robot_hash={env.robot_hash} env_hash={env.env_hash}")
    traj = env.rollout(steps)
    print("rollout batch shape:", tuple(traj.shape))
    print("task observations:", sorted(traj["next", "task"].keys()))
    print("final cube z in the robot's base frame (m):", traj["next", "task", "cube_pos"][:, -1, 2].tolist())


if __name__ == "__main__":
    main()
