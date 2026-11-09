"""Pins the hashes of existing ``ParametricArm`` configs.

A hash identifies a config in logs, caches and result tables, so it must not change by accident. If a change to
the hashed fields, their defaults or the canonical form is intended, regenerate the file with
``python -m tests.test_golden_hashes`` in the same pull request and add a row to the decision log in docs/design.md.
"""

import json
from pathlib import Path

import pytest

import tests.conftest  # noqa: F401  (registers the toy task provider; also when run as a script)
from robotdesigner.config import (
    BaseEnvConfig,
    ControlConfig,
    EnvConfig,
    FixedBox,
    FixedGains,
    InertiaScaledGains,
    MujocoOptions,
    Pose,
    SimConfig,
    TaskConfig,
    TwoFingerGripper,
    chain,
)
from tests.toy_provider import ReachParams

GOLDEN = Path(__file__).with_name("golden_hashes.json")

_PHYSICS = {"radius": 0.03, "damping": 0.5, "armature": 0.01, "frictionloss": 0.0, "effort_limit": 50.0}


def _arm(n, axes, **kw):
    return chain(n, axes, total_length=0.3 * n, total_mass=1.0 * n, **_PHYSICS, **kw)


def _env(arm, gains=None, mount=None):
    return EnvConfig(
        robot=arm,
        control=ControlConfig(mode="joint_pos_abs", gains=gains or FixedGains(kp=60.0, kd=6.0), action_scale=None),
        task=TaskConfig(
            params=ReachParams(name="reach", reset_noise=0.0, terminate_on_success=False),
            max_episode_steps=200,
            success_tolerance=0.02,
            target_region=FixedBox(low=(0.2, -0.3, 0.0), high=(0.6, 0.3, 0.4)),
            mount=mount or Pose.identity(),
        ),
        sim=SimConfig(simulator="mujoco", num_envs=8, device="cpu", dt=0.002, integrator="euler", control_freq=50,
                      seed=0, scene="plain_floor", options=MujocoOptions(self_collision=False)),
    )


def configs() -> dict[str, EnvConfig]:
    gripper = TwoFingerGripper.default()
    return {
        "chain_n1": _env(_arm(1, "z")),
        "chain_n2": _env(_arm(2, "zy")),
        "chain_n5": _env(_arm(5, "zyyzy")),
        "gripper": _env(_arm(3, "zyz", gripper=gripper)),
        "inertia_scaled_gains": _env(_arm(3, "zyz"), gains=InertiaScaledGains(bandwidth=20.0, damping_ratio=1.0)),
        "mount": _env(_arm(3, "zyz"), mount=Pose(pos=(0.5, -0.2, 0.1), quat=(0.7071067811865476, 0.0, 0.0, 0.7071067811865476))),
        "base_env_default": BaseEnvConfig().resolve(),
    }


def current() -> dict[str, dict[str, str]]:
    return {k: {"robot_hash": c.robot.full_hash, "env_hash": c.full_env_hash} for k, c in configs().items()}


_HINT = (
    "{name}: {field} changed ({old} -> {new}). If this change is intended, regenerate tests/golden_hashes.json with "
    "`python -m tests.test_golden_hashes` in the same pull request and add a decision-log row to docs/design.md."
)


@pytest.mark.parametrize("name", sorted(configs()))
def test_hashes_match_golden(name):
    golden = json.loads(GOLDEN.read_text())[name]
    now = current()[name]
    for field in ("robot_hash", "env_hash"):
        assert now[field] == golden[field], _HINT.format(name=name, field=field, old=golden[field][:12], new=now[field][:12])


def test_golden_file_covers_every_config():
    assert sorted(json.loads(GOLDEN.read_text())) == sorted(configs())


if __name__ == "__main__":
    GOLDEN.write_text(json.dumps(current(), indent=2, sort_keys=True) + "\n")
    print(f"wrote {GOLDEN}")
