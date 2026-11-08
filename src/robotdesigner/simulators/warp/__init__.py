"""MuJoCo Warp (GPU) simulator. Importing this module never imports warp, torch or mujoco."""

from __future__ import annotations

from typing import Any

from ..base import Capabilities, Problem


def _installed(dist: str) -> str | None:
    from importlib import metadata

    try:
        return metadata.version(dist)
    except metadata.PackageNotFoundError:
        return None


class WarpSimulator:
    name = "warp"

    def capabilities(self) -> Capabilities:
        # What the CPU-versus-Warp parity tests (tests/test_warp_parity.py) have shown to match.
        return Capabilities(
            control_modes=frozenset({"joint_pos_abs", "joint_pos_delta", "joint_torque", "joint_vel"}),
            devices=frozenset({"cuda"}),
            robot_kinds=frozenset({"parametric", "external"}),
            integrators=frozenset({"euler", "implicit", "implicitfast", "rk4"}),
        )

    def validate(self, cfg: Any) -> list[Problem]:
        problems: list[Problem] = []
        if type(cfg.robot).__name__ == "ExternalRobot":
            problems.extend(self._external_problems(cfg))
        if getattr(cfg.robot, "gripper", None) is not None:
            problems.append(
                Problem("robot.gripper", "the warp simulator does not build grippers yet; use the mujoco simulator")
            )
        mujoco_version, warp_version = _installed("mujoco"), _installed("mujoco-warp")
        if warp_version is None:
            problems.append(
                Problem("sim.simulator", "mujoco_warp is not installed; install the extra: pip install 'robotdesigner[warp]'")
            )
        elif mujoco_version is not None and warp_version != mujoco_version:
            problems.append(
                Problem(
                    "sim.simulator",
                    f"mujoco-warp {warp_version} does not match mujoco {mujoco_version}; install one release of both",
                )
            )
        try:
            from ..mujoco.scenes import SCENES
        except ImportError:
            problems.append(Problem("sim.scene", "cannot check the scene: mujoco is not installed"))
        else:
            if cfg.sim.scene not in SCENES:
                problems.append(Problem("sim.scene", f"unknown scene {cfg.sim.scene!r}; available: {sorted(SCENES)}"))
        problems.extend(self._device_problems(cfg.sim.device))
        return problems

    @staticmethod
    def _external_problems(cfg: Any) -> list[Problem]:
        """What the installed MuJoCo Warp does not run in the model of an external robot, at the path ``robot``. The
        model is the robot provider's, attached to the scene preset: the task's own assets are checked at build time."""
        try:
            from ..mujoco.compile import compile_robot
            from ..mujoco.external import problems_of
            from .sim import check_model_supported
        except ImportError:
            return []  # a missing library is reported below
        opened = problems_of(cfg)  # the provider's checks, as for the mujoco simulator
        if opened:
            return opened
        try:
            model = compile_robot(cfg).model
        except Exception as e:  # not a feature of the robot: the other checks say what is wrong
            return [Problem("robot", f"the model could not be compiled: {type(e).__name__}: {e}")]
        return [Problem("robot", p.message) for p in check_model_supported(model)]

    @staticmethod
    def _device_problems(device: str) -> list[Problem]:
        if not device.startswith("cuda"):
            return []  # reported by the capability check
        try:
            import torch
        except ImportError:
            return [Problem("sim.device", f"cannot use {device!r}: torch is not installed")]
        if not torch.cuda.is_available():
            return [Problem("sim.device", f"CUDA is not available, so device {device!r} cannot be used")]
        index, count = int(device.split(":")[1]), torch.cuda.device_count()
        if index >= count:
            return [Problem("sim.device", f"device {device!r} is not visible; {count} CUDA device(s) found")]
        return []

    def make_env(self, cfg: Any):
        from ...config.providers import get_task_provider
        from .env import WarpEnv

        provider = get_task_provider(cfg.task.params.provider)
        return WarpEnv(cfg, provider.build(cfg, self.name))
