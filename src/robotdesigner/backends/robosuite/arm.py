"""Turn a ``ParametricArm`` into a robosuite robot class, and a ``ControlConfig`` into its controller config."""

from __future__ import annotations

import os
import tempfile
import threading

import mujoco
import numpy as np

from ...config import ControlConfig, InertiaScaledGains, ParametricArm
from ..mujoco.arm_spec import add_arm

_LOCK = threading.Lock()
_CACHE_DIR: str | None = None  # robosuite robot classes read their XML from a file path


def robot_name(arm: ParametricArm, base_placement: str) -> str:
    """The robosuite robot name: derived from the physical config hash, so equal arms share a class."""
    return f"RD{arm.config_hash[:10]}{base_placement[0]}"


def arm_xml(arm: ParametricArm) -> str:
    """Robot-only MJCF in the layout robosuite expects.

    Root body ``base``; mount body ``link0`` carrying ``arm.base_pose`` and the ``right_center`` site that
    robosuite's controllers use as the arm's base frame; the chain from the shared arm builder
    (``link_i`` / ``joint_i`` / ``geom_i``); a ``right_hand`` body at the end-effector pose, where the gripper
    attaches; one torque motor per joint limited to ``effort_limit`` (robosuite's controllers command torques).
    Robot geoms collide with the scene, not with each other, as in the native backend.
    """
    spec = mujoco.MjSpec()
    spec.compiler.degree = False  # MjSpec reads angles as degrees by default; all angles here are radians
    root = spec.worldbody.add_body(name="base")
    mount = root.add_body(name="link0", pos=list(arm.base_pose.pos), quat=list(arm.base_pose.quat))
    mount.add_site(name="right_center", pos=[0.0, 0.0, 0.0], size=[0.01, 0.0, 0.0], group=2)  # controller base frame
    last, tip = add_arm(mount, arm, conaffinity=0)
    last.add_body(name="right_hand", pos=list(tip + np.asarray(arm.end_effector.pos)), quat=list(arm.end_effector.quat))
    for i, seg in enumerate(arm.segments):
        effort = seg.joint.effort_limit
        act = spec.add_actuator(name=f"torq_j{i}", target=f"joint_{i}", trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
        act.ctrlrange = [-effort, effort]
        act.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        act.forcerange = [-effort, effort]
    return spec.to_xml()


def register_arm(arm: ParametricArm, base_placement: str) -> str:
    """Create and register the robosuite robot class for ``arm`` (once per process) and return its name.

    robosuite keeps its robot registries global to the process, so the class is registered by name and reused.
    ``base_placement`` is ``"env_default"`` (mount where the environment mounts a stock arm) or ``"explicit"``
    (root at the arena origin; ``arm.base_pose`` places the arm).
    """
    global _CACHE_DIR
    from robosuite.models.robots import Panda
    from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
    from robosuite.models.robots.robot_model import REGISTERED_ROBOTS
    from robosuite.robots import register_robot_class

    name = robot_name(arm, base_placement)
    with _LOCK:
        if name in REGISTERED_ROBOTS:
            return name
        if _CACHE_DIR is None:
            _CACHE_DIR = tempfile.mkdtemp(prefix="robotdesigner_arms_")
        path = os.path.join(_CACHE_DIR, f"{name}.xml")
        with open(path, "w") as f:
            f.write(arm_xml(arm))

        tool = arm.tool
        if base_placement == "env_default":
            offsets = Panda.base_xpos_offset.fget(None)  # the stock mounting rule of the library itself
        else:
            offsets = {"bins": (0, 0, 0), "empty": (0, 0, 0), "table": lambda table_length: (0, 0, 0)}
        home = np.array(arm.home_qpos, dtype=float)
        reach = float(sum(np.linalg.norm(s.link.tip_offset) for s in arm.segments))
        mount_pos = None if tool is None else list(tool.mount_pos)
        mount_quat = None if tool is None else list(tool.mount_quat)
        gripper = None if tool is None else tool.gripper
        namespace = dict(
            arms=["right"],
            arm_type=property(lambda self: "single"),
            default_base=property(lambda self: "NullMount"),
            default_gripper=property(lambda self, g=gripper: {"right": g}),
            gripper_mount_pos_offset=property(lambda self, p=mount_pos: {} if p is None else {"right": p}),
            gripper_mount_quat_offset=property(lambda self, q=mount_quat: {} if q is None else {"right": q}),
            default_controller_config=property(lambda self: {"right": "default_panda"}),
            init_qpos=property(lambda self, q=home: q),
            base_xpos_offset=property(lambda self, o=offsets: o),
            top_offset=property(lambda self: np.array((0.0, 0.0, 1.0))),
            _horizontal_radius=property(lambda self, r=reach: r),
            __init__=lambda self, idn=0, p=path: ManipulatorModel.__init__(self, p, idn=idn),
        )
        cls = type(ManipulatorModel)(name, (ManipulatorModel,), namespace)  # the metaclass registers the model
        register_robot_class("FixedBaseRobot")(cls)
        return name


def controller_config(control: ControlConfig) -> dict:
    """robosuite composite controller config for ``joint_pos_delta`` control with ``InertiaScaledGains``.

    robosuite's joint-position controller commands ``M (kp e - kd qd)`` plus gravity compensation, with
    ``kp = w^2`` and ``kd = 2 z w``: our inertia-scaled gain law with ``w = bandwidth`` and ``z = damping_ratio``.
    Differences that remain: robosuite re-evaluates ``M`` at every step (ours is taken at ``home_qpos``), does not
    subtract the joint damping of the model from ``kd``, and compensates gravity, which the native position servo
    does not.

    The controller takes absolute joint targets. The environment keeps the accumulated target
    (``q_target += action * action_scale``, clipped to the joint range), exactly as the native backend does, so one
    ``ControlConfig`` means the same action on both backends. (robosuite's own delta input would add the increment
    to the current position instead, and the arm would settle far below the commanded rate.)
    """
    gains = control.gains
    if control.mode != "joint_pos_delta" or not isinstance(gains, InertiaScaledGains):
        raise ValueError("only joint_pos_delta with InertiaScaledGains maps onto robosuite's joint controller")
    kp = gains.bandwidth**2
    arm_part = {
        "type": "JOINT_POSITION",
        "input_type": "absolute",
        "input_max": 1000.0,  # joint targets in radians; the action limits are ours
        "input_min": -1000.0,
        "output_max": control.action_scale,
        "output_min": -control.action_scale,
        "kp": kp,
        "damping_ratio": gains.damping_ratio,
        "impedance_mode": "fixed",
        "kp_limits": [0, max(300.0, kp)],
        "damping_ratio_limits": [0, max(10.0, gains.damping_ratio)],
        "qpos_limits": None,
        "interpolation": None,
        "ramp_ratio": 0.2,
        "gripper": {"type": "GRIP"},
    }
    return {"type": "BASIC", "body_parts": {"right": arm_part}}
