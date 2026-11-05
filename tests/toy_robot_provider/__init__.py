"""A robot provider for the tests only: namespace ``toyrobot``, two small models written inline (no download).

``arm3`` is a 3-hinge arm with position actuators and a ``home`` keyframe; ``floater`` has a free root, two hinge legs with
torque actuators and a keyframe. ``tests/conftest.py`` registers the provider lazily; it shows what a robot provider looks like.
The engine side (``describe``, ``spec``, ``kinematics``) imports MuJoCo inside the methods, so importing this module needs
no engine.
"""

from __future__ import annotations

import hashlib
from typing import Any

import numpy as np

from robotdesigner.config.kinematics import pose_to_matrix
from robotdesigner.errors import ConfigError, Problem
from robotdesigner.robot import EndEffectorRef, ExternalJoint, ExternalRobot, Pose

ARM3 = """
<mujoco model="arm3">
  <compiler angle="radian"/>
  <worldbody>
    <body name="base">
      <body name="l1" pos="0 0 0.1">
        <joint name="j1" type="hinge" axis="0 0 1" range="-2 2"/>
        <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1"/>
        <body name="l2" pos="0.3 0 0">
          <joint name="j2" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
          <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1"/>
          <body name="l3" pos="0.3 0 0">
            <joint name="j3" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
            <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1"/>
            <site name="ee" pos="0.3 0 0"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="a1" joint="j1" kp="100" forcerange="-40 40"/>
    <position name="a2" joint="j2" kp="100" forcerange="-30 30"/>
    <position name="a3" joint="j3" kp="100" forcerange="-20 20"/>
  </actuator>
  <keyframe>
    <key name="home" qpos="0.1 -0.2 0.3"/>
  </keyframe>
</mujoco>
"""

FLOATER = """
<mujoco model="floater">
  <compiler angle="radian"/>
  <worldbody>
    <body name="root" pos="0 0 0.5">
      <freejoint name="root"/>
      <geom type="box" size="0.1 0.05 0.03" mass="2"/>
      <body name="leg1" pos="0.1 0.05 0">
        <joint name="hip1" type="hinge" axis="0 1 0" range="-1 1"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.2" size="0.01" mass="0.2"/>
      </body>
      <body name="leg2" pos="0.1 -0.05 0">
        <joint name="hip2" type="hinge" axis="0 1 0" range="-1 1"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.2" size="0.01" mass="0.2"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="m1" joint="hip1" ctrlrange="-1 1" gear="10"/>
    <motor name="m2" joint="hip2" ctrlrange="-1 1" gear="10"/>
  </actuator>
  <keyframe>
    <key name="home" qpos="0 0 0.5 1 0 0 0 0.1 0.2"/>
  </keyframe>
</mujoco>
"""

MODELS = {"arm3": ("arm3.xml", ARM3), "floater": ("floater.xml", FLOATER)}


def source_of(xml: str) -> str:
    return hashlib.sha256(xml.encode()).hexdigest()[:16]


class ToyRobots:
    """The robot provider: ``describe`` reads the model, ``spec`` opens it, ``kinematics`` is a MuJoCo oracle."""

    def names(self) -> tuple[str, ...]:
        return tuple(MODELS)

    def describe(self, name: str, **options: Any) -> ExternalRobot:
        import mujoco

        if name not in MODELS:
            raise ConfigError("name", f"unknown toy robot {name!r}; known: {list(MODELS)}")
        entry, xml = MODELS[name]
        model = mujoco.MjModel.from_xml_string(xml)
        joints, kinds = [], set()
        for a in range(model.nu):
            j = int(model.actuator_trnid[a, 0])
            lo, hi = model.jnt_range[j] if model.jnt_limited[j] else (None, None)
            effort = float(np.abs(model.actuator_forcerange[a]).max()) if model.actuator_forcelimited[a] else float(
                np.abs(model.actuator_ctrlrange[a]).max() * model.actuator_gear[a, 0]
            )
            joints.append(ExternalJoint(
                name=mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j),
                range=None if lo is None else (float(lo), float(hi)),
                effort_limit=effort,
            ))
            bias = model.actuator_biasprm[a]
            kinds.add("position" if bias[1] < 0 else "velocity" if bias[2] < 0 else "torque")
        key = model.key_qpos[0]
        qpos = [float(key[model.jnt_qposadr[int(model.actuator_trnid[a, 0])]]) for a in range(model.nu)]
        free = any(model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in range(model.njnt))
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee")
        return ExternalRobot(
            provider="toyrobot", name=name, entry=entry, source=source_of(xml), joints=tuple(joints),
            home_qpos=tuple(qpos),
            home_root=Pose(tuple(float(x) for x in key[:3]), tuple(float(x) for x in key[3:7])) if free else None,
            keyframe="home",
            end_effector=None if site < 0 else EndEffectorRef(site="ee", body=None, offset=Pose.identity()),
            actuator_kind=kinds.pop() if len(kinds) == 1 else "mixed",
        )

    def spec(self, robot: ExternalRobot) -> Any:
        import mujoco

        xml = MODELS[robot.name][1]
        if source_of(xml) != robot.source:
            raise ConfigError("robot.source", f"the model of {robot.name!r} is not the one the config was made from")
        return mujoco.MjSpec.from_string(xml)

    def validate(self, robot: ExternalRobot, simulator: str) -> list[Problem]:
        if simulator != "mujoco":
            return [Problem("sim.simulator", f"toyrobot robots run on 'mujoco', not on {simulator!r}")]
        return []

    def kinematics(self, robot: ExternalRobot) -> "ToyKinematics":
        return ToyKinematics(robot)


class ToyKinematics:
    """``forward_ee_batch(q, base)`` and ``sample_joint_positions(num, rng)`` from the compiled model."""

    def __init__(self, robot: ExternalRobot):
        import mujoco

        self._mj = mujoco
        self.robot = robot
        self.model = mujoco.MjModel.from_xml_string(MODELS[robot.name][1])
        self.data = mujoco.MjData(self.model)
        self.adr = [int(self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j.name)])
                    for j in robot.joints]
        self.site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, robot.end_effector.site)

    def forward_ee_batch(self, q, base: Pose | None = None) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        T = np.eye(4) if base is None else pose_to_matrix(base)
        out = np.empty((len(q), 3))
        for i, qi in enumerate(q):
            self.data.qpos[:] = self.model.key_qpos[0]
            self.data.qpos[self.adr] = qi
            self._mj.mj_kinematics(self.model, self.data)
            out[i] = T[:3, :3] @ self.data.site_xpos[self.site] + T[:3, 3]
        return out

    def sample_joint_positions(self, num: int, rng: np.random.Generator) -> np.ndarray:
        lo = np.array([-np.pi if j.range is None else j.range[0] for j in self.robot.joints])
        hi = np.array([np.pi if j.range is None else j.range[1] for j in self.robot.joints])
        return rng.uniform(lo, hi, size=(num, self.robot.n))


PROVIDER = ToyRobots()
