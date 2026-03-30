"""
Robot arm MuJoCo XML composer.

Builds N-DOF robot arm models by chaining base + link fragments
using MjSpec attachment (MuJoCo 3.x).

Usage:
    from robotdesigner.mujoco.composer import build_arm, save_arm
    spec = build_arm(ndof=3, control="pos")
    spec = build_arm(ndof=3, control="pos", joint_orientation=["z", "y", "z"])
    save_arm(spec, ndof=3, control="pos", joint_orientation=["z", "y", "z"])

    # Or from CLI:
    python composer.py --ndof 3 --control pos
    python composer.py --ndof 3 --control pos --joint-orientation z y z
"""

import mujoco
import numpy as np
from pathlib import Path

FRAGMENTS_DIR = Path(__file__).parent / "assets" / "fragments"
ASSETS_DIR = Path(__file__).parent / "assets"

# Default joint orientations: alternate z (yaw) and y (pitch)
DEFAULT_JOINT_ORIENTATIONS = ["z", "y"]

# PD gains computed for critically-damped response
POS_KP = 403.74
POS_KV = 37.37

# Torque motor gear and control range
TORQUE_GEAR = 200.0

SCENE_XML = """
<mujoco model="scene">
  <compiler angle="radian" inertiafromgeom="true"/>
  <option integrator="RK4" timestep="0.002"/>
  <visual>
    <map znear="0.02"/>
  </visual>
  <worldbody>
    <light cutoff="100" diffuse="1 1 1" dir="-0 0 -1.3" directional="true"
           exponent="1" pos="0 0 1.3" specular=".1 .1 .1"/>
    <geom conaffinity="1" condim="3" name="floor" pos="0 0 0"
          rgba="0.8 0.9 0.8 1" size="20 20 .125" type="plane" material="MatPlane"/>
  </worldbody>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1=".4 .5 .6" rgb2="0 0 0"
             width="100" height="100"/>
    <texture builtin="flat" height="1278" mark="cross" markrgb="1 1 1"
             name="texgeom" random="0.01" rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4"
             type="cube" width="127"/>
    <texture builtin="checker" height="100" name="texplane" rgb1="0 0 0"
             rgb2="0.8 0.8 0.8" type="2d" width="100"/>
    <material name="MatPlane" reflectance="0.5" shininess="1" specular="1"
              texrepeat="60 60" texture="texplane"/>
    <material name="geom" texture="texgeom" texuniform="true"/>
  </asset>
</mujoco>
"""

_AXIS_MAP = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}


def _make_link_spec(orientation: str) -> mujoco.MjSpec:
    """Load link fragment and set joint axis from 'z' or 'y' orientation."""
    spec = mujoco.MjSpec.from_file(str(FRAGMENTS_DIR / "link.xml"))
    joint = spec.worldbody.first_body().first_joint()
    joint.axis = _AXIS_MAP[orientation.strip().lower()]
    return spec


def _add_pos_actuator(spec: mujoco.MjSpec, joint_name: str, act_name: str):
    act = spec.add_actuator()
    act.name = act_name
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.target = joint_name
    act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
    act.gainprm[0] = POS_KP
    act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    act.biasprm[1] = -POS_KP
    act.biasprm[2] = -POS_KV
    act.ctrllimited = True
    act.ctrlrange = [-np.pi, np.pi]


def _add_torque_actuator(spec: mujoco.MjSpec, joint_name: str, act_name: str):
    act = spec.add_actuator()
    act.name = act_name
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.target = joint_name
    act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
    act.gainprm[0] = TORQUE_GEAR
    act.biastype = mujoco.mjtBias.mjBIAS_NONE
    act.ctrllimited = True
    act.ctrlrange = [-1.0, 1.0]


def build_arm(
    ndof: int,
    control: str = "pos",
    joint_orientation: list[str] | None = None,
) -> mujoco.MjSpec:
    """
    Build an N-DOF robot arm model.

    Args:
        ndof:              Number of joints / degrees of freedom.
        control:           "pos" for position control, "torque" for torque control.
        joint_orientation: List of N orientations, each "z" or "y".
                           If shorter than ndof, cycles through the list.
                           Defaults to alternating z/y.

    Returns:
        mujoco.MjSpec of the complete model.
    """
    assert ndof >= 1, "ndof must be >= 1"
    assert control in ("pos", "torque"), "control must be 'pos' or 'torque'"

    orientations = joint_orientation if joint_orientation is not None else DEFAULT_JOINT_ORIENTATIONS

    # Start with scene (floor, lights, assets, options)
    spec = mujoco.MjSpec.from_string(SCENE_XML)

    # Attach base to worldbody via a frame
    base_spec = mujoco.MjSpec.from_file(str(FRAGMENTS_DIR / "base.xml"))
    frame = spec.worldbody.add_frame()
    spec.attach(base_spec, frame=frame, prefix="")

    # Navigate to base body's tip site
    current_body = spec.worldbody.first_body()  # 'base'
    current_tip = current_body.first_site()     # 'tip'

    for i in range(ndof):
        prefix = f"link{i + 1}_"
        orientation = orientations[i % len(orientations)]
        link_spec = _make_link_spec(orientation)
        spec.attach(link_spec, site=current_tip, prefix=prefix)

        # Add actuator for this joint
        joint_name = f"{prefix}joint"
        act_name = f"{prefix}act"
        if control == "pos":
            _add_pos_actuator(spec, joint_name, act_name)
        else:
            _add_torque_actuator(spec, joint_name, act_name)

        # Move to the newly attached link's tip site for next iteration
        current_body = current_body.first_body()
        current_tip = current_body.first_site()

    # Mark final tip as end-effector frame
    current_tip.name = current_tip.name.replace("tip", "ee_frame")
    current_tip.rgba = [0, 1, 0, 1]

    spec.modelname = f"{ndof}_arm_{control}"

    return spec


def save_arm(
    spec: mujoco.MjSpec,
    ndof: int,
    control: str = "pos",
    joint_orientation: list[str] | None = None,
    output_dir: Path = ASSETS_DIR,
) -> Path:
    """Save MjSpec arm to XML file in assets directory."""
    name_map = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five"}
    prefix = name_map.get(ndof, str(ndof))

    if joint_orientation is not None:
        axes_str = "".join(o.strip().lower() for o in joint_orientation)
    else:
        default = DEFAULT_JOINT_ORIENTATIONS
        axes_str = "".join(default[i % len(default)] for i in range(ndof))

    filename = output_dir / f"{prefix}_arm_{control}_{axes_str}.xml"

    xml = spec.to_xml()
    xml = xml.replace("<default/>\n", "")
    filename.write_text(xml)
    print(f"Saved: {filename}")
    return filename


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Compose robot arm MuJoCo XML")
    parser.add_argument("--ndof", type=int, required=True, help="Number of DOF")
    parser.add_argument("--control", choices=["pos", "torque"], default="pos")
    parser.add_argument(
        "--joint-orientation", nargs="+", metavar="AXIS",
        help="Joint orientations: z or y per joint. Cycles if fewer than ndof. "
             "Example: --joint-orientation z y z",
    )
    parser.add_argument("--output", type=str, default=None,
                        help="Output file path (overrides default naming)")
    args = parser.parse_args()

    spec = build_arm(args.ndof, args.control, args.joint_orientation)
    if args.output:
        xml = spec.to_xml().replace("<default/>\n", "")
        Path(args.output).write_text(xml)
        print(f"Saved: {args.output}")
    else:
        save_arm(spec, args.ndof, args.control, args.joint_orientation)
