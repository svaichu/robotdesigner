"""
Robot arm MuJoCo XML composer.

Builds N-DOF robot arm models by chaining base + link fragments
using MjSpec attachment (MuJoCo 3.x).

Usage:
    from composer import build_arm
    xml = build_arm(ndof=3, control="pos")
    xml = build_arm(ndof=3, control="pos", joint_axes=[[0,0,1],[0,1,0],[1,0,0]])

    # Or from CLI:
    python composer.py --ndof 3 --control pos
    python composer.py --ndof 3 --control pos --joint-axes "0 0 1" "0 1 0" "1 0 0"
"""

import mujoco
import numpy as np
from pathlib import Path
from typing import Sequence

FRAGMENTS_DIR = Path(__file__).parent / "assets" / "fragments"
ASSETS_DIR = Path(__file__).parent / "assets"

# Default joint axes: alternate z (yaw) and y (pitch)
DEFAULT_JOINT_AXES = ["z", "y"]

# PD gains computed for critically-damped response
POS_KP = 403.74
POS_KV = 37.37

# Torque motor gear and control range
TORQUE_GEAR = 200.0
TORQUE_CTRLRANGE = "-1.0 1.0"

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


_AXIS_SHORTCUTS = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}

def _parse_axis(axis) -> list:
    """Accept axis as 'x'/'y'/'z', [x,y,z] list/tuple, or 'x y z' string."""
    if isinstance(axis, str):
        key = axis.strip().lower()
        if key in _AXIS_SHORTCUTS:
            return _AXIS_SHORTCUTS[key]
        return [float(x) for x in axis.split()]
    return [float(x) for x in axis]


def _make_link_spec(axis) -> mujoco.MjSpec:
    """Load link fragment and set joint axis."""
    spec = mujoco.MjSpec.from_file(str(FRAGMENTS_DIR / "link.xml"))
    joint = spec.worldbody.first_body().first_joint()
    joint.axis = _parse_axis(axis)
    return spec


def _add_pos_actuator(spec: mujoco.MjSpec, joint_name: str, act_name: str):
    act = spec.add_actuator()
    act.name = act_name
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.target = joint_name
    act.set_to_position(kp=POS_KP, kv=POS_KV)
    act.ctrllimited = True
    act.ctrlrange = [-np.pi, np.pi]


def _add_torque_actuator(spec: mujoco.MjSpec, joint_name: str, act_name: str):
    act = spec.add_actuator()
    act.name = act_name
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.target = joint_name
    act.set_to_motor()
    act.gainprm[0] = TORQUE_GEAR
    act.ctrllimited = True
    act.ctrlrange = [-1.0, 1.0]


def build_arm(
    ndof: int,
    control: str = "pos",
    joint_axes: Sequence | None = None,
) -> str:
    """
    Build an N-DOF robot arm model.

    Args:
        ndof:        Number of joints / degrees of freedom.
        control:     "pos" for position control, "torque" for torque control.
        joint_axes:  List of N axes, each as [x,y,z] or "x y z" string.
                     If shorter than ndof, cycles through the list.
                     Defaults to alternating z/y axes.

    Returns:
        MuJoCo XML string of the complete model.
    """
    assert ndof >= 1, "ndof must be >= 1"
    assert control in ("pos", "torque"), "control must be 'pos' or 'torque'"

    axes = joint_axes if joint_axes is not None else DEFAULT_JOINT_AXES

    # Start with scene (floor, lights, assets, options)
    spec = mujoco.MjSpec.from_string(SCENE_XML)

    # Attach base to worldbody via a frame
    base_spec = mujoco.MjSpec.from_file(str(FRAGMENTS_DIR / "base.xml"))
    frame = spec.worldbody.add_frame()
    spec.attach(base_spec, frame=frame, prefix="")

    # Walk the chain: each iteration finds the current tip and attaches next link
    # After attaching base, navigate to base body's tip site
    current_body = spec.worldbody.first_body()  # 'base'
    current_tip = current_body.first_site()     # 'tip'

    for i in range(ndof):
        prefix = f"link{i + 1}_"
        axis = axes[i % len(axes)]
        link_spec = _make_link_spec(axis)
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

    # Add end-effector site label at final tip
    current_tip.name = current_tip.name.replace("tip", "ee_frame")
    current_tip.rgba = [0, 1, 0, 1]

    # Set model name
    spec.modelname = f"{ndof}_arm_{control}"

    xml = spec.to_xml()
    # MjSpec emits a spurious empty <default/> child inside <default> — remove it
    xml = xml.replace("<default/>\n", "")
    return xml


def save_arm(
    ndof: int,
    control: str = "pos",
    joint_axes: Sequence | None = None,
    output_dir: Path = ASSETS_DIR,
):
    """Build arm and save to XML file."""
    xml = build_arm(ndof, control, joint_axes)
    name_map = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five"}
    prefix = name_map.get(ndof, f"{ndof}")
    if joint_axes is not None:
        axes_str = "_".join(str(a).strip().lower() for a in joint_axes)
        filename = output_dir / f"{prefix}_arm_{control}_{axes_str}.xml"
    else:
        filename = output_dir / f"{prefix}_arm_{control}.xml"
    filename.write_text(xml)
    print(f"Saved: {filename}")
    return filename


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Compose robot arm MuJoCo XML")
    parser.add_argument("--ndof", type=int, required=True, help="Number of DOF")
    parser.add_argument("--control", choices=["pos", "torque"], default="pos")
    parser.add_argument(
        "--joint-axes", nargs="+", metavar="AXIS",
        help='Joint axes per joint: use x/y/z shorthand or "x y z" vector. '
             'Cycles if fewer than ndof. Example: --joint-axes z y z',
    )
    parser.add_argument("--output", type=str, default=None,
                        help="Output file path (default: assets/<name>_arm_<control>.xml)")
    args = parser.parse_args()

    xml = build_arm(args.ndof, args.control, args.joint_axes)
    if args.output:
        Path(args.output).write_text(xml)
        print(f"Saved: {args.output}")
    else:
        save_arm(args.ndof, args.control, args.joint_axes)
