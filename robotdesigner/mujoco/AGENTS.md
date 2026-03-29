
Generate a robot arm mujoco xml file

Args:
- ndof: int, number of degrees of freedom or number of joints in the robot arm
- control: str, either "pos" for position control or "torque" for torque control
- joint_orientation: [str], list of joint orientations, each being either "z" or "y". Length should be equal to ndof. "z" means the joint rotates about the z-axis, and "y" means the joint rotates about the y-axis. If not provided, default first to "z" then "y" for all joints.

Returns:
- xml_str: str, the generated mujoco xml string for the robot arm. 

Save the xml file. Name the file based on the number of joints, control type, and joint orientations. For example, a 3-joint arm with torque control and joint orientations ["z", "y", "z"] would be saved as "three_arm_torque_zyz.xml".

Args:
- xml_str: str, the mujoco xml string for the robot arm

Return:
- None, but saves the xml file to the assets directory.

Some additional assumptions:
1. The robot arm base is fixed.
2. Each link has a length of 1 unit and a mass of 1 unit.
