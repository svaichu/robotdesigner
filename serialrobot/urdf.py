

header = """
<?xml version="1.0"?>
<robot name="robot">"""
footer = """
    <material name="yellow">
        <color rgba="1 0.83 0 1"/>
    </material>
    
    <material name="blue">
        <color rgba="0.18 0.16 0.31 1"/>
    </material>
    
    <material name="pink">
        <color rgba="0.85 0.01 0.41 1"/>
    </material>
    
    <material name="purple">
        <color rgba="0.51 0.01 0.39 1"/>
    </material>
</robot>"""
template_link = """
    <link name="{name}">
        <visual>
            <geometry>
                <box size="{len} {breadth} {height}"/>
            </geometry>
            <origin rpy="{r} {p} {y}" xyz="{x} {y} {z}"/>
            <material name="{color}"/>
        </visual>
    </link>
"""
template_joint ="""
    <joint name="{name}" type="{type}">
        <parent link="{parent}"/>
        <child link="{child}"/>
        <origin rpy="{r} {p} {y}" xyz="{x} {y} {z}"/>
        <axis xyz="{ax} {ay} {az}"/>
        <limit effort="1000.0" lower="-3.14" upper="3.14" velocity="0.5"/>
    </joint>
""" 
def urdf_write(self, robot_order):
    all_links = [template_link.format(**link) for link in robot_order]
    all_joints = [template_joint.format(**link) if link.isLast is not True else None for link in robot_order]
    with open(file, "w") as f:
        f.write(self.header)
        f.write(all_links)
        f.write(all_joints)



# 1 create ws and src
# 2 create package
# 3 generate URDF
# 4 generate rviz
# 5 generate launch.py
# colcon build
