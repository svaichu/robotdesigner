

header = """
<?xml version="1.0"?>
<robot name="{name}">"""
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
    <joint name="{jname}" type="{jtype}">
        <parent link="{parent.name}"/>
        <child link="{name}"/>
        <origin rpy="0 0 0" xyz="{jx} {jy} {jz}"/>
        <axis xyz="{ax} {ay} {az}"/>
        <limit effort="1000.0" lower="-3.14" upper="3.14" velocity="0.5"/>
    </joint>
""" 
def urdf_write(self, robot):
    all_links = [template_link.format(**link) for link in robot.links]
    all_joints = [template_joint.format(**link) if link.isLast is not True else None for link in robot.links]
    header = header.format(**robot)
    with open(robot.urdf_path, "w") as f:
        f.write(header)
        f.write(all_links)
        f.write(all_joints)
