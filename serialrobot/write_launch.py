

from pathlib import Path


header_import = """
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
"""

start_func = """
def generate_launch_description():
    declared_arguments = []
    nodes_to_start = []
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'robot.urdf'
    urdf = os.path.join(
        get_package_share_directory('{name}'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    """

add_description_package_argument = """
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="{name}",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="robot.urdf",
            description="URDF/XACRO description file with the robot.",
        )
    )
    """

open_rviz_config = """
    description_package = LaunchConfiguration("description_package")
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )
    """


add_jont_state_publisher_gui_node = """
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    nodes_to_start.append(joint_state_publisher_node)
    """

add_robot_state_publisher_node = """
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[urdf])
    nodes_to_start.append(robot_state_publisher_node)
    """

add_rviz_node = """
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    nodes_to_start.append(rviz_node)    
    """

function_footer = """
    return LaunchDescription(declared_arguments + nodes_to_start)
    """

def writeLaunch(robot, launch_path):
    with open(launch_path, "w+") as f:
        f.write(header_import)
        f.write(start_func.format(**vars(robot)))
        f.write(add_description_package_argument.format(**vars(robot)))
        f.write(open_rviz_config)
        f.write(add_jont_state_publisher_gui_node)
        f.write(add_robot_state_publisher_node)
        f.write(add_rviz_node)
        f.write(function_footer)
