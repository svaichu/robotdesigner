#!/usr/bin/env bash

package_name="first"

source /opt/ros/humble/setup.bash
cd ~
mkdir -p ros2_ws/src
cd ros2_ws/src
[ ! -d "$package_name" ] && ros2 pkg create $package_name --build-type ament_python --dependencies rclpy
cd $package_name
echo "entered into directory"
mkdir -p urdf
mkdir -p rviz
cd ~/ros2_ws