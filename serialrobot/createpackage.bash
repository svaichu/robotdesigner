#!/usr/bin/env bash

package_name="first"
ws_name="my_ws

source /opt/ros/humble/setup.bash
cd ~
mkdir -p $ws_name/src
cd $ws_name/src
[ ! -d "$package_name" ] && ros2 pkg create $package_name --build-type ament_python --dependencies rclpy
cd $package_name
echo "entered into directory"
mkdir -p urdf
mkdir -p rviz
cd ~/$ws_name