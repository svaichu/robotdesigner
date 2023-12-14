#!/usr/bin/env bash

package_name="first"
ws_name="my_ws"

if [[ -n "$1" ]]; then
    package_name=$1
else
    package_name="robot"
    echo "no input, using default ws $ws_name"
fi

source /opt/ros/humble/setup.bash
cd ~
mkdir -p $ws_name/src
cd $ws_name/src
[ ! -d "$package_name" ] && ros2 pkg create $package_name --build-type ament_python --dependencies rclpy
cd $package_name
mkdir -p urdf
mkdir -p rviz
mkdir -p launch
cd ~/$ws_name