#!/usr/bin/env bash

package_name="first"
ws_name="my_ws"

if [[ -n "$1" ]]; then
    ws_name=$1
else
    ws_name="my_ws"
    echo "no input, using default ws $ws_name"
fi

source /opt/ros/humble/setup.bash
cd ~/$ws_name
colcon build