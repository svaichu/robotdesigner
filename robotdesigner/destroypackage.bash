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
cd ~/$ws_name
sudo rm -R src/$package_name build install log