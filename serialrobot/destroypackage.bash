#!/usr/bin/env bash

package_name="first"
ws_name="my_ws"

source /opt/ros/humble/setup.bash
cd ~/$ws_name
sudo rm -R src/$package_name build install log