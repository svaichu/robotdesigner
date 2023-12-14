#!/usr/bin/env bash

package_name="first"
ws_name="my_ws"

if [[ -n "$1" ]]; then
    package_name=$1
else
    package_name="robot"
    echo "no input, using default ws $ws_name"
fi

cp view.rviz ~/$ws_name/src/$package_name/rviz/