#!/usr/bin/env bash

if [[ -n "$1" ]]; then
    ws_name=$1
else
    ws_name="my_ws"
    echo "no input, using default ws $ws_name"
fi

cd ~
mkdir -p $ws_name/src
cd ~/$ws_name
