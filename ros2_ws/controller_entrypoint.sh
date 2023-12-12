#!/bin/bash

#This file is what Docker runs to actually pull the GUI into a user interface
#since Docker does not have a GUI

source install/setup.bash

cd launch

xpra start --bind-tcp=0.0.0.0:10000 --mdns=no --ssl=off --webcam=no --no-daemon --start "ros2 launch con_launch.py"

