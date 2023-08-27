#!/bin/bash

#source /opt/ros/foxy/setup.bash

#colcon build

source install/setup.bash

cd launch

xpra start --bind-tcp=0.0.0.0:10000 --mdns=no --ssl=off --webcam=no --no-daemon --start "ros2 launch con_launch.py"

