#!/bin/bash

#source /opt/ros/foxy/setup.bash

#colcon build

source install/setup.bash

cd launch

ros2 launch sub_launch.py

