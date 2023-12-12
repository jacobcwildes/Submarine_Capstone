#!/bin/bash

#This shell script is responsible for sourcing and launching the ROS workspace
#after the "after_netplan_sub_launch.service" executes
#Written by Jacob Wildes

source /home/ubuntu/Submarine_Capstone/ros2_ws/install/setup.bash
ros2 launch submarine_coms sub_launch.py
