#!/bin/bash

#This shell script is run as a part of the "after_netplan_controller_launch_service". 
#This actually sources ROS2 workspace and properly runs the launchfile for the ROS nodes
#Written by Jacob Wildes

source /home/controller/Submarine_Capstone/ros2_ws/install/setup.bash
ros2 launch controller_gui con_launch.py

