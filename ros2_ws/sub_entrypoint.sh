#!/bin/bash

#This entrypoint allows the Dockerfile for the sub to launch the submarine properly

source install/setup.bash

cd launch

ros2 launch sub_launch.py

