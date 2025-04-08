#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash
source install/setup.bash

fastdds discovery -i 0 &

ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/lidar
