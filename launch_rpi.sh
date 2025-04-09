#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash
source install/setup.bash

fastdds discovery -i 0 &

ros2 launch kpi_rover launch_irl.launch.py
