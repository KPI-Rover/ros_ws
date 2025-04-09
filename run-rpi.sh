#!/bin/bash

set -e

# show_help () {
#         cat << EOH
# Usage: $0 -w <path/to/ros_ws> -b <bbb_ip>
# Run kpi_rover on real hardware.

# Options:
#         -w      specify path to ros_ws folder
#         -b	specify BBB IP-address
#         -h      print usage
# EOH
# }



ws_path="/home/ubuntu/ros_ws"
# bbb_ip=""


# while getopts 'w:b:h' opt
# do
#         case $opt in
#                 h) show_help; exit 0;;
#                 w) ws_path=$OPTARG;;
#                 b) bbb_ip=$OPTARG;;
#         esac
# done


# shift "$(( OPTIND - 1 ))"

# if [ -z "$bbb_ip" ]; then
#         echo '-b is mandatory' >&2
#         exit 1
# fi

echo "Using workspace path: $ws_path"

ifconfig

docker run --rm --name=kpi_rover --init \
 --network=host \
 --user $(id -u):$(id -g) \
 -v $(pwd):/workspace \
 -w /workspace \
 -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/super_client_cfg_file.xml \
 -e ROS_DOMAIN_ID=1 \
 --device=/dev/lidar \
 kpi-rover bash -c "./launch_rpi.sh"

# -c "source /opt/ros/jazzy/setup.bash \
# && source install/setup.bash \
# && ros2 launch kpi_rover launch_irl.launch.py ecu_ip:=$bbb_ip"