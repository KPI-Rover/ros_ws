#!/bin/bash

show_help () {
        cat << EOH
Usage: $0 -w <path/to/ros_ws> -b <bbb_ip>
Run kpi_rover on real hardware.

Options:
        -w      specify path to ros_ws folder
        -b	specify BBB IP-address
        -u      specify UDP port to listen IMU data to
        -h      print usage
EOH
}



ws_path="$HOME/ros_ws"
bbb_ip=""
udp_port_opt=""


while getopts 'w:b:h' opt
do
        case $opt in
                h) show_help; exit 0;;
                w) ws_path=$OPTARG;;
                b) bbb_ip=$OPTARG;;
                u) udp_port="udp_port:=$OPTARG";;
        esac
done


shift "$(( OPTIND - 1 ))"

if [ -z "$bbb_ip" ]; then
        echo '-b is mandatory' >&2
        exit 1
fi

docker run --rm --name=kpi_rover --user root --init --network=host \
 -v $ws_path:/workspace -w /workspace \
 -e ROS_DISCOVERY_SERVER=172.17.0.1:11811 -e ROS_DOMAIN_ID=1 \
 -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/super_client_cfg_file.xml \
 --cap-add=sys_nice \
 --ulimit rtprio=99 \
 --ulimit memlock=-1 \
 --device=/dev/sc_mini --device=/dev/video0 kpi-rover bash -c "source /opt/ros/jazzy/setup.bash \
&& source install/setup.bash \
&& ros2 launch kpi_rover launch_irl.launch.py ecu_ip:=$bbb_ip $udp_port_opt"
