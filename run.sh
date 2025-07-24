#!/bin/bash

show_help () {
        cat << EOH
Usage: $0 -w <path/to/ros_ws> -b <bbb_ip>
Run kpi_rover on real hardware.

Options:
        -w      path to ros_ws folder
        -b	BBB IP-address
        -u      UDP port to listen IMU data to
        -l      log verbosity level: debug, info, error, fatal
        -h      print usage
EOH
}



ws_path="$HOME/ros_ws"
bbb_ip=""
udp_port_opt=""
log_level_opt=""


while getopts 'w:b:u:l:h' opt
do
        case $opt in
                h) show_help; exit 0;;
                w) ws_path=$OPTARG;;
                b) bbb_ip=$OPTARG;;
                u) udp_port_opt="udp_port:=$OPTARG";;
                l) log_level_opt="log_level:=$OPTARG";;
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
 --device=/dev/sc_mini --device=/dev/video0 kpi-rover bash -c "source /opt/ros/jazzy/setup.bash \
&& source install/setup.bash \
&& ros2 launch kpi_rover launch_irl_rpi.launch.py ecu_ip:=$bbb_ip $udp_port_opt $log_level_opt"
