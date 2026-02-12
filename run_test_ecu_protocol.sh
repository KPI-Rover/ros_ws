#!/bin/bash
# Script to run the protocol stress test.
# Usage: ./run_test_ecu_protocol.sh [PORT] [SPEED] [ITERATIONS]

DEVICE=${1:-"/dev/ttyAMA2"}
SPEED=${2:-921600}
ITERATIONS=${3:-10000}

docker run --rm -it \
  --init --network=host \
  --user $(id -u):$(id -g) --group-add dialout --privileged \
  --device=$DEVICE \
  -v $(pwd):/workspace \
  -w /workspace \
  -e HOME=/workspace \
  kpi-rover \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source install/setup.bash && \
           ros2 run kpi_rover test_ecu_protocol --ros-args \
             -p serial_device:=$DEVICE \
             -p baud_rate:=$SPEED \
             -p load_test_iterations:=$ITERATIONS"
