#!/bin/bash
docker run --rm -it \
  --name kpi_rover \
  --init --network=host \
  --user $(id -u):$(id -g) --group-add dialout --privileged \
  --device=/dev/ttyAMA2 \
  -v $(pwd):/workspace \
  -w /workspace \
  -e HOME=/workspace \
  -e ROS_DOMAIN_ID=1 \
  kpi-rover \
  bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch kpi_rover launch_hw.launch.py"
  