#!/bin/bash
xhost +local:docker > /dev/null

docker run -it --rm \
  --name kpi-rover \
  --network=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri:/dev/dri \
  -v $(pwd):/workspace \
  -w /workspace \
  -e ROS_DOMAIN_ID=1 \
  kpi-rover \
  bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && rviz2"