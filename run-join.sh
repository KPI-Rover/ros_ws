#!/bin/bash

docker exec -it kpi-rover \
bash -c "source /opt/ros/jazzy/setup.bash \
    && source install/setup.bash \
    && bash"

