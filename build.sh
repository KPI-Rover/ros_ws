#!/bin/bash

set -e

docker run --rm -it \
  --user $(id -u):$(id -g) \
  -v $(pwd):/workspace \
  -w /workspace \
  kpi-rover colcon build
