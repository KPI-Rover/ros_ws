ARG ROS_DISTRO=jazzy

FROM osrf/ros:${ROS_DISTRO}-desktop

# Install dependencies for rosdep and sudo
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    net-tools \
    iputils-ping \
    nano \
    vim

WORKDIR /workspace

COPY src /workspace/src
RUN rosdep install --from-paths /workspace/src --ignore-src -r -y
RUN rm -rf /workspace/src
