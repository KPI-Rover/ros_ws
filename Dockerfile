ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}

# Install dependencies for rosdep and sudo
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    python3-rosdep 

WORKDIR /workspace
COPY src /workspace/src
RUN rosdep install --from-paths /workspace/src --ignore-src -r -y
# Install ROS dependencies

RUN rm -rf /workspace/src
