# KPI Rover ROS2 Software
<!-- The markdown-toc utilitity is used to generate the Table of Contents -->
<!-- Installation: npm install -g markdown-toc -->
<!-- Usage: markdown-toc -i README.md -->
<!-- toc -->

- [KPI Rover ROS2 Software](#kpi-rover-ros2-software)
  - [Get Source Code](#get-source-code)
  - [Build and Launch Simulation Using Docker](#build-and-launch-simulation-using-docker)
  - [Build and Launch Simulation Without Docker](#build-and-launch-simulation-without-docker)
  - [Build and Launch on RPI](#build-and-launch-on-rpi)
  - [Other Notes](#other-notes)
    - [Start Keyboard Control](#start-keyboard-control)
    - [Launch CSPC Lidar Demo](#launch-cspc-lidar-demo)
    - [Start DDS Server](#start-dds-server)
    - [Launch in Docker when DDS is used](#launch-in-docker-when-dds-is-used)
  - [TODO List](#todo-list)

<!-- tocstop -->

## Get Source Code

**Clone Repository**
```bash
git clone git@github.com:KPI-Rover/ros_ws.git
cd ros_ws
```

**Install Packages Using vcstool**
```bash
vcs import src < kpi-rover.repos
```

❗ **Important:** Run all next commands from the root of the ROS workspace (`ros_ws` folder) unless specified otherwise.

## Build and Launch Simulation Using Docker

❗ **Important:**  All pull requests must include source code that has been built and executed using Docker.
CI integration following this approach will be added soon.

**Build Docker**
```bash
docker build -t kpi-rover .
```

**Build Project**
```bash
docker run --rm -it \
  --user $(id -u):$(id -g) \
  -v $(pwd):/workspace \
  -w /workspace \
  kpi-rover colcon build
```

**Launch Simulation**

```bash
docker run --rm -it \
  --init \
  --network=host \
  --user $(id -u):$(id -g) \
  -v $(pwd):/workspace \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/.gz:/home/ubuntu/.gz \
  -w /workspace \
  -e ROS_DOMAIN_ID=1 \
  -e GZ_PARTITION=1 \
  kpi-rover \
  bash -c "
    source /opt/ros/jazzy/setup.bash \
    && source install/setup.bash \
    && ros2 launch kpi_rover launch_sim.launch.py"
```

## Build and Launch Simulation Without Docker

Gazebo runs very slowly in Docker. Because of this, we need to run the project locally to make it work faster.

To avoid problems with missing packages, always add new packages to the package.xml file. This helps to install them easily using the rosdep tool.

**Install ROS Dependencies**
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**Source ROS2 (If Not Added to `.bashrc`)**
```bash
source /opt/ros/jazzy/setup.bash
```

**Build Project**
```bash
colcon build
```

**Launch Simulation**

```bash
source install/setup.bash
ros2 launch kpi_rover launch_sim.launch.py
```

## Build and Launch on RPI
TBD

## Other Notes
This section holds temporary, unorganized notes.

### Start Keyboard Control
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args \
-r /cmd_vel:=/diff_drive_base_controller/cmd_vel \
-p stamped:=True \
-p frame_id:=base_link
```

### Launch CSPC Lidar Demo
```bash
ros2 launch cspc_lidar demo.launch.py
```

### Start DDS Server
A DDS (Data Distribution Service) server facilitates real-time data exchange between distributed ROS2 nodes over the network. Using a DDS server ensures seamless communication between components, particularly when a PC has multiple network interfaces. It helps in optimizing discovery mechanisms and maintaining robust connectivity.

```bash
fastdds discovery -i 0
```

### Launch in Docker when DDS is used
```bash
docker run --rm -it \
  --init \
  --user $(id -u):$(id -g) \
  -v $(pwd):/workspace \
  -w /workspace \
  -e ROS_DISCOVERY_SERVER=172.17.0.1:11811 \
  -e ROS_DOMAIN_ID=1 \
  -e IGN_DISCOVERY_URI=tcp://172.17.0.1:11811 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/super_client_cfg_file.xml \
  -e GZ_PARTITION=1 \
  kpi-rover \
  bash -c "
    source /opt/ros/jazzy/setup.bash \
    && source install/setup.bash \
    && ros2 launch apricotka-robot-car launch_sim.launch.py"
```

## TODO List
- [ ] Add entrypoint to surce source install/setup.bash
- [ ] Create Docker container for RPI
- [ ] How to use gamepad
