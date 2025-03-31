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
docker build -t kpi-rover . -f docker/Dockerfile.sim
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

Note: RPI, BBB and PC should be connected to the common network.

### To set up automated startup on power-on(RPI):

**Install udev rules for the lidar**

```bash
cd ~/ros_ws/src/lidar_coin_d4a
sudo cp sc_mini.rules /etc/udev/rules.d
```

**Build Docker**

```bash
cd ~/ros_ws
docker build -t kpi-rover . -f docker/Dockerfile.rpi
```

**Build Project**
```bash
docker run --rm -it \
  --user $(id -u):$(id -g) \
  -v $(pwd):/workspace \
  -w /workspace \
  kpi-rover colcon build
```

**Create system services**

1. For DDS server to establish communication between nodes through the network
- Create /etc/systemd/system/fastdds.service system service file with the following contents. You need sudo permission to do this.

```
[Unit]
After=network-online.target
Description=DDS server for ROS2 node comunication through network

[Service]
User=vik
Group=vik
ExecStart=fastdds discovery -i 0
Restart=always
RestartSec=5s

[Install]
WantedBy=default.target

```
- Enable and start the service.

```bash
sudo systemctl enable fastdds.service
sudo systemctl start fastdds.service
```


2. For running ROS nodes
- Create /etc/systemd/system/kpi-rover.service system service file with the following contents. You need sudo permission to do this.

```
[Unit]
After=network-online.target
After=docker.service
Requires=docker.service
After=fastdds.service
Description=ROS2 autonomous robot services

[Service]
User=<user>
Group=<group>
ExecStart= bash -c "${HOME}/ros_ws/run.sh -b <bbb_ip>"
TimeoutStartSec=0
Restart=always
RestartSec=2s

[Install]
WantedBy=default.target

```

- Enable and start the service.

```bash
sudo systemctl enable kpi-rover.service
sudo systemctl start kpi-rover.service
```



### To run visualization on your host machine:
```bash
export ROS_DISCOVERY_SERVER=<ip of the RPI>:11811
```
```bash
cd ~/ros_ws
source install/setup.bash
ros2 launch kpi_rover launch_visualization.launch.py 
```

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
