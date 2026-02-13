# KPI Rover ROS2 Software
<!-- The markdown-toc utilitity is used to generate the Table of Contents -->
<!-- Installation: npm install -g markdown-toc -->
<!-- Usage: markdown-toc -i README.md -->

<!-- toc -->

- [Get Source Code](#get-source-code)
- [Build and Launch Simulation Using Docker](#build-and-launch-simulation-using-docker)
- [Build and Launch Simulation Without Docker](#build-and-launch-simulation-without-docker)
- [Setup RPI](#setup-rpi)
  * [Update bootloader](#update-bootloader)
  * [Enable UARTs](#enable-uarts)
  * [Install udev rules for the lidar**](#install-udev-rules-for-the-lidar)
- [Build and Launch on RPI](#build-and-launch-on-rpi)
  * [On RPI5 over ssh](#on-rpi5-over-ssh)
  * [On PC](#on-pc)

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

> ❗ **Important:** Run all next commands from the root of the ROS workspace (`ros_ws` folder) unless specified otherwise.

## Build and Launch Simulation Using Docker

❗ **Important:**  All pull requests must include source code that has been built and executed using Docker.
CI integration following this approach will be added soon.

**Build Docker**
```bash
docker compose build
```

**Build Project**
```bash
docker compose run builder
```

**Launch Simulation**

```bash
docker compose up sim
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

## Setup RPI

### Update bootloader

Mandatory because there is bug: https://github.com/raspberrypi/rpi-eeprom/issues/514

```
sudo apt update
sudo apt install rpi-eeprom

sudo rpi-eeprom-update -a
```

### Enable UARTs
**Enable uart0 and uart2**
```bash
 sudo nano /boot/firmware/config.txt
```
Add the following line:
```
[pi5]
dtoverlay=uart0-pi5
dtoverlay=uart2-pi5

```

**Disable serial console on uart0 interface.**
```
sudo nano /boot/firmware/cmdline.txt
```

Remove the following text from file
```
console=serial0,115200
```
### Install udev rules for the lidar**
```bash
cd ~/ros_ws/src/lidar_coin_d4a
sudo cp sc_mini.rules /etc/udev/rules.d
```

## Build and Launch on RPI

> ❗ **Important:** RPI and PC should be connected to the same network.

> ℹ️ **Note:** Currently we build Docker images and software right on the RPI. In the future, we are planning to build Docker images for the RPI on a host and push them to the RPI.

### On RPI5 over ssh

**Clone Repository**
```bash
git clone git@github.com:KPI-Rover/ros_ws.git
cd ros_ws
```

**Install Packages Using vcstool**
```bash
vcs import src < kpi-rover.repos
```
**Save environment variables for docker compose**
```bash
echo "UID=$(id -u)" > .env
echo "GID=$(id -g)" >> .env
```

**Build Docker image**
```bash
docker compose build hw
```

**Start Docker container**
```bash
docker compose run --rm hw
```
> ℹ️ **Note:** All next commands perform inside Docker container.

**Build and lunch ROS2 project**
```bash
colcon build

source install/setup.bash

ros2 launch kpi_rover launch_hw.launch.py
```

### On PC
**Clone Repository**
```bash
git clone git@github.com:KPI-Rover/ros_ws.git
cd ros_ws
```

**Install Packages Using vcstool**
```bash
vcs import src < kpi-rover.repos
```
**Save environment variables for docker compose**
```bash
echo "UID=$(id -u)" > .env
echo "GID=$(id -g)" >> .env
```

**Build Docker image**
```bash
docker compose build pc
```

**Start Docker container**
```bash
docker compose run --rm pc
```

**Lunch Rviz in Docker**
```bash
rviz2
```



**Start Keyboard Control**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args \
-r /cmd_vel:=/diff_drive_controller/cmd_vel \
-p stamped:=True \
-p frame_id:=base_link
```
 [TODO] : How to use gamepad