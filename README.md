The vcstool is used to simplify package installation.
https://github.com/dirk-thomas/vcstool

## Get Source Code

### Clone Repository
```bash
git clone git@github.com:KPI-Rover/ros_ws.git
cd ros_ws
```

### Install Packages Using vcstool
```bash
vcs import src < kpi-rover-ros-ws-sim.repos
```

## Build and Launch Simulation Using Docker

This is the preferred approach for working with the project.  
All pull requests must be submitted with the source code built and executed using Docker.  
CI integration based on this approach will be added soon.

### Build Docker
```bash
docker build -t kpi-rover .
```

### Build Project
```bash
docker run --rm -it \
  --user $(id -u):$(id -g) \
  -v $(pwd):/workspace \
  -w /workspace \
  kpi-rover colcon build

exit
```

### Launch Simulation

#### Start DDS Server
A DDS (Data Distribution Service) server facilitates real-time data exchange between distributed ROS2 nodes over the network. Using a DDS server ensures seamless communication between components, particularly when a PC has multiple network interfaces. It helps in optimizing discovery mechanisms and maintaining robust connectivity.

**Terminal 1**
```bash
fastdds discovery -i 0
```

#### Launch no UI Nodes
**Terminal 2**
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
- `--rm`: Removes the container once it exits.
- `-it`: Runs interactively with a terminal.
- `--init`: Ensures proper handling of child processes.
- `--user $(id -u):$(id -g)`: Runs the container as the current user.
- `-v $(pwd):/workspace`: Mounts the current directory to `/workspace` inside the container.
- `-w /workspace`: Sets the working directory inside the container.
- Environment variables:
  - `ROS_DISCOVERY_SERVER`: Specifies the DDS discovery server's address.
  - `ROS_DOMAIN_ID`: Defines the ROS2 domain to avoid conflicts.
  - `IGN_DISCOVERY_URI`: Specifies the Gazebo discovery server.
  - `FASTRTPS_DEFAULT_PROFILES_FILE`: Points to the Fast RTPS configuration file.
  - `GZ_PARTITION`: The GZ_PARTITION environment variable is used in Gazebo (GZ) to define a logical partition for simulations. It allows multiple instances of Gazebo to run independently on the same network, preventing interference between different simulations.

#### Launch Gazebo UI Client
This step also launches the joystick node for simplicity. In the future, the joystick node will be moved inside Docker.

**Terminal 3**
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DISCOVERY_SERVER=172.17.0.1:11811
export ROS_DOMAIN_ID=1
export IGN_DISCOVERY_URI=tcp://172.17.0.1:11811
export GZ_PARTITION=1
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/super_client_cfg_file.xml
ros2 launch apricotka-robot-car launch_gzui.launch.py
```
- `ROS_DISCOVERY_SERVER=172.17.0.1:11811`: Specifies the DDS discovery server's IP and port.
- `ROS_DOMAIN_ID=1`: Defines the ROS2 domain ID.
- `IGN_DISCOVERY_URI=tcp://172.17.0.1:11811`: Specifies the Gazebo discovery service.
- `GZ_PARTITION=1`: Defines the Gazebo partition.
- `FASTRTPS_DEFAULT_PROFILES_FILE`: Points to the Fast RTPS configuration file.

`172.17.0.1` is the default gateway IP for Docker bridge networks, ensuring communication between the host and Docker containers.

## Build and Launch Simulation Without Docker
This approach is **not recommended**.

Running without Docker increases complexity due to manual dependency management, potential system conflicts, and inconsistent runtime environments. Docker ensures a reproducible and isolated development environment.

### Install ROS Dependencies
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Source ROS2 (If Not Added to `.bashrc`)
```bash
source /opt/ros/jazzy/setup.bash
```

### Build Source Code
```bash
colcon build
```

### Launch Simulation

**Terminal 1**
```bash
source install/setup.bash
ros2 launch apricotka-robot-car launch_sim.launch.py
```

**Terminal 2**
```bash
source install/setup.bash
ros2 launch apricotka-robot-car launch_gzui.launch.py
```

## Other Notes
This section holds temporary, unorganized notes.

### Start Keyboard Control (New Terminal Window)
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

