The vcstool is used to simplify packages installation.
https://github.com/dirk-thomas/vcstool 


## Get source code
### Clone repository
```bash
git clone git@github.com:KPI-Rover/ros_ws.git
cd ros_ws
```

### Install packages using vcstool 
```bash
vcs import src < kpi-rover-ros-ws-sim.repos
```

### Install ROS dependencies
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Source ROS2 (If not added to the .bashrc file)
```bash
source /opt/ros/jazzy/setup.bash
```

## Build

### Build Docker
```bash
docker build -t kpi-rover .
```

### Build project
```bash
docker run --rm -it \
  --user $(id -u):$(id -g) \
  -v $(pwd):/workspace \
  -w /workspace \
  kpi-rover colcon build

exit
```
## Run

### Source workspace
```bash
source install/setup.bash
```

### Launch simulation
```bash
ros2 launch apricotka-robot-car launch_sim.launch.py
```

### Start Keyboar control (new terminal window)
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
