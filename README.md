The vcstool is used to simplify packages instaletion.
https://github.com/dirk-thomas/vcstool 

1. Clone repository
```bash
git clone git@github.com:KPI-Rover/ros_ws.git
```
2. Install packages using vcstool 
```bash
vcs import src < kpi-rover-ros-ws-sim.repos
```
3. Source ROS2
```bash
source /opt/ros/jazzy/setup.bash
```  
4. Build
```bash
colcon build --install
```
5. Source workspace
```bash
source install/setup.bash
```
6. Start 
```bash
ros2 launch apricotka-robot-car launch_sim.launch.py
```
7. Start Keyboar control (new terminal window)
```bash
TODO
```
