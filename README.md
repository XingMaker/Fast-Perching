# Fast-Perching

## 0. Overview
**Fast-Perching** presents a novel trajectory planning method for real-time aerial perching, which adaptively adjusts terminal states and the trajectory duration. This feature is especially notable on micro aerial robots with low maneuverability or scenarios where the space is not enough.

**Authors**: Jialin Ji, Tiankai Yang and [Fei Gao](https://ustfei.com/) from the [ZJU FAST Lab](http://zju-fast.com/).

**Paper**: [Real-Time Trajectory Planning for Aerial Perching](https://arxiv.org/abs/2203.01061), Jialin Ji, Tiankai Yang, Chao Xu, Fei Gao, Accepted in IEEE/RSJ International Conference on Intelligent Robots and Systems (__IROS 2022__).

**Video Links**: [bilibili](https://www.bilibili.com/video/BV14q4y147uz)
<a href="https://www.bilibili.com/video/BV14q4y147uz" target="blank">
  <p align="center">
    <img src="figs/cover.png" width="500"/>
  </p>
</a>

## ROS2 Humble Migration

This repository has been migrated from ROS1 Noetic to **ROS2 Humble**.

## 1. Simulation of Aerial Perching

>Prerequisites:
- ROS2 Humble
- colcon (build tool)
- Eigen3, PCL, Armadillo

>Build:
```bash
git clone https://github.com/ZJU-FAST-Lab/Fast-Perching
cd Fast-Perching
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# Source
source install/setup.zsh  # or setup.bash
```

>Preparation and visualization:
```bash
chmod +x sh_utils/pub_triger.sh
ros2 launch planning perching.launch.py
```

>Start the perching planner:
```bash
./sh_utils/pub_triger.sh
```
<p align="center">
    <img src="figs/perching1.gif" width="400"/>
</p>

>Change the position, veliocity and orientation of the landing plate:
```python
# DIR: src/planning/launch/perching.launch.py
    declare_perching_px = DeclareLaunchArgument('perching_px', default_value='0.5')
    declare_perching_py = DeclareLaunchArgument('perching_py', default_value='0.0')
    declare_perching_pz = DeclareLaunchArgument('perching_pz', default_value='2.0')
    declare_perching_vx = DeclareLaunchArgument('perching_vx', default_value='2.0')
    declare_perching_vy = DeclareLaunchArgument('perching_vy', default_value='0.0')
    declare_perching_vz = DeclareLaunchArgument('perching_vz', default_value='0.0')
    declare_perching_axis_x = DeclareLaunchArgument('perching_axis_x', default_value='0.0')
    declare_perching_axis_y = DeclareLaunchArgument('perching_axis_y', default_value='1.0')
    declare_perching_axis_z = DeclareLaunchArgument('perching_axis_z', default_value='0.0')
    declare_perching_theta = DeclareLaunchArgument('perching_theta', default_value='-1.5708')
```

<p align="center">
    <img src="figs/perching2.gif" width="400"/>
</p>

## 2. Other Settings or Functions

>Enable replan module of the planner:
```python
# DIR: src/planning/launch/perching.launch.py
    declare_replan = DeclareLaunchArgument('replan', default_value='true')
```
<p align="center">
    <img src="figs/replan.gif" width="400"/>
</p>

>Enable pause debug module of the planner:
```python
# DIR: src/planning/launch/perching.launch.py
    declare_pause_debug = DeclareLaunchArgument('pause_debug', default_value='true')
```

<p align="center">
    <img src="figs/debug.gif" width="400"/>
</p>

## 3. Key Changes from ROS1 to ROS2

- **Build system**: `catkin_make` → `colcon build`
- **Package format**: `package.xml` format 2 → format 3
- **CMake**: `catkin` → `ament_cmake`
- **Node type**: `nodelet` → `rclcpp::Node` (component)
- **Launch files**: `.launch` (XML) → `.launch.py` (Python)
- **Parameters**: `nh.getParam()` → `node->declare_parameter()` / `node->get_parameter()`
- **Publishers/Subscribers**: `nh.advertise/subscribe` → `node->create_publisher/subscription`
- **Timers**: `nh.createTimer` → `node->create_wall_timer`
- **TF**: `tf::TransformBroadcaster` → `tf2_ros::TransformBroadcaster`
- **Time**: `ros::Time` → `rclcpp::Time` / `node->now()`
- **Logging**: `ROS_INFO/WARN` → `RCLCPP_INFO/WARN`

## 4. Acknowledgement
We use [**MINCO**](https://github.com/ZJU-FAST-Lab/GCOPTER) as our trajectory representation.
