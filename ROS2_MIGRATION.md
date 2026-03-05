# ROS2 Humble Migration Guide

This document describes the changes made to migrate Fast-Perching from ROS1 Noetic to ROS2 Humble.

## Build Instructions

### Prerequisites

- ROS2 Humble Hawksbill
- Colcon build tool
- Eigen3
- PCL (Point Cloud Library)
- Armadillo (for odom_visualization)

### Build Steps

```bash
cd ~/ros2_ws/src  # or your ROS2 workspace
git clone https://github.com/ZJU-FAST-Lab/Fast-Perching
cd ~/ros2_ws
colcon build --packages-select traj_opt vis_utils pose_utils odom_visualization planning
```

### Source the Environment

```bash
source install/setup.bash
```

## Running the Simulation

### Terminal 1: Launch RViz2 and visualization

```bash
ros2 launch planning perching.launch.py
```

### Terminal 2: Trigger the planner

```bash
./sh_utils/pub_triger.sh
```

Or manually publish:

```bash
ros2 topic pub /triger geometry_msgs/msg/PoseStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" --once
```

## Key Changes from ROS1 to ROS2

### Package Structure

- **CMakeLists.txt**: Changed from `catkin` to `ament_cmake`
- **package.xml**: Updated to format 3 with `ament` build type
- **Launch files**: Changed from `.launch` (XML) to `.launch.py` (Python)

### API Changes

| ROS1 | ROS2 |
|------|------|
| `ros::NodeHandle` | `rclcpp::Node::SharedPtr` |
| `ros::Subscriber` | `rclcpp::Subscription<T>::SharedPtr` |
| `ros::Publisher` | `rclcpp::Publisher<T>::SharedPtr` |
| `ros::Timer` | `rclcpp::TimerBase::SharedPtr` |
| `ros::Time::now()` | `node->now()` |
| `nh.getParam()` | `node->get_parameter()` |
| `nh.advertise<T>()` | `node->create_publisher<T>()` |
| `nh.subscribe()` | `node->create_subscription<T>()` |
| `ROS_INFO/ROS_WARN` | `RCLCPP_INFO/RCLCPP_WARN` |
| `nodelet::Nodelet` | `rclcpp::Node` |

### Message Changes

Messages now use `.msg` extension and are in `msg` namespace:
- `geometry_msgs/PoseStamped.h` → `geometry_msgs/msg/pose_stamped.hpp`
- `nav_msgs/Odometry.h` → `nav_msgs/msg/odometry.hpp`
- `visualization_msgs/Marker.h` → `visualization_msgs/msg/marker.hpp`

### TF Changes

- `tf::TransformBroadcaster` → `tf2_ros::TransformBroadcaster`
- `tf::StampedTransform` → `geometry_msgs::msg::TransformStamped`

## Configuration

Launch parameters are now declared using `DeclareLaunchArgument` in the Python launch file. Modify `src/planning/launch/perching.launch.py` to change:

- Target position: `perching_px`, `perching_py`, `perching_pz`
- Target velocity: `perching_vx`, `perching_vy`, `perching_vz`
- Landing orientation: `perching_axis_x/y/z`, `perching_theta`
- Algorithm parameters: `K`, `vmax`, `thrust_max`, etc.

## Known Issues

1. **Mesh resources**: The mesh file path format may need adjustment for ROS2 package sharing.
2. **Armadillo dependency**: The odom_visualization package still requires Armadillo library.

## Troubleshooting

### Build Errors

If you encounter missing dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Runtime Errors

Check that all nodes are running:
```bash
ros2 node list
```

Check topic connections:
```bash
ros2 topic list
ros2 topic echo /triger
```
