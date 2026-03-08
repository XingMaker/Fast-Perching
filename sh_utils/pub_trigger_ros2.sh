#!/bin/bash

# 触发规划器的脚本

echo "Publishing trigger message to /triger topic..."

ros2 topic pub /triger geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "world", stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: 5.0, y: 0.0, z: 2.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' \
  --once

echo "Trigger sent!"
