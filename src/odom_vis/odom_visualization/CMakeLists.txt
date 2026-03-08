cmake_minimum_required(VERSION 3.8)
project(odom_visualization)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
  ${pose_utils_INCLUDE_DIRS}
)

# odom_visualization executable
add_executable(odom_visualization src/odom_visualization.cpp)
ament_target_dependencies(odom_visualization
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pose_utils
  Eigen3
  PCL
)
target_link_libraries(odom_visualization
  ${ARMADILLO_LIBRARIES}
  pose_utils::pose_utils
)

# odom_visualization_plate executable
add_executable(odom_visualization_plate src/odom_visualization_plate.cpp)
ament_target_dependencies(odom_visualization_plate
  rclcpp
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2_ros
  Eigen3
)

install(TARGETS odom_visualization odom_visualization_plate
  DESTINATION lib/${PROJECT_NAME}
)

# Install meshes
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
