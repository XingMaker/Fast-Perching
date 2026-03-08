cmake_minimum_required(VERSION 3.8)
project(planning)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_BUILD_TYPE "Release")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(traj_opt REQUIRED)
find_package(vis_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# Create executable
add_executable(planning_node
  src/planning_node.cpp
)

target_include_directories(planning_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(planning_node
  rclcpp
  traj_opt
  vis_utils
  geometry_msgs
  nav_msgs
  std_msgs
  Eigen3
)

# Install executable
install(TARGETS planning_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
