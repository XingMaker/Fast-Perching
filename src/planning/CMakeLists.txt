cmake_minimum_required(VERSION 3.8)
project(planning)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(traj_opt REQUIRED)
find_package(vis_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# Create executable node
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  std_msgs
  traj_opt
  vis_utils
)

# Create component library
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  std_msgs
  traj_opt
  vis_utils
)

rclcpp_components_register_nodes(${PROJECT_NAME} "planning::PlanningNode")

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
