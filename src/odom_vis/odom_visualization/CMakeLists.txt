cmake_minimum_required(VERSION 3.8)
project(odom_visualization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pose_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)

# Create executable nodes
add_executable(${PROJECT_NAME} src/odom_visualization.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  pose_utils
  Eigen3
)

target_link_libraries(${PROJECT_NAME}
  ${ARMADILLO_LIBRARIES}
)

add_executable(${PROJECT_NAME}_plate src/odom_visualization_plate.cpp)

ament_target_dependencies(${PROJECT_NAME}_plate
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
)

target_link_libraries(${PROJECT_NAME}_plate
  ${ARMADILLO_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plate
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
