cmake_minimum_required(VERSION 3.8)
project(waypoint_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(waypoint_follower_msgs REQUIRED)

add_executable(waypoint_follower_node
  src/new_main.cpp
  src/new_process.cpp
  src/new_path_tracking.cpp
)

target_include_directories(waypoint_follower_node PRIVATE include)

ament_target_dependencies(waypoint_follower_node
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  waypoint_follower_msgs
)

install(TARGETS waypoint_follower_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config test
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

ament_package()
