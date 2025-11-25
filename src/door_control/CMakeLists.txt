cmake_minimum_required(VERSION 3.8)
project(door_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(include include/common include/nlohmann)
link_directories(src)
set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
  rclcpp_action 
  door_interface 
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(door_interface REQUIRED)
find_package(Threads REQUIRED)

find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)


find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(door_control_node src/door_control_node.cpp include/cmd_sender.hpp src/common/ros2_sport_client.cpp)
target_compile_features(door_control_node PRIVATE cxx_std_20)

ament_target_dependencies(door_control_node ${DEPENDENCY_LIST})



install(TARGETS
  door_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
