cmake_minimum_required(VERSION 3.10)
project(dual_arm_transport_native)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(cascade_pid src/cascade_pid.cpp)
target_include_directories(cascade_pid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(dual_arm_transport_native_node
  src/main.cpp
  src/dual_arm_transport_node.cpp
)
add_executable(dual_arm_transport_full_runner
  src/dual_arm_transport_full_runner.cpp
)

target_include_directories(dual_arm_transport_native_node PRIVATE include)
target_include_directories(dual_arm_transport_full_runner PRIVATE include)

target_link_libraries(dual_arm_transport_native_node cascade_pid)
target_link_libraries(dual_arm_transport_full_runner cascade_pid)

ament_target_dependencies(dual_arm_transport_native_node
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  control_msgs
  controller_manager_msgs
  builtin_interfaces
  geometry_msgs
  moveit_msgs
  tf2
  tf2_ros
)
ament_target_dependencies(dual_arm_transport_full_runner
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  control_msgs
  controller_manager_msgs
  builtin_interfaces
  geometry_msgs
  moveit_msgs
  tf2
  tf2_ros
)

install(TARGETS
  cascade_pid
  dual_arm_transport_native_node
  dual_arm_transport_full_runner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
