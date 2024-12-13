cmake_minimum_required(VERSION 3.5)
project(start_finish_detection)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(kvaser_reader_driver REQUIRED)

# Executable and linking
add_executable(start_finish_detection_node src/start_finish_detection_node.cpp)

ament_target_dependencies(start_finish_detection_node
  rclcpp
  std_msgs
  std_srvs
  eufs_msgs
  kvaser_reader_driver
)

install(TARGETS
  start_finish_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
