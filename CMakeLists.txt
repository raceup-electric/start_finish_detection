cmake_minimum_required(VERSION 3.5)
project(start_finish_detection)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eufs_msgs REQUIRED) 

# Add executable
add_executable(start_finish_detection_node src/start_finish_detection_node.cpp)

# Link dependencies
ament_target_dependencies(start_finish_detection_node
  rclcpp
  std_msgs
  geometry_msgs
  eufs_msgs 
)

# Install the executable
install(TARGETS
  start_finish_detection_node
  DESTINATION lib/${PROJECT_NAME})

# Install other resources (e.g., launch files)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Ament package
ament_package()

