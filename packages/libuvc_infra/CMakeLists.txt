cmake_minimum_required(VERSION 3.8)
project(libuvc_infra)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories for OpenCV
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add the executable
add_executable(uvc_cam_node src/uvc_cam_node.cpp)

# Link dependencies
ament_target_dependencies(uvc_cam_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# Linking OpenCV and libuvc libraries
target_link_libraries(uvc_cam_node
  ${OpenCV_LIBRARIES}
  uvc
)

# Install the executable
install(TARGETS
  uvc_cam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
