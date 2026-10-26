cmake_minimum_required(VERSION 3.8)
project(jetson_v4l2_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)

add_executable(jetson_v4l2_camera_node
  src/jetson_v4l2_camera_node.cpp
  src/jetson_v4l2_camera.cpp
)
target_include_directories(jetson_v4l2_camera_node PRIVATE include)
ament_target_dependencies(jetson_v4l2_camera_node rclcpp sensor_msgs camera_info_manager)
target_link_libraries(jetson_v4l2_camera_node PkgConfig::GST)

install(TARGETS jetson_v4l2_camera_node DESTINATION lib/${PROJECT_NAME})

ament_package()
