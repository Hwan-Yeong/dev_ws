cmake_minimum_required(VERSION 3.8)
project(airbot_sensor_to_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(sensor_to_pointcloud
  src/node.cpp
  src/airbot_sensor_to_pointcloud/sensor_to_pointcloud.cpp
  src/airbot_sensor_to_pointcloud/tof/pointcloud_tof.cpp
  src/airbot_sensor_to_pointcloud/camera/pointcloud_camera.cpp
  src/airbot_sensor_to_pointcloud/cliff/pointcloud_cliff.cpp
  src/utils/frame_converter.cpp
  src/utils/pointcloud_generator.cpp
  src/utils/boundingbox_generator.cpp
  src/logger/camera_object_logger.cpp
)

ament_target_dependencies(sensor_to_pointcloud 
  rclcpp 
  sensor_msgs 
  robot_custom_msgs
  builtin_interfaces
  tf2
  tf2_geometry_msgs
  vision_msgs
  visualization_msgs
)

install(TARGETS sensor_to_pointcloud
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
