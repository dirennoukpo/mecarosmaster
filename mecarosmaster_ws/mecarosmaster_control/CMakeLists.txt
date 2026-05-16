cmake_minimum_required(VERSION 3.14)
project(mecarosmaster_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(mecarosmaster_node src/mecarosmaster_node.cpp)
target_include_directories(mecarosmaster_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mecarosmaster_node
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs
  trajectory_msgs std_srvs tf2_ros tf2 tf2_geometry_msgs)

add_library(mecarosmaster_hardware SHARED src/mecarosmaster_hardware.cpp)
target_include_directories(mecarosmaster_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mecarosmaster_hardware
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(
  hardware_interface mecarosmaster_hardware_plugin.xml)

install(TARGETS mecarosmaster_node mecarosmaster_hardware
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY urdf config launch parms
  DESTINATION share/${PROJECT_NAME})

install(FILES mecarosmaster_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
