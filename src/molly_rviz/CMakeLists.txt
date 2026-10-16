cmake_minimum_required(VERSION 3.8)
project(molly_rviz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(molly_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/ros_panel.cpp
  src/control_panel.cpp
  src/status_panel.cpp
  src/tuning_panel.cpp
  src/tag_panel.cpp
  src/cmd_vel_widget.cpp
  include/molly_rviz/ros_panel.hpp
  include/molly_rviz/control_panel.hpp
  include/molly_rviz/status_panel.hpp
  include/molly_rviz/tuning_panel.hpp
  include/molly_rviz/tag_panel.hpp
  include/molly_rviz/cmd_vel_widget.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  pluginlib
  std_srvs
  sensor_msgs
  std_msgs
  geometry_msgs
  diagnostic_msgs
  molly_interfaces
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp rviz_common pluginlib std_srvs sensor_msgs std_msgs geometry_msgs diagnostic_msgs
  molly_interfaces)

ament_package()