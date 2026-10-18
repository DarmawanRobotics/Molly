cmake_minimum_required(VERSION 3.8)
project(molly_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TourCommand.msg"
  "msg/TourState.msg"
  "msg/SpeechStatus.msg"
  "srv/TourControl.srv"
  "srv/GetTourState.srv"
  "srv/Say.srv"
  "srv/Ask.srv"
  "srv/Gesture.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
