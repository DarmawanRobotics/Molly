cmake_minimum_required(VERSION 3.8)
project(molly_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # sites: points of interest and tours (stored with the site)
  "msg/Poi.msg"
  "msg/PoiArray.msg"
  "msg/Tour.msg"
  "msg/TourArray.msg"
  # the guide
  "msg/TourCommand.msg"
  "msg/TourState.msg"
  # visitors and interaction
  "msg/Visitors.msg"
  "msg/SpeechStatus.msg"
  "msg/GreeterStatus.msg"
  "msg/Conversation.msg"
  # services
  "srv/SavePoi.srv"
  "srv/SaveTour.srv"
  "srv/DeleteById.srv"
  "srv/GoToPoi.srv"
  "srv/TourControl.srv"
  "srv/GetTourState.srv"
  "srv/Say.srv"
  "srv/Ask.srv"
  "srv/Gesture.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
