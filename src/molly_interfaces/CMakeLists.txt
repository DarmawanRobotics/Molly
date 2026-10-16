cmake_minimum_required(VERSION 3.8)
project(molly_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # navigation (from Pashupati)
  "msg/SectorScan.msg"
  "msg/AvoidanceCommand.msg"
  "msg/Waypoint.msg"
  "msg/WaypointPath.msg"
  "msg/NavigationStatus.msg"
  "msg/MissionStatus.msg"
  "msg/StopPointEvent.msg"
  "srv/MarkStopPoint.srv"
  "srv/LoadPath.srv"
  "srv/SetMode.srv"
  # sites, mapping and the path finder
  "msg/Poi.msg"
  "msg/PoiArray.msg"
  "msg/Tour.msg"
  "msg/TourArray.msg"
  "msg/SiteStatus.msg"
  "msg/MappingStatus.msg"
  "msg/PlannerStatus.msg"
  "srv/ListSites.srv"
  "srv/LoadSite.srv"
  "srv/StartMapping.srv"
  "srv/SavePoi.srv"
  "srv/SaveTour.srv"
  "srv/DeleteById.srv"
  "srv/LoadFile.srv"
  "srv/SetPath.srv"
  "srv/PlanPath.srv"
  "srv/GoTo.srv"
  # guide and interaction
  "msg/TourCommand.msg"
  "msg/TourState.msg"
  "msg/SpeechStatus.msg"
  "msg/Visitors.msg"
  "srv/TourControl.srv"
  "srv/GetTourState.srv"
  "srv/Say.srv"
  "srv/Ask.srv"
  "srv/Gesture.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
