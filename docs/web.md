# Web contract (MollyDashboard)

How [MollyDashboard](https://github.com/DarmawanWeb/MollyDashboard) talks to the robot. Every
name and type is defined in [interfaces.md](interfaces.md); this page lists what the dashboard
uses and how.

## Connections

| | Default | Started by |
|---|---|---|
| rosbridge WebSocket | `ws://<robot>:9090` | `molly` `robot.launch.py` (`web:=true`, `WEB=true` in the env file) |
| Camera (MJPEG) | `http://<robot>:8080/stream?topic=/camera/camera/color/image_raw&type=mjpeg&quality=60&width=960` | `web_video_server`, same switch |
| Camera snapshot | `http://<robot>:8080/snapshot?topic=/camera/camera/color/image_raw` | same |

rosbridge only exposes the names below (`topics_glob`, `services_glob`, `params_glob` in
`molly_bridge/config/web_params.yaml`): the dashboard can never publish on the driver's
`/<robot_ns>/cmd_vel`, only on `/bridge/remote_cmd_vel` through the control mux (e-stop, speed
envelope, protective field). Keep the robot's web ports on the robot's own network.

## Read (subscribe)

| Topic | Type | Use |
|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | map canvas (latched; live while mapping, throttle 1000 ms) |
| `/localization/pose` | `geometry_msgs/msg/PoseStamped` | robot on the map (throttle 100 ms) |
| `/localization/status` | `std_msgs/msg/String` | localized or not, source |
| `/site/status` | `molly_interfaces/msg/SiteStatus` | active site, site list, counts |
| `/site/pois` | `molly_interfaces/msg/PoiArray` | POIs on the map and in the editor |
| `/site/tours` | `molly_interfaces/msg/TourArray` | tours |
| `/mapping/status` | `molly_interfaces/msg/MappingStatus` | mapping session |
| `/planner/status` | `molly_interfaces/msg/PlannerStatus` | walking to a goal |
| `/planner/path` | `nav_msgs/msg/Path` | planned path on the map |
| `/navigation/status` | `molly_interfaces/msg/NavigationStatus` | follower state (`BLOCKED`, ...) |
| `/tour/state` | `molly_interfaces/msg/TourState` | tour progress, the authority for the tour UI |
| `/interaction/speech_status` | `molly_interfaces/msg/SpeechStatus` | what the robot is saying |
| `/interaction/greeter` | `std_msgs/msg/String` | JSON `{enabled, state, last_greet_age}` |
| `/interaction/conversation` | `std_msgs/msg/String` | JSON `{question, answer, ms, ok}` |
| `/perception/visitors` | `molly_interfaces/msg/Visitors` | visitors around the robot (throttle 500 ms) |
| `/bridge/mode` | `std_msgs/msg/String` | `auto` \| `remote` |
| `/bridge/estop_engaged`, `/bridge/estop_reason` | `std_msgs/msg/Bool`, `std_msgs/msg/String` | e-stop banner |
| `/bridge/protective_field` | `std_msgs/msg/String` | `clear`, `limited`, `stop`, `stale`, `off` |
| `/health/status` | `std_msgs/msg/String` | JSON (see molly_health): level, issues, CPU / GPU / RAM, temperature, topics |
| `/odometry/status` | `std_msgs/msg/String` | odometry state |
| `/robot/battery` | `sensor_msgs/msg/BatteryState` | battery (`percentage` 0–1) |
| `/robot/imu` | `sensor_msgs/msg/Imu` | attitude (throttle 200 ms) |
| `/robot/state` | `lite3_interfaces/msg/RobotState` | Lite3 state: basic state, gait, ultrasound (robot specific, optional) |
| `/robot/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | driver diagnostics |
| `/rosout` | `rcl_interfaces/msg/Log` | log viewer |

## Write (publish)

| Topic | Type | Use |
|---|---|---|
| `/bridge/remote_cmd_vel` | `geometry_msgs/msg/Twist` | teleop at 10–20 Hz, only effective in `remote` mode; send zero on release |
| `/tour/cmd` | `molly_interfaces/msg/TourCommand` | tour control without an answer (prefer `/tour/control`) |
| `/interaction/say_text` | `std_msgs/msg/String` | quick phrases |

## Call (services)

| Service | Type | Use |
|---|---|---|
| `/site/list`, `/site/load` | `ListSites`, `LoadSite` | site picker |
| `/mapping/start`, `/mapping/stop`, `/mapping/save` | `StartMapping`, `std_srvs/srv/Trigger` | mapping page |
| `/localization/start`, `/localization/record_tags` | `std_srvs/srv/Trigger` | localize, record tags |
| `/site/save_poi`, `/site/delete_poi` | `SavePoi`, `DeleteById` | POI editor (map click: x, y; yaw from a drag) |
| `/site/save_tour`, `/site/delete_tour` | `SaveTour`, `DeleteById` | tour editor |
| `/planner/plan` | `PlanPath` | preview a path |
| `/planner/go_to`, `/planner/cancel` | `GoTo`, `std_srvs/srv/Trigger` | walk to a POI / pose |
| `/navigation/pause` | `std_srvs/srv/SetBool` | hold / continue |
| `/tour/control`, `/tour/get_state` | `TourControl`, `GetTourState` | tour control, state on (re)connect |
| `/interaction/say`, `/interaction/stop_speech` | `Say`, `std_srvs/srv/Trigger` | speak |
| `/interaction/gesture` | `Gesture` | gestures |
| `/interaction/greet`, `/interaction/set_greeter` | `std_srvs/srv/Trigger`, `std_srvs/srv/SetBool` | greeter |
| `/interaction/ask` | `Ask` | conversation panel |
| `/bridge/set_mode`, `/bridge/estop` | `SetMode`, `std_srvs/srv/SetBool` | AUTO / REMOTE, e-stop |
| `/robot/stand_up`, `/robot/sit_down`, gaits | `std_srvs/srv/Trigger` | robot actions |
| `/path_follower_node/get_parameters`, `/path_follower_node/set_parameters` | `rcl_interfaces/srv/GetParameters`, `SetParameters` | walking speed and tuning |
| `/rosapi/topics`, `/rosapi/nodes` | rosapi | system page |

All `molly_interfaces` types are `molly_interfaces/msg/<Name>` and `molly_interfaces/srv/<Name>`
on the wire.

## Flows

**Map a place.** `/mapping/start {site, reset: true}` → drive in REMOTE (teleop panel or the
handheld) while `/map` grows → at each AprilTag `/localization/record_tags` → add POIs here
(`/site/save_poi {poi: {name, ...}, at_robot_pose: true}`) or later by clicking the map →
`/mapping/save` → `/mapping/stop`.

**Edit POIs.** Click on the map: `x, y` in the map frame from the canvas projection (`origin`,
`resolution`, the grid row flip), drag for the heading → `/site/save_poi {poi, at_robot_pose:
false}`. The robot answers with the saved POI (id assigned); `/site/pois` updates every client.

**Run a tour.** `/tour/control {command: {kind: KIND_START, tour_id}}` (or `poi_ids`), the robot
must be localized and in `auto`. Follow `/tour/state`; PAUSE / RESUME / SKIP / STOP the same way.
On (re)connect, `/tour/get_state` gives the current state before the first message.

**Walk somewhere.** `/planner/go_to {poi_id}` or `{x, y, yaw}`; `/planner/status` and
`/planner/path` show the walk; `/planner/cancel` stops it.
