# Web contract (MollyDashboard)

How [MollyDashboard](https://github.com/DarmawanWeb/MollyDashboard) talks to the robot. Every
name and type is defined in [interfaces.md](interfaces.md); this page lists what the dashboard
uses and how.

## Connections

|                     | Default                                                                                           | Started by                                                                                                  |
| ------------------- | ------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| rosbridge WebSocket | `ws://<robot>:9090`                                                                               | `molly_bringup` `robot.launch.py` (`web:=true`, `WEB=true` in the env file), `molly_bridge` `web.launch.py` |
| Camera (MJPEG)      | `http://<robot>:8080/stream?topic=/camera/camera/color/image_raw&type=mjpeg&quality=60&width=960` | `web_video_server`, same switch                                                                             |
| Camera snapshot     | `http://<robot>:8080/snapshot?topic=/camera/camera/color/image_raw`                               | same                                                                                                        |

rosbridge only exposes the names below (the whitelist in `molly_bridge/core/web.py`, given to
rosbridge as `topics_glob`, `services_glob`, `params_glob`): the dashboard can never publish on
the driver's `/<robot_ns>/cmd_vel`, only on `/control/remote/dashboard/cmd_vel` through the
control mux (e-stop, speed envelope, protective field, health speed scale). Keep the robot's web ports on the robot's own network.

## Read (subscribe)

| Topic                        | Type                                          | Use                                                                                     |
| ---------------------------- | --------------------------------------------- | --------------------------------------------------------------------------------------- |
| `/map`                       | `nav_msgs/msg/OccupancyGrid`                  | map canvas (latched; live while mapping, throttle 1000 ms)                              |
| `/localization/pose`         | `geometry_msgs/msg/PoseWithCovarianceStamped` | robot on the map (throttle 100 ms)                                                      |
| `/localization/status`       | `dharma_interfaces/msg/LocalizationStatus`    | localized or not, source                                                                |
| `/site/status`               | `dharma_interfaces/msg/SiteStatus`            | active site, site list, counts                                                          |
| `/site/pois`                 | `molly_interfaces/msg/PoiArray`               | POIs on the map and in the editor                                                       |
| `/site/tours`                | `molly_interfaces/msg/TourArray`              | tours                                                                                   |
| `/mapping/status`            | `dharma_interfaces/msg/MappingStatus`         | mapping session                                                                         |
| `/planner/status`            | `dharma_interfaces/msg/PlannerStatus`         | walking to a goal                                                                       |
| `/planner/path`              | `nav_msgs/msg/Path`                           | planned path on the map                                                                 |
| `/navigation/status`         | `dharma_interfaces/msg/NavigationStatus`      | follower state, pause reasons                                                           |
| `/navigation/path`           | `nav_msgs/msg/Path`                           | the loaded route                                                                        |
| `/docking/status`            | `dharma_interfaces/msg/DockingStatus`         | docking state                                                                           |
| `/tour/state`                | `molly_interfaces/msg/TourState`              | tour progress, the authority for the tour UI                                            |
| `/interaction/speech_status` | `molly_interfaces/msg/SpeechStatus`           | what the robot is saying                                                                |
| `/interaction/greeter`       | `molly_interfaces/msg/GreeterStatus`          | greeter state, last greeting                                                            |
| `/interaction/conversation`  | `molly_interfaces/msg/Conversation`           | question, answer, latency, success                                                      |
| `/perception/visitors`       | `molly_interfaces/msg/Visitors`               | visitors around the robot (throttle 500 ms)                                             |
| `/control/status`            | `dharma_interfaces/msg/ControlStatus`         | mode (`auto` / `remote`, source), e-stop and its reasons, protective field, speed scale |
| `/health/status`             | `dharma_interfaces/msg/HealthStatus`          | level, issues, sensors, CPU / GPU / RAM, temperature                                    |
| `/odometry/status`           | `dharma_interfaces/msg/OdometryStatus`        | odometry state                                                                          |
| `/robot/battery`             | `sensor_msgs/msg/BatteryState`                | battery (`percentage` 0–1)                                                              |
| `/robot/imu`                 | `sensor_msgs/msg/Imu`                         | attitude (throttle 200 ms)                                                              |
| `/robot/state`               | `lite3_interfaces/msg/RobotState`             | Lite3 state: basic state, gait, ultrasound (robot specific, optional)                   |
| `/robot/diagnostics`         | `diagnostic_msgs/msg/DiagnosticArray`         | driver diagnostics                                                                      |
| `/rosout`                    | `rcl_interfaces/msg/Log`                      | log viewer                                                                              |

## Write (publish)

| Topic                               | Type                               | Use                                                                                                   |
| ----------------------------------- | ---------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `/control/remote/dashboard/cmd_vel` | `geometry_msgs/msg/Twist`          | teleop at 10–20 Hz, only effective in `remote` mode with the source `dashboard`; send zero on release |
| `/control/estop_request`            | `std_msgs/msg/String`              | engage the e-stop with a reason                                                                       |
| `/tour/cmd`                         | `molly_interfaces/msg/TourCommand` | tour control without an answer (prefer `/tour/control`)                                               |
| `/interaction/say_text`             | `std_msgs/msg/String`              | quick phrases                                                                                         |

## Call (services)

| Service                                                                        | Type                                                | Use                                                                       |
| ------------------------------------------------------------------------------ | --------------------------------------------------- | ------------------------------------------------------------------------- |
| `/site/list`, `/site/load`                                                     | `ListSites`, `LoadSite`                             | site picker                                                               |
| `/mapping/start`, `/mapping/stop`, `/mapping/save`                             | `StartMapping`, `std_srvs/srv/Trigger`              | mapping page                                                              |
| `/localization/start`, `/localization/record_tags`                             | `std_srvs/srv/Trigger`                              | localize, record tags                                                     |
| `/site/save_poi`, `/site/delete_poi`                                           | `SavePoi`, `DeleteById`                             | POI editor (map click: x, y; yaw from a drag)                             |
| `/site/save_tour`, `/site/delete_tour`                                         | `SaveTour`, `DeleteById`                            | tour editor                                                               |
| `/planner/plan`                                                                | `PlanPath`                                          | preview a path                                                            |
| `/planner/go_to`, `/planner/cancel`                                            | `GoTo`, `std_srvs/srv/Trigger`                      | walk to a POI / pose                                                      |
| `/navigation/pause`, `/navigation/resume`                                      | `std_srvs/srv/Trigger`                              | hold / continue                                                           |
| `/navigation/start`, `/navigation/stop`, `/navigation/load_path`               | `Trigger`, `LoadPath`                               | taught routes                                                             |
| `/route/start`, `/route/stop`, `/route/mark_stop_point`                        | `Trigger`, `MarkStopPoint`                          | teach a route                                                             |
| `/guide/go_to_poi`                                                             | `GoToPoi`                                           | walk to one POI                                                           |
| `/docking/dock`, `/docking/undock`, `/docking/cancel`                          | `std_srvs/srv/Trigger`                              | docking                                                                   |
| `/tour/control`, `/tour/get_state`                                             | `TourControl`, `GetTourState`                       | tour control, state on (re)connect                                        |
| `/interaction/say`, `/interaction/stop_speech`                                 | `Say`, `std_srvs/srv/Trigger`                       | speak                                                                     |
| `/interaction/gesture`                                                         | `Gesture`                                           | gestures                                                                  |
| `/interaction/greet`, `/interaction/set_greeter`                               | `std_srvs/srv/Trigger`, `std_srvs/srv/SetBool`      | greeter                                                                   |
| `/interaction/ask`                                                             | `Ask`                                               | conversation panel                                                        |
| `/control/set_mode`                                                            | `SetMode`                                           | `{mode: remote, source: dashboard}` to drive, `{mode: auto}` to hand back |
| `/control/estop/engage`, `/control/estop/release`                              | `std_srvs/srv/Trigger`                              | e-stop                                                                    |
| `/robot/stand_up`, `/robot/sit_down`, `/robot/emergency_stop`, gaits, gestures | `std_srvs/srv/Trigger`                              | robot actions                                                             |
| `/path_follower_node/get_parameters`, `/path_follower_node/set_parameters`     | `rcl_interfaces/srv/GetParameters`, `SetParameters` | walking speed and tuning                                                  |
| `/rosapi/topics`, `/rosapi/nodes`                                              | rosapi                                              | system page                                                               |

On the wire the types are `<package>/msg/<Name>` and `<package>/srv/<Name>`; the POI, tour,
visitor, speech and greeter types are `molly_interfaces` (in Molly 1.x some were
`dharma_interfaces` or JSON strings), the status types `dharma_interfaces`.

## Flows

**Map a place.** `/mapping/start {site, reset: true}` → drive in REMOTE (teleop panel or the
handheld) while `/map` grows → at each AprilTag `/localization/record_tags` → add POIs here
(`/site/save_poi {poi: {name, ...}, at_robot_pose: true}`) or later by clicking the map →
`/mapping/save` → `/mapping/stop`.

**Edit POIs.** Click on the map: `x, y` in the map frame from the canvas projection (`origin`,
`resolution`, the grid row flip), drag for the heading → `/site/save_poi {poi, at_robot_pose:
false}`. The robot answers with the saved POI (id assigned); `/site/pois` updates every client.

**Run a tour.** `/tour/control {command: {kind: 0, tour_id}}` (START) (or `poi_ids`), the robot
must be localized and in `auto`. Follow `/tour/state`; PAUSE / RESUME / SKIP / STOP the same way.
On (re)connect, `/tour/get_state` gives the current state before the first message.

**Walk somewhere.** `/guide/go_to_poi {poi_id}` or `/planner/go_to {x, y, yaw}`;
`/planner/status` and `/planner/path` show the walk; `/planner/cancel` stops it.

**Drive.** `/control/set_mode {mode: remote, source: dashboard}`, then
`/control/remote/dashboard/cmd_vel` at 10–20 Hz (zeros first: the mux arms a source on a
neutral command), `/control/set_mode {mode: auto}` to hand back; a running tour goes on.
