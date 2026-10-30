# Molly interfaces

The single reference for every topic, service and file of Molly 2.0 on Dharma 0.2. The handheld
([PROTOCOL.md](../src/molly_bridge/PROTOCOL.md)) and the dashboard ([web.md](web.md)) are built
against it. Names are absolute; nodes declare them relative and run in the root namespace. The
robot driver lives under `/<robot_ns>/` (default `/robot/`), see
[src/xtras/README.md](../src/xtras/README.md). Each package README has the details; the types
are in `dharma_interfaces` and `molly_interfaces`.

## Frames

```
map ─ odom ─ camera_init ─ body ─ base_link ─┬─ livox_frame ─ livox_imu_frame
 │  (localization)  (FAST-LIO2)  (static)     ├─ front_camera_link ─ front_camera_optical_frame
 │                                            └─ legs, imu_link, ultrasound_* (robot URDF)
 └─ the site map, AprilTags, POIs and routes are all in `map`
```

- `map → odom` comes from `localization_node` (an EKF fusing the AprilTags in view, an RViz or
  simulator pose estimate, or the start pose when a new map is recorded). The map is used for
  path finding, visitor filtering and display.
- The frame names are the robot profile's `robot.frames` (`config/robot.yaml`).
- `odom → camera_init` and `body → base_link` are static (from the URDF: `base_link →
livox_imu_frame`).
- The robot driver never publishes `odom → base_link`.

## Sites

A site is one mapped place: its occupancy map, the AprilTags that anchor it and its routes
(Dharma's site manager), and its POIs and tours (Molly's `site_content_node`). One directory
per site under `sites_dir` (`SITES_DIR`, default `~/dev/Molly/map`):

```
map/<site>/
├── map.yaml        # map_server format: image, resolution, origin [x, y, yaw], negate 0,
│                   # occupied_thresh 0.65, free_thresh 0.25, mode trinary
├── map.pgm         # 8-bit: 0 occupied, 254 free, 205 unknown; row 0 is the top (max y)
├── tags.json       # {tag_frame: {x, y, z, qx, qy, qz, qw}} in map (localization/record_tags)
├── pois.json       # {"pois": [Poi fields...]} in list order (molly_guide)
├── tours.json      # {"tours": [Tour fields...]} (molly_guide)
└── routes/         # taught routes, <date>_<time>.csv: x, y, yaw_deg, dwell_sec (> 0: stop point)
```

Site names and POI / tour ids: `^[a-z0-9][a-z0-9_-]{0,47}$`. The active site is remembered in
`~/.molly/site`, the last route loaded in `~/.molly/last_route`. A mapping reset moves the map,
tags, POIs and tours into `<site>/backup-<time>/`. JSON files are written atomically (temp file + rename) with a `.bak` of the
previous version.

## Topics

| Topic                                                                   | Type                                     | Publisher                                    | Notes                                                                      |
| ----------------------------------------------------------------------- | ---------------------------------------- | -------------------------------------------- | -------------------------------------------------------------------------- |
| `/map`                                                                  | `nav_msgs/OccupancyGrid`                 | site_manager_node                            | latched; the live grid while mapping, the site map otherwise               |
| `/site/status`                                                          | `dharma_interfaces/SiteStatus`           | site_manager_node                            | latched: site, sites, has_map, mapping, tags, routes                       |
| `/site/pois`, `/site/tours`                                             | `molly_interfaces/PoiArray`, `TourArray` | site_content_node                            | latched, the active site's                                                 |
| `/mapping/grid`, `/mapping/status`                                      | `OccupancyGrid`, `MappingStatus`         | grid_mapper_node                             | latched                                                                    |
| `/localization/status`                                                  | `dharma_interfaces/LocalizationStatus`   | localization_node                            | latched: state (`NONE`, localized), source, tags in view                   |
| `/localization/pose`                                                    | `PoseWithCovarianceStamped`              | localization_node                            | `map → base_link` while localized                                          |
| `/localization/tag_markers`                                             | `visualization_msgs/MarkerArray`         | localization_node                            | latched                                                                    |
| `/odometry/status`                                                      | `dharma_interfaces/OdometryStatus`       | odometry_monitor_node                        | latched                                                                    |
| `/planner/status`                                                       | `dharma_interfaces/PlannerStatus`        | planner_node                                 | latched: state, goal, remaining, replans                                   |
| `/planner/path`, `/planner/costmap`                                     | `Path`, `OccupancyGrid`                  | planner_node                                 | latched                                                                    |
| `/navigation/waypoints`, `/navigation/path`                             | `WaypointPath`, `Path`                   | path_loader_node, path_follower_node         | latched, the loaded route                                                  |
| `/navigation/status`                                                    | `dharma_interfaces/NavigationStatus`     | path_follower_node                           | latched: state, pause reasons, index, remaining                            |
| `/navigation/stop_point`                                                | `dharma_interfaces/StopPointEvent`       | path_follower_node                           | route stop points                                                          |
| `/perception/sector_scan`, `/perception/safety_scan`                    | `dharma_interfaces/SectorScan`           | lidar_sector_node                            | front sectors / 360°                                                       |
| `/perception/visitors`                                                  | `molly_interfaces/Visitors`              | visitor_tracker_node (molly_interaction)     | in front, behind, beside                                                   |
| `/perception/visitor_markers`                                           | `visualization_msgs/MarkerArray`         | visitor_tracker_node                         | RViz                                                                       |
| `/tour/state`                                                           | `molly_interfaces/TourState`             | guide_node                                   | latched: state, tour, stop, activity, message                              |
| `/tour/cmd`                                                             | `molly_interfaces/TourCommand`           | the dashboard                                | like `/tour/control`, without an answer                                    |
| `/interaction/speech_status`                                            | `molly_interfaces/SpeechStatus`          | speech_node                                  | latched                                                                    |
| `/interaction/say_text`                                                 | `std_msgs/String`                        | anyone                                       | speak, fire and forget                                                     |
| `/interaction/greeter`                                                  | `molly_interfaces/GreeterStatus`         | interaction_node                             | latched                                                                    |
| `/interaction/conversation`                                             | `molly_interfaces/Conversation`          | conversation_node                            | one per question                                                           |
| `/control/status`                                                       | `dharma_interfaces/ControlStatus`        | control_mux_node                             | latched: mode, remote source, e-stop and reasons, field state, speed scale |
| `/control/nav_cmd_vel`                                                  | `geometry_msgs/Twist`                    | path_follower_node / obstacle_avoidance_node | to the mux (AUTO)                                                          |
| `/control/remote/handheld/cmd_vel`, `/control/remote/dashboard/cmd_vel` | `geometry_msgs/Twist`                    | teleop_udp_node, the dashboard               | to the mux (REMOTE, the selected source only)                              |
| `/control/estop_request`                                                | `std_msgs/String`                        | any node                                     | engages the e-stop with a reason                                           |
| `/docking/status`                                                       | `dharma_interfaces/DockingStatus`        | docking_node                                 | latched                                                                    |
| `/health/status`                                                        | `dharma_interfaces/HealthStatus`         | health_monitor_node                          | latched: level, issues, sensors, speed scale, CPU, GPU, RAM                |
| `/<robot_ns>/cmd_vel`                                                   | `geometry_msgs/Twist`                    | control_mux_node                             | only the mux drives the robot                                              |
| `/<robot_ns>/odom`, `/battery`, `/imu`, `/joint_states`, `/diagnostics` |                                          | robot driver                                 | [contract](../src/xtras/README.md)                                         |

## Services

| Service                                                                 | Type                                      | Server                                                                      |
| ----------------------------------------------------------------------- | ----------------------------------------- | --------------------------------------------------------------------------- |
| `/site/list`, `/site/load`                                              | `ListSites`, `LoadSite`                   | site_manager_node                                                           |
| `/site/save_poi`, `/site/delete_poi`                                    | `molly_interfaces/SavePoi`, `DeleteById`  | site_content_node                                                           |
| `/site/save_tour`, `/site/delete_tour`                                  | `molly_interfaces/SaveTour`, `DeleteById` | site_content_node                                                           |
| `/mapping/start`                                                        | `dharma_interfaces/StartMapping`          | site_manager_node (drives grid_mapper_node and localization)                |
| `/mapping/stop`, `/mapping/save`                                        | `std_srvs/Trigger`                        | site_manager_node: save writes the grid into the active site and reloads it |
| `/localization/start`                                                   | `std_srvs/Trigger`                        | localization_node: localize from a visible tag                              |
| `/localization/record_tags`                                             | `std_srvs/Trigger`                        | localization_node: save the visible tags into the site's `tags.json`        |
| `/localization/load_tags`                                               | `dharma_interfaces/LoadFile`              | localization_node (the site manager calls it)                               |
| `/localization/assume_origin`, `/localization/reset`                    | `std_srvs/Trigger`                        | localization_node                                                           |
| `/planner/go_to`                                                        | `dharma_interfaces/GoTo`                  | planner_node: plan to x, y, yaw, load, start                                |
| `/planner/plan`, `/planner/cancel`                                      | `PlanPath`, `std_srvs/Trigger`            | planner_node                                                                |
| `/navigation/load_path`                                                 | `dharma_interfaces/LoadPath`              | path_loader_node: a route CSV inside `sites_dir`                            |
| `/navigation/set_path`                                                  | `dharma_interfaces/SetPath`               | path_follower_node                                                          |
| `/navigation/start`, `/stop`, `/pause`, `/resume`                       | `std_srvs/Trigger`                        | path_follower_node                                                          |
| `/route/start`, `/route/stop`                                           | `std_srvs/Trigger`                        | route_recorder_node: teach a route                                          |
| `/route/mark_stop_point`                                                | `dharma_interfaces/MarkStopPoint`         | route_recorder_node                                                         |
| `/control/set_mode`                                                     | `dharma_interfaces/SetMode`               | control_mux_node: `auto`, or `remote` with a source                         |
| `/control/estop/engage`, `/control/estop/release`                       | `std_srvs/Trigger`                        | control_mux_node                                                            |
| `/docking/dock`, `/docking/undock`, `/docking/cancel`                   | `std_srvs/Trigger`                        | docking_node                                                                |
| `/tour/control`                                                         | `molly_interfaces/TourControl`            | guide_node: START, PAUSE, RESUME, STOP, SKIP                                |
| `/tour/get_state`                                                       | `molly_interfaces/GetTourState`           | guide_node                                                                  |
| `/guide/go_to_poi`                                                      | `molly_interfaces/GoToPoi`                | guide_node: walk to one POI (refused during a tour)                         |
| `/interaction/say`, `/interaction/stop_speech`                          | `molly_interfaces/Say`, `Trigger`         | speech_node                                                                 |
| `/interaction/gesture`                                                  | `molly_interfaces/Gesture`                | interaction_node → `/<robot_ns>/<name>`                                     |
| `/interaction/greet`, `/interaction/set_greeter`                        | `std_srvs/Trigger`, `SetBool`             | interaction_node: greet now, automatic greeting on / off                    |
| `/interaction/ask`                                                      | `molly_interfaces/Ask`                    | conversation_node                                                           |
| `/<robot_ns>/stand_up`, `/sit_down`, `/emergency_stop`, gaits, gestures | `std_srvs/Trigger`                        | robot driver                                                                |

## Behaviour

**Mapping** (`start_mapping:=true site:=<name>`, or `mapping/start` at runtime).
`mapping/start {site, reset}`:

1. creates `map/<site>/` when new and makes it the active site;
2. switches localization to `map/<site>/tags.json` (`localization/load_tags`);
3. when localization has no source yet, takes the current pose as the map origin
   (`localization/assume_origin`);
4. starts `grid_mapper_node` (`reset`: empty grid; otherwise it continues the saved map).

A site that already has a map keeps its frame: continuing it needs the robot localized on one
of its tags first, or `reset: true` to start it over (its files, POIs and tours included, move
to `map/<site>/backup-<time>/`). Loading another site (`site/load`) resets localization.

While mapping, drive the robot (handheld REMOTE) through the whole place, stand still in view
of each AprilTag and `localization/record_tags`, and add POIs at the robot pose (`site/save_poi`
with `at_robot_pose`). `mapping/save` writes `map.pgm` + `map.yaml` and reloads the site;
`mapping/stop` ends the session.

**Guiding.** The active site's map is published on `/map`, its tags are loaded into
localization (a tag in view localizes the robot), and:

- `guide/go_to_poi {poi_id}` or `planner/go_to {x, y, yaw}`: A\* on the costmap; the path goes to
  the follower, which walks it (obstacle avoidance steers around people) and turns to the
  goal's heading (`planner/status` reaches the goal).
- `/tour/control {START, tour_id}` (AUTO and localized): for each POI the guide walks there
  (activity `NAVIGATING`, `WAITING_VISITORS` when nobody follows), plays the POI gesture
  (`PERFORMING_MOTION`), says the narration (`NARRATING`) and waits `dwell_sec` (`DWELLING`); then the
  next one, and walks to `return_poi_id` at the end (`RETURNING`).
- REMOTE or an e-stop pauses navigation and the tour with it; back in AUTO the tour goes on to
  the same stop (after an e-stop only when told: RESUME).
- Visitors in front of an idle robot are greeted (speech + gesture), questions typed on the
  dashboard are answered by the conversation model (optional).
