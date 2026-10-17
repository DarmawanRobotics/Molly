# Molly interfaces

The single reference for every topic, service and file of the Molly stack. The handheld
([PROTOCOL.md](../src/molly_bridge/PROTOCOL.md)) and the dashboard ([web.md](web.md)) are built
against it. Names are absolute; nodes declare them relative and run in the root namespace. The
robot driver lives under `/<robot_ns>/` (default `/robot/`), see
[src/xtras/README.md](../src/xtras/README.md).

## Frames

```
map ─ odom ─ camera_init ─ body ─ base_link ─┬─ livox_frame ─ livox_imu_frame
 │  (localization)  (FAST-LIO2)  (static)     ├─ front_camera_link ─ front_camera_optical_frame
 │                                            ├─ camera_link ─ camera_*_optical_frame (RealSense)
 │                                            └─ legs, imu_link, ultrasound_* (robot URDF)
 └─ the site map, AprilTags, POIs and routes are all in `map`
```

- `map → odom` is a planar correction from an AprilTag (`localization_node`), an RViz pose
  estimate, or the start pose when a new map is recorded. Localization stays tag based; the map
  is used for path finding, visitor filtering and display.
- `odom → camera_init` and `body → base_link` are static (from the URDF: `base_link →
  livox_imu_frame`).
- The robot driver never publishes `odom → base_link`.

## Sites

A site is one mapped place: its occupancy map, the AprilTags that anchor it, its POIs and tours,
and recorded routes. One directory per site under `sites_dir` (`$MOLLY_DIR/map`, default
`~/dev/Molly/map`):

```
map/<site>/
├── map.yaml        # map_server format: image, resolution, origin [x, y, yaw], negate 0,
│                   # occupied_thresh 0.65, free_thresh 0.25, mode trinary
├── map.pgm         # 8-bit: 0 occupied, 254 free, 205 unknown; row 0 is the top (max y)
├── tags.json       # {tag_frame: {x, y, z, qx, qy, qz, qw}} in map (localization/record_tags)
├── pois.json       # {"pois": [Poi fields...]} in list order
├── tours.json      # {"tours": [Tour fields...]}
└── routes/         # recorded routes, <date>_<time>_path.csv: x, y, yaw_deg, dwell_sec
```

Site names and POI / tour ids: `^[a-z0-9][a-z0-9_-]{0,47}$`. The active site is remembered in
`~/.molly/site`. JSON files are written atomically (temp file + rename) with a `.bak` of the
previous version.

## Topics

| Topic | Type | Publisher | Notes |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | site_manager_node | latched; the live grid while mapping, the site map otherwise |
| `/site/status` | `molly_interfaces/SiteStatus` | site_manager_node | latched |
| `/site/pois` | `molly_interfaces/PoiArray` | site_manager_node | latched |
| `/site/tours` | `molly_interfaces/TourArray` | site_manager_node | latched |
| `/mapping/grid` | `nav_msgs/OccupancyGrid` | grid_mapper_node | latched, `publish_hz` while mapping |
| `/mapping/status` | `molly_interfaces/MappingStatus` | grid_mapper_node | latched |
| `/localization/status` | `std_msgs/String` | localization_node | latched: `none`, `tag:<frame>`, `pose_estimate`, `origin` |
| `/localization/pose` | `geometry_msgs/PoseStamped` | localization_node | `map → base_link` at `pose_hz` (10) while localized |
| `/localization/tag_markers` | `visualization_msgs/MarkerArray` | localization_node | latched |
| `/odometry/status` | `std_msgs/String` | odometry_monitor_node | latched: `initializing`, `ok`, `degraded: …`, `lost: …` |
| `/planner/status` | `molly_interfaces/PlannerStatus` | planner_node | latched |
| `/planner/path` | `nav_msgs/Path` | planner_node | latched, the current plan (map) |
| `/planner/costmap` | `nav_msgs/OccupancyGrid` | planner_node | latched, inflated map (debug, `publish_costmap`) |
| `/navigation/waypoints` | `molly_interfaces/WaypointPath` | path_loader_node | latched |
| `/navigation/status` | `molly_interfaces/NavigationStatus` | path_follower_node | `state`: `NAV_INACTIVE`, `PAUSED`, `NO_PATH`, `TF_UNAVAILABLE`, `JOINING`, `FOLLOWING`, `APPROACHING`, `DWELLING`, `GOAL_REACHED`, `BLOCKED`, `OFF_PATH`, `EMERGENCY_STOP`, `AVOIDANCE_STALE` |
| `/navigation/mission_status`, `/navigation/stop_point_event` | molly_interfaces | path_follower_node | route stop points |
| `/perception/sector_scan`, `/perception/safety_scan` | `molly_interfaces/SectorScan` | lidar_sector_node | front sectors / 360° |
| `/perception/visitors` | `molly_interfaces/Visitors` | visitor_tracker_node | `rate_hz` (5) |
| `/perception/visitor_markers` | `visualization_msgs/MarkerArray` | visitor_tracker_node | RViz: in front, behind, beside |
| `/perception/people`, `/perception/people_status` | `PoseArray`, `std_msgs/String` (JSON) | person_detector_node | camera, optional |
| `/tour/state` | `molly_interfaces/TourState` | guide_node | latched |
| `/tour/cmd` | `molly_interfaces/TourCommand` | dashboard | |
| `/interaction/speech_status` | `molly_interfaces/SpeechStatus` | speech_node | latched |
| `/interaction/say_text` | `std_msgs/String` | anyone | speak, fire and forget |
| `/interaction/greeter` | `std_msgs/String` | interaction_node | latched JSON `{enabled, state, last_greet_age, gestures}` |
| `/interaction/conversation` | `std_msgs/String` | conversation_node | JSON `{question, answer, ms, ok}` per question |
| `/bridge/mode` | `std_msgs/String` | control_mux_node | latched: `auto`, `remote` |
| `/bridge/estop_engaged`, `/bridge/estop_reason` | `Bool`, `String` | control_mux_node | latched |
| `/bridge/protective_field` | `std_msgs/String` | control_mux_node | latched: `clear`, `limited`, `stop`, `stale`, `off` |
| `/bridge/nav_cmd_vel` | `geometry_msgs/Twist` | path_follower_node | to the mux |
| `/bridge/remote_cmd_vel` | `geometry_msgs/Twist` | teleop_udp_node, dashboard | to the mux, used in `remote` |
| `/bridge/estop_request` | `std_msgs/String` | any node | engages the e-stop with a reason |
| `/health/status` | `std_msgs/String` | health_monitor_node | latched JSON |
| `/<robot_ns>/cmd_vel` | `geometry_msgs/Twist` | control_mux_node | only the mux drives the robot |
| `/<robot_ns>/odom`, `/battery`, `/imu`, `/joint_states`, `/diagnostics` | | robot driver | [contract](../src/xtras/README.md) |

## Services

| Service | Type | Server |
|---|---|---|
| `/site/list` | `molly_interfaces/ListSites` | site_manager_node |
| `/site/load` | `molly_interfaces/LoadSite` | site_manager_node |
| `/site/save_poi` | `molly_interfaces/SavePoi` | site_manager_node |
| `/site/delete_poi` | `molly_interfaces/DeleteById` | site_manager_node |
| `/site/save_tour` | `molly_interfaces/SaveTour` | site_manager_node |
| `/site/delete_tour` | `molly_interfaces/DeleteById` | site_manager_node |
| `/mapping/start` | `molly_interfaces/StartMapping` | site_manager_node (drives grid_mapper_node and localization) |
| `/mapping/stop` | `std_srvs/Trigger` | site_manager_node |
| `/mapping/save` | `std_srvs/Trigger` | site_manager_node: saves the grid into the active site and reloads it |
| `/grid_mapper/start`, `/grid_mapper/stop` | `std_srvs/SetBool` (`data`: reset), `std_srvs/Trigger` | grid_mapper_node (internal) |
| `/grid_mapper/save` | `molly_interfaces/LoadFile` (`path`: site directory) | grid_mapper_node (internal) |
| `/grid_mapper/load` | `molly_interfaces/LoadFile` (`path`: site directory) | grid_mapper_node (internal): the saved map to continue |
| `/localization/start` | `std_srvs/Trigger` | localization_node: localize from a visible tag |
| `/localization/record_tags` | `std_srvs/Trigger` | localization_node: save visible tags into the active tags file |
| `/localization/load_tags` | `molly_interfaces/LoadFile` | localization_node: switch the tags file (`create`: new empty one) |
| `/localization/assume_origin` | `std_srvs/Trigger` | localization_node: `map → odom` = identity, source `origin` |
| `/localization/reset` | `std_srvs/Trigger` | localization_node: forget `map → odom` (another site), relocalize from its tags |
| `/planner/plan` | `molly_interfaces/PlanPath` | planner_node: plan only |
| `/planner/go_to` | `molly_interfaces/GoTo` | planner_node: plan, load, start |
| `/planner/cancel` | `std_srvs/Trigger` | planner_node: stop the walk |
| `/navigation/load_path` | `molly_interfaces/LoadPath` | path_loader_node: route CSV |
| `/navigation/set_path` | `molly_interfaces/SetPath` | path_loader_node: planned path |
| `/navigation/start_nav`, `/navigation/pause` | `std_srvs/SetBool` | path_follower_node |
| `/route/record` | `std_srvs/SetBool` | route_recorder_node |
| `/route/mark_stop_point` | `molly_interfaces/MarkStopPoint` | route_recorder_node |
| `/tour/control` | `molly_interfaces/TourControl` | guide_node |
| `/tour/get_state` | `molly_interfaces/GetTourState` | guide_node |
| `/interaction/say` | `molly_interfaces/Say` | speech_node |
| `/interaction/stop_speech` | `std_srvs/Trigger` | speech_node |
| `/interaction/gesture` | `molly_interfaces/Gesture` | interaction_node → `/<robot_ns>/<name>` |
| `/interaction/greet` | `std_srvs/Trigger` | interaction_node: greet now |
| `/interaction/set_greeter` | `std_srvs/SetBool` | interaction_node: automatic greeting on / off |
| `/interaction/ask` | `molly_interfaces/Ask` | conversation_node |
| `/bridge/set_mode` | `molly_interfaces/SetMode` | control_mux_node |
| `/bridge/estop` | `std_srvs/SetBool` | control_mux_node: `true` engages, `false` releases |
| `/<robot_ns>/stand_up`, `/sit_down`, `/emergency_stop`, gaits, gestures | `std_srvs/Trigger` | robot driver |

## Behaviour

**Mapping** (`mode:=mapping`, or `mapping/start` at runtime). `mapping/start {site, reset}`:
1. creates `map/<site>/` when new and makes it the active site;
2. switches localization to `map/<site>/tags.json` (`localization/load_tags`, `create: true`);
3. when localization has no source yet, takes the current pose as the map origin
   (`localization/assume_origin`);
4. starts `grid_mapper_node` (`reset`: empty grid; otherwise it continues the saved map).

A site that already has a map, tags or POIs keeps its frame: continuing it needs the robot
localized on one of its tags first (the site manager tries `localization/start` and refuses
otherwise), or `reset: true` to start it over (its files move to `map/<site>/backup-<time>/`).
Loading another site (`site/load`) resets localization (`localization/reset`).

While mapping, drive the robot (handheld REMOTE) through the whole place, stand still in view of
each AprilTag and `localization/record_tags`, and add POIs at the robot pose
(`site/save_poi` with `at_robot_pose`). `mapping/save` writes `map.pgm` + `map.yaml` and reloads
the site; `mapping/stop` ends the session.

**Guiding** (`mode:=guide`, default). The active site's map is published on `/map`, its tags are
loaded into localization (auto localization when a tag is visible), and:

- `planner/go_to {poi_id}`: A* on the map inflated by the robot footprint, line-of-sight
  shortcuts, resample + smoothing; the path goes to `navigation/set_path` and
  `navigation/start_nav`. The follower walks it (holonomic controller, path shift around
  people) and ends with the final x/y/yaw approach on the POI (`GOAL_REACHED`). When the follower
  reports `BLOCKED`, the planner replans around the obstacles of the safety scan.
- `/tour/control {START, tour_id}`: for each POI the guide walks there (`navigating`, pausing
  with `waiting_visitors` when nobody follows), performs the POI gesture
  (`performing_motion`), says the narration (`narrating`) and waits `dwell_sec` (`dwelling`);
  then the next one, and `returning` to `return_poi_id` at the end.
- Visitors in front of an idle robot are greeted (speech + gesture), questions typed on the
  dashboard are answered by the conversation model (optional).

The software e-stop pauses everything; the guide resumes only when told.
