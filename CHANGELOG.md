# Changelog

All notable changes to the Molly robot stack. Format: [Keep a Changelog](https://keepachangelog.com),
versions follow [SemVer](https://semver.org).

## [Unreleased]

### Fixed

- Dharma pinned to `main 2026-10-30` (`molly.repos`): nodes stop cleanly on Ctrl-C (no
  `take_message` RuntimeError, no publication after `destroy_node`), `planner/go_to` waits up to
  `pose_wait_sec` (2 s) for the robot pose right after localization, dharma_rviz builds with
  GCC 15. Later on the same day: `navigation/start` waits for the pose too, and a node still
  alive 8 s after a stop signal prints every thread's stack and exits with 1 instead of being
  SIGKILLed without a trace.

## [2.0.0] - 2026-10-28

Molly on Dharma 0.2 (ROS 2 Humble). The packages follow the Dharma layout and the bringup is
`molly_bringup`; settings, launch commands and some services changed (see Changed).

### Added

- `molly_bringup` (from the Dharma hub's `robot_bringup`): the Lite3 robot profile
  (`config/robot.yaml`, `robot.*` shared by every node), `config/stack.yaml` (Molly's overrides
  of the Dharma defaults, the field-tested values of 1.x), `config/site.example.yaml`,
  `robot.launch.py` and `sim.launch.py` with every argument validated, the parameter files
  merged into one, the started nodes added to the health monitor, unknown node sections
  reported.
- POIs and tours are Molly's own: `molly_interfaces` `Poi`, `Tour`, `SavePoi`, `SaveTour`,
  `DeleteById`, `GoToPoi`; `site_content_node` (molly_guide) keeps `<site>/pois.json` and
  `tours.json` next to the Dharma site and archives them with a mapping reset;
  `guide/go_to_poi` walks to one POI.
- The visitor tracker moved into `molly_interaction` (`visitor_tracker_node`, `Visitors`): the
  band above the floor from `robot.base_height`, the robot's own body from `robot.footprint`.
- Typed status for the operators: `SpeechStatus`, `GreeterStatus`, `TourState` with an
  `activity` (walking, gesture, narrating, dwelling, waiting for visitors).
- Docking through `dharma_navigation`'s docking node (handheld `dock`, `undock`,
  `dock_cancel`; the dock tag `charging_dock`); see the open items in the bringup's
  `stack.yaml`: the Lite3 driver does not report charging yet.
- Handheld protocol (still `molly-1`, wire format unchanged): `dock`, `undock`, `dock_cancel`,
  `nav_pause` / `nav_resume` on the Dharma services, `state.pause`, `state.estop_reasons`,
  `state.site.routes`, `state.health.speed_scale` and the docking state; `nav_speed` sets
  `path_follower_node` `speed.cruise`. See "Changes in Molly 2.0" in `PROTOCOL.md`.
- Repository: the Humble dev container with a VNC desktop (`compose.yaml`, `docker/`,
  `.devcontainer/`), `scripts/start_tmux.sh` with yaml layouts (`scripts/tmux/robot.yaml`,
  `sim.yaml`), pre-commit with the hub's ruff, prettier and clang-format settings, CI that
  builds the whole stack from `molly.repos` on Humble and runs the simulator tour.
- End-to-end test (`molly_bringup/test/e2e`): the simulator localizes, walks the demo tour to its
  end, the handheld takes over through the control mux in the middle and hands back to AUTO.

### Changed

- **Packages**: the bringup `molly` is now `molly_bringup`
  (`ros2 launch molly_bringup robot.launch.py`, `sim.launch.py`); WebRTC video moved into
  `molly_bridge`.
- **Settings** (`/etc/molly/molly.env`): `ROBOT_BRINGUP` is `ROBOT_DRIVER`, `MODE=mapping` is
  `START_MAPPING=true`, `ROUTE` loads a route at start; `scripts/setup.sh install config` adds the
  new keys. The directory `script/` is `scripts/`.
- **Site file** (`/etc/molly/site.yaml`): the robot's body and limits are no longer set per node
  (`field.front`, `max_vx`, `footprint.*`, `robot_radius`, `self_filter_*`, the teleop `max_*`):
  they come from the robot profile. Renamed: `path_follower_node.target_linear_velocity` is
  `speed.cruise`, `low_battery_percentage` is `low_battery`, `planner_node.inflation_radius` is
  `costmap.inflation_radius`, `obstacle_avoidance_node.emergency_distance` is
  `shared.emergency_distance`, `localization_node.tag_frames` is `tag.frames`,
  `lidar_sector_node.obstacle_z_min` is `filter.height_*` (above the floor). Compare yours with
  `config/site.example.yaml`.
- **Driving**: the control mux is the only publisher of `/robot/cmd_vel`; the handheld publishes
  `control/remote/handheld/cmd_vel`, the dashboard `control/remote/dashboard/cmd_vel`. Mode and
  e-stop: `control/set_mode` (`dharma_interfaces/srv/SetMode`, `mode` and `source`),
  `control/estop/engage` and `control/estop/release` (`std_srvs/srv/Trigger`) instead of the
  bridge's services.
- **Navigation**: the guide walks with `planner/go_to` and follows the typed `planner/status` and
  `navigation/status`; it pauses with navigation (REMOTE, e-stop) and resumes at the same stop.
- Every node is a `DharmaNode` (typed parameters checked at start and on `ros2 param set`, the
  robot profile); pure logic in `core/`, ROS adapters in `utils/`, thin `*_node.py`; ruff
  replaces flake8 and pep257.
- `lite3_ros2` submodule at its `main` (3c4e66e).

### Removed

- The handheld's `people_set`, `people` and `mission` messages and `MissionStatus`: the people
  layer and the mission status are gone from Dharma 0.2; the tour state carries what the
  handheld shows.
- `docker-compose.yaml` (now `compose.yaml`), `.flake8` and the ament lint tests.

## [1.0.0] - 2026-10-18

First release of the Lite3 tour guide.

### Added

- Tour guide stack on the Dharma packages (pinned in `molly.repos`, `script/setup.sh install
dharma`): odometry, localization, mapping and sites, the path finder and navigation, LiDAR
  perception, the control mux, the health monitor, RViz panels and the simulator. Molly keeps
  `molly` (bringup, configs, simulation launch and end-to-end tests), `molly_bridge`,
  `molly_guide`, `molly_interaction` and `molly_interfaces` (tour and speech types).
- Mapping (`dharma_mapping`): an occupancy grid built live from the FAST-LIO scan (log-odds, ray-cast free
  space, saved as a map_server map) and the site manager: one directory per place with its map,
  AprilTags, POIs, tours and routes; `/map`, `site/*` and `mapping/*` services; sites switched at
  runtime; `mode:=mapping site:=<name>` maps a new site from the start.
- Localization per site: `localization/load_tags`, `localization/assume_origin` for a new map,
  `localization/reset`, `localization/pose`.
- Path finder (`planner_node`): A\* on the map inflated by the robot footprint, start and goal
  snapping, line-of-sight shortcuts, replanning around the safety scan when the follower is
  blocked; `planner/go_to` to a POI or a pose, `navigation/set_path` on the path loader.
- `molly_guide`: tours from POI to POI with gestures, narration and dwell, intro and outro,
  pause, resume, skip, stop, retry or skip of unreachable POIs, waiting for visitors who fall
  behind; `/tour/control`, `/tour/state`, `/tour/get_state`.
- `molly_interaction`: speech with Piper voices or eSpeak NG, robot gestures, a greeter for
  visitors who stop in front, questions answered by a local LLM (Ollama) from the site's POIs.
- Visitor tracker: people around the robot from the 360° LiDAR, walls and pillars of the map
  filtered out, front and rear counts for the guide and the greeter.
- Handheld protocol Molly v1 (UDP 9880, `MLY1`): sites, mapping, POIs, tours, go-to and
  interaction commands, the map (PNG) and site streams.
- Web: rosbridge with a whitelist of names and web_video_server for the dashboard
  (`web:=true`), `script/setup.sh install web`; `install speech` (Piper pinned by checksum, the
  Indonesian and English voices), `install ollama` for the conversation model.
- The Lite3 robot package `lite3_ros2` as a submodule, its site file values in
  `site.example.yaml`.
- Simulation (`ros2 launch molly sim.launch.py` on `dharma_sim`): a ready site drawn from the
  world with a demo tour; end-to-end tests of a tour, mapping a new site, the path finder and
  a taught route.

### Docs

- Verified every README against the code; fixed stale references (Leg-KILO is not a submodule,
  the person detector feeds the visitor tracker, not a crowd heatmap, PRs target `main`, route
  looping is optional).
- One layout for the package READMEs (purpose, Nodes, Topics and services, Parameters, Launch,
  Test), a quick start in the README and an index of all docs in `docs/README.md`.

### Changed

- The camera person detector is gone (it was off by default); visitors come from the LiDAR.
- Modes are `guide` and `mapping` (`patrol` and `teach` still accepted); routes are no longer
  looped by default; the AprilTags file comes from the active site instead of `TAGS`.
