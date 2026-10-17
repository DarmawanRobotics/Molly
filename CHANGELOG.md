# Changelog

All notable changes to the Molly robot stack. Format: [Keep a Changelog](https://keepachangelog.com),
versions follow [SemVer](https://semver.org).

## [Unreleased]

### Added
- Workspace started from Pashupati `dev@be99273`: odometry, localization, perception, navigation,
  bridge, health, RViz panels and the simulator, renamed to `molly_*`. The NETRA uplink and the
  VLM inspection are left out.
- `molly_mapping`: an occupancy grid built live from the FAST-LIO scan (log-odds, ray-cast free
  space, saved as a map_server map) and the site manager: one directory per place with its map,
  AprilTags, POIs, tours and routes; `/map`, `site/*` and `mapping/*` services; sites switched at
  runtime; `mode:=mapping site:=<name>` maps a new site from the start.
- Localization per site: `localization/load_tags`, `localization/assume_origin` for a new map,
  `localization/reset`, `localization/pose`.
- Path finder (`planner_node`): A* on the map inflated by the robot footprint, start and goal
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
- Simulator: a ready site drawn from the world with a demo tour; end-to-end test of a tour,
  mapping a new site and the path finder.

### Docs
- Verified every README against the code; fixed stale references (Leg-KILO is not a submodule,
  the person detector feeds the visitor tracker, not a crowd heatmap, PRs target `main`, route
  looping is optional).
- One layout for the package READMEs (purpose, Nodes, Topics and services, Parameters, Launch,
  Test), a quick start in the README and an index of all docs in `docs/README.md`.

### Changed
- Modes are `guide` and `mapping` (`patrol` and `teach` still accepted); routes are no longer
  looped by default; the AprilTags file comes from the active site instead of `TAGS`.
