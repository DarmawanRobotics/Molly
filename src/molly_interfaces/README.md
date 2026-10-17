# molly_interfaces

Messages and services shared by the Molly stack, the handheld gateway and the dashboard
(rosbridge). Definitions: [msg/](msg) and [srv/](srv).

## Navigation (from Pashupati)

| Kind | Name | Used by |
|---|---|---|
| msg | `SectorScan` | lidar_sector_node → obstacle_avoidance_node, path follower, control mux |
| msg | `AvoidanceCommand` | obstacle_avoidance_node → path_follower_node |
| msg | `Waypoint`, `WaypointPath` | path_loader_node → path_follower_node |
| msg | `NavigationStatus`, `MissionStatus`, `StopPointEvent` | path_follower_node → bridge, planner, guide |
| srv | `LoadPath` | `navigation/load_path` (route CSV) |
| srv | `SetPath` | `navigation/set_path` (planned path) |
| srv | `MarkStopPoint` | `route/mark_stop_point` |
| srv | `SetMode` | `bridge/set_mode` (`auto` \| `remote`) |

## Sites, mapping and the path finder

| Kind | Name | Used by |
|---|---|---|
| msg | `Poi`, `PoiArray` | `site/pois` (latched) |
| msg | `Tour`, `TourArray` | `site/tours` (latched) |
| msg | `SiteStatus` | `site/status` (latched) |
| msg | `MappingStatus` | `mapping/status` (latched) |
| msg | `PlannerStatus` | `planner/status` (latched) |
| srv | `ListSites`, `LoadSite` | `site/list`, `site/load` |
| srv | `StartMapping` | `mapping/start` |
| srv | `SavePoi`, `SaveTour`, `DeleteById` | `site/save_poi`, `site/save_tour`, `site/delete_poi`, `site/delete_tour` |
| srv | `LoadFile` | `localization/load_tags` |
| srv | `PlanPath`, `GoTo` | `planner/plan`, `planner/go_to` |

## Guide and interaction

| Kind | Name | Used by |
|---|---|---|
| msg | `TourCommand`, `TourState` | `/tour/cmd`, `/tour/state` (latched) |
| msg | `SpeechStatus` | `interaction/speech_status` (latched) |
| msg | `Visitors` | `perception/visitors` |
| srv | `TourControl`, `GetTourState` | `/tour/control`, `/tour/get_state` |
| srv | `Say`, `Ask`, `Gesture` | `interaction/say`, `interaction/ask`, `interaction/gesture` |

The full list of topics and services is in [docs/interfaces.md](../../docs/interfaces.md).
