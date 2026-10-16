# molly_interfaces

Messages and services shared by the patrol stack.

| Kind | Name | Used by |
|---|---|---|
| msg | `SectorScan` | lidar_sector_node → obstacle_avoidance_node |
| msg | `AvoidanceCommand` | obstacle_avoidance_node → path_follower_node |
| msg | `Waypoint`, `WaypointPath` | path_loader_node → path_follower_node |
| msg | `NavigationStatus`, `MissionStatus`, `StopPointEvent` | path_follower_node → panel, bridge, fleet |
| srv | `LoadPath` | `navigation/load_path` |
| srv | `MarkStopPoint` | `route/mark_stop_point` |
| srv | `SetMode` | `bridge/set_mode` (`auto` \| `remote`) |
