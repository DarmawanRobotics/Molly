# molly_navigation

The path finder and path following: walk to a POI on the site map, or repeat a taught route.

## Nodes

| Node | Role |
|---|---|
| `planner_node` | path finder: A* on the site map inflated by the robot, to a POI or a pose; loads the path into the follower and replans around blockages |
| `path_loader_node` | loads a route CSV or takes a planned path (`navigation/set_path`), resamples and smooths it, publishes it latched |
| `path_follower_node` | follows the path, speed profile, final x/y/yaw approach, stop points, optional route looping (`loop_route`), safety supervisor |
| `obstacle_avoidance_node` | reactive avoidance on `SectorScan` (`braitenberg`, `vfh`, `potential_field`, `follow_gap`), and the front-cone emergency stop in every mode |

## Path finder: `planner_node`

`planner/go_to {poi_id}` (or `{x, y, yaw}` in the map frame):

1. The site map (`/map`) becomes a costmap at `planning_resolution`: occupied cells, and unknown
   ones (`unknown_is_obstacle`), are max-pooled, then everything closer than `robot_radius` is
   lethal and cells up to `inflation_radius` cost more the closer they are to a wall.
2. A* (8-connected, no corner cutting) from the robot to the goal. A robot standing inside the
   inflation walks out of it first (`start_escape`); a goal inside an obstacle moves to the nearest
   free cell within `goal_tolerance`.
3. Line-of-sight shortcuts that never get closer to a wall than the search did, waypoints every
   `waypoint_spacing`, smoothing when it stays clear, heading along the path and the goal heading
   at the end.
4. `navigation/set_path`, then `navigation/start_nav`. The follower walks the path with path shift
   around people and converges on the goal pose.

The planner watches `navigation/status`: `GOAL_REACHED` within `arrival_tolerance` of the goal is
`arrived` (and the follower is stopped, `stop_on_arrival`); `BLOCKED` for `replan_after_sec` or
`OFF_PATH` plans again from where the robot is, with the people of the safety scan in the costmap,
up to `max_replans`; navigation stopped by someone else is `cancelled`. `planner/plan` returns a
plan without moving (dashboard preview).

Every parameter applies at runtime (Tuning panel or `ros2 param set`) except `control_rate` and the
service names.

## Following on a legged robot: `controller: holonomic` (default)

A robot dog can walk sideways and backwards, so the follower uses all three axes, x, y and yaw:

- **Translation** heads for a carrot `holonomic.lookahead_distance` further along the route, in any
  direction of the body. A cross-track error closes sideways, smoothly and without overshoot,
  while the heading stays on the route. The robot cannot weave around the route the way
  steering-only trackers do, because the heading does not depend on the cross-track error.
- **Heading** follows the route direction `holonomic.heading_lookahead` ahead, with a proportional,
  rate-limited yaw command.
- **When the route is beside or behind the robot**, it moves that way, e.g. after a stop point
  where it faced a shop, after avoidance, or when navigation starts off the route. It strafes at up
  to `holonomic.max_lateral_velocity` and backs up at up to `holonomic.max_reverse_velocity`, while
  it turns to face the route. Translation slows down while the heading is far off
  (`holonomic.min_alignment_scale`), so it mostly walks forward.

With `holonomic: false` it never strafes: it turns toward the carrot and walks forward. The
steering-only controllers `pure_pursuit`, `pid`, `mppi`, `lqr` and `stanley` are still available.

## Obstacles: `avoidance_mode: path_shift` (default with the holonomic controller)

Instead of steering away, the robot slides sideways onto a line parallel to the route and keeps
its heading. It uses the 360° `perception/safety_scan` and the robot footprint (`footprint.*`).

- It takes the offset nearest the route whose corridor is free, from the rear edge to
  `path_shift.look_ahead` beyond the front edge, with `path_shift.clearance` to spare.
- It never leaves the route by more than `path_shift.max_offset`. Without a gap, it slows down and
  stops `path_shift.stop_margin` before the obstacle, and reports `BLOCKED` after
  `blocked_timeout_sec`.
- It keeps its side while passing (`path_shift.switch_cost`, `path_shift.switch_margin`). It
  returns to the route only once the route corridor is free along its whole body for
  `path_shift.return_delay_sec`.
- The speed is limited so it can always stop before the first obstacle, both where it is and
  where it is sliding to.

The obstacle avoidance node still provides the front-cone emergency stop. With
`avoidance_mode: reactive`, or any other controller, its steering is blended in instead. A
holonomic robot then turns its direction of travel, not its heading.

## Starting a route

`navigation/start_nav` starts from wherever the robot is on the route, not only at its start:

- A paused walk resumes where it was while the robot is still within `off_path_slow_distance`
  of its progress; otherwise (route finished, robot moved) the robot is located on the route again.
- A robot further away than that walks slowly onto the nearest route point (`JOINING`, final
  approach gains) from up to `join_max_distance`, then follows. Further away the start is refused.
- Loading a route again, even the same file, starts it over; a redelivered route is ignored.

## Topics and services

| Interface | Type | |
|---|---|---|
| `planner/go_to` | `molly_interfaces/GoTo` | plan to a POI or pose, load it and walk |
| `planner/plan` | `molly_interfaces/PlanPath` | plan only |
| `planner/cancel` | `std_srvs/Trigger` | stop walking |
| `planner/status` | `molly_interfaces/PlannerStatus`, latched | `idle`, `planning`, `following`, `replanning`, `arrived`, `failed`, `cancelled`; remaining distance |
| `planner/path` | `nav_msgs/Path`, latched | the current plan |
| `planner/costmap` | `nav_msgs/OccupancyGrid`, latched | with `publish_costmap` (debug) |
| `navigation/start_nav`, `navigation/pause` | `std_srvs/SetBool` | run / hold |
| `navigation/load_path` | `molly_interfaces/LoadPath` | load a route |
| `navigation/set_path` | `molly_interfaces/SetPath` | take a planned path (not remembered as the route) |
| `navigation/status`, `navigation/mission_status` | status messages | state, lap, next stop, path offset |
| `navigation/stop_point_event` | `molly_interfaces/StopPointEvent` | arrived / inspected / departed / skipped |
| `navigation/markers`, `navigation/route_markers`, `navigation/avoidance_markers` | `MarkerArray` | debug drawing |
| `perception/safety_scan` | `molly_interfaces/SectorScan` | input of path shift |
| `cmd_vel` | `geometry_msgs/Twist` | output: `linear.x`, `linear.y`, `angular.z` (route it through `molly_bridge`) |

## Parameters

All parameters: [config/navigation_params.yaml](config/navigation_params.yaml). Molly's defaults
differ from Pashupati's: `loop_route: false` (paths end at their goal), `target_linear_velocity:
0.35` (visitors follow the robot) and `blocked_timeout_sec: 10` (the planner replans after that).

| `planner_node` parameter | Default | |
|---|---|---|
| `robot_radius` | `0.35` | m, lethal distance to obstacles |
| `inflation_radius`, `cost_scaling` | `0.7`, `3.0` | wall clearance preferred up to here |
| `unknown_is_obstacle` | `true` | never plan through unmapped space |
| `planning_resolution` | `0.1` | m |
| `goal_tolerance`, `start_escape` | `0.5`, `1.0` | m |
| `arrival_tolerance`, `stop_on_arrival` | `0.35`, `true` | |
| `replan_after_sec`, `max_replans`, `use_live_obstacles` | `3.0`, `5`, `true` | |

## Launch

```bash
ros2 launch molly_navigation navigation.launch.py cmd_vel_topic:=/bridge/nav_cmd_vel
ros2 service call /planner/go_to molly_interfaces/srv/GoTo "{poi_id: fountain}"
ros2 topic echo /planner/status
```

`navigation.launch.py` starts `path_loader_node`, `obstacle_avoidance_node`, `planner_node` and
`path_follower_node`; `path_follower.launch.py` and `obstacle_avoidance.launch.py` start one part.

## Test

`colcon test --packages-select molly_navigation` runs the planner (costmap, A*, shortcuts, a room
with a door, the node against a fake follower), the controller, route and path-shift
simulations. The simulator's end-to-end test ([molly_sim](../molly_sim)) drives the whole
stack past a person on the route and stops it for a crowd.
