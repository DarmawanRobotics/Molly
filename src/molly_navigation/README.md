# molly_navigation

Route following for patrols.

| Node | Role |
|---|---|
| `path_loader_node` | loads a route CSV, resamples and smooths it between stop points, publishes it latched |
| `path_follower_node` | follows the route, speed profile, final x/y/yaw approach, inspection at stop points, loop patrol, safety supervisor |
| `obstacle_avoidance_node` | reactive avoidance on `SectorScan` (`braitenberg`, `vfh`, `potential_field`, `follow_gap`), and the front-cone emergency stop in every mode |

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

## Starting a patrol

`navigation/start_nav` starts from wherever the robot is on the route, not only at its start:

- A paused patrol resumes where it was while the robot is still within `off_path_slow_distance`
  of its progress; otherwise (route finished, robot moved) the robot is located on the route again.
- A robot further away than that walks slowly onto the nearest route point (`JOINING`, final
  approach gains) from up to `join_max_distance`, then follows. Further away the start is refused.
- Loading a route again, even the same file, starts it over; a redelivered route is ignored.

## Interfaces

| Interface | Type | |
|---|---|---|
| `navigation/start_nav`, `navigation/pause` | `std_srvs/SetBool` | run / hold |
| `navigation/load_path` | `molly_interfaces/LoadPath` | load a route |
| `navigation/status`, `navigation/mission_status` | status messages | state, lap, next stop, path offset |
| `navigation/stop_point_event` | `molly_interfaces/StopPointEvent` | arrived / inspected / departed / skipped |
| `navigation/markers`, `navigation/route_markers`, `navigation/avoidance_markers` | `MarkerArray` | debug drawing |
| `perception/safety_scan` | `molly_interfaces/SectorScan` | input of path shift |
| `cmd_vel` | `geometry_msgs/Twist` | output: `linear.x`, `linear.y`, `angular.z` (route it through `molly_bridge`) |

Parameters: [config/navigation_params.yaml](config/navigation_params.yaml).
`colcon test --packages-select molly_navigation` runs the controller, route and path-shift
simulations. The simulator's end-to-end test ([molly_sim](../molly_sim)) drives the whole
stack past a person on the route and stops it for a crowd.
