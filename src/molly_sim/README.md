# molly_sim

Software-in-the-loop quadruped for the whole stack, with end-to-end tests.

The production stack runs unchanged against a simulated robot:
localization, mapping and sites, perception and the visitor tracker, the path finder and
navigation, the guide, speech (silent), the control mux with its protective field and e-stop,
the teleop gateway and the health monitor. It needs no Gazebo and no GPU. One Python node holds a
2D world, a kinematic body and a ray-cast LiDAR.

## Launch

```bash
ros2 launch molly_sim sim.launch.py rviz:=true
# a tour through the demo site
ros2 service call /tour/control molly_interfaces/srv/TourControl "{command: {kind: 0, tour_id: demo}}"
ros2 topic echo /tour/state
```

The site `mall_loop` (under `/tmp/molly_sim/sites`) is drawn from the world the first time:
its occupancy map, two POIs (`east_gate`, `fountain`) and the tour `demo`. The robot localizes
itself, so a tour or `planner/go_to` runs right away. `autostart:=true` walks the taught route
of `config/routes/mall_loop.csv` instead (teach and repeat, one stop point).

| Argument | Default | |
|---|---|---|
| `world` | `config/worlds/mall_loop.yaml` | walls, pillars, start pose |
| `site` | `mall_loop` | site, drawn from the world when it has no map yet |
| `sites_dir` | `/tmp/molly_sim/sites` | |
| `route` | `config/routes/mall_loop.csv` | route CSV, as on the robot |
| `autostart` | `false` | walk the route once localized; otherwise `navigation/start_nav` |
| `web` | `false` | rosbridge + web_video_server, for the dashboard |
| `rviz` | `false` | RViz with the `molly_rviz` panels |
| `robot_ns` | `robot` | driver namespace |

## What is simulated

| On the robot | In `sim_robot_node` |
|---|---|
| robot driver: `/robot/cmd_vel` → motion, `/robot/odom`, `/robot/battery` | holonomic body with acceleration limits; stops when commands are older than `cmd_timeout_sec` |
| Livox MID-360 + FAST-LIO2 | ray-cast scan at three heights; `/cloud_registered_body` in `body`, `/Odometry` and `camera_init → body` (drift-free) |
| URDF | `base_link → livox_frame → livox_imu_frame` |
| AprilTag localization | `/initialpose` at the robot's true pose whenever `localization/status` is `none` (every simulated site shares the world frame) |
| stop-point hook of routes | `sim/inspect` (`std_srvs/Trigger`) answers `clear` |
| People and boxes | `sim/obstacles` (`geometry_msgs/PoseArray`, map frame): circles of `obstacle_radius`, seen by the LiDAR and the visitor tracker |

Everything else is `molly/robot.launch.py`, started with `odometry:=external` and no robot
package (`robot_bringup:=''`). `sim_robot_node` implements the same interface a real robot
package must provide ([src/xtras/README.md](../xtras/README.md)). The site file
[config/sim_site.yaml](config/sim_site.yaml) switches the LiDAR input to the point cloud, makes
speech silent and fast, does not wait for followers, and keeps every file under `/tmp/molly_sim`.

## Usage

```bash
# walk to a POI with the path finder
ros2 service call /planner/go_to molly_interfaces/srv/GoTo "{poi_id: fountain}"
ros2 topic echo /planner/status

# map a new site from where the robot stands, save it
ros2 service call /mapping/start molly_interfaces/srv/StartMapping "{site: scratch, reset: true}"
ros2 service call /mapping/save std_srvs/srv/Trigger
ros2 service call /mapping/stop std_srvs/srv/Trigger
ros2 service call /site/load molly_interfaces/srv/LoadSite "{site: mall_loop}"

# a person steps into the corridor: the protective field slows, then stops the robot
ros2 topic pub --once /sim/obstacles geometry_msgs/msg/PoseArray "{poses: [{position: {x: 2.0, y: -3.0}}]}"
ros2 topic echo /perception/visitors
ros2 topic pub --once /sim/obstacles geometry_msgs/msg/PoseArray "{poses: []}"

# software e-stop; after the release, navigation stays paused until resumed
ros2 service call /bridge/estop std_srvs/srv/SetBool "{data: true}"
ros2 service call /bridge/estop std_srvs/srv/SetBool "{data: false}"
ros2 service call /navigation/pause std_srvs/srv/SetBool "{data: false}"
```

The handheld connects to the sim as it would to a robot (UDP 9880, token `sim`), and the
dashboard with `web:=true` (rosbridge and web_video_server installed).

## Worlds

```yaml
walls:                     # polylines in the map frame, closed by repeating the first point
  - [[-6.0, -4.5], [6.0, -4.5], [6.0, 4.5], [-6.0, 4.5], [-6.0, -4.5]]
pillars:                   # [x, y, radius]
  - [0.0, 3.9, 0.2]
start: [-4.5, -3.0, 0.0]   # x, y, yaw_deg: where the robot starts
```

Walls and pillars are infinitely tall for the LiDAR. `molly_sim/site_fixture.py` draws them into
the site map (walls 10 cm thick, pillars as disks) with the demo POIs and tour.

## Tests

- `test/test_world.py` covers the ray casting, the kinematics and the FAST-LIO frame maths;
  `test/test_site_fixture.py` the drawn site.
- `test/test_sim_guide.py`, end to end in CI, on a fresh sites directory:
  1. The demo tour walks to both POIs, narrates at each (silent speech) and ends idle at the last
     POI, within 30 cm and 15°.
  2. `mapping/start` on a new site builds a grid from the simulated LiDAR, `mapping/save` writes
     it, `mapping/stop` leaves the new site active with its map.
  3. `site/load` brings the demo site back (the robot localizes again) and `planner/go_to` walks
     around the shops back to the start pose.
- `test/test_sim_patrol.py`, end to end in CI, teach and repeat with obstacles:
  1. A person on the route: the robot slides around them without contact, stays within
     `path_shift.max_offset` of the route, and still completes the stop.
  2. A crowd blocks the whole corridor: the robot stops short of it, then drives on once it leaves.
  3. In REMOTE, the handheld stick pushes straight at a person 1.5 m away: only the protective
     field stops the robot short of them.
- Both fail if any node died or printed a traceback, including during shutdown.

The end-to-end tests take about three minutes together. Set `MOLLY_SKIP_SIM=1` to skip them.
Their launch logs are kept in `/tmp/molly_sim_*.log` and `/tmp/molly_guide_*.log`.
