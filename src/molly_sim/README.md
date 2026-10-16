# molly_sim

Software-in-the-loop quadruped. The production stack runs unchanged against a simulated robot:
localization, perception, navigation, the control mux with its protective field and e-stop, the
teleop gateway, the NETRA uplink and the health monitor. It needs no Gazebo and no GPU. One Python
node holds a 2D world, a kinematic body and a ray-cast LiDAR.

```bash
ros2 launch molly_sim sim.launch.py autostart:=true rviz:=true
```

The robot localizes itself and patrols a loop around a block of shops with one inspection stop.
`loop_route` is on, so it keeps patrolling until stopped.

## What is simulated

| On the robot | In `sim_robot_node` |
|---|---|
| robot driver: `/robot/cmd_vel` → motion, `/robot/odom`, `/robot/battery` | holonomic body with acceleration limits; stops when commands are older than `cmd_timeout_sec` |
| Livox MID-360 + FAST-LIO2 | ray-cast scan at three heights; `/cloud_registered_body` in `body`, `/Odometry` and `camera_init → body` (drift-free) |
| URDF | `base_link → livox_frame → livox_imu_frame` |
| AprilTag relocalization | `/initialpose` at the world's start pose until `localization/status` reports a source |
| VLM inspection | `sim/inspect` (`std_srvs/Trigger`) answers `clear` |
| People and boxes | `sim/obstacles` (`geometry_msgs/PoseArray`, map frame): circles of `obstacle_radius` |

Everything else is `molly/robot.launch.py`, started with `odometry:=external` and no
robot package (`robot_bringup:=''`). `sim_robot_node` implements the same interface a real robot
package must provide ([src/xtras/README.md](../xtras/README.md)). Person detection and the VLM are off. The site file
[config/sim_site.yaml](config/sim_site.yaml) switches the LiDAR input to the point cloud and sends
routes, the spool and the route state to `/tmp/molly_sim`.

| Argument | Default | |
|---|---|---|
| `world` | `config/worlds/mall_loop.yaml` | walls, pillars, start pose |
| `route` | `config/routes/mall_loop.csv` | route CSV, as on the robot |
| `autostart` | `false` | start the patrol once localized; otherwise `navigation/start_nav` |
| `rviz` | `false` | RViz with the `molly_rviz` panels |
| `cc_url`, `cc_token` | empty | report to NETRA as `sim-01` |
| `robot_ns` | `robot` | driver namespace |

## Things to try

```bash
# a person steps onto the bottom corridor: the protective field slows, then stops the robot
ros2 topic pub --once /sim/obstacles geometry_msgs/msg/PoseArray "{poses: [{position: {x: 2.0, y: -3.0}}]}"
ros2 topic echo /bridge/protective_field
# they walk away: the patrol goes on
ros2 topic pub --once /sim/obstacles geometry_msgs/msg/PoseArray "{poses: []}"

# software e-stop; after the release, navigation stays paused until resumed
ros2 service call /bridge/estop std_srvs/srv/SetBool "{data: true}"
ros2 service call /bridge/estop std_srvs/srv/SetBool "{data: false}"
ros2 service call /navigation/pause std_srvs/srv/SetBool "{data: false}"

ros2 topic echo /health/status
```

The handheld connects to the sim as it would to a robot (UDP 9870, token `sim`). From there you
can drive in REMOTE, record routes and mark stops. Recorded routes are saved under
`/tmp/molly_sim/routes`.

## Worlds

```yaml
walls:                     # polylines in the map frame, closed by repeating the first point
  - [[-6.0, -4.5], [6.0, -4.5], [6.0, 4.5], [-6.0, 4.5], [-6.0, -4.5]]
pillars:                   # [x, y, radius]
  - [0.0, 3.9, 0.2]
start: [-4.5, -3.0, 0.0]   # x, y, yaw_deg: where the robot starts and is localized
```

Walls and pillars are infinitely tall for the LiDAR, which is all the obstacle sectors need. A
route for a new world uses the normal route CSV (`x,y,yaw_deg,dwell_sec` in the map frame). You can
write one by hand or record it by driving the sim.

## Tests

- `test/test_world.py` covers the ray casting, the kinematics and the FAST-LIO frame maths.
- `test/test_sim_patrol.py` is the end-to-end test of the whole stack, and it runs in CI:
  1. It launches the simulation and puts a person on the route before the stop. The robot must
     slide around them without contact, stay within `path_shift.max_offset` of the route, and
     still complete the stop (`arrived` → `inspected` → `departed`).
  2. A crowd blocks the whole corridor: the robot must stop short of it, then drive on once it
     leaves.
  3. In REMOTE, it pushes the handheld stick straight at a person 1.5 m away. Avoidance is out of
     the loop there, so only the protective field can stop the robot short of them.
  4. It fails if any node died or printed a traceback, including during shutdown.

The end-to-end test takes about a minute. Set `MOLLY_SKIP_SIM=1` to skip it. Its launch log is
kept in `/tmp/molly_sim_*.log`.
