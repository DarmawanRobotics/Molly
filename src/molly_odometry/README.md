# molly_odometry

LiDAR-inertial odometry on the Livox Mid-360 and teach-and-repeat route recording.

The odometry publishes `camera_init → body` and `/Odometry`. It starts at the robot's pose on every
launch and drifts slowly. `map → odom` is set by [molly_localization](../molly_localization).

`odometry.launch.py` starts four things.

**The estimator, chosen with `method:=`** (default `$ODOMETRY` or `fast_lio`):

| Method | Source | Notes |
|---|---|---|
| `fast_lio` | `src/xtras/slam/fast_lio2` | [config/fast_lio.yaml](config/fast_lio.yaml) is tuned indoors: 0.3 m voxels, 4 iterations, 50 m range, no ever-growing `/path` |
| `leg_kilo` | `src/xtras/slam/leg_kilo` (optional) | [config/leg_kilo.yaml](config/leg_kilo.yaml): Gaussian voxel map (plane + NDT), two-stage ESKF built for legged-robot dynamics, headless |
| `external` | started elsewhere | no estimator, the monitor and recorder still run; used by [molly_sim](../molly_sim), which publishes `/Odometry` itself |

**`wait_still`**, the launch prefix of the estimator. Both estimators estimate gravity and the IMU
biases from their first IMU samples and assume the robot is not moving. `wait_still` holds the
start until the IMU has been still and level for `still_sec`. It runs again on every respawn.

**`odometry_monitor_node`**, which publishes `odometry/status`. It checks for:

- stale `/Odometry`
- jumps
- z drift on a flat floor, which points to a tilted start
- LIO speed that disagrees with the robot's own `/<robot_ns>/odom`, which points to a degenerate corridor
  or slip
- restarts, published on `odometry/restarted`; `localization_node` then drops `map → odom`

**`route_recorder_node`** (`record:=true`) for teach. It samples `map → base_link` and writes
`map/<site>/routes/<date>_<time>_path.csv` (`x,y,yaw_deg,dwell_sec`) into the active site
(`site/status`, `sites_dir`), or `map/<date>/<time>_path.csv` while no site is active.

With `localization:=true tags_config_file:=…`, the launch also starts `localization_node`
(development, `script/start_tmux.sh`).

| Interface | Type | |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | estimator, `camera_init → body` |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | deskewed scan in `body`, feeds lidar_sector_node |
| `odometry/status` | `std_msgs/String`, latched | `initializing`, `ok`, `degraded: …`, `lost: …` |
| `odometry/restarted` | `std_msgs/Empty` | odom frame reset |
| `route/record` | `std_srvs/SetBool` | start / stop recording |
| `route/mark_stop_point` | `molly_interfaces/MarkStopPoint` | stop point at the current pose |
| `route/saved` | `std_msgs/String` | path of the CSV just written |

```bash
ros2 launch molly_odometry odometry.launch.py                          # FAST-LIO, guide
ros2 launch molly_odometry odometry.launch.py method:=leg_kilo save_map:=true
ros2 launch molly_odometry odometry.launch.py method:=leg_kilo leg_kilo_viewer:=true  # with a display
ros2 topic echo /odometry/status
```

## Leg-KILO

Leg-KILO is a submodule from the DarmawanRobotics fork, branch `molly`, which adds options for
running on the robot:

- `--viewer=false`
- `odom_frame` / `body_frame`
- `publish_path: false`
- `backend_enable`
- the `/cloud_registered_body` output
- `COLCON_IGNORE` on its bundled copy of `livox_ros_driver2`

Build dependencies come from `script/setup.sh install legkilo`. Without the submodule, the rest of
the workspace builds as before.

Leg-KILO runs in **LIO** mode here. Its `KILO` mode adds leg kinematics, but it needs per-foot
contact forces in a Unitree `HighState`, which most robot drivers do not publish. On a
wheel-legged robot, the wheels also break its static-foot assumption.

In teach (`save_map:=true`) its backend runs: submaps, loop closure, and recording under
`leg_kilo/legkilo/result/temp/`. While guiding the backend is off.

Gate and monitor thresholds and recorder settings:
[config/odometry_params.yaml](config/odometry_params.yaml).
