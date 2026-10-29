# xtras: third-party code and the robot package

All the robot-specific parts live in one **robot package**: driver, URDF and sensor launch.
Molly's own packages do not depend on a robot brand. The Deep Robotics Lite3 package is
[lite3_ros2](https://github.com/DarmawanRobotics/lite3_ros2), a submodule here.

| Path                        | What                                                      |
| --------------------------- | --------------------------------------------------------- |
| `drivers/livox_ros_driver2` | Livox Mid-360 driver (submodule)                          |
| `slam/fast_lio2`            | FAST-LIO2, `odometry:=fast_lio` (submodule)               |
| `robots/lite3_ros2`         | Lite3 driver, URDF, Mid-360 and camera launch (submodule) |

```bash
git submodule update --init --recursive
script/setup.sh install robot     # ROBOT_SETUP: the robot package's own installer
```

In `/etc/molly/molly.env`:

| Variable         | Meaning                                                                                                                                  |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `ROBOT_BRINGUP`  | `<package>[/<file>.launch.py]` (default file `bringup.launch.py`), started with `namespace:=<ROBOT_NS>`; Lite3: `lite3_bringup`          |
| `ROBOT_NS`       | namespace of the driver topics and services (default `robot`)                                                                            |
| `ROBOT_SETUP`    | optional: the package's installer (`<script> check\|install`); Lite3: `src/xtras/robots/lite3_ros2/lite3_bringup/scripts/lite3_setup.sh` |
| `ROBOT_PRESTART` | optional: a command run before the stack starts, e.g. to stop a vendor autostart that would fight for the robot                          |
| `ROBOT_IP`       | optional: address of the robot's motion computer, checked by `script/setup.sh check` (Lite3: `192.168.1.120`)                            |

Robot-specific tuning goes in `/etc/molly/site.yaml` (see [below](#per-robot-parameters)).

## What the robot package must provide

[`dharma_sim`](https://github.com/DarmawanRobotics/dharma_sim) implements exactly this interface for the simulator. Use it as the
reference.

### Launch

`launch/<file>.launch.py` takes a `namespace` argument and starts:

- the driver
- `robot_state_publisher` with the URDF (joint states from the driver, in the namespace)
- the sensor drivers: Livox Mid-360 and the camera

It must not start any Molly node.

### Topics and services, all under `/<ROBOT_NS>/`

| Name                                                            | Type                              | Direction | Contract                                                                                                                                                                                                                                                                                                 |
| --------------------------------------------------------------- | --------------------------------- | --------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `cmd_vel`                                                       | `geometry_msgs/Twist`             | subscribe | Body frame (`base_link`): `linear.x` forward m/s, `linear.y` left m/s, `angular.z` yaw rad/s (counter-clockwise). Ignore `linear.y` if the robot cannot strafe. **Stop if no command arrives for 0.5 s.** Only `control_mux_node` publishes here, after the e-stop, speed envelope and protective field. |
| `odom`                                                          | `nav_msgs/Odometry`               | publish   | The robot's own velocity estimate, ≥ 10 Hz. Molly uses `twist`, in `base_link`. Cross-checks the LiDAR odometry.                                                                                                                                                                                         |
| `battery`                                                       | `sensor_msgs/BatteryState`        | publish   | `percentage` 0–1 and `voltage`, ≥ 0.2 Hz.                                                                                                                                                                                                                                                                |
| `imu`                                                           | `sensor_msgs/Imu`                 | publish   | Optional: the robot's attitude, for the dashboard.                                                                                                                                                                                                                                                       |
| `joint_states`                                                  | `sensor_msgs/JointState`          | publish   | Optional: legs in RViz and the dashboard.                                                                                                                                                                                                                                                                |
| `diagnostics`                                                   | `diagnostic_msgs/DiagnosticArray` | publish   | Optional: driver state for the handheld and the dashboard.                                                                                                                                                                                                                                               |
| `stand_up`, `sit_down`, `emergency_stop`                        | `std_srvs/Trigger`                | serve     | The handheld buttons. Add more with `robot_commands`.                                                                                                                                                                                                                                                    |
| gestures, e.g. `hello`, `nod`, `bow`, `look_left`, `look_right` | `std_srvs/Trigger`                | serve     | Optional: what `interaction/gesture` may call (`interaction_node` `gestures`). The robot must refuse while it walks.                                                                                                                                                                                     |

Navigation can also call two `std_srvs/Trigger` services of the driver when walking starts and
stops: `auto_mode_on_service` / `auto_mode_off_service` in `navigation_params.yaml`. Use them on
robots that must hand motion control from the vendor remote to the SDK.

### TF (URDF)

```
base_link ─┬─ livox_frame ── livox_imu_frame     (LiDAR and its IMU)
           └─ camera_link ── camera_color_optical_frame
```

- `base_link` is the centre of the body: x forward, z up (REP-103).
- `livox_imu_frame` is where FAST-LIO's `body` sits. `localization_node` needs `base_link → livox_imu_frame` before it starts.
- **The driver must not publish `odom → base_link`.** Molly owns `map → odom → camera_init → body → base_link`, so turn off the driver's odometry TF.

### Sensors

- **LiDAR: Livox Mid-360** through `livox_ros_driver2`, with `xfer_format: 1`. The cloud goes to `/livox/lidar`, the IMU to `/livox/imu`. FAST-LIO, the obstacle sectors, the visitor tracker and the odometry start gate use them.
  - Another LiDAR needs its own FAST-LIO preprocess settings (`dharma_odometry/config/fast_lio.yaml`)
    and `input_type: pointcloud2` for `lidar_sector_node` and `visitor_tracker_node`.
- **Camera** (AprilTag localization, people, handheld and dashboard video). Whatever the source
  (RealSense, the robot's own RTSP camera, USB), it publishes:
  - `/camera/camera/color/image_raw` (rectified when the lens distorts: the tag detector expects it)
  - `/camera/camera/color/image_raw/compressed`
  - `/camera/camera/color/camera_info`
  - with `frame_id` `camera_color_optical_frame`

## Per-robot parameters

These go in `/etc/molly/site.yaml`, applied on top of every package default:

```yaml
control_mux_node:
  ros__parameters:
    field.front: 0.40 # m, base_link to the front edge of the body
    field.rear: 0.40
    field.half_width: 0.28
    field.max_decel: 0.6 # m/s^2, MEASURE: braking from full speed on this floor
    field.reaction_time: 0.25 # s, command to the robot actually braking
    max_vx: 1.0 # m/s, nothing may command more
    max_vy: 0.5 # 0.0 for a robot that cannot strafe
    max_wz: 1.5 # rad/s

path_follower_node:
  ros__parameters:
    holonomic: true # false if the robot cannot strafe

planner_node:
  ros__parameters:
    robot_radius: 0.35 # m, half the body diagonal plus a margin

lidar_sector_node:
  ros__parameters:
    obstacle_z_min: -0.20 # -(height of base_link above the floor) + 0.10
    self_filter_x_min: -0.40 # the robot's own body and legs, ignored
    self_filter_x_max: 0.40
    self_filter_y_min: -0.28
    self_filter_y_max: 0.28

teleop_udp_node:
  ros__parameters:
    max_vx: 1.0
    max_vy: 0.5
    max_wz: 1.5
    robot_commands: [stand_up, sit_down, emergency_stop] # + this robot's own Trigger services

interaction_node:
  ros__parameters:
    gestures: [hello, nod, bow, look_left, look_right] # the driver's gesture services
```

## Checklist for a new robot

1. **Robot package alone:**
   ```bash
   ros2 launch <pkg> <file> namespace:=<ns>
   ros2 topic hz /<ns>/odom
   ros2 topic hz /livox/lidar
   ros2 topic hz /livox/imu
   ros2 run tf2_ros tf2_echo base_link livox_imu_frame
   ```
2. **Motion and the watchdog:** with the robot lifted or in an open area, this should drive it
   slowly forward. When the publisher stops, the robot must stop within 0.5 s.
   ```bash
   ros2 topic pub -r 20 /<ns>/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
   ```
   Then `{angular: {z: 0.3}}`: the robot must turn **left** (counter-clockwise seen from above).
3. **Whole stack:**
   ```bash
   ros2 launch molly robot.launch.py robot_bringup:=<pkg> robot_ns:=<ns>
   ros2 topic echo /health/status
   ```
   `/health/status` should report `level: ok` once odometry is running.
4. **Braking distance:** measure it, then set `field.max_decel` and `field.reaction_time`
   before the first tour.
