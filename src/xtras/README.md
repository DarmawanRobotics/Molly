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
scripts/setup.sh install robot     # ROBOT_SETUP: the robot package's own installer
```

In `/etc/molly/molly.env`:

| Variable         | Meaning                                                                                                                                  |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `ROBOT_DRIVER`   | `<package>[/<file>.launch.py]` (default file `bringup.launch.py`), started with `namespace:=<ROBOT_NS>`; Lite3: `lite3_bringup`          |
| `ROBOT_NS`       | namespace of the driver topics and services (default `robot`)                                                                            |
| `ROBOT_SETUP`    | optional: the package's installer (`<script> check\|install`); Lite3: `src/xtras/robots/lite3_ros2/lite3_bringup/scripts/lite3_setup.sh` |
| `ROBOT_PRESTART` | optional: a command run before the stack starts, e.g. to stop a vendor autostart that would fight for the robot                          |
| `ROBOT_IP`       | optional: address of the robot's motion computer, checked by `scripts/setup.sh check` (Lite3: `192.168.1.120`)                           |

The robot's body, limits and frames are the robot profile (see [below](#per-robot-parameters)).

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

Navigation can also call `std_srvs/Trigger` services of the driver: `path_follower_node`
`hooks.start` before walking (robots that must hand motion control from the vendor remote to
the SDK), and `docking_node` `hooks.sit`, `hooks.charge`, `hooks.stop_charge`, `hooks.stand` for
docking onto a charger. Docking also needs `battery` with `power_supply_status` (charging or
not); the Lite3 driver sits and stands but does not report charging yet.

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
  - Another LiDAR needs its own FAST-LIO preprocess settings (`dharma_odometry/config/fast_lio.yaml`),
    `input.type: pointcloud2` for `lidar_sector_node` and `input_type: pointcloud2` for
    `visitor_tracker_node`.
- **Camera** (AprilTag localization and docking, handheld and dashboard video). Whatever the source
  (RealSense, the robot's own RTSP camera, USB), it publishes:
  - `/camera/camera/color/image_raw` (rectified when the lens distorts: the tag detector expects it)
  - `/camera/camera/color/image_raw/compressed`
  - `/camera/camera/color/camera_info`
  - with `frame_id` `camera_color_optical_frame`

## Per-robot parameters

The robot profile (`robot.*`), read by every Dharma and Molly node, describes the body:
[molly_bringup/config/robot.yaml](../molly_bringup/config/robot.yaml) for the Lite3. Another
robot gets its own file, passed with `robot_params:=` (the launch validates it):

```yaml
/**:
  ros__parameters:
    robot:
      name: my_robot
      kinematics: omni # omni (strafes) or diff
      base_height: 0.30 # m, base_link above the floor (the LiDAR's obstacle and visitor bands)
      footprint: # m, base_link to the body edges, legs included
        front: 0.40
        rear: 0.40
        half_width: 0.28
      limits:
        max_vx: 1.0 # m/s, nothing may command more (the mux, the follower, the handheld)
        max_vy: 0.5 # 0.0 for a robot that cannot strafe
        max_wz: 1.5 # rad/s
        max_accel: 0.5 # m/s^2
        max_decel: 0.6 # m/s^2, MEASURE: braking from full speed on this floor
        max_yaw_accel: 1.5 # rad/s^2
        reaction_time: 0.25 # s, MEASURE: command to the robot actually braking
      frames:
        map: map
        odom: odom
        base: base_link
```

What is not the body goes in `/etc/molly/site.yaml`
([example](../molly_bringup/config/site.example.yaml)): the driver's services for the handheld
and the gestures:

```yaml
teleop_udp_node:
  ros__parameters:
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
   ros2 launch molly_bringup robot.launch.py robot_driver:=<pkg> robot_ns:=<ns> \
     robot_params:=<profile.yaml>
   ros2 topic echo /health/status
   ```
   `/health/status` should report `level: ok` once odometry is running.
4. **Braking distance:** measure it, then set `robot.limits.max_decel` and
   `robot.limits.reaction_time` before the first tour.
