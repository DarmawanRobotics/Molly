# molly_bridge

Who drives the robot, and how the handheld talks to it without ROS.

## Nodes

### control_mux_node

The single gate between every velocity source and the robot: forwards either navigation or teleop
velocity, and holds the robot still while the software e-stop is engaged.

| | Name | Type |
|---|---|---|
| sub | `bridge/nav_cmd_vel` | `geometry_msgs/Twist` (path follower output) |
| sub | `bridge/remote_cmd_vel` | `geometry_msgs/Twist` (teleop) |
| sub | `bridge/estop_request` | `std_msgs/String`: engage the e-stop with this reason, no reply |
| pub | `cmd_vel` | `geometry_msgs/Twist` (remap to the robot, e.g. `/robot/cmd_vel`) |
| pub | `bridge/mode` | `std_msgs/String`, latched |
| pub | `bridge/estop_engaged`, `bridge/estop_reason` | `std_msgs/Bool`, `std_msgs/String`, latched |
| srv | `bridge/set_mode` | `molly_interfaces/SetMode` (`auto` \| `remote`) |
| srv | `bridge/estop` | `std_srvs/SetBool`: `true` engages, `false` releases |

In `remote` mode navigation is paused (and resumed when switching back). The robot gets one zero
command as soon as the active source goes silent: teleop after `remote_timeout_sec`, navigation
after `nav_timeout_sec`.

**Speed envelope.** No source can command more than `max_vx` / `max_vy` / `max_wz` (1.0 m/s,
0.5 m/s, 1.5 rad/s): x and y are scaled down together, so the robot keeps its heading. A command
with a NaN or infinite value, from a controller bug or a bad route, is replaced by zero and logged.

**Protective field.** Every forwarded command is limited to a speed the robot can still stop at,
after `field.reaction_time` and braking at `field.max_decel`, `field.margin` short of the nearest
obstacle in its swept path (box `field.front` / `field.rear` / `field.half_width`, from the 360°
`perception/safety_scan`). It works in every direction, so sideways and backward motion during a
holonomic stop-point approach are protected as well as forward driving. Without a fresh scan
(`safety_scan_timeout_sec`) navigation stops; a blind LiDAR reports every sector blocked. Rotation
is not limited. The handheld in REMOTE is not limited by default, the operator watches the robot
(`field_in_remote: true` protects it too; without a fresh scan it then creeps at
`stale_remote_speed`). State on `bridge/protective_field` (`clear`, `limited`, `stop`, `stale`,
`off`), also on the handheld and the dashboard. Robots without the safety scan set
`field_in_auto` to false.

**Software e-stop.** Engaged from the handheld (`estop`, and also its `emergency_stop` driver
command), the dashboard (`bridge/estop`), RViz / CLI (`ros2 service call /bridge/estop std_srvs/srv/SetBool
"{data: true}"`) or any node via `bridge/estop_request`. While engaged nothing but zero reaches the
robot (sent at `estop_hold_hz`), navigation is paused and a mode switch cannot resume it. Release
is explicit (`estop_release`, or the service with `false`); navigation then stays paused until an
operator resumes it.

### teleop_udp_node

UDP gateway for the [Molly Controller](https://github.com/DarmawanRobotics/MollyController) handheld: joystick, commands (mode, navigation, route recording, stop
points, routes, localization, robot commands), 10 Hz telemetry, JPEG camera, downsampled lidar and
the loaded route. The wire format is in [PROTOCOL.md](PROTOCOL.md). With `video_port` set (the
`molly` launch does it when MediaMTX runs) it offers the handheld the WebRTC video instead of
the JPEG frames.

Set the same `token` here (site file) and in the handheld; a wrong one is answered with `denied`.
`robot_commands` are the driver's `std_srvs/Trigger` services shown in the handheld's action
menu, in this order. The camera, the cloud and TF are only subscribed while a handheld is connected
and has that stream on, so an idle gateway costs next to nothing.

### video_stream_node

H.264 of the camera for the handheld's WebRTC video. MediaMTX starts it when the handheld opens
the video and stops it 10 s after the last viewer, so it costs nothing while nobody watches. It
encodes `image_topic` (raw `rgb8`, `bgr8`, `mono8`, `yuv422`, ...) at `fps`, no wider than
`max_width`, with the first encoder that works: the Jetson hardware encoder (`nvv4l2h264enc`, not
on the Orin Nano), x264, OpenH264. Then it publishes over RTSP to MediaMTX on the loopback, and
MediaMTX serves WebRTC (WHEP) to the handheld. MediaMTX, its configuration and the ports come from
`molly` `robot.launch.py`; GStreamer and MediaMTX from `script/setup.sh install webrtc`.

## Run

```bash
ros2 launch molly_bridge bridge.launch.py robot_ns:=robot
ros2 launch molly_navigation navigation.launch.py cmd_vel_topic:=/bridge/nav_cmd_vel ...
```

Parameters: [config/bridge_params.yaml](config/bridge_params.yaml). For a lighter camera stream
publish a compressed image (`camera_compressed: true`); raw images are JPEG-encoded with OpenCV.

## Test

```bash
colcon test --packages-select molly_bridge && colcon test-result --verbose
```
