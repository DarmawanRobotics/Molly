# molly_bridge

How the handheld and the dashboard reach the robot without ROS on their side: the UDP gateway
for the [Molly Controller](https://github.com/DarmawanRobotics/MollyController), its WebRTC
video, and the rosbridge whitelist and camera server for the dashboard. None of them drives the
robot: operator velocities go to dharma_control's mux as operator sources
(`control/remote/handheld/cmd_vel`, `control/remote/dashboard/cmd_vel`), and the mux alone
publishes the driver's `cmd_vel`.

## Nodes

### teleop_udp_node

UDP gateway (protocol in [PROTOCOL.md](PROTOCOL.md)): joystick, commands, 10 Hz telemetry,
JPEG camera, downsampled LiDAR, the loaded route, the site's map, POIs and tours. Set the same
`token` here (site file) and in the handheld; a wrong one is answered with `denied`. One driver
at a time; a silent client is dropped after `client_timeout_sec` with one zero velocity.

- **Driving.** `joy` is clamped to `robot.limits` and published on
  `control/remote/<remote_source>/cmd_vel`. `set_mode` calls `control/set_mode` with that
  source; `estop` publishes `control/estop_request` at once (no service needed to latch) and
  calls `control/estop/engage`; `estop_release` calls `control/estop/release`. The mux
  re-arms a source only after neutral commands, so a stick held during a release never moves the
  robot.
- **Commands** call the Dharma services (`navigation/start|stop|pause|resume`,
  `route/start|stop|mark_stop_point`, `navigation/load_path`, `localization/start|record_tags`,
  `mapping/*`, `site/list|load`, `planner/go_to|cancel`, `docking/dock|undock|cancel`), Molly's
  (`site/save_poi` ..., `tour/control`, `guide/go_to_poi`, `interaction/*`) and the driver's
  `robot_commands` (`std_srvs/Trigger` under `robot_command_prefix`). `nav_speed` sets the
  follower's live `speed.cruise` within `patrol_speed_min` .. `robot.limits.max_vx`.
- **State** from the typed status messages: `control/status`, `navigation/status`,
  `localization/status`, `health/status` (with CPU, GPU, RAM and temperature from the health
  monitor's `/diagnostics`), `planner/status`, `mapping/status`, `site/status`,
  `docking/status`, `tour/state`, `interaction/speech_status`, `interaction/greeter`,
  `perception/visitors`, the driver's `battery`, `diagnostics` and the `cmd_vel` the mux sends.
- **Streams.** The camera, the cloud, the AprilTag detections and TF are subscribed only while a
  handheld is connected and has that stream on. Raw images and driver JPEGs bigger than
  `camera_max_bytes` are encoded with OpenCV. With `video_port` set (the launch does it when
  MediaMTX runs) the handheld gets WebRTC video instead.

### video_stream_node

H.264 of the camera for the handheld's WebRTC video. MediaMTX starts it when the handheld opens
the video and stops it 10 s after the last viewer. It encodes `image_topic` (raw `rgb8`, `bgr8`,
`mono8`, `yuv422`, ...) at `fps`, no wider than `max_width`, with the first encoder that works:
the Jetson hardware encoder (`nvv4l2h264enc`), x264, OpenH264; then publishes over RTSP to
MediaMTX on the loopback, and MediaMTX serves WHEP to the handheld.

## Launch

```bash
ros2 launch molly_bridge bridge.launch.py robot_ns:=robot sites_dir:=~/dev/Molly/map
ros2 launch molly_bridge web.launch.py robot_ns:=robot      # rosbridge 9090, MJPEG 8080
```

| `bridge.launch.py` argument                                | Default |                                                            |
| ---------------------------------------------------------- | ------- | ---------------------------------------------------------- |
| `params_file`, `robot_params`, `use_sim_time`, `log_level` |         | the Dharma launch contract                                 |
| `robot_ns`                                                 | `robot` | driver namespace: battery, diagnostics, `robot_commands`   |
| `sites_dir`                                                | (files) | routes listed and loaded from here                         |
| `webrtc`                                                   | `true`  | MediaMTX when installed (`scripts/setup.sh install webrtc`) |
| `mediamtx`                                                 |         | MediaMTX binary; empty: `/opt/molly/mediamtx`, then `PATH` |

MediaMTX v1.11 or newer: WHEP on TCP 8890, media on UDP 8190, the encoder publishes on
`127.0.0.1:8654`. The launch writes its configuration to `~/.molly/mediamtx.yml` (mode 600) at
every start: reading needs the teleop token (user `teleop`), publishing only from the robot.

`web.launch.py` starts `rosbridge_websocket` and `rosapi_node` with the whitelist of
[molly_bridge/core/web.py](molly_bridge/core/web.py) (`topics_glob`, `services_glob`,
`params_glob`; never the driver's `cmd_vel`) and `web_video_server`, each only when installed
(`scripts/setup.sh install web`). The names are listed in [docs/web.md](../../docs/web.md).

## Parameters

Typed (`ros2 param describe /teleop_udp_node <name>`), defaults in
[molly_bridge/core/config.py](molly_bridge/core/config.py), notes in
[config/bridge_params.yaml](config/bridge_params.yaml). The joystick limits and the frames come
from the robot profile.

| `teleop_udp_node`                                             | Default                                       |                                         |
| ------------------------------------------------------------- | --------------------------------------------- | --------------------------------------- |
| `port`, `token`, `robot_name`                                 | 9880, empty, hostname                         |                                         |
| `client_timeout_sec`, `telemetry_hz`                          | 2 s, 10 Hz                                    |                                         |
| `remote_source`                                               | `handheld`                                    | a `remote_sources` entry of the mux     |
| `robot_command_prefix`, `robot_commands`                      | `/robot/`, stand_up, sit_down, emergency_stop | driver Trigger services                 |
| `sites_dir`                                                   | `~/dev/Molly/map`                             | routes                                  |
| `camera_topic`, `camera_compressed`, `camera_fps`             | `.../image_raw/compressed`, true, 8           |                                         |
| `camera_jpeg_quality`, `camera_max_width`, `camera_max_bytes` | 60, 640, 60000                                |                                         |
| `video_port`, `video_path`                                    | 0, `camera`                                   | WebRTC (the launch sets the port)       |
| `tags_topic`, `tags_camera_info_topic`, `tags_max_age_sec`    | `/perception/apriltag/...`, 0.6 s             | tag overlay                             |
| `lidar_topic`, `lidar_hz`, `lidar_voxel`, `lidar_max_points`  | `/cloud_registered_body`, 2, 0.1, 3000        |                                         |
| `navigation_node`, `patrol_speed_min`                         | `/path_follower_node`, 0.1 m/s                | walking speed from the handheld         |
| `map_max_px`, `map_push_sec`                                  | 1024, 3 s                                     | map stream                              |
| `gestures`                                                    | hello, nod, bow, look_left, look_right        | until interaction/greeter names its own |

## Layout and tests

| Module                                                                        |                                                                        |
| ----------------------------------------------------------------------------- | ---------------------------------------------------------------------- |
| `core/protocol.py`                                                            | the codec: JSON, chunks, points, the map payload                       |
| `core/session.py`                                                             | the one driving client: token, busy, denied, acks                      |
| `core/args.py`                                                                | command arguments: validated and cut to their limits                   |
| `core/summaries.py`                                                           | the state message from typed status messages                           |
| `core/map_image.py`                                                           | the map as a small PNG (no OpenCV)                                     |
| `core/routes.py`                                                              | routes listed and loaded inside the sites directory                    |
| `core/mediamtx.py`, `core/gstreamer.py`                                       | MediaMTX configuration, the H.264 pipelines                            |
| `core/web.py`                                                                 | the rosbridge whitelist                                                |
| `utils/commands.py`                                                           | every command as a service client and a request factory                |
| `utils/telemetry.py`, `utils/streams.py`, `utils/pusher.py`, `utils/speed.py` | state, on-demand streams, route / map / site pushes, the walking speed |
| `utils/link.py`                                                               | the UDP socket and its reader thread                                   |

`test/unit` covers the cores; `test/ros` runs `teleop_udp_node` in-process against a real UDP
client and fake services.

```bash
colcon test --packages-select molly_bridge && colcon test-result --verbose
```
