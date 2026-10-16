# molly

The robot package. `robot.launch.py` starts the whole stack in one process tree, the way it runs in
production under systemd (`script/setup.sh install service`). Every Molly node respawns 3 s after
it dies, and the service restarts the whole launch if it exits.

```bash
ros2 launch molly robot.launch.py                       # patrol, last loaded route
ros2 launch molly robot.launch.py mode:=teach           # keep the map while teaching a route
ros2 launch molly robot.launch.py odometry:=leg_kilo    # Leg-KILO instead of FAST-LIO
ros2 launch molly robot.launch.py route:=/path/route.csv people:=false
ros2 launch molly robot.launch.py robot_bringup:=my_dog_bringup robot_ns:=dog
```

The stack is robot-agnostic. The robot's driver, URDF and sensors come from a robot package
started with `robot_bringup`; [src/xtras/README.md](../xtras/README.md) lists what it must
provide.

| Argument | Default | |
|---|---|---|
| `robot_id` | `$ROBOT_ID` | fleet id reported to NETRA; empty: `fleet_params.yaml` / site file |
| `robot_ns` | `$ROBOT_NS` or `robot` | robot driver namespace (`/robot/cmd_vel`, `/robot/odom`, ...) |
| `robot_bringup` | `$ROBOT_BRINGUP` | robot package `<package>[/<file>.launch.py]`, started with `namespace:=<robot_ns>`; empty: the robot runs elsewhere (simulator, vendor service) |
| `mode` | `$MODE` or `patrol` | `teach` keeps the map (FAST-LIO PCD / Leg-KILO backend, RAM grows); `mapping` still works |
| `odometry` | `$ODOMETRY` or `fast_lio` | `fast_lio`, `leg_kilo` or `external` (simulator) ([molly_odometry](../molly_odometry)) |
| `route` | `$ROUTE` | route CSV; empty reloads the last route loaded through `navigation/load_path` |
| `tags` | `$TAGS` | AprilTag map poses (JSON), created by `localization/record_tags` |
| `cc_url`, `cc_token` | `$CC_URL`, `$CC_TOKEN` | NETRA command center |
| `site_config` | `$SITE_CONFIG` or `/etc/molly/site.yaml` | per-robot parameter overrides, see below |
| `people`, `inspection`, `health` | `true` (`$PEOPLE`, `$INSPECTION`) | optional parts |
| `webrtc` | `$WEBRTC` or `true` | handheld video over WebRTC through MediaMTX; off (or no MediaMTX): JPEG frames |
| `mediamtx` | `$MEDIAMTX` | MediaMTX binary; empty: `/opt/molly/mediamtx/mediamtx` (`script/setup.sh install webrtc`), then `PATH` |

Invalid values (`mode:=auto`, `odometry:=gmapping`, `robot_ns:=dog-2`, a robot package that is
not built) stop the launch with a clear error instead of starting a half-configured robot. The first log line states the version, git
commit (`MOLLY_GIT`, set by `script/run_robot.sh`), robot id, mode, odometry and site file.

## Site configuration

Tuning for one mall or one robot belongs in `/etc/molly/site.yaml`, not in the repository. It
is an ordinary ROS 2 parameter file keyed by node name and is applied on top of the package
defaults of every node (including FAST-LIO and the odometry gate):

```yaml
path_follower_node:
  ros__parameters:
    target_linear_velocity: 0.35
teleop_udp_node:
  ros__parameters:
    token: "change-me"
```

A missing file means no overrides. A broken file, or a section without `ros__parameters`, stops
the launch, and a section that matches no node (a typo) is reported in the log. Non-empty launch
arguments (`ROUTE`, `TAGS`, `CC_URL`, ...) win over the site file. Start from
[config/site.example.yaml](config/site.example.yaml).

Started nodes: the robot package (`robot_bringup`), odometry (held until the robot is
still) + odometry monitor + route recorder, localization, lidar sectors, AprilTag, person detector,
path loader / follower / avoidance, control mux, teleop UDP gateway, MediaMTX (it runs
`video_stream_node` while the handheld watches), NETRA uplink, VLM anomaly detector, health
monitor. `script/start_tmux.sh` remains for development (one pane per part, RViz).

## WebRTC video

MediaMTX v1.11 or newer serves the handheld's video: WHEP on TCP 8890, media on UDP 8190, and the
encoder publishes to it over RTSP on `127.0.0.1:8654`. The launch writes its configuration to
`~/.molly/mediamtx.yml` (mode 600) at every start: reading needs the `teleop_udp_node` token
from the site file (user `teleop`), publishing is allowed from the robot itself only. The ports
differ from the MediaMTX defaults, so a MediaMTX the robot already runs for something else can
stay. Encoder settings (`fps`, `max_width`, `bitrate_kbps`, `encoder`) are `video_stream_node`
parameters in the site file.
