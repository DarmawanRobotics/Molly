# molly

The robot bringup: one launch for the whole stack, the site configuration and the web gateway.

## Launch

`robot.launch.py` starts the whole stack in one process tree, the way it runs in
production under systemd (`script/setup.sh install service`). Every Molly node respawns 3 s after
it dies, and the service restarts the whole launch if it exits.

```bash
ros2 launch molly robot.launch.py                              # guide, the site of the last run
ros2 launch molly robot.launch.py site:=museum                 # guide on another site
ros2 launch molly robot.launch.py mode:=mapping site:=lobby    # map a new site from the start
ros2 launch molly robot.launch.py conversation:=true web:=false
ros2 launch molly robot.launch.py robot_bringup:=lite3_bringup robot_ns:=robot
```

The stack is robot-agnostic. The robot's driver, URDF and sensors come from a robot package
started with `robot_bringup` (the Lite3 one is [lite3_ros2](https://github.com/DarmawanRobotics/lite3_ros2));
[src/xtras/README.md](../xtras/README.md) lists what it must provide.

### Arguments

| Argument | Default | |
|---|---|---|
| `robot_ns` | `$ROBOT_NS` or `robot` | robot driver namespace (`/robot/cmd_vel`, `/robot/odom`, ...) |
| `robot_bringup` | `$ROBOT_BRINGUP` | robot package `<package>[/<file>.launch.py]`, started with `namespace:=<robot_ns>`; empty: the robot runs elsewhere (simulator, vendor service) |
| `mode` | `$MODE` or `guide` | `guide`, or `mapping`: maps `site` right after start when it is new, and FAST-LIO keeps its PCD. A site can also be mapped at any time with `mapping/start` |
| `site` | `$SITE` | site to load (guide) or to map (mapping); empty: the site of the last run |
| `sites_dir` | `$SITES_DIR` | directory of the sites; empty: `~/dev/Molly/map` (site manager, route recorder, handheld) |
| `odometry` | `$ODOMETRY` or `fast_lio` | `fast_lio`, `leg_kilo` or `external` (simulator) ([molly_odometry](../molly_odometry)) |
| `route` | `$ROUTE` | route CSV for teach and repeat; empty reloads the last route loaded through `navigation/load_path` |
| `site_config` | `$SITE_CONFIG` or `/etc/molly/site.yaml` | per-robot parameter overrides, see below |
| `visitors` | `$VISITORS` or `true` | visitors around the robot from the LiDAR |
| `people` | `$PEOPLE` or `false` | camera people (YOLO11n) |
| `speech`, `greeter` | `$SPEECH`, `$GREETER` or `true` | text to speech; greeting visitors |
| `conversation` | `$CONVERSATION` or `false` | answering questions with a local LLM (Ollama) |
| `web`, `web_port`, `web_video_port` | `$WEB` (`true`), `9090`, `8080` | rosbridge (whitelisted names) and web_video_server for the dashboard |
| `health` | `true` | health monitor |
| `webrtc` | `$WEBRTC` or `true` | handheld video over WebRTC through MediaMTX; off (or no MediaMTX): JPEG frames |
| `mediamtx` | `$MEDIAMTX` | MediaMTX binary; empty: `/opt/molly/mediamtx/mediamtx` (`script/setup.sh install webrtc`), then `PATH` |

Invalid values (`mode:=auto`, `site:=Lobby`, `odometry:=gmapping`, `robot_ns:=dog-2`, a robot
package that is not built, `mode:=mapping` without a site) stop the launch with a clear error
instead of starting a half-configured robot. The first log line states the version, git commit
(`MOLLY_GIT`, set by `script/run_robot.sh`), mode, site, robot, odometry and site file.

## Nodes

Started nodes: the robot package (`robot_bringup`), odometry (held until the robot is still) +
odometry monitor + route recorder, localization, grid mapper, site manager, LiDAR sectors, the
visitor tracker, AprilTag, the person detector, path loader / follower / avoidance, the planner
(path finder), the guide, the interaction node (gestures, greeter), speech, conversation, the
control mux, the teleop UDP gateway, MediaMTX (it runs `video_stream_node` while the handheld
watches), rosbridge + rosapi + web_video_server, and the health monitor.
`script/start_tmux.sh` remains for development (one pane per part, RViz).

## Site configuration

Tuning for one place or one robot belongs in `/etc/molly/site.yaml`, not in the repository. It
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
arguments (`SITE`, `SITES_DIR`, `ROUTE`, ...) win over the site file. Start from
[config/site.example.yaml](config/site.example.yaml) (values for the Lite3).

## Web

With `web:=true` the dashboard reaches the robot through `rosbridge_websocket` (port 9090) and
the camera through `web_video_server` (port 8080), both from apt (`script/setup.sh install web`;
without them the launch logs it and goes on). rosbridge and rosapi only expose the names of
[docs/web.md](../../docs/web.md) (`topics_glob`, `services_glob`, `params_glob` from
`molly/bringup.py`): the dashboard drives through `/bridge/remote_cmd_vel` and the control mux,
never the driver's `cmd_vel`.

## WebRTC video

MediaMTX v1.11 or newer serves the handheld's video: WHEP on TCP 8890, media on UDP 8190, and the
encoder publishes to it over RTSP on `127.0.0.1:8654`. The launch writes its configuration to
`~/.molly/mediamtx.yml` (mode 600) at every start: reading needs the `teleop_udp_node` token
from the site file (user `teleop`), publishing is allowed from the robot itself only. The ports
differ from the MediaMTX defaults, so a MediaMTX the robot already runs for something else can
stay. Encoder settings (`fps`, `max_width`, `bitrate_kbps`, `encoder`) are `video_stream_node`
parameters in the site file.
