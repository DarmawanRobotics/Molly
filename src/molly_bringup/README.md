# molly_bringup

Molly on the Dharma stack: the Lite3's robot profile, the per-node overrides of the package
defaults, and one launch for the whole robot (the Dharma stack, Molly's packages, the robot
driver), plus the same on the simulator. Made from the Dharma hub's `robot_bringup` template.

## Files

| File                            |                                                                                                       |
| ------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `config/robot.yaml`             | the Lite3 robot profile, `/**: ros__parameters: robot: {...}` (kinematics, footprint, limits, frames) |
| `config/stack.yaml`             | per-node overrides for Molly on the Lite3 (only what differs from the package defaults)               |
| `config/sim.yaml`               | on top of `stack.yaml` in `sim.launch.py` (PointCloud2 input, silent speech, the sim's dock)          |
| `config/site.example.yaml`      | the site file for a Lite3, copied to `/etc/molly/site.yaml` by `scripts/setup.sh install config`      |
| `config/demo_site/`             | demo POIs and tour copied into a simulated site that has none                                         |
| `launch/robot.launch.py`        | the robot driver (`robot_driver:=lite3_bringup`) and the whole stack                                  |
| `launch/sim.launch.py`          | dharma_sim and the whole stack (odometry external, no camera)                                         |
| `molly_bringup/stack.py`        | the composition: package launch files, their arguments, the nodes they start                          |
| `molly_bringup/bringup.py`      | argument validation, the robot profile check, parameter file merging, the demo site                   |
| `molly_bringup/launch_utils.py` | the launch actions shared by both launch files                                                        |

## Launch

```bash
ros2 launch molly_bringup robot.launch.py robot_driver:=lite3_bringup site:=museum
ros2 launch molly_bringup robot.launch.py site:=lobby start_mapping:=true   # map a new site
ros2 launch molly_bringup robot.launch.py conversation:=true web:=false
ros2 launch molly_bringup sim.launch.py rviz:=true
```

In production `molly.service` runs `robot.launch.py` with the environment of
`/etc/molly/molly.env` (`scripts/setup.sh install service`); the arguments marked with a
variable default to it.

| Argument                            | Default (env)                            |                                                                                                |
| ----------------------------------- | ---------------------------------------- | ---------------------------------------------------------------------------------------------- |
| `robot_params`                      | `config/robot.yaml`                      | robot profile file                                                                             |
| `params_file`                       | (`SITE_CONFIG`)                          | the site file on top of `config/stack.yaml`, e.g. `/etc/molly/site.yaml`; must exist when set  |
| `use_sim_time` / `log_level`        | / `info`                                 |                                                                                                |
| `robot_ns`                          | `robot` (`ROBOT_NS`)                     | driver namespace: `/<ns>/cmd_vel`, `/<ns>/odom`, `/<ns>/battery`, driver services              |
| `robot_driver`                      | (`ROBOT_DRIVER`)                         | `<package>[/<file>.launch.py]`, started with `namespace:=<robot_ns>`; empty: started elsewhere |
| `odometry`                          | `fast_lio` (`ODOMETRY`)                  | `fast_lio`, `leg_kilo` or `external` (robot launch only)                                       |
| `site` / `sites_dir`                | (`SITE`, `SITES_DIR`)                    | empty: from the parameter files (or the last site)                                             |
| `route`                             | (`ROUTE`)                                | route CSV inside `sites_dir` loaded at start; empty: the last one loaded                       |
| `start_mapping`                     | `false` (`START_MAPPING`)                | map `site` from the start when it is new                                                       |
| `fiducials` / `fiducials_detector`  | `true` / `apriltag`                      | camera throttle + tag detector (`aruco`: dharma_perception's ArUco detector)                   |
| `camera_image` / `camera_info`      |                                          | empty: the perception defaults (`/camera/camera/color/...`, the Lite3 camera)                  |
| `health`                            | `true`                                   | the health monitor                                                                             |
| `speech`, `greeter`, `visitors`     | `true` (`SPEECH`, `GREETER`, `VISITORS`) | text to speech, the greeter, the visitor tracker                                               |
| `conversation`                      | `false` (`CONVERSATION`)                 | questions answered by a local model (Ollama)                                                   |
| `web`, `web_port`, `web_video_port` | `true`, 9090, 8080 (`WEB`, ...)          | rosbridge (whitelisted names) and web_video_server, when installed                             |
| `webrtc`, `mediamtx`                | `true` (`WEBRTC`, `MEDIAMTX`)            | the handheld's video over WebRTC when MediaMTX is installed                                    |
| `world` / `scenario` / `rviz`       | / / `false`                              | simulator only                                                                                 |

What the launch does:

1. Validates every argument and the robot profile (unknown names and out-of-range values stop
   the launch with the reason).
2. Merges `config/stack.yaml` (+ `config/sim.yaml`) and `params_file` into one parameter file
   (later wins), gives `sites_dir` to the route loader, the POI store and the teleop gateway and
   `route` to the route loader, adds every started node to the health monitor's
   `nodes.expected` (the Lite3 package's nodes are listed in `stack.yaml`), and reports
   sections that match no node (typos).
3. Includes each package launch (`dharma_odometry`, `dharma_localization`, `dharma_perception`,
   `dharma_mapping`, `dharma_navigation`, `dharma_control`, `dharma_health`, `molly_guide`,
   `molly_interaction`, `molly_bridge` `bridge.launch.py` and `web.launch.py`) with
   `robot_params`, the merged `params_file` and only the arguments that are set, each in its own
   launch scope.
4. The control mux is the only publisher of `/<robot_ns>/cmd_vel`; the handheld and the
   dashboard are its `remote_sources` (`handheld`, `dashboard`).

The first log line names the Molly version, the git commit (`MOLLY_GIT`, set by
`scripts/run_robot.sh`), the robot, the namespace, the odometry, the site and the files.

## The simulator

`sim.launch.py` draws the site `mall_loop` (under `/tmp/molly_sim/sites`) from dharma_sim's world
when it has no map, and copies the demo POIs and tour (`east_gate`, `fountain`, `west_hall`)
next to it. The simulator stands in for the Lite3 driver, the Mid-360 and FAST-LIO, sends a pose
estimate while not localized, answers the route's inspection hook and has a charging dock with
a tag camera on the west wall. The handheld connects with the token `sim`.

```bash
ros2 launch molly_bringup sim.launch.py
ros2 service call /tour/control molly_interfaces/srv/TourControl "{command: {kind: 0, tour_id: demo}}"
ros2 topic echo /tour/state
```

## Tests

`test/unit` (argument validation, parameter merging, the composition), `test/ros` (both launch
files against stub packages, and `ros2 launch --show-args` when installed), `test/e2e` (the
whole simulator: a tour, teleop through the mux; `MOLLY_SKIP_SIM=1` skips it), `test/test_style.py`.

```bash
colcon test --packages-select molly_bringup && colcon test-result --verbose
```
