<div align="center">

# Molly

**Tour guide stack for the Deep Robotics Jueying Lite3: an occupancy map recorded together with
the AprilTags that anchor it, a path finder between points of interest, tours with narration,
and interaction with visitors — driven from a handheld and a web dashboard.**

[![CI](https://github.com/DarmawanRobotics/Molly/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/DarmawanRobotics/Molly/actions/workflows/ci.yml)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Dharma](https://img.shields.io/badge/stack-Dharma%200.2-4A78C2)](molly.repos)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
[![Docker](https://img.shields.io/badge/dev-Docker%20%2B%20VNC-2496ED?logo=docker&logoColor=white)](compose.yaml)
![Version](https://img.shields.io/badge/version-2.0.0-informational)
![Jetson](https://img.shields.io/badge/Jetson-Orin%20NX-76B900?logo=nvidia&logoColor=white)
![Robot](https://img.shields.io/badge/robot-Deep%20Robotics%20Lite3-1F9E93)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue)](LICENSE)

[Quick start](#quick-start) · [Features](#features) · [System](#system) · [Architecture](#architecture) · [Install](#install) · [Workflow](#workflow) ·
[Operations](#operations) · [Develop](#develop) · [Docs](docs/README.md)

</div>

---

Molly walks visitors from one point of interest to the next, tells them about each one, waits
when they fall behind, and greets people who come up to it. Everything runs on the robot's Jetson
as one supervised service.

Molly runs on **Dharma 0.2** (ROS 2 Humble), the shared autonomy stack: LiDAR-inertial
odometry, AprilTag localization, the map and its sites, the path finder, the holonomic path
follower with obstacle avoidance, docking, the health monitor, and the protective field and
e-stop in the control mux come from the `dharma_*` repositories, at the commits pinned in
[molly.repos](molly.repos). This repository adds the tours and their POIs, the interaction with
visitors (speech, gestures, greeter, the LiDAR visitor tracker, conversation), the gateways for
the handheld and the dashboard, and the bringup for the Lite3 (`molly_bringup`).

## Quick start

```bash
# 1. Install on the Jetson (details: Install below)
git clone --recursive https://github.com/DarmawanRobotics/Molly.git ~/dev/Molly && cd ~/dev/Molly
scripts/setup.sh install config && sudoedit /etc/molly/molly.env /etc/molly/site.yaml
scripts/setup.sh install && scripts/setup.sh install service && sudo systemctl start molly

# 2. Map a site: drive it in REMOTE, record the tags, save
ros2 service call /mapping/start dharma_interfaces/srv/StartMapping "{site: museum, reset: true}"
ros2 service call /localization/record_tags std_srvs/srv/Trigger
ros2 service call /mapping/save std_srvs/srv/Trigger
ros2 service call /mapping/stop std_srvs/srv/Trigger

# 3. POIs and a tour (or the handheld / the dashboard's editors)
ros2 service call /site/save_poi molly_interfaces/srv/SavePoi \
  "{poi: {name: Lobby, narration: 'Selamat datang', language: id, dwell_sec: 5.0}, at_robot_pose: true}"
ros2 service call /site/save_tour molly_interfaces/srv/SaveTour "{tour: {name: Main, poi_ids: [lobby]}}"

# 4. Guide: localize from a tag, switch to AUTO, start the tour
ros2 service call /localization/start std_srvs/srv/Trigger
ros2 service call /control/set_mode dharma_interfaces/srv/SetMode "{mode: auto}"
ros2 service call /tour/control molly_interfaces/srv/TourControl "{command: {kind: 0, tour_id: main}}"

# 5. No robot: the simulator runs the same stack with a demo site and tour
ros2 launch molly_bringup sim.launch.py rviz:=true
```

Run `ros2` commands in a shell with the service's environment (`source scripts/env.sh`, see
[Install](#install)). The [workflow](#workflow) explains each step. Coming from Molly 1.x: the
renamed packages, settings and services are listed in [CHANGELOG.md](CHANGELOG.md#200---2026-10-28).

## Features

**Map**

- Drive the robot once through the place (handheld REMOTE, or the dashboard): a 2D occupancy grid
  is built live from the LiDAR, together with the AprilTags recorded in the same session.
- Add points of interest where the robot stands, or by clicking the map on the dashboard: name,
  description, narration in Indonesian or English, a gesture and a dwell time.
- Each place is a **site** (map, tags, POIs, tours, routes) on the robot; switch sites at runtime.

**Guide**

- **Path finder**: A\* on the map inflated by the robot footprint, then the holonomic follower
  walks it at a guide's pace and turns to the POI's heading on arrival. People and obstacles in
  the way are steered around (obstacle avoidance) inside the control mux's protective field.
- **Tours**: POI after POI — walk, gesture, narrate, dwell — with an introduction and a goodbye,
  pause / resume / skip / stop, and an optional return point.
- **Followers**: the 360° LiDAR sees who walks behind; when nobody follows, the robot stops and
  asks the visitors to follow.
- Localization stays tag based: a tag in view localizes the robot on its map (and corrects drift).
- The tour pauses with navigation (REMOTE, e-stop, a blocked path) and goes on from the same stop.

**Interact**

- Speech with Piper voices (Indonesian and English), eSpeak NG as the fallback.
- Greets visitors who stop in front of it, with a gesture if wanted.
- Lite3 gestures: hello, nod, bow, look left / right.
- Questions typed on the dashboard answered by a local LLM (Ollama), from the site's POIs.

**Stay safe**

- A 360° protective field on every command (AUTO and REMOTE), the latched software e-stop, the
  health monitor's speed scale, navigation stops on stale odometry or LiDAR.
- The dashboard reaches only whitelisted names through rosbridge; it drives through the control
  mux like the handheld.

**Operate**

- [Molly Controller](https://github.com/DarmawanRobotics/MollyController) on a Retroid Pocket:
  drive, WebRTC video, the map with POIs, sites and mapping, tours, go-to, speech and gestures.
- [Molly Dashboard](https://github.com/DarmawanWeb/MollyDashboard) in a browser: the same, plus
  the POI and tour editors, the conversation panel and system health.
- On the robot: one systemd service (`molly_bringup robot.launch.py`) with respawning nodes
  and health monitoring.

## System

![Molly system: Dharma and Molly packages on the Jetson, the Lite3 driver and hardware below, the handheld and the dashboard off the robot](docs/diagrams/system.drawio.svg)

<sub>The diagram is [docs/diagrams/system.drawio.svg](docs/diagrams/system.drawio.svg): open it in
draw.io (desktop app, [app.diagrams.net](https://app.diagrams.net) or the VS Code Draw.io
extension), edit and save, and this picture changes with it.</sub>

| Repository                                                             |                                                                                                                                                                          |
| ---------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Molly** (this)                                                       | the robot workspace: bringup, tours, interaction, handheld and dashboard gateways                                                                                        |
| Dharma                                                                 | the shared autonomy stack, one repository per part (`dharma_mapping`, `dharma_navigation`, ...), pinned in [molly.repos](molly.repos) and checked out into `src/dharma/` |
| [lite3_ros2](https://github.com/DarmawanRobotics/lite3_ros2)           | Lite3 driver (UDP motion host protocol), URDF, Mid-360 and camera launch — submodule in `src/xtras/robots/` ([contract](src/xtras/README.md))                            |
| [MollyController](https://github.com/DarmawanRobotics/MollyController) | handheld app (Retroid Pocket / Android), [protocol](src/molly_bridge/PROTOCOL.md)                                                                                        |
| [MollyDashboard](https://github.com/DarmawanWeb/MollyDashboard)        | web operator console (Next.js), [web contract](docs/web.md)                                                                                                              |

### Architecture

```
            ┌─────────────────────────────────────────────────────────────────┐  ┌─────────────────┐
 operators  │ molly_bridge: handheld UDP (molly-1), WebRTC video, rosbridge   │  │ safety          │
            ├─────────────────────────────────────────────────────────────────┤  │                 │
 Molly      │ molly_guide: tours, POIs │ molly_interaction: speech, gestures, │  │ control mux     │
            │                          │ greeter, visitors, conversation      │  │ (dharma_control)│
            ├─────────────────────────────────────────────────────────────────┤  │ the only        │
 navigation │ planner · route loader · path follower · avoidance · docking    │  │ /robot/cmd_vel, │
            ├─────────────────────────────────────────────────────────────────┤  │ e-stop,         │
 world      │ occupancy grid · sites (map, tags, routes)                      │  │ protective      │
            ├─────────────────────────────────────────────────────────────────┤  │ field           │
 estimation │ FAST-LIO odometry · AprilTag localization (EKF) · sector scans  │  │ health monitor  │
            ├─────────────────────────────────────────────────────────────────┤  │ speed scale     │
 robot      │ lite3_ros2 (driver, Mid-360, camera) or dharma_sim              │  │                 │
            └─────────────────────────────────────────────────────────────────┘  └─────────────────┘
 base         dharma_utils (robot profile, config, DharmaNode) · dharma_interfaces · molly_interfaces
```

- **One robot profile** ([robot.yaml](src/molly_bringup/config/robot.yaml), `robot.*`): the
  Lite3's footprint, limits and frames, given to every node by `molly_bringup`.
- **Every node** is a `DharmaNode`: typed parameters checked at start and on `ros2 param set`,
  the robot profile, structured logs.
- **Who drives**: navigation publishes `control/nav_cmd_vel`; the handheld and the dashboard
  publish `control/remote/<source>/cmd_vel` and take over with `control/set_mode`. The control
  mux arbitrates them, applies the protective field and the health speed scale, and is the only
  publisher of `/robot/cmd_vel`.
- **The guide** walks with `planner/go_to`, pauses with `navigation/pause` and follows the typed
  `control/status`, `navigation/status`, `planner/status` and `localization/status`.

<details>
<summary><b>Hardware</b></summary>

| Component | Model                                                                | Role                                        |
| --------- | -------------------------------------------------------------------- | ------------------------------------------- |
| Robot     | Deep Robotics Jueying **Lite3** (Venture)                            | motion host at `192.168.1.120`, UDP `43893` |
| Compute   | NVIDIA **Jetson Orin NX** (JetPack 6), added                         | the whole stack                             |
| LiDAR     | Livox **MID-360**, added at the LiDAR edition's mount                | odometry, map, obstacles, visitors          |
| Camera    | the Lite3's front wide-angle camera over RTSP (or a RealSense D435i) | AprilTags, video                            |
| Speaker   | USB or the Jetson's audio out                                        | speech                                      |

</details>

<details>
<summary><b>Packages</b></summary>

| Package                                                                          | Role                                                                                                                  |
| -------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| [`molly_bringup`](src/molly_bringup)                                             | the robot: Lite3 robot profile, stack and site configuration, `robot.launch.py` and `sim.launch.py`, end-to-end tests |
| [`molly_guide`](src/molly_guide)                                                 | tours (walk, gesture, narrate, dwell, wait for visitors), the site's POIs and tours                                   |
| [`molly_interaction`](src/molly_interaction)                                     | speech (Piper / eSpeak NG), gestures and greeter, the LiDAR visitor tracker, conversation (Ollama)                    |
| [`molly_bridge`](src/molly_bridge)                                               | handheld UDP gateway ([protocol](src/molly_bridge/PROTOCOL.md)), WebRTC video, the dashboard's rosbridge              |
| [`molly_interfaces`](src/molly_interfaces)                                       | messages and services of the POIs, tours, visitors and the interaction                                                |
| [`dharma_utils`](https://github.com/DarmawanRobotics/dharma_utils)               | robot profile, typed configuration, `DharmaNode`, math, test helpers                                                  |
| [`dharma_interfaces`](https://github.com/DarmawanRobotics/dharma_interfaces)     | typed status messages and the services of the stack                                                                   |
| [`dharma_odometry`](https://github.com/DarmawanRobotics/dharma_odometry)         | FAST-LIO2 (or Leg-KILO, or external) behind a still-start gate, odometry monitor                                      |
| [`dharma_localization`](https://github.com/DarmawanRobotics/dharma_localization) | `map -> odom` from AprilTags (EKF), per-site tags, map origin for new sites                                           |
| [`dharma_mapping`](https://github.com/DarmawanRobotics/dharma_mapping)           | occupancy grid per site, the site manager, the route recorder                                                         |
| [`dharma_perception`](https://github.com/DarmawanRobotics/dharma_perception)     | LiDAR sector scans (front and 360° safety), AprilTag camera throttle, ArUco detector                                  |
| [`dharma_navigation`](https://github.com/DarmawanRobotics/dharma_navigation)     | planner, route loader, path follower, obstacle avoidance, docking; `dharma_control`: the control mux and e-stop       |
| [`dharma_health`](https://github.com/DarmawanRobotics/dharma_health)             | sensor rates, TF ages, stack states and the computer as one `HealthStatus`                                            |
| [`dharma_rviz`](https://github.com/DarmawanRobotics/dharma_rviz)                 | RViz panels and the robot layout                                                                                      |
| [`dharma_sim`](https://github.com/DarmawanRobotics/dharma_sim)                   | simulated quadruped, Mid-360 and site for the whole stack                                                             |

The `dharma_*` packages come from their own repositories: `scripts/setup.sh install dharma`
(or `vcs import src < molly.repos`) checks them out into `src/dharma/` at the versions in
[molly.repos](molly.repos).

</details>

## Install

On the Jetson: Ubuntu 22.04 with ROS 2 Humble (JetPack 6), internet access, wired to the Lite3's
network, and the Jetson's SSH key added to GitHub with read access to the `dharma_*` repositories.

```bash
git clone --recursive https://github.com/DarmawanRobotics/Molly.git ~/dev/Molly
cd ~/dev/Molly
scripts/setup.sh install config          # creates /etc/molly/molly.env and site.yaml
sudoedit /etc/molly/molly.env           # ROBOT_DRIVER=lite3_bringup, ROBOT_IP, SITE, ...
sudoedit /etc/molly/site.yaml           # teleop token, speeds, AprilTag ids
scripts/setup.sh install                 # everything missing (Dharma, SDKs, apt, speech, web), then the build
scripts/setup.sh install service         # systemd unit, enabled at boot
sudo systemctl start molly && journalctl -fu molly
scripts/setup.sh                         # every line ok?
```

**Dharma** is checked out with vcstool into `src/dharma/` (git-ignored), at the commits pinned in
[molly.repos](molly.repos). `scripts/setup.sh` reports a checkout that is missing or not at its
pin, and `scripts/setup.sh install dharma` moves it back.

**The Lite3 sends its state only to the hosts in its `network.toml`.** Add the Jetson once
(`src/xtras/robots/lite3_ros2/lite3_bringup/scripts/lite3_setup.sh network --apply`, see the
[lite3_ros2 README](https://github.com/DarmawanRobotics/lite3_ros2)), and make sure the vendor's
own navigation does not drive the robot at the same time.

**Update.** Pull, rebuild and restart:

```bash
git pull && git submodule update --init --recursive \
  && scripts/setup.sh install dharma build config service && sudo systemctl restart molly
```

`install dharma` follows the pins when a new Dharma release is pinned, `install build` rebuilds
whenever the checkout has moved, `install config` adds env keys that are new in the example, and
`install service` rewrites an outdated unit.

| Extra          | `molly.env`         | Install                           | Gives                                                               |
| -------------- | ------------------- | --------------------------------- | ------------------------------------------------------------------- |
| Speech         | `SPEECH=true`       | `scripts/setup.sh install speech` | Piper voices (id, en), eSpeak NG, aplay                             |
| Dashboard      | `WEB=true`          | `scripts/setup.sh install web`    | rosbridge on 9090, camera MJPEG on 8080                             |
| Handheld video | `WEBRTC=true`       | `scripts/setup.sh install webrtc` | H.264 over WebRTC through MediaMTX; JPEG without it                 |
| Questions      | `CONVERSATION=true` | `scripts/setup.sh install ollama` | a local LLM (`CHAT_MODEL`, qwen2.5:3b) answers from the site's POIs |

- **`/etc/molly/molly.env`** describes the deployment: robot driver, namespace, site, route,
  `START_MAPPING`, extras. systemd, bash and `scripts/setup.sh` all read it. Quote any value with spaces, and never
  put a comment after a value.
- **`/etc/molly/site.yaml`** sets the parameters of this robot and place, on top of the
  bringup's [stack.yaml](src/molly_bringup/config/stack.yaml)
  ([example for the Lite3](src/molly_bringup/config/site.example.yaml)). It only needs a restart.
  The robot's body, limits and frames are the robot profile
  [robot.yaml](src/molly_bringup/config/robot.yaml) (`robot.*`, shared by every node).
- **ROS stays on the robot** (`ROS_LOCALHOST_ONLY=1`); the handheld and the dashboard come in
  through their gateways only.

**A shell on the robot** needs the service's environment:

```bash
echo "alias molly='source ~/dev/Molly/scripts/env.sh'" >> ~/.bashrc   # once
molly
ros2 topic echo /tour/state
pnode molly_guide guide_node guide_params.yaml   # one node with the robot profile and stack.yaml
```

## Workflow

**1. Map a place** (`START_MAPPING=true SITE=museum`, or at any time from the handheld /
dashboard / `mapping/start`):

1. Drive the robot (REMOTE) through every corridor the tours will use; the map grows on the
   handheld and the dashboard.
2. Stand still in view of each AprilTag and **Record tags**: the tags anchor the map.
3. **Add POIs** where the robot stands (handheld) or on the map (dashboard), with their narration.
4. **Save** the map, then stop mapping. Build tours from the POIs.

**2. Guide.**

- **Localize** from a tag (automatic when one is in view at start, or **Localize**).
- Switch to **AUTO** and start a tour (handheld Guide page, dashboard, or
  `ros2 service call /tour/control molly_interfaces/srv/TourControl "{command: {kind: 0, tour_id: main}}"`).
- **Go to** any POI or map point on its own with the path finder.
- Say something, greet, or play a gesture whenever the robot stands.

**Emergency stop.** The software e-stop is latched in the control mux: the handheld **STOP**,
the dashboard, or `ros2 service call /control/estop/engage std_srvs/srv/Trigger` (release:
`/control/estop/release`). Only zero reaches the robot while it is engaged; the tour pauses and
resumes only when told, after the release. The driver's
own stop is `ros2 service call /robot/emergency_stop std_srvs/srv/Trigger`.

<details>
<summary><b>Guide behaviour</b></summary>

| Feature          | What it does                                                               | Key parameters                               |
| ---------------- | -------------------------------------------------------------------------- | -------------------------------------------- |
| Grid mapping     | occupancy grid per site from the LiDAR scan, saved crash-safe              | `grid_mapper_node.*`                         |
| Sites            | map + tags + routes (Dharma) and POIs + tours (Molly) per place            | `site_manager_node.sites_dir`, `SITES_DIR`   |
| Path finder      | A\* on the costmap inflated around the footprint                           | `planner_node.costmap.*`                     |
| Following        | holonomic follower at a guide's pace, final approach to the POI's heading  | `path_follower_node.speed.*`                 |
| Avoidance        | VFH steering around obstacles and people, emergency stop in the front cone | `obstacle_avoidance_node.shared.*`           |
| Tours            | walk, gesture, narrate, dwell; retry, then skip or stop at a POI it misses | `guide_node.go_to_retries`, `on_failure`     |
| Followers        | pause and ask when nobody is behind for `visitor_lost_sec`                 | `guide_node.wait_for_visitors`, `visitor_*`  |
| Greeter          | greet a visitor who stays in front, once per `cooldown_sec`                | `interaction_node.*`                         |
| Protective field | v·t + v²/2a ≤ clearance − margin in every direction, AUTO and REMOTE       | `control_mux_node.field.*`, `robot.limits.*` |

</details>

## Operations

|            |                                                                                                                                               |
| ---------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| Health     | `ros2 topic echo /health/status`: `level` ok, warn or error, with sensor rates, CPU, GPU, RAM, temperature; on the handheld and the dashboard |
| Tour       | `ros2 topic echo /tour/state`                                                                                                                 |
| Logs       | `journalctl -fu molly`; ROS logs older than 7 days are pruned at start                                                                        |
| Interfaces | every topic, service and file: [docs/interfaces.md](docs/interfaces.md)                                                                       |

## Develop

**Dev container** (Linux / macOS, Docker): ROS 2 Humble with every dependency and an XFCE
desktop over VNC ([docker/README.md](docker/README.md)). `scripts/dev.sh up` checks out the
Dharma repositories from [molly.repos](molly.repos) and the submodules first.

```bash
scripts/dev.sh up      # import the stack, build the image, start it
scripts/dev.sh build   # colcon build
scripts/dev.sh test    # colcon test + test-result
scripts/dev.sh vnc     # TigerVNC localhost:5901 or http://localhost:6080/vnc.html
scripts/dev.sh shell
```

The same image opens in VS Code as a dev container (`.devcontainer/`).
`scripts/start_tmux.sh sim` (or `robot`) starts the stack one window per subsystem
([scripts/tmux/](scripts/tmux)), for development; the service runs `robot.launch.py`.

**Simulation.** The whole stack runs against a simulated Lite3 with a Mid-360 and a site
([dharma_sim](https://github.com/DarmawanRobotics/dharma_sim)), with no robot, Gazebo or GPU.
`sim.launch.py` draws the site `mall_loop` with demo POIs (`east_gate`, `fountain`,
`west_hall`) and the tour `demo`; the robot localizes from the simulator's pose estimate.

```bash
ros2 launch molly_bringup sim.launch.py rviz:=true
ros2 service call /tour/control molly_interfaces/srv/TourControl "{command: {kind: 0, tour_id: demo}}"
ros2 topic echo /tour/state
```

**Tests.**

```bash
colcon test --packages-select molly_interfaces molly_guide molly_interaction molly_bridge \
  molly_bringup --executor sequential && colcon test-result --verbose
pre-commit run --all-files    # ruff, prettier, clang-format, shellcheck, as CI
```

Every package has unit tests of its pure `core/`, in-process node tests (`test/ros`) and the
style check (`test/test_style.py`). `molly_bringup`'s end-to-end test starts `sim.launch.py`,
runs the demo tour to its end, takes over with the handheld through the control mux in the
middle and hands back to AUTO (`MOLLY_SKIP_SIM=1` skips it). CI
([ci.yml](.github/workflows/ci.yml)) lints, builds the whole stack from
[molly.repos](molly.repos) on Humble and runs all of it. Branch and commit rules are in
[CONTRIBUTING.md](CONTRIBUTING.md), changes in [CHANGELOG.md](CHANGELOG.md).

```
Molly/
├── .github/workflows/        # ci.yml: lint, scripts, build + tests + simulator tour on Humble
├── .devcontainer/ compose.yaml docker/   # dev container (Humble, VNC desktop)
├── docs/                     # interfaces, web contract, diagrams
├── map/                      # sites (map, tags, POIs, tours, routes) + example/
├── molly.repos               # the Dharma versions (vcstool)
├── scripts/                  # setup.sh, run_robot.sh (service), env.sh (shell), start_tmux.sh, dev.sh
└── src/
    ├── molly_*               # Molly's packages
    ├── dharma/               # Dharma repositories, checked out from molly.repos (git-ignored)
    └── xtras/                # submodules: livox_ros_driver2, FAST_LIO, robots/lite3_ros2
```

Molly is released under the [Apache License 2.0](LICENSE).
