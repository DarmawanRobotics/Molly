# Molly docs

Start with the [README](../README.md) (quick start, install, workflow, operations).

## Reference

| Document | For |
|---|---|
| [interfaces.md](interfaces.md) | every topic, service, frame and site file of the stack |
| [web.md](web.md) | the dashboard's contract: rosbridge names, the camera, flows |
| [PROTOCOL.md](../src/molly_bridge/PROTOCOL.md) | the handheld's UDP protocol (Molly v1) |
| [src/xtras/README.md](../src/xtras/README.md) | what a robot package must provide; per-robot parameters |
| [site.example.yaml](../src/molly/config/site.example.yaml) | the site file for a Lite3 |
| [lite3_ros2](https://github.com/DarmawanRobotics/lite3_ros2) | the Lite3 driver: motion host network, UDP protocol, URDF, camera |

## Packages

| Package | README |
|---|---|
| `molly` | [robot bringup, site configuration, web, WebRTC](../src/molly/README.md) |
| `molly_interfaces` | [messages and services](../src/molly_interfaces/README.md) |
| `molly_odometry` | [FAST-LIO2 / Leg-KILO, stillness gate, route recording](../src/molly_odometry/README.md) |
| `molly_localization` | [`map → odom` from AprilTags](../src/molly_localization/README.md) |
| `molly_mapping` | [grid mapper and sites](../src/molly_mapping/README.md) |
| `molly_perception` | [LiDAR sectors, tags, people, visitors](../src/molly_perception/README.md) |
| `molly_navigation` | [path finder, follower, avoidance](../src/molly_navigation/README.md) |
| `molly_guide` | [tours](../src/molly_guide/README.md) |
| `molly_interaction` | [speech, gestures, greeter, conversation](../src/molly_interaction/README.md) |
| `molly_bridge` | [control mux, handheld gateway, video](../src/molly_bridge/README.md) |
| `molly_health` | [health monitor](../src/molly_health/README.md) |
| `molly_rviz` | [RViz panels](../src/molly_rviz/README.md) |
| `molly_sim` | [simulator and end-to-end tests](../src/molly_sim/README.md) |

## Development

| Document | For |
|---|---|
| [CONTRIBUTING.md](../CONTRIBUTING.md) | branches, commits, build and test, rules from real crashes |
| [docker/README.md](../docker/README.md) | the dev container (VNC desktop) |
| [CHANGELOG.md](../CHANGELOG.md) | notable changes |
