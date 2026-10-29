# Molly docs

Start with the [README](../README.md) (quick start, install, workflow, operations).

## Reference

| Document                                                                                                   | For                                                                    |
| ---------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------- |
| [interfaces.md](interfaces.md)                                                                             | every topic, service, frame and site file of the stack                 |
| [system.drawio.svg](diagrams/system.drawio.svg)                                                            | the system diagram; edit it in draw.io                                 |
| [web.md](web.md)                                                                                           | the dashboard's contract: rosbridge names, the camera, flows           |
| [PROTOCOL.md](../src/molly_bridge/PROTOCOL.md)                                                             | the handheld's UDP protocol (`molly-1`, with the changes of Molly 2.0) |
| [src/xtras/README.md](../src/xtras/README.md)                                                              | what a robot package must provide; per-robot parameters                |
| [site.example.yaml](../src/molly_bringup/config/site.example.yaml)                                         | the site file for a Lite3                                              |
| [robot.yaml](../src/molly_bringup/config/robot.yaml), [stack.yaml](../src/molly_bringup/config/stack.yaml) | the Lite3 robot profile, Molly's overrides of the Dharma defaults      |
| [lite3_ros2](https://github.com/DarmawanRobotics/lite3_ros2)                                               | the Lite3 driver: motion host network, UDP protocol, URDF, camera      |

## Packages

| Package                               | README                                                                                                                     |
| ------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| `molly_bringup`                       | [robot and simulator launch, robot profile, stack and site configuration](../src/molly_bringup/README.md)                  |
| `molly_interfaces`                    | [messages and services of the tours and the interaction](../src/molly_interfaces/README.md)                                |
| `molly_guide`                         | [tours](../src/molly_guide/README.md)                                                                                      |
| `molly_interaction`                   | [speech, gestures, greeter, visitor tracker, conversation](../src/molly_interaction/README.md)                             |
| `molly_bridge`                        | [handheld gateway, WebRTC video, the dashboard's rosbridge](../src/molly_bridge/README.md)                                 |
| `dharma_interfaces`                   | [messages and services of the stack](https://github.com/DarmawanRobotics/dharma_interfaces)                                |
| `dharma_odometry`                     | [FAST-LIO2 / Leg-KILO, stillness gate, route recording](https://github.com/DarmawanRobotics/dharma_odometry)               |
| `dharma_localization`                 | [`map → odom` from AprilTags](https://github.com/DarmawanRobotics/dharma_localization)                                     |
| `dharma_mapping`                      | [grid mapper and sites](https://github.com/DarmawanRobotics/dharma_mapping)                                                |
| `dharma_perception`                   | [LiDAR sectors, safety scan, AprilTag throttle](https://github.com/DarmawanRobotics/dharma_perception)                     |
| `dharma_navigation`, `dharma_control` | [path finder, follower, avoidance, docking; control mux and e-stop](https://github.com/DarmawanRobotics/dharma_navigation) |
| `dharma_utils`                        | [robot profile, typed configuration, `DharmaNode`](https://github.com/DarmawanRobotics/dharma_utils)                       |
| `dharma_health`                       | [health monitor](https://github.com/DarmawanRobotics/dharma_health)                                                        |
| `dharma_rviz`                         | [RViz panels](https://github.com/DarmawanRobotics/dharma_rviz)                                                             |
| `dharma_sim`                          | [simulator and end-to-end tests](https://github.com/DarmawanRobotics/dharma_sim)                                           |

The `dharma_*` packages are checked out into `src/dharma/` at the versions in
[molly.repos](../molly.repos) (`scripts/setup.sh install dharma`).

## Development

| Document                                | For                                                        |
| --------------------------------------- | ---------------------------------------------------------- |
| [CONTRIBUTING.md](../CONTRIBUTING.md)   | branches, commits, build and test, rules from real crashes |
| [docker/README.md](../docker/README.md) | the dev container (Humble, VNC desktop)                    |
| [scripts/tmux/](../scripts/tmux)        | `scripts/start_tmux.sh` layouts (robot, sim)               |
| [CHANGELOG.md](../CHANGELOG.md)         | notable changes                                            |
