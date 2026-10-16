# Molly

Tour guide stack for the Deep Robotics **Jueying Lite3** (ROS 2 Humble on a Jetson Orin NX).

Molly starts from [Pashupati](https://github.com/DarmawanRobotics/Pashupati) `dev@be99273`
(LiDAR-inertial odometry, AprilTag localization, route following, path shift avoidance, control
mux, handheld gateway) and grows it into a guide: an occupancy map recorded together with the
AprilTags, a path finder between points of interest, tours, and interaction with visitors.

| Repository | |
|---|---|
| **Molly** (this) | the robot workspace |
| [lite3_ros2](https://github.com/DarmawanRobotics/lite3_ros2) | Lite3 driver, URDF and sensors (robot package) |
| [MollyController](https://github.com/DarmawanRobotics/MollyController) | handheld app (Retroid Pocket) |
| [MollyDashboard](https://github.com/DarmawanWeb/MollyDashboard) | web operator console |

Work in progress, see [CHANGELOG.md](CHANGELOG.md).

Molly is released under the [Apache License 2.0](LICENSE).
