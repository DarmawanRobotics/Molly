# molly_rviz

RViz panels for operating and tuning the robot, each addable from **Panels → Add New Panel**.

## Panels

| Panel | |
|---|---|
| Mission (`ControlPanel`) | localize, record route and stop points, load route, start / stop / pause |
| Status (`StatusPanel`) | battery, motor temperature, driver diagnostics, cmd_vel |
| Tuning (`TuningPanel`) | live parameter editing with controller / algorithm selection and YAML export |
| AprilTags (`TagPanel`) | record tag poses, localize from a tag |

## Usage

`rviz/robot.rviz` loads Mission, Status and Tuning with the navigation, avoidance, route and tag
markers. Robot topics are relative; remap them on the command line, e.g.
`rviz2 -d robot.rviz --ros-args -r battery:=/robot/battery -r cmd_vel:=/robot/cmd_vel`.
