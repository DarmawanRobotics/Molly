# molly_localization

Tag-based localization: `map → odom` from AprilTags, per-site tags files and the robot pose.

## Nodes

| Node | Role |
|---|---|
| `localization_node` | bridges FAST-LIO into the standard tree and keeps `map → odom` correct |

`localization_node` bridges FAST-LIO into the standard tree and keeps `map → odom` correct.

```
map → odom → camera_init → body → base_link
```

`odom → camera_init` and `body → base_link` come from the URDF (`base_link → livox_imu_frame`) and
are static. `map → odom` is a planar (x, y, yaw) correction set from an AprilTag, an RViz 2D Pose
Estimate or the current pose taken as the map origin (a new map); it is published on `/tf` at
`map_odom_hz` (REP-105), so every correction reaches the other nodes at once.

The tags file is per site: the site manager (`molly_mapping`) switches it with
`localization/load_tags` when a site is loaded or mapped, and `localization/record_tags` writes
the visible tags into it.

## Topics and services

| Interface | Type | |
|---|---|---|
| `localization/start` | `std_srvs/Trigger` | localize from a visible tag |
| `localization/record_tags` | `std_srvs/Trigger` | average and save the map pose of visible tags into the active tags file |
| `localization/load_tags` | `molly_interfaces/LoadFile` | switch the tags file; `create`: start an empty one when it does not exist |
| `localization/assume_origin` | `std_srvs/Trigger` | `map → odom` = identity: the current pose becomes the map origin |
| `localization/reset` | `std_srvs/Trigger` | forget `map → odom` (another site), localize from its tags again |
| `localization/status` | `std_msgs/String`, latched | `none`, `tag:<frame>`, `pose_estimate`, `origin` |
| `localization/pose` | `geometry_msgs/PoseStamped` | `map → base_link` at `pose_hz` while localized |
| `localization/tag_markers` | `visualization_msgs/MarkerArray`, latched | known tags |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz pose estimate |

## Parameters

Optional periodic drift correction from any visible tag is gated by `max_correction_m/deg`.

| Parameter | Default | |
|---|---|---|
| `tags_config_file` | `""` | tags file at start (the site manager switches it) |
| `auto_localize_on_start` | `true` | retry tag localization every second until the first success, also after `load_tags` and `reset` |
| `assume_start_origin` | `false` | the start pose is the map origin, no tag needed |
| `tag_frames` | `[base_map, charging_dock]` | tag TF frames `record_tags` looks for (the `apriltag` `tag.frames`) |
| `auto_correct_period_sec` | `0.0` | > 0: correct drift from any visible tag every N s |
| `pose_hz` | `10.0` | rate of `localization/pose`; 0: off |

All parameters: [config/localization_params.yaml](config/localization_params.yaml).

## Launch

```bash
ros2 launch molly_localization localization.launch.py
ros2 service call /localization/start std_srvs/srv/Trigger
ros2 topic echo /localization/status
```
