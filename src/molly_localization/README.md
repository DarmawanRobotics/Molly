# molly_localization

`localization_node` bridges FAST-LIO into the standard tree and keeps `map → odom` correct.

```
map → odom → camera_init → body → base_link
```

`odom → camera_init` and `body → base_link` come from the URDF (`base_link → livox_imu_frame`) and
are static. `map → odom` is a planar (x, y, yaw) correction set from an AprilTag, an RViz 2D Pose
Estimate or, with `assume_start_origin`, the start pose; it is published on `/tf` at `map_odom_hz`
(REP-105), so every correction reaches the other nodes at once.

| Interface | Type | |
|---|---|---|
| `localization/start` | `std_srvs/Trigger` | localize from a visible tag |
| `localization/record_tags` | `std_srvs/Trigger` | average and save the map pose of visible tags |
| `localization/status` | `std_msgs/String`, latched | active source, `none` until localized |
| `localization/tag_markers` | `visualization_msgs/MarkerArray`, latched | known tags |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz pose estimate |

Optional periodic drift correction from any visible tag is gated by `max_correction_m/deg`.
Parameters: [config/localization_params.yaml](config/localization_params.yaml).
