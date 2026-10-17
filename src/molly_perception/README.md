# molly_perception

| Node | Output |
|---|---|
| `lidar_sector_node` | `perception/sector_scan` — nearest obstacle per angular sector in `base_link` (Livox CustomMsg or any PointCloud2), floor and robot-body filtered, front `fov_deg` for avoidance; `perception/safety_scan` — the same over 360° (`safety_num_sectors`) for the protective field, every sector reported blocked (0 m) when a cloud has fewer than `min_raw_points` points (LiDAR covered or failed) |
| `apriltag_node` (apriltag_ros) | tag TFs used by localization |
| `image_throttle_node` (`apriltag_throttle`) | the camera for the tag detector at `rate_hz` (5), relayed as serialized bytes |
| `person_detector_node` | `perception/people` — people on the floor in the `map` frame (YOLO + floor-plane projection, no depth needed) |
| `visitor_tracker_node` | `perception/visitors` — the people around the robot from the 360° LiDAR, in front and behind, for the guide and the greeter |

`input_type: livox` (default) reads the Livox CustomMsg straight from its serialized bytes with
numpy (`livox_cdr.py`, ~1 ms per scan); `pointcloud2` takes any PointCloud2, e.g. for another
LiDAR. FAST-LIO's `/cloud_registered_body` misses everything within its `blind` radius, so it is
not a good input for obstacles next to the robot.

## Person detection

`person_detector_node` runs YOLO11n, the smallest YOLO, exported to ONNX at 416 px
(`script/setup.sh install people_model` downloads it from the `people-model` release, which CI
exports and checks). It needs no PyTorch: ONNX Runtime runs it (its CUDA provider when a Jetson
GPU build is installed, else 2 CPU threads), or OpenCV DNN without ONNX Runtime. `rate_hz: 0`
(default) detects 5 times a second on the GPU (smooth boxes on the handheld) and 2 times on the
CPU, a small share of one Orin NX core. `.pt` and TensorRT `.engine` models still work
through Ultralytics. Set `floor_z` to minus the height of `base_link`.

People are counted in every inspected frame and placed on the map (`perception/people`, for the
visitor tracker with `use_camera_people`) while the robot is localized. `enabled`, `rate_hz` and `confidence` change live
(handheld, `ros2 param set /person_detector_node enabled false`); the on/off choice is kept in
`~/.molly/people.json`. `PEOPLE=false` in the env file does not start the node at all.

| Topic | Type | |
|---|---|---|
| `perception/people` | `geometry_msgs/PoseArray` | people on the floor, `map` frame; `orientation.w` is the score |
| `perception/people_status` | `std_msgs/String` (JSON, latched, every 2 s and per detection) | `{enabled, state (loading, running, off, error), error, backend, model, count, boxes ([x1, y1, x2, y2, score] as fractions of the image), placed, latency_ms, at, rate_hz}` |

```bash
ros2 launch molly_perception perception.launch.py people:=true draw_tags:=false
```

Parameters: [config/perception_params.yaml](config/perception_params.yaml).

## Visitors

`visitor_tracker_node` finds the people around the robot in the LiDAR, all the way round, so the
guide knows whether anybody is following and the greeter whether somebody stands in front. The
camera only looks ahead; the Mid-360 also sees the visitors walking behind the robot.

1. Points between `z_min` and `z_max` above / below `base_link` (legs and torso), off the robot's
   body and within `max_range`.
2. Grid clustering (`cluster_tolerance`), kept when a cluster is `min_width`–`max_width` long
   (along its main axis) with at least `min_points` points.
3. Clusters on occupied cells of `/map`, grown by `map_margin`, are walls, pillars or furniture
   and are dropped. Without a map or while the robot is not localized every cluster is kept.
4. With `use_camera_people`, people from `person_detector_node` raise the confidence of a cluster
   next to them, or are added where the LiDAR has nothing.
5. A visitor counts once seen in `confirm_scans` processed scans in a row.

`front` is the sector `front_angle_deg` wide straight ahead, `behind` the one `behind_angle_deg`
wide straight back; people beside the robot only count in `count`.

| Topic | Type | |
|---|---|---|
| `perception/visitors` | `molly_interfaces/Visitors` | positions on the floor in `base_link`, confidences, `count`, `front`, `behind`, nearest distances (-1: nobody), at most `rate_hz` |
| `perception/visitor_markers` | `visualization_msgs/MarkerArray` | cylinders: blue in front, green behind, yellow beside |

```bash
ros2 launch molly_perception perception.launch.py visitors:=true people:=false
ros2 topic echo /perception/visitors
```

