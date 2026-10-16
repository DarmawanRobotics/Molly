# molly_perception

| Node | Output |
|---|---|
| `lidar_sector_node` | `perception/sector_scan` — nearest obstacle per angular sector in `base_link` (Livox CustomMsg or any PointCloud2), floor and robot-body filtered, front `fov_deg` for avoidance; `perception/safety_scan` — the same over 360° (`safety_num_sectors`) for the protective field, every sector reported blocked (0 m) when a cloud has fewer than `min_raw_points` points (LiDAR covered or failed) |
| `apriltag_node` (apriltag_ros) | tag TFs used by localization |
| `image_throttle_node` (`apriltag_throttle`) | the camera for the tag detector at `rate_hz` (5), relayed as serialized bytes |
| `person_detector_node` | `perception/people` — people on the floor in the `map` frame (YOLO + floor-plane projection, no depth needed); feeds the crowd heatmap |

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
crowd heatmap) while the robot is localized. `enabled`, `rate_hz` and `confidence` change live
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
