# molly_health

Health monitoring: sensor rates, the computer, node liveness and the robot's safety state.

## Nodes

| Node | Role |
|---|---|
| `health_monitor_node` | watches what a tour depends on and says when something is off |

Checks:

- **Sensor rates**: every `topic:min_hz` in `topics` is subscribed raw (no deserialisation, cheap
  even for point clouds) as soon as it appears; below 80 % of the minimum is an issue.
  `{robot_ns}` in a topic is the robot namespace (`robot_ns`, set by `robot.launch.py`).
- **Computer**: hottest thermal zone, memory use, free disk at `disk_path`, load average, CPU
  busy share and the Jetson GPU load (sysfs `load`, none on other computers).
- **Nodes**: every name in `expected_nodes` (filled by `robot.launch.py` with the nodes it starts)
  must be running once `startup_grace_sec` has passed.
- **Robot state**: odometry `lost` / `degraded`, a stale protective-field scan and an engaged e-stop
  are critical issues (`level: error`, diagnostics ERROR): the robot cannot guide like this.

## Topics and services

| Topic | Type | |
|---|---|---|
| `molly/diagnostics` | `diagnostic_msgs/DiagnosticArray` | one status per check (rqt_robot_monitor) |
| `health/status` | `std_msgs/String` (JSON, latched) | `{ok, level (ok\|warn\|error), issues[], topics{topic: hz}, odometry, localization, field, estop, cpu_temp_c, memory_percent, memory_used_gb, memory_total_gb, cpu_percent, gpu_percent, disk_free_gb, load}` — shown on the handheld and the dashboard |

## Parameters

| Parameter | Default | |
|---|---|---|
| `robot_ns` | `robot` | fills `{robot_ns}` in `topics` (set by `robot.launch.py`) |
| `topics` | see the file | `topic:min_hz` entries |
| `max_cpu_temp_c`, `max_memory_percent`, `min_disk_free_gb` | `85.0`, `92.0`, `5.0` | issue thresholds |
| `disk_path` | `~` | disk checked for free space |
| `publish_hz` | `1.0` | rate of `health/status` |
| `expected_nodes` | `[""]` | nodes that must run (filled by `robot.launch.py`) |
| `startup_grace_sec` | `30.0` | no missing-node issues during start-up |

All parameters: [config/health_params.yaml](config/health_params.yaml).

## Launch

```bash
ros2 launch molly_health health.launch.py
ros2 topic echo /health/status
```
