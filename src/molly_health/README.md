# molly_health

`health_monitor_node` watches what a tour depends on and says when something is off.

- **Sensor rates**: every `topic:min_hz` in `topics` is subscribed raw (no deserialisation, cheap
  even for point clouds) as soon as it appears; below 80 % of the minimum is an issue.
  `{robot_ns}` in a topic is the robot namespace (`robot_ns`, set by `robot.launch.py`).
- **Computer**: hottest thermal zone, memory use, free disk at `disk_path`, load average, CPU
  busy share and the Jetson GPU load (sysfs `load`, none on other computers).
- **Nodes**: every name in `expected_nodes` (filled by `robot.launch.py` with the nodes it starts)
  must be running once `startup_grace_sec` has passed.
- **Robot state**: odometry `lost` / `degraded`, a stale protective-field scan and an engaged e-stop
  are critical issues (`level: error`, diagnostics ERROR): the robot cannot guide like this.

| Topic | Type | |
|---|---|---|
| `molly/diagnostics` | `diagnostic_msgs/DiagnosticArray` | one status per check (rqt_robot_monitor) |
| `health/status` | `std_msgs/String` (JSON, latched) | `{ok, level (ok\|warn\|error), issues[], topics{topic: hz}, odometry, localization, field, estop, cpu_temp_c, memory_percent, memory_used_gb, memory_total_gb, cpu_percent, gpu_percent, disk_free_gb, load}` — shown on the handheld and the dashboard |

```bash
ros2 launch molly_health health.launch.py
ros2 topic echo /health/status
```
