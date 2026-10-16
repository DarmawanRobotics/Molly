# Teleop UDP protocol (v1)

One UDP port on the robot (default `9870`). The handheld sends JSON datagrams; the robot answers
with JSON datagrams (control) and chunked binary datagrams (streams). No ROS on the handheld.

## Session

1. Client sends `hello` (and again every 1 s as a keep-alive when idle).
2. Robot replies `welcome` and the loaded route, then streams `state` at `telemetry_hz`.
3. Any datagram refreshes the session; after `client_timeout_sec` of silence the robot drops the
   client and zeroes the remote velocity.
4. One driver at a time: a `hello` from another device (IP address) is answered with `busy` while
   the current client is alive. The same device may reconnect from a new port (app restart).

If the robot is configured with a `token`, every client message must include `"k": "<token>"`
(compared in constant time). A `hello` with a missing or wrong token is answered with `denied`
(at most once per second per address); every other message without it is dropped silently.
Without a token anyone on the network can drive the robot; the robot logs a warning at start.

Malformed datagrams are dropped. Joystick values that are not finite numbers (NaN, inf, strings,
booleans) count as 0, so a broken client stops the robot instead of driving it at full speed.

## Discovery

Any device may broadcast `{"t": "discover"}` to UDP port 9870 (no token needed). Every robot answers
the sender with `{"t": "beacon", "name", "secured" (token required), "in_use_by" (name of the
connected handheld or ""), "mode", "battery"}`; the handheld lists robots without typing an IP.

## Client → robot

| `t` | Fields | Meaning |
|---|---|---|
| `hello` | `name`, `streams: {camera: bool, lidar: bool, video: bool, camera_fps?: number}` | register / keep alive |
| `joy` | `vx`, `vy` (m/s), `wz` (rad/s) | teleop velocity, send at 20–30 Hz; only used in `remote` mode |
| `cmd` | `id` (int, unique per command), `name`, `args` (object) | run a command, see below |
| `streams` | `camera`, `lidar`, `video` (bool), `camera_fps` (number, capped by the `camera_fps` parameter) | change subscriptions and camera rate |
| `ping` | `ts` | latency probe, echoed as `pong` |
| `bye` | | leave |

Commands are idempotent per `id` and `name` within a session: resend the same `cmd` until its
`ack` arrives (e.g. every 200 ms); the robot runs it once and resends the cached `ack`. A new
session (another port or device, `bye`, or a timeout) forgets the old acks, and acks of commands
from an earlier session are not delivered. Start ids at a random value per app start anyway.

| `name` | `args` | Effect |
|---|---|---|
| `estop` | | latch the software e-stop: only zero reaches the robot, navigation pauses |
| `estop_release` | | release it; navigation stays paused until `nav_resume` |
| `set_mode` | `mode`: `auto` \| `remote` | choose who drives; `remote` pauses navigation |
| `nav_start`, `nav_stop` | | start / stop navigation |
| `nav_pause`, `nav_resume` | | hold / continue without losing progress |
| `record_start`, `record_stop` | | route recording |
| `mark_stop` | `dwell` (s, 0–3600) | mark an inspection stop at the current pose |
| `list_routes` | | `ack.data` = route CSV paths, newest first |
| `load_route` | `file` | load a route CSV; only files under `routes_dir` |
| `get_route` | | resend the loaded route stream |
| `localize` | | localize from a visible AprilTag |
| `record_tags` | | record AprilTag poses |
| `inspection_set` | any of `enabled` (bool), `model` (Ollama model name), `interval` (s: `0` continuous, `> 0` every N s, `< 0` at stop points only), `categories` (the finding types, all of them in the order the model gets them: `[{name (a-z, 0-9, _), desc (what the model looks for, up to 200 characters; empty keeps the built-in text of a known type), report (bool: sent to NETRA)}]`, 1 to 12) | change the VLM inspection live (`anomaly_detector_node` parameters, kept across restarts) |
| `inspection_models` | | `ack.data` = models to choose from: `[{name, installed (bool or null when unknown), active}]` |
| `inspection_categories` | | `ack.data` = the finding types: `[{name, desc (as the model reads it), report, builtin (known type with its own text), own_desc}]` |
| `people_set` | `enabled` (bool) | person detection on or off (`person_detector_node`, kept across restarts) |
| `nav_speed` | `speed` (m/s, `state.speed.min`–`max`) | patrol speed (`path_follower_node` `target_linear_velocity`) until the next restart |
| `stand_up`, `sit_down`, `emergency_stop` (also latches the software e-stop), `move_mode`, `balance_stand_mode`, `lock_mode`, `slow_speed`, `normal_speed`, `fast_speed` | | robot driver commands (`robot_commands` parameter) |

## Robot → client (JSON)

| `t` | Fields |
|---|---|
| `welcome` | `name` (robot), `commands` (every command name), `robot_commands` (the driver commands of this robot, in the order of the `robot_commands` parameter), `limits` ([vx, vy, wz] maxima), `video` ({port, path}: WebRTC video, see below; absent when the robot only sends JPEG) |
| `ack` | `id`, `ok` (bool, `null` while pending), `msg`, optional `data` |
| `state` | `seq` (increments per state, for packet-loss estimates), `mode`, `estop` (bool), `field` (protective field: `clear`, `limited`, `stop`, `stale`, `off` or `null`), `nav`, `nav_msg`, `mission`, `localization`, `recording`, `pose` ([x, y, yaw] in map or `null`), `battery` ({pct, v, a, temp} or `null`), `driver` ({level, msg} or `null`), `cmd` ([vx, vy, wz] sent to the robot), `health` ({ok, issues[], `cpu` / `gpu` / `ram` (% in use; `gpu` `null` off a Jetson), `ram_used` / `ram_total` (GB), `temp` (hottest zone, °C)} or `null`), `tags` (AprilTags in the camera image while the camera or video stream is on: [[id, x0, y0, x1, y1, x2, y2, x3, y3], ...], corners as fractions of the image width and height), `inspection` (VLM inspection, `null` when no detector runs: {`on`, `state` (`loading`, `running`, `off`, `error`, `offline` = no status for 6 s), `model`, `interval` (as in `inspection_set`), `error`, `count` (inspections), `reports` (findings sent to NETRA), `queue` / `queue_max` (interval mode: frames waiting for the model, first in first out), `last`: {`cat` (category or `none`), `reported`, `ms` (latency), `age` (s), `waited` (s the frame waited in the queue), `desc`, `source` (`periodic` or `stop_point`)} or `null`}), `people` (person detection, `null` when no detector runs: {`on`, `state` (`loading`, `running`, `off`, `error`, `offline`), `count` (people in the last inspected frame, `null` before the first), `ms` (latency), `age` (s since that frame), `boxes` (people in that frame, `[]` when older than 2 s: [[x1, y1, x2, y2, score], ...] as fractions of the image), `backend`, `error`}), `speed` (patrol speed {`target`, `min`, `max`} in m/s, `null` without navigation) |
| `pong` | `ts` |
| `busy` | `msg`: another device is driving, retry later |
| `denied` | `msg`: the `hello` had a missing or wrong token |
| `bye` | robot shutting down |

## Binary streams

Every datagram starts with a 13-byte little-endian header:

| Offset | Type | Field |
|---|---|---|
| 0 | 4 bytes | magic `MLY1` |
| 4 | u8 | stream: 1 camera, 2 lidar, 3 route |
| 5 | u32 | frame number (increasing per stream) |
| 9 | u16 | chunk index |
| 11 | u16 | chunk count |

Payloads are split into chunks of at most 1400 bytes (no IP fragmentation). Reassemble per stream;
when a chunk of a newer frame arrives, drop the incomplete older frame. The robot subscribes to the
camera, the cloud and TF only while a client is connected and has the stream enabled.

| Stream | Payload |
|---|---|
| 1 camera | one JPEG image |
| 2 lidar | N × (int16 x, int16 y, int16 z) in centimetres, robot frame (`base_link`), little-endian |
| 3 route | JSON `{"t": "route", "points": [[x, y, dwell_sec], ...]}` in the map frame |

## WebRTC video

When the robot runs MediaMTX (`molly` `robot.launch.py`, `script/setup.sh install webrtc`), the
`welcome` carries `video: {port, path}` and the handheld can play the camera as H.264 over WebRTC
instead of the JPEG stream: smoother, a fraction of the bandwidth, and no lost frames when one
datagram is lost.

1. POST an SDP offer (receive-only video, all ICE candidates gathered) to
   `http://<robot>:<port>/<path>/whep` with `Content-Type: application/sdp` and HTTP basic auth,
   user `teleop`, password the token (any user and password when the robot has no token).
2. `201 Created` returns the SDP answer; its `Location` header is the session to `DELETE` when
   done. `401` means a wrong token. The first answer can take a few seconds: MediaMTX starts the
   encoder (`video_stream_node`) when the first viewer comes and stops it 10 s after the last one.
3. While the WebRTC video plays, send `streams` with `camera: false, video: true`: the JPEG frames
   stop and the `tags` keep coming for the overlay. Turn `camera` back on when WebRTC fails.

The media goes over UDP (`8190` with the default launch); WHEP signalling over TCP `<port>`
(`8890`).
