# Teleop UDP protocol (Molly v1)

One UDP port on the robot (default `9880`). The handheld
([MollyController](https://github.com/DarmawanRobotics/MollyController)) sends JSON datagrams;
the robot answers with JSON datagrams (control) and chunked binary datagrams (streams). No ROS on
the handheld. Molly v1 is Pashupati's teleop protocol v1 with its own port and stream magic (a
Molly handheld never talks to a Pashupati robot by mistake), without the VLM inspection, and
with sites, mapping, POIs, tours, the path finder and interaction.

## Session

1. Client sends `hello` (and again every 1 s as a keep-alive when idle).
2. Robot replies `welcome`, the loaded route, the map and the site, then streams `state` at
   `telemetry_hz`.
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
Text fields from the client are trimmed and cut to their limit (names 64 characters, narration and
`say` 1000, questions 500).

## Discovery

Any device may broadcast `{"t": "discover"}` to UDP port 9880 (no token needed). Every robot answers
the sender with `{"t": "beacon", "name", "secured" (token required), "in_use_by" (name of the
connected handheld or ""), "mode", "battery", "site" (active site or "")}`; the handheld lists
robots without typing an IP.

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
`ack` arrives (e.g. every 200 ms); the robot runs it once and resends the cached `ack`. A command
that takes long (`ask`, `go_to_poi`, `site_load`) is first answered with `ok: null` (pending) and
then with the result. A new session (another port or device, `bye`, or a timeout) forgets the old
acks, and acks of commands from an earlier session are not delivered. Start ids at a random
value per app start anyway.

**Driving and safety**

| `name` | `args` | Effect |
|---|---|---|
| `estop` | | latch the software e-stop: only zero reaches the robot, navigation and the tour pause |
| `estop_release` | | release it; navigation stays paused until `nav_resume` or `tour_resume` |
| `set_mode` | `mode`: `auto` \| `remote` | choose who drives; `remote` pauses navigation |
| `stand_up`, `sit_down`, `emergency_stop` (also latches the software e-stop), gaits, ... | | robot driver commands (`robot_commands` parameter, listed in `welcome.robot_commands`) |

**Sites and mapping**

| `name` | `args` | Effect |
|---|---|---|
| `site_list` | | `ack.data` = `{sites: [name, ...], active}` |
| `site_load` | `site` | make a site active: map, POIs, tours, AprilTags (pending ack) |
| `map_start` | `site` (a-z, 0-9, `_`, `-`), `reset` (bool, default `true`) | start mapping into this site, new or existing |
| `map_stop` | | stop integrating scans |
| `map_save` | | write the map into the active site and reload it |
| `get_map` | | resend the map stream (4) |
| `get_site` | | resend the site stream (5) |
| `localize` | | localize from a visible AprilTag |
| `record_tags` | | record the visible AprilTags into the active site |

**POIs, tours and walking**

| `name` | `args` | Effect |
|---|---|---|
| `poi_add` | `name`, optional `narration`, `language`, `gesture`, `dwell` (s, 0–600), `description` | new POI at the robot's current pose; `ack.data` = the POI |
| `poi_update` | `poi`: {`id` (required), any POI field; `here: true` moves it to the robot's pose} | change a POI; `ack.data` = the POI |
| `poi_delete` | `id` | remove a POI (also from the tours) |
| `tour_save` | `tour`: {`id` (empty: new), `name`, `poi_ids`, `return_poi_id`} | create or change a tour; `ack.data` = the tour |
| `tour_delete` | `id` | remove a tour |
| `tour_start` | `tour_id`, or `poi_ids` (list) | start a tour (the robot must be localized, mode `auto`) |
| `tour_pause`, `tour_resume`, `tour_stop`, `tour_skip` | | control the running tour |
| `go_to_poi` | `id` | find a path and walk to a POI |
| `go_to` | `x`, `y` (m), `yaw` (rad), map frame | find a path and walk to a pose |
| `go_cancel` | | stop walking |
| `nav_start`, `nav_stop`, `nav_pause`, `nav_resume` | | navigation along the loaded path or route |
| `nav_speed` | `speed` (m/s, `state.speed.min`–`max`) | walking speed (`path_follower_node` `target_linear_velocity`) until the next restart |
| `record_start`, `record_stop` | | route recording (teach and repeat) |
| `mark_stop` | `dwell` (s, 0–3600) | stop point on the recorded route |
| `list_routes` | | `ack.data` = route CSV paths, newest first |
| `load_route` | `file` | load a route CSV; only files under `routes_dir` |
| `get_route` | | resend the loaded route stream |

**Interaction**

| `name` | `args` | Effect |
|---|---|---|
| `say` | `text`, optional `lang`, `interrupt` (bool) | speak through the robot's speaker; `ack.data` = `{id}` |
| `stop_speech` | | stop talking, drop the queue |
| `greet` | | greet the visitor in front now |
| `greeter_set` | `enabled` (bool) | greet approaching visitors automatically (kept across restarts) |
| `gesture` | `name` (one of `welcome.gestures`) | robot gesture, only while standing still |
| `ask` | `question`, optional `lang`, `speak` (bool, default `true`) | ask the conversation model; `ack.data` = `{answer}` (pending ack, may take seconds) |
| `people_set` | `enabled` (bool) | camera person detection on or off (kept across restarts) |

## Robot → client (JSON)

| `t` | Fields |
|---|---|
| `welcome` | `name` (robot), `protocol` (`"molly-1"`), `commands` (every command name the robot runs), `robot_commands` (the driver commands of this robot, in the order of the `robot_commands` parameter), `gestures` (gesture names for `gesture`), `features` (any of `mapping`, `planner`, `guide`, `speech`, `greeter`, `conversation`, `people`), `limits` ([vx, vy, wz] maxima), `video` ({port, path}: WebRTC video, see below; absent when the robot only sends JPEG) |
| `ack` | `id`, `ok` (bool, `null` while pending), `msg`, optional `data` |
| `state` | see below |
| `pong` | `ts` |
| `busy` | `msg`: another device is driving, retry later |
| `denied` | `msg`: the `hello` had a missing or wrong token |
| `bye` | robot shutting down |

`state` fields (every one may be `null` when its source is not running):

| Field | Content |
|---|---|
| `seq` | increments per state, for packet-loss estimates |
| `mode`, `estop`, `field` | `auto` \| `remote`; e-stop engaged (bool); protective field `clear`, `limited`, `stop`, `stale`, `off` |
| `nav`, `nav_msg`, `mission` | navigation state and message (`navigation/status`), route mission |
| `localization` | `none`, `tag:<frame>`, `pose_estimate`, `origin` |
| `recording` | route recording (bool) |
| `pose` | [x, y, yaw] in the map |
| `battery` | {pct, v, a, temp} |
| `driver` | {level, msg} |
| `cmd` | [vx, vy, wz] sent to the robot |
| `health` | {ok, issues[], `cpu` / `gpu` / `ram` (% in use; `gpu` `null` off a Jetson), `ram_used` / `ram_total` (GB), `temp` (hottest zone, °C)} |
| `tags` | AprilTags in the camera image while the camera or video stream is on: [[id, x0, y0, x1, y1, x2, y2, x3, y3], ...], corners as fractions of the image width and height |
| `people` | {`on`, `state` (`loading`, `running`, `off`, `error`, `offline`), `count`, `ms`, `age`, `boxes` ([[x1, y1, x2, y2, score], ...] as fractions of the image, `[]` when older than 2 s), `backend`, `error`} |
| `speed` | walking speed {`target`, `min`, `max`} in m/s |
| `site` | {`name`, `map` (has a map), `pois`, `tours`, `tags`, `rev` (changes whenever the site stream changes)} |
| `mapping` | {`active`, `site`, `scans`, `area` (m², known)} |
| `planner` | {`state` (`idle`, `planning`, `following`, `replanning`, `arrived`, `failed`, `cancelled`), `goal` (POI id or ""), `remaining` (m), `msg`} |
| `tour` | {`state` (`idle`, `starting`, `running`, `paused`, `stopping`, `error`), `tour_id`, `name`, `index`, `total`, `poi_id`, `poi_name`, `activity`, `msg`} |
| `speech` | {`state` (`idle`, `speaking`, `done`, `failed`, `cancelled`), `text` (first 80 characters), `queued`} |
| `visitors` | {`count`, `front`, `behind`, `nearest` (m or `null`)} |
| `greeter` | {`enabled`, `state`} |
| `map_rev` | changes whenever the map stream changes: ask `get_map` when it differs from the last map received |

## Binary streams

Every datagram starts with a 13-byte little-endian header:

| Offset | Type | Field |
|---|---|---|
| 0 | 4 bytes | magic `MLY1` |
| 4 | u8 | stream: 1 camera, 2 lidar, 3 route, 4 map, 5 site |
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
| 3 route | JSON `{"t": "route", "points": [[x, y, dwell_sec], ...]}` in the map frame (route or planned path) |
| 4 map | u32 little-endian `n`, then `n` bytes of JSON `{"t": "map", "rev", "res" (m per pixel), "ox", "oy" (map position of the bottom-left pixel corner), "w", "h" (pixels), "site"}`, then a grayscale PNG of `w` × `h`: 0 occupied, 254 free, 205 unknown, row 0 at the top (largest y). The robot scales maps larger than `map_max_px` down (`res` is the scaled one). |
| 5 site | JSON `{"t": "site", "rev", "name", "pois": [{id, name, description, x, y, yaw, narration, language, gesture, dwell}], "tours": [{id, name, description, poi_ids, return_poi_id}]}` |

The map and the site are sent on `welcome`, whenever they change, and on `get_map` / `get_site`.
A lost chunk loses that frame: compare `state.map_rev` / `state.site.rev` and ask again.

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
