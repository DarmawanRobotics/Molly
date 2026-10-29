# molly_interaction

Molly with visitors: it talks (text to speech), moves (robot gestures), greets the people who stop
in front of it, sees who walks around it (the LiDAR) and, optionally, answers their questions
with a local language model.

## Nodes

| Node                   | Role                                                                          |
| ---------------------- | ----------------------------------------------------------------------------- |
| `speech_node`          | speaks queued texts through Piper or eSpeak NG on the robot's speaker         |
| `interaction_node`     | robot gestures on request; greets visitors who stay in front of the robot     |
| `conversation_node`    | answers questions about the site with Ollama (optional, `conversation:=true`) |
| `visitor_tracker_node` | the people around the robot from the 360° LiDAR (`perception/visitors`)       |

### speech_node

One text after the other, in a background thread. `engine: auto` uses
[Piper](https://github.com/rhasspy/piper) (natural voices, offline) when its binary and the
language's voice are installed (`scripts/setup.sh install speech`: `/opt/molly/piper/piper` and
`/opt/molly/piper/voices/<voice>.onnx`), else eSpeak NG (apt `espeak-ng`), else `none`, which
only takes the time the text would take (`none_chars_per_sec`, for the simulator). Sound goes out
through `aplay` (`alsa-utils`) on the default card or `audio_device`. Texts reach the synthesizer
on stdin, never through a shell; they are put on one line and cut to `max_chars`.

| Interface                   | Type                                     |                                                                                                                                      |
| --------------------------- | ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `interaction/say`           | `molly_interfaces/Say`                   | queue a text in a language (`id`, `en`, empty: `default_language`); `interrupt` cancels what is said and waits first; returns the id |
| `interaction/stop_speech`   | `std_srvs/Trigger`                       | stop talking, drop the queue                                                                                                         |
| `interaction/say_text`      | `std_msgs/String`                        | queue a text, no answer                                                                                                              |
| `interaction/speech_status` | `molly_interfaces/SpeechStatus`, latched | per utterance: `SPEAKING`, then `DONE`, `FAILED` or `CANCELLED`; `IDLE` at start                                                     |

| Parameter                                   | Default                                                  |                                   |
| ------------------------------------------- | -------------------------------------------------------- | --------------------------------- |
| `engine`                                    | `auto`                                                   | `auto`, `piper`, `espeak`, `none` |
| `default_language`                          | `id`                                                     |                                   |
| `piper_bin`, `voices_dir`                   | `/opt/molly/piper/piper`, `/opt/molly/piper/voices`      |                                   |
| `voices`                                    | `["id:id_ID-news_tts-medium", "en:en_US-lessac-medium"]` | Piper voice per language          |
| `piper_length_scale`                        | `1.0`                                                    | > 1 speaks slower                 |
| `espeak_bin`, `espeak_voices`, `espeak_wpm` | `espeak-ng`, `["id:id", "en:en-us"]`, `150`              |                                   |
| `aplay_bin`, `audio_device`                 | `aplay`, `""`                                            | `aplay -L` lists the devices      |
| `max_queue`, `max_chars`                    | `10`, `1000`                                             |                                   |
| `none_chars_per_sec`                        | `15.0`                                                   |                                   |

### interaction_node

**Gestures.** `interaction/gesture` (`molly_interfaces/Gesture`) runs one of `gestures` by calling
the robot driver's `std_srvs/Trigger` `/<robot_ns>/<name>` and answers when the driver does: the
Lite3 driver's gesture services return once the gesture is over, and refuse while the robot walks.
The node also refuses while navigation moves the robot, and runs one gesture at a time.

**Greeter.** A visitor who stays within `approach_distance` in front of the robot
(`perception/visitors`) for `dwell_sec` is greeted with the next of `greeting_texts_<language>`
(and `greeting_gesture`, if set). Not again before `cooldown_sec`, and never the same visitor
twice: the front has to empty first. Never while a tour runs, the robot walks or talks. The on /
off is live and kept in `state_file`.

| Interface                 | Type                                      |                                                                                                                                                         |
| ------------------------- | ----------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `interaction/gesture`     | `molly_interfaces/Gesture`                | run a gesture, wait for it                                                                                                                              |
| `interaction/greet`       | `std_srvs/Trigger`                        | greet now                                                                                                                                               |
| `interaction/set_greeter` | `std_srvs/SetBool`                        | automatic greeting on / off                                                                                                                             |
| `interaction/greeter`     | `molly_interfaces/GreeterStatus`, latched | `enabled`, `state` (`OFF`, `WAITING`, `WATCHING`, `GREETING`, `GREETED`, `COOLDOWN`, `PAUSED`, `UNAVAILABLE`), `last_greet_age` (-1: never), `gestures` |

| Parameter                                        | Default                                    |                                                      |
| ------------------------------------------------ | ------------------------------------------ | ---------------------------------------------------- |
| `robot_ns`                                       | `robot`                                    | driver namespace (the bringup sets it)               |
| `gestures`                                       | `[hello, nod, bow, look_left, look_right]` | the driver's gesture services                        |
| `gesture_timeout_sec`                            | `20.0`                                     |                                                      |
| `greeter_available`                              | `true`                                     | `false` (launch `greeter:=false`): no greeter        |
| `enabled`                                        | `true`                                     | live; kept in `state_file` (`~/.molly/greeter.json`) |
| `approach_distance`, `dwell_sec`, `cooldown_sec` | `1.5`, `1.5`, `30.0`                       |                                                      |
| `language`                                       | `id`                                       | picks `greeting_texts_id` or `greeting_texts_en`     |
| `greeting_gesture`                               | `""`                                       | e.g. `nod`; gestures need the robot standing still   |

### conversation_node

`interaction/ask` (`molly_interfaces/Ask`) sends the question to Ollama (`/api/chat`, no new
Python dependencies) with a system prompt in Indonesian or English: Molly, the four-legged tour
guide robot of the active site, short spoken answers, and the site's POIs (name, description,
narration from `site/pois`) as knowledge. The last `history_turns` questions are kept as context.
Answers lose reasoning blocks and markup and are cut after a sentence (`max_answer_chars`); with
`speak` they are said through `interaction/say`. One question at a time; the HTTP call runs in a
worker thread. Every question is logged on `interaction/conversation` (`molly_interfaces/Conversation`:
question, answer, latency, success). Ollama comes from `scripts/setup.sh install ollama`; pull the model into its service.

| Parameter                        | Default                  |                                         |
| -------------------------------- | ------------------------ | --------------------------------------- |
| `url`                            | `http://127.0.0.1:11434` | Ollama                                  |
| `model`                          | `qwen2.5:3b`             |                                         |
| `timeout_sec`                    | `30.0`                   | the first question also loads the model |
| `max_tokens`, `max_answer_chars` | `200`, `600`             |                                         |
| `language`                       | `id`                     | when the question names none            |
| `robot_name`                     | `Molly`                  |                                         |
| `history_turns`                  | `2`                      |                                         |

### visitor_tracker_node

People around the robot for the guide (is anybody following?) and the greeter (somebody in
front?). It was part of Dharma 0.1's perception; Dharma 0.2 is general, so Molly owns it. Each
scan (`rate_hz`, the newest one) is moved into `base_link` with the LiDAR mount from TF, cut to
`height_min` .. `height_max` above the floor (`robot.base_height` below `base_link`) and to
`range_min` .. `max_range`, without the body (`robot.footprint` + `self_margin`). Points are
clustered on a grid (`cluster_tolerance`); a cluster of `min_points` and `min_width` ..
`max_width` is a person unless it sits on an occupied cell of `/map` grown by `map_margin`
(walls, pillars; skipped while not localized). A visitor counts after `confirm_scans` scans in a
row and is kept `max_misses` scans after it went (`assoc_distance` per scan).

| Interface                    | Type                                                                                                 |                                                                                                                               |
| ---------------------------- | ---------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `perception/visitors`        | `molly_interfaces/Visitors`                                                                          | positions in `base_link`, `count`, `front` (`front_angle_deg`), `behind` (`behind_angle_deg`), nearest distances (-1: nobody) |
| `perception/visitor_markers` | `visualization_msgs/MarkerArray`                                                                     | `publish_markers`                                                                                                             |
| reads                        | `/livox/lidar` (CustomMsg, raw, `input_type: livox`) or `pointcloud_topic` (PointCloud2), `/map`, TF |                                                                                                                               |

## Parameters

Typed (`ros2 param describe /<node> <name>`), defaults in
[molly_interaction/core/config.py](molly_interaction/core/config.py), notes in
[config/interaction_params.yaml](config/interaction_params.yaml); the tables above list them per
node. The robot profile (`robot.*`) gives the frames, the footprint and `base_height`.

## Launch

```bash
ros2 launch molly_interaction interaction.launch.py robot_ns:=robot conversation:=false
ros2 service call /interaction/say molly_interfaces/srv/Say "{text: 'Halo, saya Molly'}"
ros2 service call /interaction/gesture molly_interfaces/srv/Gesture "{name: nod}"
ros2 service call /interaction/ask molly_interfaces/srv/Ask "{question: 'Di mana kafe?', speak: true}"
```

| Argument                                    | Default                          |                                             |
| ------------------------------------------- | -------------------------------- | ------------------------------------------- |
| `params_file`                               | `config/interaction_params.yaml` |                                             |
| `robot_ns`                                  | `robot`                          | driver namespace for the gesture services   |
| `speech`, `visitors`                        | `true`                           | start `speech_node`, `visitor_tracker_node` |
| `greeter`                                   | (files)                          | `false`: no greeter                         |
| `conversation`                              | `false`                          | start `conversation_node`                   |
| `robot_params`, `use_sim_time`, `log_level` |                                  | robot profile, the Dharma launch contract   |

## Test

`test/unit` (speech engines, queue and player, greeter, conversation, visitors; no ROS),
`test/ros` (every node in-process: a fake driver gesture, Ollama and a cloud).

```bash
colcon test --packages-select molly_interaction && colcon test-result --verbose
```
