# molly_guide

`guide_node` runs Molly's tours: it walks from POI to POI with the path finder, plays the POI's
gesture, says its narration, waits its dwell time and, while walking, waits for the visitors when
nobody follows. The robot owns the tour state; the dashboard and the handheld mirror
`/tour/state`.

```bash
ros2 launch molly_guide guide.launch.py
ros2 service call /tour/control molly_interfaces/srv/TourControl "{command: {kind: 0, tour_id: main}}"
ros2 service call /tour/control molly_interfaces/srv/TourControl "{command: {kind: 0, poi_ids: [lobby, hall]}}"
ros2 topic echo /tour/state
```

## A tour

1. **START** (a saved tour of the active site, `site/tours`, or a list of POI ids) needs the POIs
   in `site/pois`, a localized robot (`localization/status` not `none`), AUTO (`bridge/mode`) and
   no e-stop; otherwise it is refused with the reason. The state is `STARTING` until the first
   request is taken, then `RUNNING`.
2. `intro_texts` in the guide's `language`, if set (`narrating`).
3. For every POI:
   - `navigating`: `planner/go_to {poi_id}`, done when `planner/status` says `arrived` for it
     (stamped after the request). A failed walk (`go_to` refused, `failed`, `cancelled` or no
     arrival within `arrival_timeout_sec`) is tried `go_to_retries` more times, then the POI is
     skipped (`on_failure: skip`) or the tour ends in `ERROR` (`stop`).
   - `waiting_visitors`: while walking, when `perception/visitors.behind` stays 0 for
     `visitor_lost_sec`, the walk is held (`navigation/pause`) and `follow_me_texts` is said; the
     walk goes on once somebody is behind for `visitor_found_sec`, or after `visitor_wait_sec`
     (then not again on that leg). Never without fresh visitor data or in REMOTE.
   - `performing_motion`: the POI's gesture through `interaction/gesture` (answers when done).
   - `narrating`: the POI's narration through `interaction/say` in the POI's language, done when
     `interaction/speech_status` says `done`, `failed` or `cancelled` for its id (or after a
     time limit from the text length).
   - `dwelling`: the POI's `dwell_sec`.
4. `returning` to the tour's `return_poi_id`, if set, then `outro_texts`, then `IDLE`.

| Command | Accepted in | Effect |
|---|---|---|
| `START` | `IDLE`, `ERROR` | new tour |
| `PAUSE` | `RUNNING` | holds the walk, freezes the timers; what is being said is finished |
| `RESUME` | `PAUSED` (e-stop released) | continues where it was |
| `SKIP` | `RUNNING` | cancels the walk or the narration, goes on with the next POI |
| `STOP` | `STARTING`, `RUNNING`, `PAUSED`, `ERROR` | cancels the walk and the speech; `STOPPING`, then `IDLE` |

A refused command answers why (`/tour/control`) and the state is republished either way. The
software e-stop pauses a running tour; it continues only with a RESUME after the release.

## Interfaces

| Interface | Type | |
|---|---|---|
| `/tour/control` | `molly_interfaces/TourControl` | a command with an answer |
| `/tour/cmd` | `molly_interfaces/TourCommand` | a command without an answer |
| `/tour/get_state` | `molly_interfaces/GetTourState` | the state now, for a client that just connected |
| `/tour/state` | `molly_interfaces/TourState`, latched | on every change and every `publish_period_sec` |
| uses | `planner/go_to`, `planner/cancel`, `navigation/pause`, `interaction/say`, `interaction/stop_speech`, `interaction/gesture` | |
| reads | `site/pois`, `site/tours`, `planner/status`, `interaction/speech_status`, `perception/visitors`, `bridge/estop_engaged`, `bridge/mode`, `localization/status` | |

## Parameters

[config/guide_params.yaml](config/guide_params.yaml):

| Parameter | Default | |
|---|---|---|
| `language` | `id` | language of the guide's own texts |
| `intro_texts`, `outro_texts`, `follow_me_texts` | Indonesian and English | `language:text`; none for a language: `id`, then the first |
| `gestures`, `narrate` | `true` | play the POI gestures, say the narrations and texts |
| `arrival_timeout_sec` | `300.0` | walking time, waits for visitors excluded |
| `go_to_retries`, `retry_delay_sec` | `1`, `3.0` | |
| `on_failure` | `skip` | `skip` the POI or `stop` the tour |
| `gesture_timeout_sec` | `25.0` | |
| `narration_timeout_base_sec`, `narration_chars_per_sec` | `15.0`, `8.0` | narration time limit |
| `wait_for_visitors` | `true` | |
| `visitor_lost_sec`, `visitor_found_sec`, `visitor_wait_sec` | `6.0`, `1.0`, `20.0` | |
| `visitors_timeout_sec` | `2.0` | older visitor data: no waiting |
| `require_localization`, `require_auto_mode` | `true` | START conditions |
| `tick_hz`, `publish_period_sec` | `10.0`, `1.0` | |

The state machine and the steps are in `molly_guide/tour.py`, without ROS: `test/test_tour.py`
drives them with a fake clock, `test/test_guide_node.py` runs a whole tour against fake planner,
speech and gesture services.
