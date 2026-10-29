# molly_guide

Molly's tours: walk from POI to POI with the Dharma path finder, gesture, narrate, dwell and wait
for the visitors; and the POIs and tours of a site, stored next to its map.

## Nodes

| Node                | Role                                                                                   |
| ------------------- | -------------------------------------------------------------------------------------- |
| `guide_node`        | runs the tours and owns the tour state; walks to single POIs (`guide/go_to_poi`)       |
| `site_content_node` | the POIs and tours of the active site: `site/pois`, `site/tours` and the edit services |

### A tour

1. **START** (a saved tour of the active site or a list of POI ids) needs the POIs in
   `site/pois`, a localized robot (`localization/status` not `NONE`), the control mux in AUTO
   (`control/status`) and no e-stop; otherwise it is refused with the reason. The state is
   `STARTING` until the first step is taken, then `RUNNING`.
2. `intro_texts` in the guide's `language`, if set (`ACTIVITY_NARRATING`).
3. For every POI:
   - `ACTIVITY_NAVIGATING`: `planner/go_to {x, y, yaw}` of the POI; done when `planner/status`
     says `ARRIVED` for that goal (within `goal_tolerance`, stamped after the request). A failed
     walk (refused, `FAILED`, `CANCELLED` or no arrival within `arrival_timeout_sec`) is tried
     `go_to_retries` more times, then the POI is skipped (`on_failure: skip`) or the tour ends
     in `ERROR` (`stop`).
   - `ACTIVITY_WAITING_VISITORS`: while walking, when `perception/visitors.behind` stays 0 for
     `visitor_lost_sec`, the walk is held (`navigation/pause`) and `follow_me_texts` is said; it
     goes on (`navigation/resume`) once somebody is behind for `visitor_found_sec`, or after
     `visitor_wait_sec` (then not again on that leg). Never without fresh visitor data or in
     REMOTE; the wait does not count as walking time.
   - `ACTIVITY_PERFORMING_MOTION`: the POI's gesture through `interaction/gesture`.
   - `ACTIVITY_NARRATING`: the POI's narration through `interaction/say`, done when
     `interaction/speech_status` reports the utterance `DONE`, `FAILED` or `CANCELLED` (or after
     `narration_timeout_base_sec` + length / `narration_chars_per_sec`).
   - `ACTIVITY_DWELLING`: the POI's `dwell_sec`.
4. `ACTIVITY_RETURNING` to the tour's `return_poi_id`, if set, then `outro_texts`, then `IDLE`.

| Command  | Accepted in                              | Effect                                                                      |
| -------- | ---------------------------------------- | --------------------------------------------------------------------------- |
| `START`  | `IDLE`, `ERROR`                          | new tour                                                                    |
| `PAUSE`  | `RUNNING`                                | holds the walk (`navigation/pause`), freezes the timers                     |
| `RESUME` | `PAUSED`, e-stop released                | continues where it was (`navigation/resume`)                                |
| `SKIP`   | `RUNNING`                                | cancels the walk or the narration, goes on with the next POI                |
| `STOP`   | `STARTING`, `RUNNING`, `PAUSED`, `ERROR` | cancels the walk (`planner/cancel`) and the speech; `STOPPING`, then `IDLE` |

The e-stop (`control/status.estop`) pauses a running tour; it continues only with a `RESUME`
after the release (the path follower's own e-stop pause is sticky in the same way). An
operator driving in REMOTE pauses the follower, not the tour; back in AUTO the walk goes on.

### POIs and tours of a site

`site_content_node` follows dharma_mapping's `site/status`. The active site's
`<sites_dir>/<site>/pois.json` and `tours.json` (written atomically, the previous version as
`.bak`; broken entries are skipped and logged) are published latched on `site/pois` and
`site/tours`, re-read when they change on disk. `site/save_poi` with `at_robot_pose` takes the
robot's `map -> base_link` pose. Deleting a POI removes it from every tour. A mapping reset of
the site (`mapping/start {reset: true}`) moves its map, tags and routes into `backup-<time>/`;
the POIs and tours are poses in the old map frame and go into the same backup
(`archive_with_map`).

## Topics and services

| Interface                                                                | Type                                                                                                                                                |                                                |
| ------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------- |
| `tour/control`, `tour/cmd`                                               | `molly_interfaces/TourControl`, `TourCommand`                                                                                                       | a command with or without answer               |
| `tour/get_state`, `tour/state`                                           | `molly_interfaces/GetTourState`, `TourState` latched                                                                                                | on every change and every `publish_period_sec` |
| `guide/go_to_poi`                                                        | `molly_interfaces/GoToPoi`                                                                                                                          | walk to one POI (not during a tour)            |
| `site/pois`, `site/tours`                                                | `molly_interfaces/PoiArray`, `TourArray` latched                                                                                                    |                                                |
| `site/save_poi`, `site/save_tour`, `site/delete_poi`, `site/delete_tour` | `molly_interfaces/SavePoi`, `SaveTour`, `DeleteById`                                                                                                |                                                |
| uses                                                                     | `planner/go_to`, `planner/cancel`, `navigation/pause`, `navigation/resume`, `interaction/say`, `interaction/stop_speech`, `interaction/gesture`     |                                                |
| reads                                                                    | `planner/status`, `control/status`, `localization/status`, `site/status`, `interaction/speech_status`, `perception/visitors`, TF `map -> base_link` |                                                |

## Parameters

Typed (`ros2 param describe /guide_node <name>`), defaults in
[molly_guide/core/config.py](molly_guide/core/config.py), notes in
[config/guide_params.yaml](config/guide_params.yaml).

| Parameter                                                          | Default                        |                                                            |
| ------------------------------------------------------------------ | ------------------------------ | ---------------------------------------------------------- |
| `language`                                                         | `id`                           | language of the guide's own texts                          |
| `intro_texts`, `outro_texts`, `follow_me_texts`                    | Indonesian and English         | `language:text`; none for a language: `id`, then the first |
| `gestures`, `narrate`                                              | `true`                         | play the POI gestures, say narrations and texts            |
| `arrival_timeout_sec`                                              | 300 s                          | walking time, waits for visitors excluded                  |
| `go_to_retries`, `retry_delay_sec`                                 | 1, 3 s                         |                                                            |
| `on_failure`                                                       | `skip`                         | `skip` the POI or `stop` the tour                          |
| `gesture_timeout_sec`                                              | 25 s                           |                                                            |
| `narration_timeout_base_sec`, `narration_chars_per_sec`            | 15 s, 8                        | narration time limit                                       |
| `wait_for_visitors`                                                | `true`                         |                                                            |
| `visitor_lost_sec`, `visitor_found_sec`, `visitor_wait_sec`        | 6, 1, 20 s                     |                                                            |
| `visitors_timeout_sec`                                             | 2 s                            | older visitor data: no waiting                             |
| `require_localization`, `require_auto_mode`                        | `true`                         | START conditions                                           |
| `goal_tolerance`                                                   | 0.05 m                         | a planner goal counts as the POI within                    |
| `tick_hz`, `publish_period_sec`, `stop_settle_sec`                 | 10 Hz, 1 s, 0.5 s              |                                                            |
| `site_content_node` `sites_dir`, `refresh_sec`, `archive_with_map` | `~/dev/Molly/map`, 2 s, `true` |                                                            |

## Launch

```bash
ros2 launch molly_guide guide.launch.py robot_params:=<robot.yaml> sites_dir:=~/dev/Molly/map
ros2 service call /tour/control molly_interfaces/srv/TourControl "{command: {kind: 0, tour_id: main}}"
ros2 service call /tour/control molly_interfaces/srv/TourControl "{command: {kind: 0, poi_ids: [lobby, hall]}}"
ros2 service call /site/save_poi molly_interfaces/srv/SavePoi "{poi: {name: Lobby, narration: 'Selamat datang', language: id, dwell_sec: 5.0}, at_robot_pose: true}"
ros2 service call /site/save_tour molly_interfaces/srv/SaveTour "{tour: {name: Main, poi_ids: [lobby]}}"
ros2 service call /guide/go_to_poi molly_interfaces/srv/GoToPoi "{id: lobby}"
ros2 topic echo /tour/state
```

`kind` is a `TourCommand` constant: `0` START, `1` PAUSE, `2` RESUME, `3` STOP, `4` SKIP. The
bringup (`molly_bringup`) starts both nodes with the robot profile and the stack file.

## Layout and tests

| Module                   |                                                                    |
| ------------------------ | ------------------------------------------------------------------ |
| `core/config.py`         | `GuideConfig`, `SiteContentConfig` (typed parameters)              |
| `core/tour/model.py`     | states, commands, activities, `Stop`, the `Hooks` the engine calls |
| `core/tour/steps.py`     | the steps from one stop to the next                                |
| `core/tour/engine.py`    | `TourEngine`: commands, answers, timers                            |
| `core/tour/followers.py` | `FollowerWatch`: when the walk waits for the visitors              |
| `core/tour/request.py`   | resolving a START, readiness                                       |
| `core/site/*.py`         | POI / tour validation, the files of a site, archive on a reset     |
| `utils/hooks.py`         | the engine's hooks as asynchronous service calls                   |
| `utils/convert.py`       | messages <-> core values                                           |

`test/unit` drives the cores with a fake clock; `test/ros` runs both nodes in-process against a
fake planner, speech, gestures, control mux and TF.

```bash
colcon test --packages-select molly_guide && colcon test-result --verbose
```
