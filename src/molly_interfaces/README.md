# molly_interfaces

The tour guide's own messages and services. Navigation, sites, POIs, tours, mapping, the path
finder and visitors come from
[dharma_interfaces](https://github.com/DarmawanRobotics/dharma_interfaces). Definitions:
[msg/](msg) and [srv/](srv).

| Kind | Name | Used by |
|---|---|---|
| msg | `TourCommand`, `TourState` | `/tour/cmd`, `/tour/state` (latched) |
| msg | `SpeechStatus` | `interaction/speech_status` (latched) |
| srv | `TourControl`, `GetTourState` | `/tour/control`, `/tour/get_state` |
| srv | `Say`, `Ask`, `Gesture` | `interaction/say`, `interaction/ask`, `interaction/gesture` |

The full list of topics and services is in [docs/interfaces.md](../../docs/interfaces.md).
