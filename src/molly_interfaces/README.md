# molly_interfaces

Molly's own messages and services. Everything the Dharma stack offers (status of every
subsystem, sites, mapping, the path finder, navigation, the control mux) is in
[dharma_interfaces](https://github.com/DarmawanRobotics/dharma_interfaces); Molly adds what a tour
guide needs on top. Definitions: [msg/](msg) and [srv/](srv).

| Kind | Name                                              | Used by                                                                                |
| ---- | ------------------------------------------------- | -------------------------------------------------------------------------------------- |
| msg  | `Poi`, `PoiArray`, `Tour`, `TourArray`            | `site/pois`, `site/tours` (latched, `site_content_node`)                               |
| msg  | `TourCommand`, `TourState`                        | `tour/cmd`, `tour/state` (latched, `guide_node`)                                       |
| msg  | `Visitors`                                        | `perception/visitors` (`visitor_tracker_node`)                                         |
| msg  | `SpeechStatus`, `GreeterStatus`, `Conversation`   | `interaction/speech_status`, `interaction/greeter` (latched), `interaction/conversation` |
| srv  | `SavePoi`, `SaveTour`, `DeleteById`               | `site/save_poi`, `site/save_tour`, `site/delete_poi`, `site/delete_tour`               |
| srv  | `GoToPoi`                                         | `guide/go_to_poi`                                                                      |
| srv  | `TourControl`, `GetTourState`                     | `tour/control`, `tour/get_state`                                                       |
| srv  | `Say`, `Ask`, `Gesture`                           | `interaction/say`, `interaction/ask`, `interaction/gesture`                            |

Status messages carry enum constants (`TourState.RUNNING`, `SpeechStatus.DONE`,
`GreeterStatus.WATCHING`), never strings or JSON. Every topic and service of the robot is listed
in [docs/interfaces.md](../../docs/interfaces.md).
