# interaction_node -> interaction/greeter (latched), on a change and every few seconds.

uint8 OFF = 0             # switched off (interaction/set_greeter)
uint8 WAITING = 1         # nobody in front
uint8 WATCHING = 2        # somebody in front, not yet for dwell_sec
uint8 GREETING = 3        # a greeting is being said
uint8 GREETED = 4         # the visitor in front was greeted; the front must empty first
uint8 COOLDOWN = 5        # the last greeting is less than cooldown_sec ago
uint8 PAUSED = 6          # a tour runs, the robot walks or talks
uint8 UNAVAILABLE = 7     # no greeter on this robot (greeter_available false)

std_msgs/Header header
bool enabled
uint8 state
float32 last_greet_age    # s since the last greeting, -1 before the first one
string[] gestures         # gesture names interaction/gesture runs
