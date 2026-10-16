# A point of interest on a site's map: a place the guide walks to and talks about (map frame).

string id                 # stable id: a-z, 0-9, _ and -, unique per site
string name               # shown to operators and visitors
string description        # what it is; also given to the conversation model
float64 x                 # m, map frame
float64 y                 # m, map frame
float64 yaw               # rad, heading the robot takes on arrival (towards the exhibit)
string narration          # spoken on arrival; empty: nothing is said
string language           # narration language ("id", "en"); empty: the speech default
string gesture            # gesture on arrival; empty: none (see interaction/gesture)
float32 dwell_sec         # wait after the narration before moving on
