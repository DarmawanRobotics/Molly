# guide_node -> tour/state (latched), on every change and every publish_period_sec.

uint8 IDLE = 0
uint8 STARTING = 1
uint8 RUNNING = 2
uint8 PAUSED = 3
uint8 STOPPING = 4
uint8 ERROR = 5

uint8 ACTIVITY_NONE = 0
uint8 ACTIVITY_NAVIGATING = 1
uint8 ACTIVITY_WAITING_VISITORS = 2
uint8 ACTIVITY_PERFORMING_MOTION = 3
uint8 ACTIVITY_NARRATING = 4
uint8 ACTIVITY_DWELLING = 5
uint8 ACTIVITY_RETURNING = 6

std_msgs/Header header
uint8 state
uint8 activity
string tour_id            # empty for an ad-hoc list of POIs
string tour_name
int32 current_poi_index   # 0-based, -1 when idle
int32 total_pois
string current_poi_id
string current_poi_name
string message            # human-readable detail of the activity
string error_reason       # set in ERROR
