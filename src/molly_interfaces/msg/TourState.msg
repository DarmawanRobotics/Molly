# guide_node -> everyone, latched on /tour/state; republished on every change and every second.

uint8 STATE_IDLE=0
uint8 STATE_STARTING=1
uint8 STATE_RUNNING=2
uint8 STATE_PAUSED=3
uint8 STATE_STOPPING=4
uint8 STATE_ERROR=5

std_msgs/Header header
uint8 state
string tour_id            # empty for an ad-hoc list of POIs
string tour_name
int32 current_poi_index   # 0-based, -1 when idle
int32 total_pois
string current_poi_id
string current_poi_name
# "", navigating, waiting_visitors, performing_motion, narrating, dwelling, returning
string activity
string message            # human-readable detail of the activity
string error_reason       # set in STATE_ERROR
