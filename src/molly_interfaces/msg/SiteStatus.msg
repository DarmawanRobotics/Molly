# site_manager_node -> latched site/status: the active site and what it holds.

std_msgs/Header header
string site               # active site, empty: none loaded
string[] sites            # every site on the robot
bool has_map
bool mapping              # a mapping session is running
uint32 pois
uint32 tours
uint32 tags               # AprilTags recorded for this site
string message
