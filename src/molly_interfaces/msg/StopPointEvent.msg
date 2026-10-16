# Published by path_follower_node at each inspection stop point.
# event: arrived | inspected | departed | skipped

std_msgs/Header header
uint32 index
float32 x
float32 y
float32 yaw_deg
float32 dwell_sec
string event
string message
