# Output of lidar_sector_node (molly_perception).
# Fixed-count angular sectors, each holding the nearest obstacle distance.
# Sector i center angle = angle_min + (i + 0.5) * angle_increment (rad, REP-103: +left/CCW).

std_msgs/Header header
uint32 num_sectors
float32 angle_min
float32 angle_increment
float32 range_max
float32[] ranges