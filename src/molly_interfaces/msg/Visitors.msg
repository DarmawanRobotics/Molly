# visitor_tracker_node -> perception/visitors: people around the robot, base_link frame.

std_msgs/Header header
geometry_msgs/Point[] positions   # on the floor in base_link (z = 0)
float32[] confidences             # 0-1, one per position
uint32 count
uint32 front              # people in the front sector
uint32 behind             # people in the rear sector (following the robot)
float32 nearest           # m, -1 when nobody
float32 nearest_front     # m, -1 when nobody in front
float32 nearest_behind    # m, -1 when nobody behind
