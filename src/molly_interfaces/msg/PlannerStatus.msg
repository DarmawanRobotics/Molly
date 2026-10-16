# planner_node -> latched planner/status.

std_msgs/Header header
string state              # idle, planning, following, replanning, arrived, failed, cancelled
string goal_id            # POI id, empty for a pose goal
float64 goal_x            # m, map frame
float64 goal_y
float64 goal_yaw          # rad
float32 path_length       # m, length of the current plan
float32 remaining         # m, still to walk along the plan
uint32 replans            # replans for this goal
string message
