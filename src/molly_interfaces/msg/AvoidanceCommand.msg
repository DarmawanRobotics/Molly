# Output of obstacle_avoidance_node, consumed by path_follower_node.

std_msgs/Header header
float32 steering_bias      # rad/s, added to the controller's angular command
float32 lateral_bias       # m/s, sideways push (+left), used only by holonomic robots
float32 velocity_scale     # 0..1 multiplier on the target linear velocity
bool emergency             # hard stop, obstacle inside the emergency cone/distance
