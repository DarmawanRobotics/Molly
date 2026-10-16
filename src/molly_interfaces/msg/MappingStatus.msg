# grid_mapper_node -> latched mapping/status.

std_msgs/Header header
bool active               # scans are being integrated into the grid
string site
uint32 scans              # scans integrated in this session
float32 resolution        # m per cell
uint32 width              # cells
uint32 height             # cells
float32 known_area_m2     # free + occupied area
string message
