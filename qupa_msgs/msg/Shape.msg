std_msgs/Header header
string    shape           # TRIANGLE | SQUARE | PENTAGON | UNKNOWN
int32     sides           # 0 if UNKNOWN
float32   side_m          # estimated side length (m)
float32   circumradius_m  # estimated circumradius (m)
float32   center_x        # figure centre in base_link (m)
float32   center_y
float32   rotation_rad    # angle of the first fitted vertex around the centre
int8      direction       # +1 CCW, -1 CW, as traced by the observed robot
float32   score           # chamfer distance to the best template (lower = better)
float32   margin          # score gap to the second-best candidate (higher = clearer)
float32[] vertices_x      # fitted polygon vertices in base_link (m)
float32[] vertices_y
float32[] path_x          # observed trajectory the decision was made on
float32[] path_y
