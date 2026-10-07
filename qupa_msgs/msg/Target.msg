string  color        # BLUE | GREEN | RED
float32 distance_m   # distance from robot centre (base_link origin) on the floor
float32 angle_rad    # bearing, REP-103: 0 = front (+x), CCW positive, (-pi, pi]
float32 x            # target position in base_link (m)
float32 y            # target position in base_link (m)
float32 distance_px  # radial distance from the calibrated mirror centre (px)
int32   area         # blob area in pixels²
bool    in_range     # false if distance_px is outside the calibrated interval
