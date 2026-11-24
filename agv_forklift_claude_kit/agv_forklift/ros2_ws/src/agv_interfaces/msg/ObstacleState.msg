# Published by obstacle_monitor on /obstacle_state (one message per scan).
# All geometry is in base_link. Corridor definitions: docs/02_interfaces.md section 5.
uint8 ZONE_CLEAR=0
uint8 ZONE_SLOW=1
uint8 ZONE_STOP=2

std_msgs/Header header       # stamp = scan stamp, frame_id = base_link
bool sensor_ok               # false => every consumer MUST treat it as ZONE_STOP
uint8 zone                   # filtered (persistence + hysteresis)
uint8 raw_zone               # this scan only
float32 front_min_distance   # m, measured from the robot FRONT EDGE; = scan range_max if no point
bool left_free               # left lateral corridor clear (visible part only)
bool right_free              # right lateral corridor clear (visible part only)
bool obstacle_beside_left
bool obstacle_beside_right
bool rotation_clear          # no point inside rotation radius + rotation_margin_m
uint16 valid_points
uint16 self_filtered_points
