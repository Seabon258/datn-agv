# Published by aruco_detector on /marker (one message per processed frame).
std_msgs/Header header       # stamp = image capture time
bool detected                # at least one marker visible in this frame
bool confirmed               # same id seen in >= confirm_frames consecutive processed frames
int32 id                     # -1 = none
uint8 confirm_count
uint8 visible_count          # number of markers visible in this frame
float32 center_x             # normalized [0, 1], 0 = left image edge
float32 center_y             # normalized [0, 1], 0 = top edge (larger = closer to the robot)
float32 image_angle          # rad, marker rotation in the image
string station_name          # from config/station_map.yaml, "" if id unknown
string default_action        # NONE | CHECKPOINT | HOME | LIFT_UP | LIFT_DOWN
