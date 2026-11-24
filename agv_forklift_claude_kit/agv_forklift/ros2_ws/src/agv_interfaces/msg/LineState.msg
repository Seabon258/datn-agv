# Published by line_follower on /line_state. PERCEPTION ONLY: no control output here.
# Sign conventions follow REP-103 (x forward, y left, z up, CCW positive). See docs/02_interfaces.md.
uint8 STATUS_TRACKING=0
uint8 STATUS_LOST=1          # not seen for longer than lost_timeout_s
uint8 STATUS_RECOVERING=2    # not seen, but for less than lost_timeout_s (broken line / gap)

std_msgs/Header header       # stamp = image capture time, frame_id = camera_link
bool found                   # line found in THIS frame
uint8 status
float32 lateral_error        # [-1, 1]; + = line is LEFT of image centerline
float32 heading_error        # rad, approx. (image space unless a homography is configured); + = line turns CCW (left)
float32 confidence           # [0, 1]
uint8 bands_found            # number of ROI bands with a valid line segment
bool corner_detected         # 90-degree corner confirmed over corner_confirm_frames
int8 corner_direction        # +1 = corner turns left, -1 = turns right, 0 = none
float32 time_since_seen_s
float32 fps
