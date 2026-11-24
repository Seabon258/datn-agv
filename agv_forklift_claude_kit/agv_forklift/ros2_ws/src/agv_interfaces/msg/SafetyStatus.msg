# Published by safety_supervisor on /safety_status (same rate as /cmd_vel).
uint8 REASON_NONE=0
uint8 REASON_OBSTACLE_STOP=1
uint8 REASON_OBSTACLE_SLOW=2
uint8 REASON_SIDE_BLOCKED=3
uint8 REASON_ROTATION_BLOCKED=4
uint8 REASON_REVERSE_BLOCKED=5
uint8 REASON_CMD_STALE=6
uint8 REASON_OBSTACLE_DATA_STALE=7
uint8 REASON_STM32_NOT_READY=8
uint8 REASON_ESTOP=9
uint8 REASON_SPEED_LIMITED=10

std_msgs/Header header
bool motion_allowed          # false = output forced to zero
uint8 limiting_reason        # most important active reason
uint32 active_reasons_mask   # bit i set = REASON i active
geometry_msgs/Twist cmd_in   # last /cmd_vel_nav
geometry_msgs/Twist cmd_out  # what was published on /cmd_vel
