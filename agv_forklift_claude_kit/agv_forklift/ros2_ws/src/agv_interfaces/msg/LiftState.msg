# Published by stm32_bridge on /lift_state (from TELEMETRY_LIFT, 10 Hz + on change).
uint8 STATE_IDLE=0
uint8 STATE_HOMING=1
uint8 STATE_MOVING_UP=2
uint8 STATE_MOVING_DOWN=3
uint8 STATE_DONE=4
uint8 STATE_ERROR=5
uint8 ACTION_STOP=0
uint8 ACTION_HOME=1
uint8 ACTION_UP=2
uint8 ACTION_DOWN=3

std_msgs/Header header
uint8 state
bool homed
bool limit_bottom
bool moving
uint8 last_action
uint16 request_id            # id of the command this state refers to
int32 position_steps
float32 position_mm          # NaN if lift hardware params are unknown
bool at_up_position
bool at_down_position
uint16 error_code            # docs/03_uart_protocol.md error code table
