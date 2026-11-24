# Published by mission_controller on /mission_status (status_rate_hz + on every transition).
uint8 STATE_IDLE=0
uint8 STATE_FOLLOW_LINE=1
uint8 STATE_TURN_CORNER=2
uint8 STATE_APPROACH_MARKER=3
uint8 STATE_LIFT_ACTION=4
uint8 STATE_BLOCKED=5
uint8 STATE_DETOUR=6
uint8 STATE_PASS=7
uint8 STATE_RETURN=8
uint8 STATE_REACQUIRE=9
uint8 STATE_PAUSED=10
uint8 STATE_ERROR=11
uint8 STATE_EMERGENCY_STOP=12
uint8 MISSION_NONE=0
uint8 MISSION_TRANSPORT=1
uint8 MISSION_PATROL=2
uint8 MISSION_GO_HOME=3

std_msgs/Header header
uint8 state
string state_name
uint8 previous_state
float32 state_elapsed_s
string mission_id            # "" if no mission
uint8 mission_type
uint8 mission_step           # index into the mission step list
string current_station       # last station reached / confirmed
string target_station
bool loaded                  # true after LIFT_UP done, false after LIFT_DOWN done
uint32 completed_missions
float32 detour_offset_m      # signed lateral offset from the line (DETOUR/PASS/RETURN), 0 otherwise
float32 pass_progress_m
string last_event
uint16 error_code            # mission error codes: docs/04_state_machines.md
string error_message
