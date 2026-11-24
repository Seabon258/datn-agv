# Published by any node on /system_events when something noteworthy happens.
# event_type values: catalog in docs/02_interfaces.md section 9 (do not invent new ones silently).
uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARNING=1
uint8 SEVERITY_ERROR=2
uint8 SEVERITY_CRITICAL=3

std_msgs/Header header
string source                # node name
string sensor_type           # CAMERA | LIDAR | IMU | ENCODER | STM32 | LIFT | MISSION | SAFETY | SYSTEM
string event_type
uint8 severity
string value                 # short value, e.g. "0.27"
string description
