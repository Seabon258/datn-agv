# Published by stm32_bridge on /stm32_status (10 Hz).
uint8 MODE_STOPPED=0
uint8 MODE_RUNNING=1
uint8 MODE_FAULT=2

std_msgs/Header header
bool link_ok                 # false if no valid frame for link_timeout_s
uint8 mode
bool estop_active
uint32 fault_flags           # bit table: docs/03_uart_protocol.md
uint16 last_error_code
uint16 battery_mv            # 0 = not measured
uint32 stm32_uptime_ms
uint32 rx_crc_errors         # Pi-side counter
uint32 rx_frames_lost        # Pi-side, from SEQ gaps
uint32 stm32_rx_crc_errors   # reported by the STM32
