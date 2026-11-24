# UART protocol Pi ↔ STM32 (v1, 2026-11-11)

USART2 (header C11), 460800 8N1. Code: firmware `agv_f411/Core/Src/link.c`, Pi `pi/stm32_link.py`.

## Frame

```
0xAA 0x55 | len | cmd | payload[len] | crc_lo | crc_hi
```
- `len` = payload length (0..64).
- CRC-16/CCITT-FALSE (init 0xFFFF, poly 0x1021) over `len, cmd, payload`. Check value of "123456789" = 0x29B1.
- All multi-byte fields little-endian.

## Pi → STM32

| cmd | Name | Payload | Effect |
|---|---|---|---|
| 0x01 | SPEED | 4 × int16 | Wheel speed setpoints M1..M4, encoder counts/s, PID |
| 0x02 | PWM | 4 × int16 | Raw PWM −5000..5000 (testing, no PID) |
| 0x03 | STOP | — | Wheels coast, lift stops, lift driver off |
| 0x04 | LIFT_MOVE | int32 steps, uint16 steps/s | Relative move, + = up. Refused at limit switch |
| 0x05 | LIFT_HOME | uint16 steps/s | Move down to BOTTOM switch, position = 0 |
| 0x06 | PING | — | Keep-alive only |

**Safety:** any valid frame refreshes the link timer. No valid frame for 300 ms → STM32 stops wheels and lift
and sets flag TIMEOUT. `stm32_link.py` resends the last command at 20 Hz and sends STOP if the main program
has not updated it for 0.25 s.

## STM32 → Pi

`0x81 TELEM`, 48 bytes, every 20 ms while the Pi link is active (`link_telem_t` in `link.h`):

| Field | Type | Unit |
|---|---|---|
| ms | uint32 | STM32 uptime |
| enc[4] | int32 | total encoder counts (ENC_SIGN applied) |
| speed[4] | int16 | counts/s, filtered |
| acc[3], gyro[3] | int16 | MPU6050 raw (±2 g, ±500 °/s) |
| batt_mv | uint16 | battery mV |
| lift_pos | int32 | steps, + = up, 0 = bottom after homing |
| flags | uint8 | 0x01 TOP, 0x02 BOT, 0x04 lift busy, 0x08 homed, 0x10 IMU OK, 0x20 timeout, 0x40 watchdog reset |
| wheel_mode | uint8 | 0 off, 1 PWM, 2 speed |

## Text console

Text commands (CH340 terminal) still work on the same port: `m`, `v`, `l`, `h`, `s`, `t`, `i`, `?`.
Typing a text command leaves binary mode; text status lines are only printed when the Pi link is idle (> 1 s).
