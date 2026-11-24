# 03 — GIAO THỨC UART Pi ↔ STM32 (Protocol Contract, version 0x01)

> Đặc tả chạy được: `tools/protocol_reference.py`. Golden vectors: `tools/protocol_vectors.json` và `firmware/host_tests/protocol_vectors.h` (sinh tự động bằng `--write`).
> Firmware C và `agv_bridge/protocol.py` **phải** tạo ra/giải mã đúng từng byte như vectors. Mọi thay đổi: tăng `VERSION`, sửa tài liệu này + script, chạy `--write`, sửa cả 2 phía, ghi `07_decisions.md`.

## 1. Lớp vật lý

| Mục | Giá trị |
|---|---|
| Kết nối khuyến nghị | UART phần cứng: Pi GPIO14 (TXD) / GPIO15 (RXD) ↔ STM32 USARTx (`hardware_params.stm32.uart_instance`), mức 3.3 V, **chung GND** |
| Lý do | PCB hiện tại từng bị nhiễu động cơ làm lỗi USB. UART + CRC + resync chịu nhiễu tốt hơn USB-CDC. Phương án thay thế: USB-TTL (CP2102/CH340) đặt gần STM32 |
| Khung | 115200 baud, 8N1, không flow control (`comm.baudrate`) |
| Dây | Xoắn đôi TX/GND và RX/GND, tránh đi song song dây động cơ, ngắn nhất có thể |
| Cấu hình Pi (Ubuntu 22.04, kiểm tra lại trên máy thật) | `/boot/firmware/config.txt`: `dtoverlay=disable-bt` (PL011 ra GPIO14/15 → `/dev/ttyAMA0`); bỏ `console=serial0,115200` khỏi `/boot/firmware/cmdline.txt`; user thuộc nhóm `dialout` |
| Băng thông dự kiến | STM32→Pi ≈ 4.8 KB/s (~42% của 115200), Pi→STM32 ≈ 0.5 KB/s |

## 2. Cấu trúc frame

| Offset | Size | Trường | Giá trị / ghi chú |
|---|---|---|---|
| 0 | 1 | SOF1 | `0xAA` |
| 1 | 1 | SOF2 | `0x55` |
| 2 | 1 | VERSION | `0x01` |
| 3 | 1 | MSG_ID | bảng §4 |
| 4 | 1 | SEQ | uint8, mỗi phía tự tăng 1 cho mỗi frame gửi, quay vòng 255→0 |
| 5 | 1 | LEN | độ dài payload, 0..64, **phải bằng** kích thước cố định của MSG_ID |
| 6 | LEN | PAYLOAD | little-endian, không padding |
| 6+LEN | 2 | CRC | CRC-16/CCITT-FALSE trên byte [2, 6+LEN) (VERSION→hết PAYLOAD), gửi **byte thấp trước** |

- CRC-16/CCITT-FALSE: poly `0x1021`, init `0xFFFF`, không reflect, xorout `0x0000`. Kiểm tra: `"123456789"` → `0x29B1`.
- Kiểu dữ liệu: `u8/u16/u32/i16/i32` little-endian; `f32` = IEEE-754 binary32 little-endian.
- Frame dài nhất: 6 + 64 + 2 = 72 byte.

## 3. Quy tắc parser (C và Python giống hệt nhau)

1. Dữ liệu nhận được đưa vào **buffer** (STM32: ring buffer ≥ 256 B, nạp bằng UART DMA circular + IDLE-line interrupt; **parse trong main loop, không parse trong ISR**).
2. Tìm cặp `AA 55`. Chưa đủ 6 byte header thì chờ thêm.
3. VERSION sai → `version_errors++`, **bỏ đúng 1 byte** (SOF1) rồi quét lại.
4. LEN > 64 → `length_errors++`, bỏ 1 byte, quét lại.
5. Chưa đủ `6 + LEN + 2` byte thì chờ. Nếu frame dở dang chờ quá **5 ms** không có byte mới (`interbyte_timeout_s`) → bỏ 1 byte, quét lại.
6. CRC sai → `crc_errors++`, bỏ 1 byte, quét lại (để không mất frame hợp lệ nằm bên trong frame hỏng).
7. CRC đúng: tiêu thụ cả frame. MSG_ID lạ → `unknown_ids++`. LEN ≠ kích thước chuẩn → `bad_payload_len++` (nếu là lệnh: gửi `ACK` result `BAD_PAYLOAD`).
8. SEQ: bên nhận đếm khoảng nhảy để tính `frames_lost`. Không loại frame vì SEQ (trừ `CMD_LIFT` trùng `request_id`, xem §6).
9. Gửi (TX) luôn không chặn: STM32 dùng DMA + hàng đợi frame; Python ghi serial trong thread riêng / timer, không trong callback ROS.

Vector `PV_STREAM` (§8) kiểm tra đủ các trường hợp 3, 6 và resync.

## 4. Bảng message

| ID | Tên | Chiều | LEN | Tần số | ACK |
|---|---|---|---|---|---|
| `0x01` | CMD_VELOCITY | Pi→STM32 | 12 | 20 Hz liên tục (kể cả 0) | không |
| `0x02` | CMD_LIFT | Pi→STM32 | 3 | theo lệnh | có |
| `0x03` | CMD_STOP | Pi→STM32 | 2 | theo lệnh | có |
| `0x04` | HEARTBEAT | cả 2 chiều (mặc định chỉ Pi gửi) | 4 | 5 Hz | không |
| `0x05` | CMD_ARM | Pi→STM32 | 4 | theo lệnh | có |
| `0x06` | CMD_SET_PID | Pi→STM32 | 13 | khi tuning | có |
| `0x81` | TELEMETRY_ODOM | STM32→Pi | 28 | 50 Hz | — |
| `0x82` | TELEMETRY_MOTOR | STM32→Pi | 41 | 20 Hz | — |
| `0x83` | TELEMETRY_LIFT | STM32→Pi | 12 | 10 Hz + khi đổi trạng thái | — |
| `0x84` | TELEMETRY_ERROR | STM32→Pi | 8 | khi có lỗi mới | — |
| `0x85` | TELEMETRY_STATUS | STM32→Pi | 16 | 10 Hz (đóng vai trò heartbeat của STM32) | — |
| `0x86` | TELEMETRY_IMU | STM32→Pi | 24 | 50 Hz (chỉ khi IMU nối vào STM32) | — |
| `0x8F` | ACK | STM32→Pi | 4 | trả lời lệnh có ACK | — |

So với prompt gốc, có thêm `CMD_ARM`, `CMD_SET_PID`, `TELEMETRY_STATUS`, `TELEMETRY_IMU`, `ACK` (lý do: `07_decisions.md` D-006).

## 5. Payload

### 0x01 CMD_VELOCITY (12 B)
| Off | Kiểu | Trường | Đơn vị |
|---|---|---|---|
| 0 | f32 | vx | m/s (base_link) |
| 4 | f32 | vy | m/s |
| 8 | f32 | wz | rad/s |
STM32 tự clamp theo giới hạn firmware. Chỉ có hiệu lực ở mode RUNNING; **mỗi frame hợp lệ nạp lại watchdog chuyển động**.

### 0x02 CMD_LIFT (3 B)
| Off | Kiểu | Trường | Giá trị |
|---|---|---|---|
| 0 | u8 | action | 0 STOP, 1 HOME, 2 UP, 3 DOWN |
| 1 | u16 | request_id | bridge cấp, tăng dần, ≠ 0 |

### 0x03 CMD_STOP (2 B)
| Off | Kiểu | Trường | Giá trị |
|---|---|---|---|
| 0 | u8 | reason | 0 NORMAL, 1 OBSTACLE, 2 OPERATOR, 3 FAULT, 4 SHUTDOWN |
| 1 | u8 | disarm | 0 = vẫn RUNNING, mục tiêu vận tốc = 0; 1 = chuyển STOPPED (cần `CMD_ARM` để chạy lại) |
Đồng thời dừng lift nếu đang chạy.

### 0x04 HEARTBEAT (4 B)
| Off | Kiểu | Trường | Ghi chú |
|---|---|---|---|
| 0 | u32 | uptime_ms | thời gian chạy của bên gửi |

### 0x05 CMD_ARM (4 B)
| Off | Kiểu | Trường | Ghi chú |
|---|---|---|---|
| 0 | u32 | magic | phải = `0x214D5241` (byte trên dây `41 52 4D 21` = "ARM!") |

### 0x06 CMD_SET_PID (13 B)
| Off | Kiểu | Trường |
|---|---|---|
| 0 | u8 | wheel (0 FL, 1 FR, 2 RL, 3 RR, 255 = tất cả) |
| 1 | f32 | kp |
| 5 | f32 | ki |
| 9 | f32 | kd |

### 0x81 TELEMETRY_ODOM (28 B)
| Off | Kiểu | Trường | Ghi chú |
|---|---|---|---|
| 0 | u32 | t_ms | thời gian STM32 |
| 4 | f32 | x | m, frame odom (gốc = lúc STM32 khởi động) |
| 8 | f32 | y | m |
| 12 | f32 | theta | rad, chuẩn hóa (−π, π] |
| 16 | f32 | vx | m/s, base_link |
| 20 | f32 | vy | m/s |
| 24 | f32 | wz | rad/s |

### 0x82 TELEMETRY_MOTOR (41 B)
| Off | Kiểu | Trường |
|---|---|---|
| 0 | f32[4] | rpm_target [FL, FR, RL, RR] (rpm tại bánh) |
| 16 | f32[4] | rpm_actual |
| 32 | i16[4] | pwm (‰, −1000..1000) |
| 40 | u8 | encoder_fault_mask (bit0 FL … bit3 RR) — encoder mất tín hiệu **hoặc** motor kẹt |

### 0x83 TELEMETRY_LIFT (12 B)
| Off | Kiểu | Trường | Giá trị |
|---|---|---|---|
| 0 | u8 | state | 0 IDLE, 1 HOMING, 2 MOVING_UP, 3 MOVING_DOWN, 4 DONE, 5 ERROR |
| 1 | u8 | flags | bit0 homed, bit1 limit_bottom, bit2 limit_top, bit3 moving, bit4 at_up, bit5 at_down |
| 2 | u8 | last_action | như `CMD_LIFT.action` |
| 3 | u8 | reserved | 0 |
| 4 | u16 | request_id | của lệnh đang/đã thực hiện |
| 6 | i32 | position_steps | 0 = vị trí home |
| 10 | u16 | error_code | §7.2 |

### 0x84 TELEMETRY_ERROR (8 B)
| Off | Kiểu | Trường |
|---|---|---|
| 0 | u16 | error_code (§7.2) |
| 2 | u8 | severity (0 INFO, 1 WARNING, 2 ERROR, 3 CRITICAL) |
| 3 | u8 | component (1 SYSTEM, 2 DRIVE, 3 LIFT, 4 IMU, 5 COMM, 6 POWER) |
| 4 | u32 | t_ms |

### 0x85 TELEMETRY_STATUS (16 B)
| Off | Kiểu | Trường | Ghi chú |
|---|---|---|---|
| 0 | u32 | t_ms | |
| 4 | u32 | fault_flags | §7.1 |
| 8 | u8 | mode | 0 STOPPED, 1 RUNNING, 2 FAULT |
| 9 | u8 | estop | 1 = nút E-stop đang nhấn |
| 10 | u16 | vbat_mv | 0 = không đo |
| 12 | u16 | rx_crc_errors | bão hòa ở 65535 |
| 14 | u16 | rx_frames_dropped | bão hòa |

### 0x86 TELEMETRY_IMU (24 B)
| Off | Kiểu | Trường |
|---|---|---|
| 0, 4, 8 | f32 ×3 | ax, ay, az (m/s², imu_link) |
| 12, 16, 20 | f32 ×3 | gx, gy, gz (rad/s) |

### 0x8F ACK (4 B)
| Off | Kiểu | Trường | Giá trị |
|---|---|---|---|
| 0 | u8 | acked_msg_id | |
| 1 | u8 | acked_seq | SEQ của frame lệnh |
| 2 | u8 | result | 0 OK, 1 REJECTED, 2 BAD_PAYLOAD, 3 BUSY |
| 3 | u8 | reason | 0 NONE, 1 NOT_ARMED, 2 FAULT_ACTIVE, 3 ESTOP_ACTIVE, 4 LIFT_NOT_HOMED, 5 LIFT_BUSY, 6 DRIVE_MOVING, 7 BAD_MAGIC, 8 INVALID_ARG |

## 6. Mode, watchdog và arming (firmware)

| Từ | Sự kiện | Sang | Hành động |
|---|---|---|---|
| (boot) | — | STOPPED | PWM = 0, lift IDLE chưa homed |
| STOPPED | `CMD_ARM` magic đúng, không E-stop, không lỗi đang hoạt động | RUNNING | xóa lỗi latched, ACK OK, nạp watchdog |
| STOPPED/FAULT | `CMD_ARM` khi E-stop nhấn / magic sai | giữ nguyên | ACK REJECTED (ESTOP_ACTIVE / BAD_MAGIC) |
| FAULT | `CMD_ARM` hợp lệ và điều kiện lỗi đã hết | RUNNING | như trên |
| RUNNING | không có `CMD_VELOCITY` hợp lệ trong **300 ms** | FAULT | dừng 4 động cơ (brake rồi coast), dừng lift (lift → ERROR `LIFT_ABORTED`), latch `WATCHDOG`, gửi `TELEMETRY_ERROR` |
| bất kỳ | E-stop nhấn | FAULT | như trên, latch `ESTOP` |
| RUNNING | lỗi bánh / lift timeout / limit fault / undervoltage | FAULT | như trên |
| RUNNING | `CMD_STOP disarm=1` | STOPPED | dừng tất cả |

- **Không bao giờ tự chạy lại sau khi kết nối phục hồi**: thoát FAULT chỉ bằng `CMD_ARM`, mà `CMD_ARM` chỉ được gửi khi `mission_controller` gọi `/stm32/arm` (START hoặc RESET_ERROR do người vận hành).
- `CMD_VELOCITY` khi STOPPED/FAULT: bỏ qua (mục tiêu = 0), không ACK.
- `CMD_LIFT` chỉ chấp nhận ở RUNNING. UP/DOWN bị từ chối nếu chưa homed (`LIFT_NOT_HOMED`), đang chạy lệnh khác (`LIFT_BUSY`), hoặc bánh còn quay (`DRIVE_MOVING`). Lệnh trùng `request_id` đang thực hiện → ACK OK, không chạy lại. `STOP` luôn được chấp nhận.
- **Interlock**: khi lift đang chạy, firmware ép mục tiêu vận tốc = 0 và bật bit `INFO_DRIVE_INHIBITED_BY_LIFT`.


## 7. Bảng mã

### 7.1 fault_flags (u32)
| Bit | Tên | Loại | Ép FAULT? |
|---|---|---|---|
| 0 | WATCHDOG_TIMEOUT | latched | có |
| 1 | ESTOP_ACTIVE | trực tiếp | có |
| 2–5 | WHEEL_FAULT_FL / FR / RL / RR (encoder mất hoặc motor kẹt: PWM cao mà rpm ≈ 0 quá T ms) | latched | có |
| 6–9 | dự phòng | | |
| 10 | LIFT_TIMEOUT | latched | có |
| 11 | LIFT_LIMIT_FAULT | latched | có |
| 12 | IMU_FAULT | trực tiếp | không (cảnh báo) |
| 13 | UART_CRC_RATE_HIGH | trực tiếp | không (cảnh báo) |
| 14 | UNDERVOLTAGE | trực tiếp | có (nếu có đo điện áp) |
| 16 | INFO_LIFT_NOT_HOMED | thông tin | không |
| 17 | INFO_DRIVE_INHIBITED_BY_LIFT | thông tin | không |

### 7.2 error_code (u16)
| Mã | Tên | Component |
|---|---|---|
| `0x0000` | NONE | — |
| `0x0101` | WATCHDOG_TIMEOUT | COMM |
| `0x0102` | ESTOP_PRESSED | SYSTEM |
| `0x0103` | BAD_ARM_MAGIC | COMM |
| `0x0201`–`0x0204` | WHEEL_FAULT_FL / FR / RL / RR | DRIVE |
| `0x0301` | LIFT_TIMEOUT | LIFT |
| `0x0302` | LIFT_LIMIT_NOT_FOUND (homing vượt hành trình mà không chạm công tắc) | LIFT |
| `0x0303` | LIFT_LIMIT_UNEXPECTED (công tắc dưới kích hoạt khi không mong đợi) | LIFT |
| `0x0304` | LIFT_NOT_HOMED | LIFT |
| `0x0305` | LIFT_SOFT_LIMIT | LIFT |
| `0x0306` | LIFT_ABORTED (bị dừng bởi STOP/watchdog/E-stop) | LIFT |
| `0x0401` | IMU_FAULT | IMU |
| `0x0501` | UART_CRC_RATE_HIGH | COMM |
| `0x0601` | UNDERVOLTAGE | POWER |

## 8. Golden vectors (sinh bởi `tools/protocol_reference.py`)

CRC check: `"123456789"` → `0x29B1`.

| Vector | Bytes (hex) |
|---|---|
| `CMD_VELOCITY_FWD` (seq 0, vx 0.25, vy 0, wz 0.5) | `aa 55 01 01 00 0c 00 00 80 3e 00 00 00 00 00 00 00 3f d8 c6` |
| `CMD_LIFT_UP` (seq 1, UP, req 7) | `aa 55 01 02 01 03 02 07 00 95 04` |
| `CMD_ARM` (seq 2) | `aa 55 01 05 02 04 41 52 4d 21 59 6a` |
| `CMD_STOP_OPERATOR` (seq 3) | `aa 55 01 03 03 02 02 00 bc 36` |
| `HEARTBEAT_PI` (seq 4, 60000 ms) | `aa 55 01 04 04 04 60 ea 00 00 ae 06` |
| `CMD_SET_PID_ALL` (seq 5, 1.5/0.25/0) | `aa 55 01 06 05 0d ff 00 00 c0 3f 00 00 80 3e 00 00 00 00 b4 b7` |
| `TELEMETRY_ODOM` (seq 10) | `aa 55 01 81 0a 1c e8 03 00 00 00 00 80 3f 00 00 00 bf 00 00 80 3e 00 00 00 3e 00 00 00 00 00 00 00 bf e2 5f` |
| `TELEMETRY_MOTOR` (seq 11) | `aa 55 01 82 0b 29 00 00 70 42 00 00 70 42 00 00 70 42 00 00 70 42 00 00 6e 42 00 00 71 42 00 00 6b 42 00 00 74 42 a4 01 61 fe ae 01 66 fe 00 87 1d` |
| `TELEMETRY_LIFT_DONE` (seq 12) | `aa 55 01 83 0c 0c 04 11 02 00 07 00 80 3e 00 00 00 00 74 e3` |
| `TELEMETRY_ERROR_WATCHDOG` (seq 13) | `aa 55 01 84 0d 08 01 01 03 05 40 e2 01 00 5c 7f` |
| `TELEMETRY_STATUS_FAULT` (seq 14) | `aa 55 01 85 0e 10 40 e2 01 00 03 00 00 00 02 01 34 30 05 00 01 00 38 e9` |
| `TELEMETRY_IMU` (seq 15) | `aa 55 01 86 0f 18 00 00 00 00 00 00 00 00 00 00 1c 41 00 00 00 00 00 00 00 00 00 00 00 3f ec bd` |
| `ACK_LIFT_OK` (seq 16) | `aa 55 01 8f 10 04 02 01 00 00 27 1e` |

**Stream test** (rác + frame hỏng CRC + header cụt + frame tốt + rác + frame tốt):
`00 13 aa aa 55 01 01 00 0c 00 00 7f 3e 00 00 00 00 00 00 00 3f d8 c6 aa 55 aa 55 01 02 01 03 02 07 00 95 04 55 aa aa 55 01 05 02 04 41 52 4d 21 59 6a`
→ kết quả bắt buộc: giải mã đúng **2 frame** `[CMD_LIFT, CMD_ARM]`, `crc_errors = 1`, `version_errors = 1`, cả khi nạp một lần lẫn nạp từng byte.

## 9. Cách kiểm thử
- Python: `python3 tools/protocol_reference.py --check`; `agv_bridge` có pytest đọc `tools/protocol_vectors.json`.
- C (trên PC): `make -C firmware/host_tests` biên dịch core protocol bằng gcc và so với `protocol_vectors.h`.
- Trên phần cứng: bridge hiển thị bộ đếm `rx_crc_errors`, `rx_frames_lost`; chạy motor tải nặng 10 phút, tỉ lệ lỗi CRC phải < 0.1% và không có watchdog giả.
