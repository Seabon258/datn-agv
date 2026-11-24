# 08 — KẾ HOẠCH KIỂM THỬ & TIÊU CHÍ HOÀN THÀNH

> Ba mức: **[auto]** agent tự chạy được (pytest / host test / launch_testing với dữ liệu giả); **[bench]** chủ dự án chạy trên bàn, bánh nhấc khỏi sàn; **[floor]** chạy trên sàn thật. Agent viết test [auto] và viết hướng dẫn từng bước cho [bench]/[floor], nhưng **không tự chạy** test làm robot di chuyển.
> Dữ liệu test thật (video, rosbag) để ở `tests/data/` (xem README ở đó).

## Phase 0 — Kickoff
- [auto] `python3 tools/protocol_reference.py --check` → OK.
- [auto] `colcon build --packages-select agv_interfaces` thành công (máy có ROS 2 Humble).

## Module 1 — STM32 firmware
| ID | Mức | Kiểm tra | Đạt khi |
|---|---|---|---|
| T-FW-01 | auto | Host test protocol (encode/decode, CRC, stream vector) | Khớp 100% `protocol_vectors.h` |
| T-FW-02 | auto | Host test kinematics: inverse rồi forward | Sai số < 1e-5 cho 100 vector ngẫu nhiên |
| T-FW-03 | auto | Host test PID: anti-windup, clamp, reset | Output không vượt giới hạn; tích phân không tràn khi bão hòa |
| T-FW-04 | auto | Host test lift SM: mọi transition docs/04 §B, timeout, soft limit, lệnh khi BUSY | Đúng bảng |
| T-FW-05 | auto | Host test mode/watchdog: 300 ms không lệnh ⇒ FAULT; ARM sai magic bị từ chối; không tự RUNNING lại | Đúng docs/03 §6 |
| T-FW-06 | bench | Encoder test: quay tay 10 vòng | Đếm khớp ±1%, đúng dấu |
| T-FW-07 | bench | PID step 0 → 60 rpm từng bánh | Không dao động mạnh, sai số xác lập < 5% |
| T-FW-08 | bench | Rút cáp UART khi đang chạy | Bánh dừng ≤ 300 ms, mode FAULT, không tự chạy khi cắm lại |
| T-FW-09 | bench | Homing lift, UP, DOWN, STOP giữa chừng | Đúng trạng thái; STOP ⇒ ERROR LIFT_ABORTED, `homed=false` |
| T-FW-10 | bench | E-stop khi motor đang quay | Động lực bị cắt, STATUS báo estop + FAULT |
| T-FW-11 | bench | Interlock: gửi vận tốc khi lift đang chạy; gửi lệnh lift khi bánh đang quay | Bánh không quay (bit `INFO_DRIVE_INHIBITED_BY_LIFT`); lệnh lift bị ACK REJECTED `DRIVE_MOVING` |
| T-FW-12 | bench | Tự hãm vít me: nâng tải tối đa, ngắt nguồn stepper (E-stop) | Càng không trôi xuống; nếu trôi ⇒ dừng dự án phần lift, xử lý H9 |

## Module 2 — stm32_bridge
| ID | Mức | Kiểm tra | Đạt khi |
|---|---|---|---|
| T-BR-01 | auto | pytest protocol.py với `tools/protocol_vectors.json` | Khớp 100% |
| T-BR-02 | auto | Chạy bridge với `sim_stm32` (PTY ảo): `/cmd_vel` → frame đúng; telemetry → `/odom`, `/lift_state`, `/stm32_status` | Đúng giá trị, đúng frame/TF |
| T-BR-03 | auto | Giả lập rút serial | `link_ok=false` ≤ 0.5 s, tự reconnect, executor không treo |
| T-BR-04 | auto | `/lift` khi sim từ chối | `accepted=false` + message lý do |
| T-COMM-02 | bench | Motor chạy tải 10 phút | Lỗi CRC < 0.1%, không có watchdog giả |

## Module 3 — camera_node + line_follower
| ID | Mức | Kiểm tra | Đạt khi |
|---|---|---|---|
| T-VIS-01 | auto | pytest `line_detector` trên ảnh tổng hợp (line thẳng, lệch trái/phải, nghiêng) | Dấu `lateral_error`/`heading_error` đúng quy ước docs/02 |
| T-VIS-02 | auto | Ảnh tổng hợp có góc vuông trái/phải | `corner_detected`, đúng `corner_direction` |
| T-VIS-03 | auto | Chuỗi frame có đoạn line mất 0.3 s và 0.8 s | RECOVERING rồi TRACKING; LOST sau 0.5 s |
| T-VIS-04 | auto | Không có frame mới | Mission nhận stale ⇒ (test ở M6) |
| T-VIS-05 | floor | Video thật trong `tests/data/` | ≥ 20 FPS trên Pi 4 ở 320×240; tracking ổn định |

## Module 4 — aruco_detector
| ID | Mức | Kiểm tra | Đạt khi |
|---|---|---|---|
| T-VIS-06 | auto | Ảnh tổng hợp có marker (sinh bằng OpenCV) | Đúng ID; confirmed sau `confirm_frames`; reset khi vắng |
| T-VIS-07 | auto | Station map: ID lạ | `station_name=""`, không crash |
| T-VIS-08 | auto | Chạy được với cả API ArUco cũ và mới (mock hoặc kiểm phiên bản) | Không lỗi import |

## Module 5 — obstacle_monitor + safety_supervisor
| ID | Mức | Kiểm tra | Đạt khi |
|---|---|---|---|
| T-OBS-01 | auto | LaserScan giả: vật ở 0.25 / 0.5 / 1.0 m phía trước | STOP / SLOW / CLEAR (sau persistence) |
| T-OBS-02 | auto | Vật ở gần góc trước xe (ngoài sector ±30°, trong hành lang) | Vẫn STOP (D-019) |
| T-OBS-03 | auto | Điểm tự va (trong footprint / cung bị che) | Bị lọc, `self_filtered_points` đếm đúng |
| T-OBS-04 | auto | `left_free/right_free`, `rotation_clear` với các bố trí vật | Đúng định nghĩa docs/02 §5 |
| T-OBS-05 | auto | Scan cũ | `sensor_ok=false` |
| T-SAFE-01 | auto | zone STOP chặn vx > 0; zone SLOW giới hạn `slow_vx` | Lệnh ra + reason đúng |
| T-SAFE-02 | auto | `left_free/right_free = false` chặn vy tương ứng | Lệnh ra + reason đúng |
| T-SAFE-03 | auto | `rotation_clear = false` chặn wz | Lệnh ra + reason đúng |
| T-SAFE-04 | auto | vx < 0 với `allow_reverse=false` / `true` | 0 / bị clamp `max_reverse_vx` |
| T-SAFE-07 | auto | `loaded = true` và `/mission_status` cũ | Nhân `loaded_speed_scale` trong cả hai trường hợp |
| T-SAFE-08 | auto | Từng input cũ: `/cmd_vel_nav`, `/obstacle_state`, `/stm32_status`; `mode ≠ RUNNING`; estop | Xuất 0, reason đúng, vẫn publish đều 20 Hz |
| T-SAFE-09 | auto | Giới hạn gia tốc khi tăng tốc; giảm tốc/dừng | Tăng bị ramp; giảm tức thì |
| T-SAFE-10 | floor | Hộp đặt trước xe khi đang chạy 0.15 m/s | Dừng trước hộp ≥ 5 cm |

## Module 6 — mission_controller
| ID | Mức | Kiểm tra | Đạt khi |
|---|---|---|---|
| T-MIS-01 | auto | pytest `state_machine` thuần: mọi transition bảng docs/04 §D | 100% transition có test |
| T-MIS-02 | auto | launch_testing: publish topic giả (line, marker, obstacle, odom, lift, status) cho kịch bản TRANSPORT | Trình tự state đúng; lệnh `/lift` đúng |
| T-MIS-03 | auto | Kịch bản né: BLOCKED → DETOUR → PASS → RETURN → REACQUIRE → FOLLOW_LINE (odom giả) | Đúng, offset về ≈ 0 |
| T-MIS-04 | auto | Mang hàng + vật cản | Không DETOUR; ERROR sau `blocked_timeout_s` |
| T-MIS-05 | auto | Ma trận lệnh docs/04 §D.3 | Từ chối/chấp nhận đúng + event |
| T-MIS-06 | auto | Từng loại stale ⇒ ERROR đúng mã | Đúng D.6 |
| T-MIS-07 | floor | Line thẳng, góc 90°, line gãy 5 cm và 10 cm | Không ra khỏi line; qua góc; qua chỗ gãy |
| T-MIS-08 | floor | Dừng trạm 10 lần | Sai lệch vị trí dừng ≤ ±2 cm (ghi lại để chỉnh `stop_offset_m`) |
| T-MIS-09 | floor | Vật cản tĩnh trên line, hai bên trống | Né và quay lại đúng line |

## Module 7 — bringup
| ID | Mức | Kiểm tra | Đạt khi |
|---|---|---|---|
| T-BU-01 | auto | `ros2 launch agv_bringup agv.launch.py sim_mode:=true` | Mọi node lên, không lỗi param |
| T-BU-02 | auto | Thiếu hardware param khi `sim_mode:=false` | Từ chối khởi động, liệt kê đủ tham số thiếu |
| T-SAFE-05 | bench | `kill -9` mission_controller khi đang chạy | supervisor xuất 0 trong ≤ 0.3 s |
| T-SAFE-06 | bench | `kill -9` toàn bộ ROS khi đang chạy | STM32 dừng ≤ 300 ms, FAULT |

## Module 8 — Dashboard (theo prompt Module 8 §14, đánh số T-DASH-01..12)
- T-DASH-01..09 [auto] với chế độ giả lập dữ liệu ROS 2.
- T-DASH-10 [auto] tắt MySQL khi robot (hoặc sim) đang chạy ⇒ điều khiển không bị ảnh hưởng, queue có giới hạn, tự reconnect.
- T-DASH-11 [auto] tắt backend ⇒ các node điều khiển không bị ảnh hưởng.
- T-DASH-12 [auto] user không có quyền gửi lệnh ⇒ 403, có ghi `command_logs`.

## Tiêu chí hoàn thành chung cho mỗi module
1. Mọi test [auto] pass; có lệnh chạy lại một dòng.
2. `interface-guardian` và `safety-reviewer` không còn lỗi mức HIGH.
3. Không có số cứng (ngưỡng, thời gian, PID, HSV) trong source.
4. PLAN.md cập nhật; hướng dẫn [bench]/[floor] đã viết; chủ dự án xác nhận.
