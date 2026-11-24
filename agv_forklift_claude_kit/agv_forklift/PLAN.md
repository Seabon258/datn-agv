# PLAN.md — Kế hoạch triển khai

> Agent cập nhật file này sau mỗi lượt làm việc: tick `[x]`, ghi ngày + ghi chú ngắn ở cột cuối mỗi phase. Mỗi **GATE** cần chủ dự án xác nhận (ghi "✅ Hai xác nhận <ngày>") trước khi sang phase tiếp theo.
> Chi tiết test: `docs/08_test_plan.md`. Thông số phần cứng: `config/hardware_params.yaml`.

## Phase 0 — Kickoff (`/kickoff`)
- [ ] 0.1 Đọc toàn bộ docs, phân tích khả thi, liệt kê rủi ro → `docs/architecture.md` mục "Kickoff review"
- [ ] 0.2 Sơ đồ kiến trúc phần cứng (nguồn, E-stop, UART, I2C, USB) — Mermaid trong `docs/architecture.md`
- [ ] 0.3 Sơ đồ luồng dữ liệu node ROS 2 (đối chiếu docs/02 §2)
- [ ] 0.4 Review `agv_interfaces`; build thử `colcon build --packages-select agv_interfaces`
- [ ] 0.5 Review state machine docs/04; liệt kê điểm cần chốt
- [ ] 0.6 `/hw-params`: chủ dự án điền những gì đã biết
- [ ] 0.7 `python3 tools/protocol_reference.py --check` → OK
- [ ] 0.8 Chủ dự án chốt các quyết định PROPOSED trong `docs/07_decisions.md`
- **GATE 0:** ☐ chủ dự án xác nhận kiến trúc + hợp đồng

## Phase 1 — Module 1: STM32 firmware (`/module 1`)
- [ ] 1.1 `tools/gen_firmware_config.py`: sinh `firmware/core/inc/agv_config.h` từ `hardware_params.yaml` + `firmware_params.yaml`; thiếu tham số ⇒ sinh `#error` liệt kê tên
- [ ] 1.2 `core/`: crc16, protocol (encode + parser buffer), pid, mecanum, odometry, lift_sm, mode/watchdog, fault manager — C thuần, không HAL
- [ ] 1.3 `host_tests/`: Makefile + test cho 1.2, dùng `protocol_vectors.h` (T-FW-01..05)
- [ ] 1.4 Nhận file `.ioc` từ chủ dự án → đối chiếu ngân sách timer (docs/06 §2)
- [ ] 1.5 `port/`: encoder timers, PWM DRV8871, UART DMA + IDLE, step timer, limit switch, E-stop input, I2C IMU, tick 10 ms; `main.c` chỉ gọi `app_init()`/`app_loop()` trong USER CODE
- [ ] 1.6 Chế độ test firmware (biên dịch có điều kiện): MOTOR_TEST, ENCODER_TEST, PID_STEP, LIFT_TEST
- [ ] 1.7 Hướng dẫn bench T-FW-06..10 (chủ dự án chạy)
- **GATE 1:** ☐ host tests pass · ☐ bench tests đạt · ☐ chủ dự án xác nhận

## Phase 2 — Module 2: stm32_bridge (`/module 2`)
- [ ] 2.1 Package `agv_common` (hw_params loader fail-fast, qos, station_map loader, stale helpers)
- [ ] 2.2 `agv_bridge/protocol.py` (pass vectors) + `sim_stm32` (PTY)
- [ ] 2.3 Node `stm32_bridge`: `/cmd_vel` → CMD_VELOCITY 20 Hz liên tục; telemetry → `/odom`+TF, `/motor_state`, `/lift_state`, `/stm32_status`, `/imu/data`; services `/lift`, `/stm32/arm`, `/stm32/set_pid`; reconnect không treo executor
- [ ] 2.4 Test T-BR-01..04; hướng dẫn T-COMM-02
- **GATE 2:** ☐

## Phase 3 — Module 3: camera_node + line_follower (`/module 3`)
- [ ] 3.1 `camera_node` (V4L2/OpenCV, MJPG 320×240, khóa exposure/WB, reconnect)
- [ ] 3.2 `line_detector.py` thuần + test ảnh tổng hợp (T-VIS-01..03)
- [ ] 3.3 Node `line_follower` (+ debug window/trackbar, debug image, chế độ video_file)
- [ ] 3.4 Đo FPS trên Pi với video thật (T-VIS-05)
- **GATE 3:** ☐

## Phase 4 — Module 4: aruco_detector (`/module 4`)
- [ ] 4.1 `aruco_tracker.py` thuần (tương thích API cũ/mới) + test (T-VIS-06..08)
- [ ] 4.2 Node `aruco_detector` + station map
- [ ] 4.3 Script in marker ID 0..N (PDF/PNG) theo `marker_size_m`
- **GATE 4:** ☐

## Phase 5 — Module 5: obstacle_monitor + safety_supervisor (`/module 5`)
- [ ] 5.1 `scan_geometry.py` thuần (self-filter, zone, corridors, rotation) + test T-OBS-01..05
- [ ] 5.2 Node `obstacle_monitor`
- [ ] 5.3 `cmd_gate.py` thuần + test T-SAFE-01..07; node `safety_supervisor`
- [ ] 5.4 Script phân tích bag để đề xuất `blocked_angle_ranges_deg`
- **GATE 5:** ☐

## Phase 6 — Module 6: mission_controller (`/module 6`)
- [ ] 6.1 `state_machine.py`, `controllers.py`, `mission_model.py` thuần + test T-MIS-01, 04, 05, 06
- [ ] 6.2 Node `mission_controller` + service `/mission/command`
- [ ] 6.3 launch_testing kịch bản T-MIS-02, 03
- [ ] 6.4 Hướng dẫn floor test T-MIS-07..09
- **GATE 6:** ☐

## Phase 7 — Module 7: bringup (`/module 7`)
- [ ] 7.1 `agv.launch.py` (tham số `sim_mode`, `AGV_CONFIG_DIR`, lidar driver, static TF từ hardware params)
- [ ] 7.2 systemd unit mẫu (tự khởi động khi bật nguồn, tùy chọn)
- [ ] 7.3 T-BU-01, 02; hướng dẫn T-SAFE-05, 06
- [ ] 7.4 Chạy tích hợp trên sàn: kịch bản TRANSPORT đầy đủ
- **GATE 7:** ☐ — robot hoạt động v1

## Phase 8 — Module 8: Dashboard (`/module 8 8.1` … `/module 8 8.8`)
- [ ] 8.1 Thiết kế DB (`database/schema.sql`, index, FK)
- [ ] 8.2 ROS 2 monitoring (tiến trình riêng, thread executor, queue)
- [ ] 8.3 FastAPI backend (REST, auth, Swagger)
- [ ] 8.4 WebSocket `/ws/agv/status` (heartbeat, seq, stale)
- [ ] 8.5 Frontend React + TS (+ chế độ dữ liệu giả lập)
- [ ] 8.6 Tích hợp MySQL (batch, retry, bộ đệm SQLite)
- [ ] 8.7 API điều khiển qua `/mission/command` (quyền, xác nhận, audit)
- [ ] 8.8 Kiểm thử T-DASH-01..12
- **GATE 8:** ☐

## Nhật ký
| Ngày | Phase | Ghi chú |
|---|---|---|
| | | |
