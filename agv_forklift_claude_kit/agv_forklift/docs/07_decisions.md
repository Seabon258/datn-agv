# 07 — NHẬT KÝ QUYẾT ĐỊNH THIẾT KẾ (ADR log)

> Trạng thái: **ACCEPTED** = từ prompt gốc hoặc chủ dự án đã đồng ý · **PROPOSED** = đề xuất khi dựng bộ khung, cần chủ dự án xác nhận ở `/kickoff`.
> Agent thêm quyết định mới vào cuối file (D-0xx tiếp theo) khi chủ dự án chấp thuận. Không xóa quyết định cũ; nếu thay đổi thì ghi "SUPERSEDED by D-0yy".

| ID | Quyết định | Lý do | Trạng thái |
|---|---|---|---|
| D-001 | Không SLAM ở v1. Camera + ArUco để bám line/nhận trạm; encoder + IMU cho odometry; lidar chỉ để phát hiện/né vật cản | Prompt gốc §2 | ACCEPTED |
| D-002 | Một đường lệnh duy nhất: `mission_controller → /cmd_vel_nav → safety_supervisor → /cmd_vel → stm32_bridge`. Thêm node `safety_supervisor` | Đảm bảo "không module nào ghi đè STOP" bằng cấu trúc, không bằng kỷ luật; supervisor nhỏ, test được độc lập | PROPOSED |
| D-003 | PID bám line đặt ở `mission_controller`; `line_follower` chỉ là perception (publish `/line_state`) | Prompt §7.3 và §10.1 mâu thuẫn về nơi đặt PID; tách perception/điều khiển giúp test line bằng video không cần robot | PROPOSED |
| D-004 | `camera_node` sở hữu camera, publish `/camera/image_raw`; `line_follower` và `aruco_detector` subscribe | Một thiết bị V4L2 không mở ổn định từ 2 tiến trình. Logic xử lý nằm trong thư viện không phụ thuộc ROS ⇒ có thể gộp thành 1 tiến trình nếu đo thấy CPU cao | PROPOSED |
| D-005 | Lift dùng service `/lift` (trả lời ngay) + topic `/lift_state` (theo dõi bằng `request_id`), không dùng ROS action | Đúng prompt (service `/lift`), đơn giản hơn action; không block executor | PROPOSED |
| D-006 | Firmware có mode STOPPED/RUNNING/FAULT + lệnh `CMD_ARM` (magic). Thêm frame `CMD_ARM`, `CMD_SET_PID`, `TELEMETRY_STATUS`, `TELEMETRY_IMU`, `ACK` | Thực hiện "không tự chạy lại khi kết nối phục hồi" và "reset lỗi có kiểm soát"; ACK để biết lệnh lift bị từ chối vì sao; tuning PID không cần nạp lại firmware | PROPOSED |
| D-007 | Vùng mù phía sau lidar (gắn giữa mặt trước, giữa 2 tầng): PASS dùng "hành lang nhìn thấy đã trống + đi thêm `pass_clearance_distance_m` theo odometry". Bỏ trường `obstacle_behind` (không đo được), dashboard hiển thị `pass_progress_m`. Cấm lùi mặc định | Lidar không thể thấy vật sau mặt phẳng x của chính nó | PROPOSED |
| D-008 | Thêm state IDLE, TURN_CORNER, PAUSED, EMERGENCY_STOP | Tuyến thật có góc vuông 90° (video); dashboard cần Pause/Resume, IDLE, EMERGENCY_STOP | PROPOSED |
| D-009 | `hardware_params.yaml` là YAML thường (cho phép null) + loader fail-fast; header firmware sinh tự động từ cùng file | ROS 2 param không chấp nhận null; một nguồn sự thật cho cả Pi và STM32 | PROPOSED |
| D-010 | Quy ước dấu: `lateral_error` dương = line bên trái; `heading_error` dương = CCW; REP-103 | Thống nhất dấu giữa vision, mission, kinematics | PROPOSED |
| D-011 | Kích hoạt dừng trạm khi `center_y` của marker vượt `marker_trigger_y`, sau đó đi thêm `stop_offset_m` theo odometry (cho phép riêng từng trạm) | Vị trí dừng lặp lại tốt hơn so với kích hoạt ngay lúc vừa phát hiện (phụ thuộc cự ly phát hiện) | PROPOSED |
| D-012 | Mô hình nhiệm vụ TRANSPORT / GO_HOME / PATROL; action ở TRANSPORT do bước nhiệm vụ quyết định, `default_action` dùng cho PATROL | Hợp nhất "ID → hành động" (prompt §8.3) với "start/target station" (Module 8) | PROPOSED |
| D-013 | Dashboard: TypeScript (`.tsx`); frontend build tĩnh, FastAPI phục vụ trên Pi; MySQL mặc định chạy trên laptop/server LAN (host cấu hình được), Pi có bộ đệm SQLite khi mất DB; monitor là tiến trình riêng, không nằm trong node điều khiển | Prompt Module 8 yêu cầu TS nhưng cây thư mục ghi `.jsx`; Pi 4 4GB đang chạy ROS + vision nên tránh thêm MySQL + Node dev server | PROPOSED |
| D-014 | Pi ↔ STM32 qua UART phần cứng (GPIO14/15) thay vì USB-CDC | PCB đã từng bị nhiễu động cơ làm lỗi USB | PROPOSED — cần xác nhận đấu dây |
| D-015 | MPU6050 nối I2C của STM32 (odometry fusion ở STM32), gửi `TELEMETRY_IMU` | Đồng bộ thời gian với encoder; giảm tải Pi | PROPOSED — cần xác nhận đấu dây |
| D-016 | Firmware chia `core/` (C thuần, không HAL, test trên PC bằng gcc) và `port/` (glue HAL); agent không bao giờ tự flash | Kiểm chứng được logic (protocol, PID, kinematics, lift SM) mà không cần phần cứng; an toàn | PROPOSED |
| D-017 | Interlock trong firmware: lift đang chạy ⇒ vận tốc = 0; bánh đang quay ⇒ từ chối lệnh lift | Phòng thủ nhiều lớp (docs/05 H8) | PROPOSED |
| D-018 | `loaded` = sau LIFT_UP DONE cho tới LIFT_DOWN DONE (load cell để v2) | Cần biết trạng thái mang hàng để cấm né | PROPOSED |
| D-019 | Front zone = sector ±30° **hợp** hành lang thẳng rộng bằng thân xe; khoảng cách tính từ mép trước xe | Sector ±30° đặt ở lidar bỏ sót vật ở gần góc xe | PROPOSED |
| D-020 | Thứ tự module giữ như prompt (1→8), thêm Phase 0 (kickoff, review hợp đồng) | Prompt gốc §16 | ACCEPTED |
