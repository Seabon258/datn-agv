# 05 — AN TOÀN (ưu tiên cao nhất)

> Tài liệu này **thắng mọi tài liệu khác** khi mâu thuẫn. Không được làm yếu bất kỳ cơ chế nào ở đây để "cho chạy được". Mọi thay đổi liên quan an toàn phải được liệt kê riêng trong bản tóm tắt module và có test đi kèm.

## 1. Các lớp bảo vệ (defense in depth)

| Lớp | Thành phần | Hoạt động khi lớp trên chết? |
|---|---|---|
| L0 | **E-stop phần cứng** (nút NC nối tiếp nguồn động lực DRV8871 + driver stepper) + tín hiệu báo về STM32 | Luôn, không phụ thuộc phần mềm |
| L1 | Firmware STM32: watchdog 300 ms, interlock lift↔drive, giới hạn tốc độ/gia tốc, soft limit lift, phát hiện lỗi bánh | Có, kể cả khi Pi/ROS chết |
| L2 | `safety_supervisor`: bộ lọc lệnh cuối cùng trước bridge, STOP luôn thắng | Có, kể cả khi mission_controller lỗi |
| L3 | `mission_controller`: logic nhiệm vụ, chờ, né, timeout mọi state | — |
| L4 | Dashboard: chỉ hiển thị + gửi yêu cầu qua `/mission/command`; không có quyền bỏ qua lớp dưới | — |

**Lệnh STOP từ ROS 2 không bao giờ được coi là E-stop.** E-stop là phần cứng.

## 2. Bảng mối nguy

| # | Mối nguy | Nguyên nhân | Biện pháp | Kiểm chứng (docs/08) |
|---|---|---|---|---|
| H1 | Đâm vật phía trước | Vật xuất hiện bất ngờ | Zone STOP/SLOW (hành lang + sector), supervisor chặn vx > 0, giới hạn tốc độ | T-SAFE-01, T-SAFE-10 |
| H2 | Đâm khi strafe | Bên cạnh có vật | `left_free/right_free` (hành lang đủ rộng cho cả thân xe + hàng), supervisor chặn vy | T-SAFE-02 |
| H3 | Quét góc xe khi quay | Quay tại chỗ gần vật | `rotation_clear`, supervisor chặn wz | T-SAFE-03 |
| H4 | Lùi vào vật | Lidar không thấy phía sau | Cấm lùi mặc định (`allow_reverse: false`) | T-SAFE-04 |
| H5 | Vật thấp dưới mặt quét lidar | Lidar 2D chỉ quét một mặt phẳng | Đo `lidar.mount_z_m`; tốc độ thấp; **khuyến nghị** thêm ToF/bumper gắn thấp (v2) — ghi rõ là giới hạn đã biết | ghi chú |
| H6 | Pi/ROS treo hoặc node chết | Phần mềm | Supervisor xuất 0 khi dữ liệu cũ; STM32 watchdog 300 ms dừng hẳn và latch FAULT; không tự chạy lại | T-SAFE-05, T-SAFE-06, T-SAFE-08, T-FW-08 |
| H7 | Nhiễu UART | Nhiễu động cơ (đã từng xảy ra) | CRC16, resync, watchdog, dây xoắn, chung GND, UART phần cứng; giám sát tỉ lệ lỗi | T-COMM-02 |
| H8 | Kẹp/đè do lift | Lift chạy khi xe chạy hoặc ngược lại | Interlock firmware (lift chạy ⇒ vận tốc = 0; xe chạy ⇒ từ chối lệnh lift), soft limit, homing bắt buộc | T-FW-11 |
| H9 | **Hàng rơi khi mất điện / E-stop** | Vít me bước lớn (vd T8×8) có thể tự trôi khi stepper mất điện | Kiểm tra tự hãm (`screw_self_locking_verified`); nếu không tự hãm: dùng vít bước nhỏ (T8×2) hoặc thêm phanh | T-FW-12 |
| H10 | Lật khi mang hàng | Trọng tâm cao + gia tốc | Giới hạn gia tốc, `loaded_speed_scale`, không né khi mang hàng | T-SAFE-07 |
| H11 | Camera mất / nhận sai line | Lỏng cáp, ánh sáng | `/line_state` cũ ⇒ ERROR; khóa exposure/WB; confidence | T-VIS-04 |
| H12 | Đọc nhầm marker | Nhiễu ảnh | Xác nhận N frame, khóa marker, chỉ hành động với marker liên quan | T-VIS-06 |
| H13 | Lệnh trái phép từ dashboard | Mạng LAN | Đăng nhập, phân quyền, xác nhận, audit log; mission_controller tự kiểm điều kiện an toàn | T-DASH-12 |
| H14 | DB/dashboard lỗi làm treo điều khiển | Thiết kế sai | Dashboard là tiến trình riêng; không I/O chặn trong node điều khiển | T-DASH-10, T-DASH-11 |
| H15 | Pin yếu | Pin 12 V | Nếu có đo điện áp: cảnh báo + UNDERVOLTAGE; nếu không: ghi rõ là chưa giám sát | ghi chú |
| H16 | Nguồn logic bị cắt cùng E-stop | Đấu nguồn sai | Pi/STM32 dùng buck riêng, E-stop chỉ cắt nguồn động lực để hệ thống vẫn báo trạng thái | kiểm tra đấu dây |

## 3. Yêu cầu bắt buộc (map với prompt gốc §13)

| Yêu cầu | Thực hiện ở |
|---|---|
| E-stop phần cứng độc lập | L0 + `estop_input_pin` → FAULT |
| Watchdog STM32 | firmware, 300 ms |
| Timeout Pi–STM32 | firmware watchdog + `link_timeout_s` ở bridge |
| Timeout cảm biến | docs/02 §8 |
| Giới hạn tốc độ / gia tốc | firmware (cứng) + supervisor (mềm) |
| Kiểm tra vùng né | obstacle_monitor + supervisor + mission |
| Không né khi mang hàng | mission BLOCKED guard |
| Giới hạn hành trình nâng | firmware soft limit + homing |
| Phát hiện lỗi encoder | firmware WHEEL_FAULT |
| Phát hiện lỗi LiDAR / mất camera | `sensor_ok`, stale check |
| Dừng an toàn khi node chết | supervisor publish liên tục; watchdog |
| Reset lỗi có kiểm soát | chỉ `RESET_ERROR` bởi người có quyền; `CMD_ARM` có magic |

## 4. Luật cho agent khi viết code

1. Không thêm bất kỳ đường nào cho phép bỏ qua supervisor, watchdog hoặc E-stop (kể cả "debug mode").
2. Giá trị mặc định khi thiếu dữ liệu luôn là **dừng**.
3. Không ghi DB, gọi mạng, sleep hay đọc file lớn trong callback của node điều khiển.
4. Không tự flash firmware, không tự chạy node điều khiển động cơ thật, không chạy lệnh làm robot di chuyển — đưa lệnh cho chủ dự án tự chạy, kèm cảnh báo kê bánh lên trước khi test motor.
5. Test firmware lần đầu luôn ở chế độ **bánh nhấc khỏi sàn** và giới hạn PWM thấp.
6. Mỗi module phải có test cho các nhánh an toàn của nó (dữ liệu cũ, mất kết nối, giá trị ngoài khoảng).
