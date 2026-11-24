# 06 — THÔNG SỐ PHẦN CỨNG CẦN XÁC ĐỊNH

> Giá trị lưu ở `config/hardware_params.yaml` (`null` = chưa biết). Agent **không được tự điền** — chỉ hỏi chủ dự án, kèm cách đo dưới đây. Module có thể viết khung trước và chạy `sim_mode`, nhưng chế độ phần cứng thật phải từ chối khởi động khi còn thiếu.
> Cột "Chặn" = module không thể chạy trên phần cứng thật nếu thiếu.

## 1. Khung xe (chassis)

| Tham số | Vì sao cần | Cách đo | Chặn |
|---|---|---|---|
| `footprint_length_m`, `footprint_width_m` | Self-filter lidar, hành lang né, bán kính quay | Đo thước hoặc bản vẽ SolidWorks. Hình render ghi 340 và 450 mm — xác nhận cạnh nào là chiều tiến | M5, M6 |
| `wheelbase_m`, `track_width_m` | Động học mecanum (lx, ly) | Từ bản vẽ khung, đo tâm trục / tâm bánh | M1 |
| `base_link_to_front_edge_m`, `..._rear_edge_m` | Khoảng cách vật cản tính từ mép xe; vùng mù | Đo từ tâm 4 điểm tiếp đất | M5, M6 |
| `mass_kg`, `max_payload_kg`, `payload_cog_height_m` | Giới hạn gia tốc khi mang hàng | Cân; ước lượng trọng tâm | M6 (tinh chỉnh) |

## 2. Truyền động (drive)

| Tham số | Cách đo |
|---|---|
| `encoder_counts_per_wheel_rev` | Nạp firmware chế độ *encoder test* (in số đếm). Đánh dấu bánh, quay tay đúng 10 vòng, lấy số đếm / 10. Làm cho cả 4 bánh |
| `encoder_direction_sign[]` | Cùng test trên: quay bánh theo chiều "xe tiến" mà số đếm giảm ⇒ −1 |
| `motor_direction_sign[]` | Chế độ *motor test* (bánh nhấc khỏi sàn, PWM 20%): lệnh "tiến" mà bánh quay lùi ⇒ −1 |
| `motor_max_rpm_measured` | PWM 100% (bánh nhấc), đọc rpm; sau đó đo lại khi xe đặt trên sàn có tải |
| `mecanum_roller_layout` | Sau khi đúng dấu: lệnh `vy > 0` (strafe trái). Xe đi sang trái ⇒ layout khớp công thức docs/02 §1.1 ("X"); sang phải ⇒ "O" (đảo dấu vy và wz trong ma trận) |
| `wheel_diameter_calibrated_m` | Chạy thẳng 2.000 m theo odometry, đo quãng thực; D_mới = D_cũ × thực / odom |
| `rotation_scale_calibrated` | Quay tại chỗ 10 vòng theo odometry, đo góc thực; hệ số = thực / odom |
| `drv8871_control_scheme` | Xem schematic PCB: mỗi DRV8871 có 2 chân PWM (`PWM_PWM`) hay 1 PWM + 1 GPIO (`PWM_GPIO`) |

### Ngân sách timer STM32F411 (phải đối chiếu file .ioc)
F411 có TIM1–TIM5 hỗ trợ encoder mode (TIM9–11 không). 4 encoder chiếm 4 trong 5 timer đó ⇒ chỉ còn 1 timer 4 kênh + TIM9 (2 kênh) + TIM10/TIM11 (1 kênh) cho: PWM 4 động cơ (4 hoặc 8 kênh), xung STEP stepper, tick 10 ms. Cấu hình `PWM_PWM` cần 8 kênh PWM ⇒ rất sát. **Agent phải đọc .ioc thật** (`stm32.ioc_file`) trước khi viết lớp port, không tự đặt pin/timer.

## 3. Cơ cấu nâng (lift)

| Tham số | Cách đo |
|---|---|
| `stepper_driver`, `microstepping` | Xem module driver và jumper/cấu hình MS pins |
| `steps_per_rev` | Datasheet motor (1.8° ⇒ 200) |
| `screw_lead_mm` | Ký hiệu vít (T8×8: lead 8 mm; T8×2: 2 mm) hoặc quay 1 vòng đo quãng đi |
| `screw_self_locking_verified` | Ngắt điện stepper, đặt tải tối đa lên càng: không trôi ⇒ `true`. Nếu trôi: xem docs/05 H9 |
| `travel_max_mm`, `up_position_mm`, `down_position_mm` | Đo hành trình cơ khí; chọn vị trí nâng/hạ an toàn (chừa ≥ 3 mm với giới hạn cơ khí) |
| `max_speed_mm_s`, `homing_speed_mm_s`, `homing_backoff_mm` | Thử tăng dần đến khi mất bước, lấy 60% |
| `limit_switch_type`, `limit_switch_active_level`, `has_top_limit_switch` | Xem đấu dây; đo mức logic khi nhấn |

## 4. STM32 / giao tiếp

| Tham số | Ghi chú |
|---|---|
| `stm32.ioc_file` | Copy file .ioc của PCB vào `firmware/cubemx/` |
| `stm32.uart_instance` | Theo .ioc |
| `stm32.estop_input_pin`, `battery_adc_pin` | Theo PCB (null nếu chưa có) |
| `comm.link_type`, `comm.pi_serial_device` | Khuyến nghị `PI_GPIO_UART` + `/dev/ttyAMA0` (docs/03 §1) |

## 5. IMU, camera, lidar, ArUco

| Tham số | Cách đo |
|---|---|
| `imu.connected_to`, `i2c_address`, `mount_rpy_deg` | Theo đấu dây; `i2cdetect` (Pi) hoặc I2C scan (STM32) |
| `camera.device` | `v4l2-ctl --list-devices`; nên dùng đường dẫn `/dev/v4l/by-id/...` |
| `camera.supported_mode_320x240_mjpg` | `v4l2-ctl -d <dev> --list-formats-ext` |
| `camera.mount_*`, `tilt_deg` | Đo từ base_link |
| `camera.view_near_m`, `view_far_m` | Đặt thước dọc trên sàn trước xe, chụp ảnh, đọc khoảng cách ở mép dưới ảnh và ở đường `roi_top_ratio` |
| `lidar.serial_device`, `scan_topic`, `frame_id` | Chạy driver, `ros2 topic list`, `ros2 topic echo --once <topic>` xem `header.frame_id` |
| `lidar.angle_convention_verified` | Đặt hộp thẳng phía trước, xem điểm nằm ở góc ≈ 0 và tăng CCW (RViz) |
| `lidar.mount_*` | Đo tâm lidar so với base_link; `mount_z_m` = chiều cao mặt quét |
| `lidar.min_range_m`, `max_range_m`, `scan_rate_hz` | Từ LaserScan + `ros2 topic hz` |
| `lidar.blocked_angle_ranges_deg` | Xe đứng giữa phòng trống, ghi bag 10 s; các cung có điểm < 0.35 m (trúng trụ, bánh, dây) là cung bị che. Agent có thể viết script phân tích bag để đề xuất |
| `aruco.marker_size_m` | Đo cạnh marker sau khi in |

## 6. Nguồn

| Tham số | Ghi chú |
|---|---|
| `battery_type`, `voltage_sense_available`, `voltage_divider_ratio` | Theo phần cứng; không có đo ⇒ ghi rõ H15 chưa giám sát |
| `estop_installed`, `estop_cuts`, `logic_power_separate` | Xác nhận đấu dây theo docs/05 L0, H16 |

## 7. Thứ tự đề xuất đo
1. Chassis (bản vẽ) → 2. .ioc + timer map → 3. motor/encoder sign + counts (bánh nhấc) → 4. layout mecanum → 5. lift (tự hãm!) → 6. camera view → 7. lidar mount + cung bị che → 8. hiệu chỉnh đường kính/quay trên sàn.
