# AGV F411 - bảng chân để vẽ PCB

Board: WeAct Black Pill STM32F411CEU6 (thạch anh 25 MHz, chạy 100 MHz).
Cấu hình: `agv_f411.ioc` (CubeMX 6.18.1). Project build OK, 0 lỗi.

## Bảng chân

| Chân | Tên trong code | Chức năng | Nối tới |
|---|---|---|---|
| PA8  | M1_PWM_IN2 | TIM1_CH1, PWM 20 kHz | DRV8871 #1 **IN2** (M1 trước trái) |
| PB12 | M1_DIR_IN1 | GPIO out | DRV8871 #1 **IN1** |
| PA9  | M2_PWM_IN2 | TIM1_CH2 | DRV8871 #2 **IN2** (M2 sau trái) |
| PB13 | M2_DIR_IN1 | GPIO out | DRV8871 #2 **IN1** |
| PA10 | M3_PWM_IN2 | TIM1_CH3 | DRV8871 #3 **IN2** (M3 sau phải) |
| PB14 | M3_DIR_IN1 | GPIO out | DRV8871 #3 **IN1** |
| PA11 | M4_PWM_IN2 | TIM1_CH4 | DRV8871 #4 **IN2** (M4 trước phải) |
| PB15 | M4_DIR_IN1 | GPIO out | DRV8871 #4 **IN1** |
| PA15 | ENC1_A | TIM2_CH1 encoder | Encoder M1 kênh A |
| PB3  | ENC1_B | TIM2_CH2 | Encoder M1 kênh B |
| PB4  | ENC2_A | TIM3_CH1 | Encoder M2 kênh A |
| PB5  | ENC2_B | TIM3_CH2 | Encoder M2 kênh B |
| PB6  | ENC3_A | TIM4_CH1 | Encoder M3 kênh A |
| PB7  | ENC3_B | TIM4_CH2 | Encoder M3 kênh B |
| PA0  | ENC4_A | TIM5_CH1 | Encoder M4 kênh A |
| PA1  | ENC4_B | TIM5_CH2 | Encoder M4 kênh B |
| PB8  | IMU_SCL | I2C1 400 kHz | MPU6050 SCL |
| PB9  | IMU_SDA | I2C1 | MPU6050 SDA |
| PA2  | UART_TX_TO_PI_RX | USART2 TX, 460800 | Pi GPIO15 (chân 10) + CH340 RX (debug) |
| PA3  | UART_RX_FROM_PI_TX | USART2 RX (DMA) | Pi GPIO14 (chân 8) |
| PB0  | STEP_STEP | GPIO out tốc độ cao (TIM10 phát xung) | TMC2209 STEP |
| PB1  | STEP_DIR | GPIO out | TMC2209 DIR |
| PA5  | STEP_EN | GPIO out, mặc định 1 = tắt | TMC2209 EN |
| PB10 | LIMIT_BOT | GPIO in, pull-up | Công tắc đáy |
| PA4  | LIMIT_TOP | GPIO in, pull-up | Công tắc đỉnh |
| PC13 | LED | GPIO out (LED có sẵn trên board) | - |
| PA13, PA14 | SWDIO, SWCLK | Nạp/debug | Header SWD có sẵn trên Black Pill |

Còn trống: **PA6, PA7, PA12, PB2, PC14, PC15**. Nên đưa PA6, PA7 ra header dự phòng (PA6 có thể dùng cho MPU6050 INT).

Timer nội: TIM10 = ngắt phát xung STEP, TIM11 = vòng điều khiển 1 kHz, SysTick = HAL.

## Đầu nối trên PCB

| Đầu nối | Số chân | Chân |
|---|---|---|
| Nguồn vào | XT60 | 12V, GND |
| Động cơ + encoder M1..M4 (cáp JGB37 6 sợi) | 6 ×4 | M+ (DRV OUT1), M- (DRV OUT2), 3V3, GND, A, B |
| TMC2209 | 2 hàng 8 chân cái (cắm module) | xem sơ đồ chân Tuyết Nga |
| Động cơ bước | 4 | M1A, M1B, M2A, M2B |
| Công tắc đáy, đỉnh | 2 ×2 | Tín hiệu, GND |
| MPU6050 | 4 (+1 INT) | 3V3, GND, SCL, SDA, (INT -> PA6) |
| Pi UART | 3 | TX(PA2), RX(PA3), GND |
| Debug CH340 | 2 | PA2, GND |
| 5V ra cho Pi/lidar (tùy chọn) | 2 | 5V, GND |

## Ghi chú khi vẽ

1. **Encoder cấp 3V3**, không cấp 5V.
2. **Tụ 100 µF 35V + 100 nF** sát chân VM-GND của TMC2209, và mỗi DRV8871 thêm 1 tụ 100 µF ở VM.
3. **Điện trở 10k kéo STEP_EN (PA5) lên 3V3**, để TMC2209 tắt trong lúc F411 đang reset hoặc chưa chạy.
4. **Công tắc hành trình dùng tiếp điểm NC** nối xuống GND. Bình thường đọc 0, chạm hoặc đứt dây đọc 1 -> dừng, an toàn hơn.
5. **Tách GND công suất** (DRV8871, TMC2209, 12V) và GND logic, nối chung tại 1 điểm gần đầu vào nguồn. Đường 12V và dây động cơ to (≥ 1 mm), đường tín hiệu 0.3-0.4 mm.
6. **PA0 trùng nút KEY** trên Black Pill: không bấm nút KEY khi đang chạy. **PA11** là USB D-: khi nạp qua Type-C thì tắt 12V.
7. Đi dây encoder xa đường PWM và dây động cơ. Có thể thêm tụ 1 nF từ A/B xuống GND nếu nhiễu.
8. **Điện trở 10k kéo PA8, PA9, PA10, PA11 xuống GND.** PA10 là USART1_RX: bootloader ROM của F411 dò cả USART1 lẫn USB, PA10 để hở bị nhiễu thì bootloader chọn UART và DFU qua Type-C lỗi -71. Kéo xuống GND còn giữ bánh đứng yên khi F411 reset.
9. Nguồn: 12V -> XT60 -> cầu chì 5A -> chống ngược -> VM; buck 5V -> chân 5V Black Pill; 3V3 lấy từ Black Pill (encoder + MPU ~50 mA).
