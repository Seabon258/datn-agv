# firmware/ — STM32F411 Black Pill

| Thư mục | Nội dung | Ai tạo |
|---|---|---|
| `cubemx/` | Project STM32CubeIDE sinh từ file **.ioc của PCB** (pin/timer mapping thật) | Chủ dự án copy vào |
| `core/` | C thuần, **không include HAL**: crc16, protocol, pid, mecanum, odometry, lift_sm, mode/watchdog, fault manager. `core/inc/agv_config.h` sinh từ `config/hardware_params.yaml` + `config/firmware_params.yaml` bằng `tools/gen_firmware_config.py` | Agent (Module 1) |
| `port/` | Glue HAL: encoder timers, PWM DRV8871, UART DMA + IDLE, step timer, GPIO limit/E-stop, I2C IMU, tick 10 ms | Agent (Module 1), sau khi đọc .ioc |
| `host_tests/` | Test `core/` trên PC bằng gcc. `protocol_vectors.h` sinh tự động — không sửa tay | Agent |

Tích hợp vào CubeIDE: thêm `core/` và `port/` làm source folder (linked folder) của project trong `cubemx/`; `main.c` chỉ gọi `app_init()` / `app_loop()` trong các vùng `USER CODE` để CubeMX sinh lại code không ghi đè.
**Agent không bao giờ tự flash.** Chủ dự án nạp bằng CubeIDE / ST-Link.
