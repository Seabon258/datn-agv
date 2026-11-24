# AGV Forklift Mecanum — bộ khung cho Claude Code

Bộ khung này biến hai prompt (Master prompt Modules 1–7 + Module 8 Dashboard) thành một repo mà **Claude Code** đọc và làm theo từng module, có hợp đồng giao tiếp cố định để các module khớp nhau.

## Có gì trong đây

| Thành phần | Vai trò |
|---|---|
| `CLAUDE.md` | Claude Code tự đọc mỗi phiên: bối cảnh, luật cứng, quy trình, bẫy môi trường |
| `PLAN.md` | Kế hoạch theo phase + gate; agent tick tiến độ |
| `docs/02`, `03`, `04` | **Hợp đồng**: topic/msg/srv, giao thức UART (byte-level), state machine |
| `docs/05_safety.md` | An toàn — thắng mọi tài liệu khác |
| `docs/06`, `config/hardware_params.yaml` | Thông số phần cứng còn thiếu (`null`) + cách đo. Agent không được bịa |
| `config/firmware_params.yaml` | Hằng chính sách firmware (watchdog, ramp, phát hiện lỗi bánh) |
| `docs/07_decisions.md` | Các chỗ đã điều chỉnh so với prompt gốc, chờ bạn chốt |
| `ros2_ws/src/agv_interfaces` | Message/service đã viết sẵn (nền chung cho mọi node) |
| `tools/protocol_reference.py` | Đặc tả UART chạy được + golden vectors cho cả C và Python |
| `.claude/commands` | `/kickoff`, `/module <n>`, `/review-module <n>`, `/hw-params`, `/status` |
| `.claude/agents` | Subagent `interface-guardian`, `safety-reviewer` (chỉ đọc, review) |

## Cách dùng

1. **Chuẩn bị** (laptop Ubuntu 22.04 + ROS 2 Humble là tốt nhất):
   ```bash
   unzip agv_forklift_claude_kit.zip && cd agv_forklift
   git init && git add -A && git commit -m "Init AGV kit"
   python3 tools/protocol_reference.py --check
   ```
2. Cài Claude Code (xem hướng dẫn chính thức), mở terminal trong thư mục `agv_forklift`, chạy `claude`.
3. Gõ `/kickoff` → agent phân tích, vẽ sơ đồ, liệt kê quyết định cần chốt và thông số cần đo. Trả lời các câu hỏi.
4. Gõ `/module 1` → làm firmware STM32. Agent dừng sau mỗi module để bạn test và xác nhận.
5. Lặp `/module 2` … `/module 7`, rồi `/module 8 8.1` … `/module 8 8.8`.

## Mẹo
- **Commit sau mỗi module** (`git commit`) để dễ quay lại.
- Dùng `/clear` giữa các module cho ngữ cảnh gọn; `CLAUDE.md` + `PLAN.md` giữ mạch công việc.
- Chuẩn bị sớm: file **.ioc** của PCB (copy vào `firmware/cubemx/`), kích thước khung từ SolidWorks, video line thật + rosbag lidar (để vào `tests/data/`).
- Agent bị cấm tự flash firmware và tự chạy lệnh làm xe di chuyển (`.claude/settings.json` + `CLAUDE.md`) — bạn tự chạy các test phần cứng theo hướng dẫn nó viết.
- Đổi giao thức UART: sửa `docs/03` + `tools/protocol_reference.py`, chạy `--write`, rồi mới sửa firmware/bridge.
