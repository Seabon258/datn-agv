---
description: Phase 0 — phân tích khả thi, sơ đồ kiến trúc, review hợp đồng, liệt kê thông số thiếu (BƯỚC 1–6 của prompt gốc)
---
Thực hiện **Phase 0** trong `PLAN.md`. KHÔNG viết code module nào trong lệnh này.

1. Đọc `CLAUDE.md`, toàn bộ `docs/00..08`, `config/*.yaml`, `ros2_ws/src/agv_interfaces/`, `tools/protocol_reference.py`.
2. Chạy `python3 tools/protocol_reference.py --check`. Nếu có ROS 2 Humble: `cd ros2_ws && colcon build --packages-select agv_interfaces`.
3. Tạo `docs/architecture.md` gồm:
   - **Kickoff review**: đánh giá khả thi toàn kiến trúc, rủi ro kỹ thuật xếp theo mức độ (ví dụ: ngân sách timer STM32F411, CPU Pi 4 cho vision + ROS, odometry mecanum trượt khi strafe, vùng mù lidar, nhiễu UART, tự hãm vít me).
   - Sơ đồ phần cứng (Mermaid): nguồn 12 V, buck logic riêng, E-stop cắt nguồn động lực, Pi ↔ STM32 UART, IMU I2C, camera USB, lidar, driver.
   - Sơ đồ luồng dữ liệu node ROS 2 (khớp docs/02 §2) và sơ đồ mission state (khớp docs/04).
4. Review hợp đồng (docs/02, 03, 04, msg/srv): liệt kê mâu thuẫn, thiếu sót, điểm chưa rõ. **Không tự sửa hợp đồng** — chỉ đề xuất.
5. Liệt kê các quyết định PROPOSED trong `docs/07_decisions.md`, mỗi cái 1 dòng tóm tắt, để chủ dự án chốt.
6. Từ `config/hardware_params.yaml`, liệt kê các trường `null`, nhóm theo module bị chặn, kèm cách đo (docs/06). Hỏi chủ dự án những gì đã biết sẵn (đặc biệt: file .ioc của PCB, kích thước khung, đấu dây UART/IMU/E-stop, loại vít me).
7. Cập nhật `PLAN.md` Phase 0 (tick các mục đã xong, ghi nhật ký).
8. DỪNG. Hỏi chủ dự án: (a) chốt từng quyết định PROPOSED, (b) điền thông số đã biết, (c) có muốn bắt đầu `/module 1` không.
