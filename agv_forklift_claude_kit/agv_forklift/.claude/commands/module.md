---
description: Triển khai MỘT module theo PLAN.md, ví dụ /module 3 hoặc /module 8 8.2
argument-hint: <số module 1-8> [phần, vd 8.2]
---
Triển khai **Module $ARGUMENTS** theo `PLAN.md`.

Quy trình bắt buộc:
0. **Kiểm tra gate**: GATE của phase trước trong `PLAN.md` đã được chủ dự án xác nhận chưa? Nếu chưa → dừng, nói rõ còn thiếu gì, hỏi có muốn tiếp tục không.
1. Đọc spec module (`docs/00_master_spec.md` cho 1–7, `docs/01_dashboard_spec.md` cho 8) + các mục liên quan trong `docs/02`, `03`, `04`, `05`, `07` + `config/hardware_params.yaml`.
2. Liệt kê thông số phần cứng module cần mà còn `null`, giải thích vì sao cần và cách đo. Nếu thiếu: viết khung + `sim_mode`, đánh dấu rõ chỗ chờ thông số; không bịa giá trị.
3. Trình bày đủ 10 bước: mục tiêu → kiến trúc → thuật toán → cấu trúc file → code đầy đủ → giải thích hàm quan trọng → build → chạy → test riêng → tiêu chí hoàn thành.
4. Đặt logic trong thư viện thuần (Python không ROS / C không HAL), node/port là lớp vỏ mỏng. Tham số mới → thêm vào `config/agv_params.yaml` kèm comment.
5. Viết test tự động theo `docs/08_test_plan.md` và chạy những test chạy được ở đây. Sửa đến khi pass. Không xóa/vô hiệu test để pass.
6. Dùng subagent `interface-guardian` và `safety-reviewer` review phần vừa viết; sửa mọi lỗi mức HIGH, ghi lại lỗi MEDIUM/LOW chưa sửa.
7. Viết hướng dẫn từng bước cho test [bench]/[floor] mà chủ dự án phải tự chạy (lệnh cụ thể, kết quả mong đợi, cảnh báo an toàn như kê bánh khỏi sàn). Agent KHÔNG tự chạy chúng.
8. Cập nhật `PLAN.md` (tick + nhật ký) và `docs/07_decisions.md` nếu có quyết định mới đã được đồng ý.
9. Tóm tắt ngắn: đã làm gì, lệnh chạy lại test, việc chủ dự án cần làm trên phần cứng, câu hỏi mở.
10. **DỪNG.** Hỏi chủ dự án có chuyển sang module/phần tiếp theo không. Không tự viết module kế tiếp.
