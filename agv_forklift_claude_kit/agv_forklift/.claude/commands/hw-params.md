---
description: Liệt kê thông số phần cứng còn thiếu (null), cách đo, và cập nhật config/hardware_params.yaml theo câu trả lời
argument-hint: [tùy chọn: nhóm, vd drive | lift | lidar]
---
1. Đọc `config/hardware_params.yaml` và `docs/06_hardware_params.md`. Nếu có đối số "$ARGUMENTS" thì chỉ xét nhóm đó.
2. Liệt kê các trường `null` thành bảng: tên, vì sao cần, cách đo ngắn gọn, module bị chặn. Sắp theo thứ tự đo đề xuất ở docs/06 §7.
3. Hỏi chủ dự án giá trị (cho phép trả lời một phần).
4. Với mỗi giá trị chủ dự án đưa ra: kiểm tra đơn vị và khoảng hợp lý (ví dụ đường kính bánh 0.05–0.15 m); nếu đáng ngờ thì hỏi lại; nếu hợp lý thì ghi vào YAML, giữ nguyên comment.
5. Không bao giờ điền giá trị chủ dự án không cung cấp. Báo lại số trường còn `null`.
