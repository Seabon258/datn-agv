---
description: Review một module đã viết bằng 2 subagent (hợp đồng giao tiếp + an toàn), ví dụ /review-module 5
argument-hint: <số module>
---
Review code của **Module $ARGUMENTS** (xác định file theo `PLAN.md` và `docs/02_interfaces.md §10`).

1. Gọi subagent `interface-guardian` với danh sách file của module.
2. Gọi subagent `safety-reviewer` với cùng danh sách file.
3. Gộp kết quả thành một bảng: mức độ (HIGH/MEDIUM/LOW), file:dòng, vấn đề, đề xuất sửa.
4. Hỏi chủ dự án có muốn sửa ngay các mục HIGH không. Không tự sửa khi chưa được đồng ý.
