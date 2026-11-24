# dashboard/ — Module 8
- `backend/`: FastAPI + Uvicorn + rclpy monitor (tiến trình riêng, không nằm trong node điều khiển). Phục vụ cả frontend đã build.
- `frontend/`: React + TypeScript + Vite + Tailwind (`.tsx`), có chế độ dữ liệu giả lập.
Spec: `docs/01_dashboard_spec.md`. Hợp đồng dữ liệu: `docs/02_interfaces.md §11`. Lệnh điều khiển chỉ qua `/mission/command`.
