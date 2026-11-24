# MODULE 8 – AGV MONITORING DASHBOARD & DATABASE MANAGEMENT

> **Nguồn:** prompt gốc của chủ dự án (Module 8 – Dashboard), giữ nguyên nội dung, chỉ đổi định dạng.
> **Thứ tự ưu tiên khi mâu thuẫn:** `05_safety` > `02_interfaces` > `04_state_machines` > `07_decisions` > file này.
> Điều chỉnh đã thống nhất (xem `07_decisions.md`): frontend dùng TypeScript nên các file là `.tsx`/`.ts` (cây thư mục gốc ghi `.jsx`/`.js`); lệnh điều khiển đi qua service `/mission/command` của `mission_controller`; trường `Obstacle Behind` được thay bằng `pass_progress_m` vì lidar gắn trước không nhìn thấy phía sau.


## 1. VAI TRÒ

Bạn là kỹ sư Software Engineering, Robotics Monitoring và IoT Dashboard.

Hãy thiết kế và lập trình một hệ thống Dashboard giám sát AGV Forklift, tích hợp trực tiếp với hệ thống ROS 2 Humble đang chạy trên Raspberry Pi 4.

Hệ thống phải có khả năng:

- Theo dõi trạng thái AGV theo thời gian thực.
- Hiển thị thông tin cảm biến và động cơ.
- Theo dõi trạng thái nhiệm vụ.
- Giám sát cơ cấu nâng hàng.
- Cảnh báo lỗi.
- Lưu lịch sử hoạt động vào Database.
- Truy vấn và thống kê dữ liệu.
- Xem lại lịch sử hành trình và các sự kiện đã xảy ra.

Dashboard phải hoạt động độc lập với các node điều khiển robot. Nếu Dashboard hoặc Database bị mất kết nối, AGV vẫn phải duy trì cơ chế điều khiển và an toàn độc lập.

---

## 2. KIẾN TRÚC TỔNG THỂ

Sử dụng kiến trúc:

```text
AGV Sensors / STM32
|
v
ROS 2 Humble
|
v
ROS 2 Monitoring Node
|
v
FastAPI Backend
|
+---------- WebSocket
|               |
|               v
|          Web Dashboard
|
+---------- MySQL
|
v
Historical Data
```

Các thành phần:

1. ROS 2 Humble: Nguồn dữ liệu robot.
2. Monitoring Node: Thu thập dữ liệu từ các ROS 2 Topic.
3. FastAPI: Backend cung cấp API.
4. WebSocket: Truyền dữ liệu thời gian thực.
5. Frontend: Dashboard hiển thị.
6. MySQL: Lưu dữ liệu lâu dài.

Không cho phép Dashboard truy cập trực tiếp vào STM32.

Không để Database tham gia vào vòng điều khiển PID hoặc vòng điều khiển an toàn.

---

## 3. CÔNG NGHỆ SỬ DỤNG

Backend:

- Python 3.
- FastAPI.
- Uvicorn.
- rclpy.
- WebSocket.
- SQLAlchemy.
- MySQL Connector hoặc PyMySQL.
- Pydantic.

Frontend:

- React.
- TypeScript.
- Vite.
- Tailwind CSS.
- Recharts hoặc thư viện biểu đồ tương đương.
- WebSocket Client.

Database:

- MySQL 8.x.

Triển khai:

- Raspberry Pi 4B.
- Ubuntu 22.04.
- ROS 2 Humble.

Ưu tiên thiết kế nhẹ, không sử dụng quá nhiều hiệu ứng hoặc thư viện nặng làm ảnh hưởng Raspberry Pi.

Có thể chạy Frontend trên máy tính khác trong cùng mạng LAN.

---

## 4. DASHBOARD – GIAO DIỆN GIÁM SÁT

Thiết kế giao diện hiện đại, phong cách Industrial Robotics Dashboard.

Giao diện phải Responsive, hiển thị tốt trên laptop, tablet và điện thoại.

Chia thành các khu vực:

### 4.1. Tổng quan AGV

Hiển thị:

- Robot Status.
- Connection Status.
- Mission State.
- Current Station.
- Battery Voltage nếu có cảm biến đo.
- Motor Status.
- Lift Status.
- Obstacle Status.
- Camera Status.
- LiDAR Status.
- STM32 Connection Status.

Các trạng thái:

- IDLE.
- FOLLOW_LINE.
- APPROACH_MARKER.
- LIFT_ACTION.
- BLOCKED.
- DETOUR.
- PASS.
- RETURN.
- REACQUIRE.
- ERROR.
- EMERGENCY_STOP.

Sử dụng màu sắc trực quan:

- Green: Normal.
- Yellow: Warning.
- Red: Error / Emergency.
- Gray: Offline.

Không giả lập trạng thái khi dữ liệu thực tế chưa có.

Nếu dữ liệu không được cập nhật trong khoảng thời gian cấu hình, phải hiển thị STALE hoặc OFFLINE.

### 4.2. Robot Telemetry

Hiển thị theo thời gian thực:

- X.
- Y.
- Theta.
- VX.
- VY.
- WZ.
- Left Front RPM.
- Right Front RPM.
- Left Rear RPM.
- Right Rear RPM.
- Encoder Error nếu có.

Có biểu đồ:

- Wheel RPM.
- Linear Velocity.
- Angular Velocity.
- Odometry X/Y.
- Robot Heading.

Cho phép chọn khoảng thời gian xem dữ liệu.

### 4.3. Line Following Monitor

Hiển thị:

- Line Found.
- Line Error.
- Line Angle.
- Confidence.
- Camera FPS.
- Camera Connection.

Có thể hiển thị camera image và ảnh debug nếu được bật.

Hiển thị trạng thái:

- TRACKING.
- LOST.
- RECOVERING.

Không bắt buộc stream video liên tục nếu làm tăng tải CPU hoặc băng thông.

### 4.4. ArUco Monitor

Hiển thị:

- Marker ID.
- Marker Position in Image.
- Marker Detection Status.
- Marker Confirmation Count.
- Current Station.
- Station Action.

Ví dụ:

ID 0 = HOME.
ID 1 = LIFT_UP.
ID 2 = LIFT_DOWN.

Mapping phải lấy từ cấu hình hệ thống, không hard-code trên Dashboard.

### 4.5. LiDAR Monitor

Hiển thị:

- Obstacle Zone.
- Front Distance.
- Left Free.
- Right Free.
- Obstacle Beside.
- Obstacle Behind.
- LiDAR Status.

Có thể tích hợp RViz2 riêng để xem dữ liệu quét LiDAR.

Dashboard phải thể hiện rõ:

- CLEAR.
- SLOW.
- STOP.

Nếu LiDAR mất kết nối hoặc dữ liệu stale, hiển thị lỗi cảm biến và không cho phép Dashboard tự ý bỏ qua lỗi để robot tiếp tục chạy.

### 4.6. Lift Monitor

Hiển thị:

- Lift State.
- Homing Status.
- Current Position nếu có.
- Limit Switch Status.
- Lift Error.
- Last Lift Action.

Các trạng thái:

- IDLE.
- HOMING.
- MOVING_UP.
- MOVING_DOWN.
- DONE.
- ERROR.

### 4.7. Mission Monitor

Hiển thị tiến trình nhiệm vụ:

- Current Mission.
- Current State.
- Current Station.
- Target Station.
- Completed Missions.
- Mission Duration.
- Last Event.
- Mission Error.

Có Timeline các trạng thái:

FOLLOW_LINE → APPROACH_MARKER → LIFT_ACTION → FOLLOW_LINE.

Khi xảy ra vật cản:

FOLLOW_LINE → BLOCKED → DETOUR → PASS → RETURN → REACQUIRE.

---

## 5. DASHBOARD CONTROL PANEL

Tạo một khu vực điều khiển có phân quyền.

Các thao tác có thể hỗ trợ:

- Start Mission.
- Pause Mission.
- Resume Mission.
- Cancel Mission.
- Request Home.
- Lift Up.
- Lift Down.
- Reset Error.

Mỗi lệnh phải được xác thực trước khi gửi.

Không được gửi trực tiếp lệnh PWM hoặc lệnh động cơ từ Dashboard.

Lệnh phải đi qua một ROS 2 Command Gateway hoặc Mission Controller.

Các lệnh điều khiển phải có:

- Confirmation Dialog.
- Permission Check.
- Command Timeout.
- Response Status.
- Audit Log.

Không cho phép Dashboard vô hiệu hóa Emergency Stop.

Emergency Stop phải là cơ chế phần cứng độc lập.

Dashboard chỉ được phép hiển thị trạng thái Emergency Stop và gửi yêu cầu dừng bổ sung nếu được thiết kế phù hợp.

Nếu robot đang mang hàng hoặc LiDAR báo STOP, các thao tác di chuyển phải tuân thủ điều kiện an toàn từ Mission Controller.

---

## 6. FASTAPI BACKEND

Tạo backend bằng FastAPI.

Cấu trúc:

```text
dashboard_backend/
├── app/
│   ├── main.py
│   ├── config.py
│   ├── ros_bridge/
│   │   └── ros_monitor.py
│   ├── api/
│   │   ├── status.py
│   │   ├── missions.py
│   │   ├── telemetry.py
│   │   ├── events.py
│   │   └── commands.py
│   ├── websocket/
│   │   └── manager.py
│   ├── database/
│   │   ├── connection.py
│   │   ├── models.py
│   │   ├── repository.py
│   │   └── migrations/
│   ├── services/
│   │   ├── logging_service.py
│   │   ├── mission_service.py
│   │   └── telemetry_service.py
│   └── schemas/
│       ├── status.py
│       ├── mission.py
│       └── telemetry.py
│
├── requirements.txt
└── README.md
```

Backend phải:

1. Kết nối ROS 2.
2. Subscribe các topic cần thiết.
3. Chuẩn hóa dữ liệu.
4. Cung cấp REST API.
5. Cung cấp WebSocket.
6. Ghi dữ liệu vào MySQL.
7. Quản lý kết nối Database.
8. Có logging.
9. Có xử lý ngoại lệ.
10. Không làm block ROS executor.

Thiết kế ROS 2 monitoring node chạy độc lập hoặc tích hợp bằng một tiến trình phù hợp.

Không chạy asyncio và rclpy theo cách làm treo executor.

Sử dụng thread hoặc cơ chế executor phù hợp để tách ROS callbacks, WebSocket và Database operations.

---

## 7. WEBSOCKET REAL-TIME

Tạo WebSocket endpoint:

/ws/agv/status

Dữ liệu gửi định kỳ:

- Robot Status.
- Mission State.
- Odometry.
- Motor RPM.
- Line State.
- ArUco State.
- Obstacle State.
- Lift State.
- Sensor Health.

Tần số cập nhật Dashboard có thể cấu hình, mặc định khoảng 5–10 Hz.

Không cần gửi toàn bộ dữ liệu LiDAR thô lên Dashboard.

Có thể sử dụng REST API để truy vấn lịch sử thay vì gửi lịch sử liên tục qua WebSocket.

Yêu cầu:

- Tự reconnect.
- Heartbeat.
- Timestamp.
- Sequence Number.
- Kiểm tra dữ liệu stale.
- Xử lý mất kết nối.
- Không gửi dữ liệu không thay đổi quá thường xuyên nếu không cần thiết.

---

## 8. MYSQL DATABASE

Thiết kế Database có cấu trúc rõ ràng, có khóa chính, khóa ngoại và index phù hợp.

Database dự kiến:

agv_database

### 8.1. Bảng robots

Lưu thông tin AGV:

- robot_id.
- robot_name.
- model.
- firmware_version.
- software_version.
- created_at.

### 8.2. Bảng missions

Lưu lịch sử nhiệm vụ:

- mission_id.
- robot_id.
- start_time.
- end_time.
- start_station.
- target_station.
- mission_status.
- total_duration.
- error_message.

### 8.3. Bảng telemetry

Lưu dữ liệu chuyển động:

- id.
- robot_id.
- timestamp.
- x.
- y.
- theta.
- vx.
- vy.
- wz.
- front_left_rpm.
- front_right_rpm.
- rear_left_rpm.
- rear_right_rpm.

Không ghi telemetry quá dày một cách không cần thiết.

Tần số lưu dữ liệu phải cấu hình được.

Có thể lưu dữ liệu telemetry tổng hợp hoặc lấy mẫu theo chu kỳ.

### 8.4. Bảng sensor_events

Lưu sự kiện cảm biến:

- event_id.
- robot_id.
- timestamp.
- sensor_type.
- event_type.
- event_value.
- severity.
- description.

Ví dụ:

LIDAR_STOP.
LIDAR_CLEAR.
LINE_LOST.
ARUCO_DETECTED.
CAMERA_OFFLINE.
IMU_ERROR.

### 8.5. Bảng mission_events

Lưu lịch sử thay đổi trạng thái:

- id.
- mission_id.
- timestamp.
- previous_state.
- current_state.
- description.

### 8.6. Bảng lift_logs

Lưu hoạt động nâng:

- id.
- robot_id.
- timestamp.
- action.
- start_position.
- end_position.
- status.
- duration.
- error_code.

### 8.7. Bảng system_errors

Lưu lỗi hệ thống:

- id.
- robot_id.
- timestamp.
- error_code.
- component.
- severity.
- description.
- resolved_at.

### 8.8. Bảng command_logs

Lưu các lệnh gửi từ Dashboard:

- id.
- user_id.
- robot_id.
- command_type.
- command_payload.
- request_time.
- response_time.
- result.
- description.

---

## 9. DATABASE PERFORMANCE

Không được ghi Database trực tiếp trong ROS callback.

Sử dụng:

- Queue.
- Background Worker.
- Batch Insert.
- Connection Pool.
- Retry Mechanism.

Nếu Database bị mất kết nối:

1. Không làm treo robot.
2. Không block ROS executor.
3. Tạm lưu dữ liệu vào queue.
4. Có giới hạn dung lượng queue.
5. Thử kết nối lại.
6. Ghi log nếu mất dữ liệu.

Có thể sử dụng SQLite làm bộ đệm cục bộ nếu cần.

MySQL là nơi lưu trữ lịch sử chính.

Không cần lưu ảnh camera liên tục vào MySQL.

Nếu cần lưu ảnh sự kiện, lưu file riêng và chỉ lưu đường dẫn cùng metadata trong Database.

---

## 10. REST API

Tạo các endpoint:

GET /api/agv/status

GET /api/agv/telemetry

GET /api/agv/missions

GET /api/agv/missions/{mission_id}

GET /api/agv/events

GET /api/agv/errors

GET /api/agv/lift-logs

POST /api/agv/mission/start

POST /api/agv/mission/pause

POST /api/agv/mission/resume

POST /api/agv/mission/cancel

POST /api/agv/lift/home

POST /api/agv/lift/up

POST /api/agv/lift/down

POST /api/agv/errors/reset

Các endpoint POST phải có kiểm tra quyền, điều kiện an toàn và xác nhận kết quả.

API phải có:

- Pydantic validation.
- HTTP status code phù hợp.
- Error handling.
- Authentication.
- API documentation bằng Swagger/OpenAPI.

---

## 11. FRONTEND DASHBOARD

Cấu trúc:

```text
dashboard_frontend/
├── src/
│   ├── components/
│   │   ├── RobotStatus.jsx
│   │   ├── MissionPanel.jsx
│   │   ├── TelemetryChart.jsx
│   │   ├── MotorMonitor.jsx
│   │   ├── LineMonitor.jsx
│   │   ├── ArucoMonitor.jsx
│   │   ├── LidarMonitor.jsx
│   │   ├── LiftMonitor.jsx
│   │   └── ErrorPanel.jsx
│   ├── pages/
│   │   ├── Dashboard.jsx
│   │   ├── Missions.jsx
│   │   ├── History.jsx
│   │   ├── Events.jsx
│   │   └── Settings.jsx
│   ├── services/
│   │   ├── api.js
│   │   └── websocket.js
│   ├── App.jsx
│   └── main.jsx
└── package.json
```

Thiết kế Dashboard có:

- Sidebar.
- Header.
- Robot overview.
- Real-time telemetry.
- Mission progress.
- Sensor status.
- Lift control.
- Error notification.
- Historical charts.

Giao diện ưu tiên dễ quan sát khi robot đang chạy thực tế.

---

## 12. BẢO MẬT

- Dashboard có đăng nhập.
- Phân quyền Operator và Administrator.
- Có Audit Log.
- Không để MySQL public ra Internet.
- Không hard-code mật khẩu trong source code.
- Sử dụng environment variables.
- Có cơ chế timeout phiên đăng nhập.
- Giới hạn API điều khiển.
- Không cho phép người không có quyền gửi lệnh robot.

---

## 13. CẤU HÌNH VÀ TRIỂN KHAI

Tạo cấu hình:

backend/.env.example

frontend/.env.example

database/schema.sql

Tất cả thông số phải cấu hình được:

- Database host.
- Database port.
- Database name.
- Database user.
- WebSocket update rate.
- Telemetry logging rate.
- Queue size.
- Retry interval.
- API port.
- Frontend URL.
- Authentication settings.

Cung cấp hướng dẫn chạy:

1. Khởi động MySQL.
2. Khởi động ROS 2.
3. Khởi động FastAPI.
4. Khởi động Frontend.
5. Truy cập Dashboard từ máy tính khác trong LAN.
6. Kiểm tra dữ liệu thời gian thực.
7. Kiểm tra dữ liệu được lưu vào Database.
8. Kiểm tra khả năng phục hồi khi Database mất kết nối.

---

## 14. KIỂM THỬ

Phải có các bài kiểm thử:

1. Dashboard nhận dữ liệu robot.
2. Dashboard nhận trạng thái line.
3. Dashboard nhận marker.
4. Dashboard nhận LiDAR STOP.
5. Dashboard nhận trạng thái nâng hàng.
6. Dashboard hiển thị trạng thái ERROR.
7. WebSocket tự kết nối lại.
8. Database lưu lịch sử nhiệm vụ.
9. Truy vấn lịch sử nhiệm vụ.
10. Database mất kết nối nhưng AGV vẫn vận hành an toàn.
11. Dashboard mất kết nối nhưng AGV không mất khả năng điều khiển.
12. Người dùng không có quyền không thể gửi lệnh.

Có chế độ giả lập dữ liệu ROS 2 để phát triển Dashboard trước khi AGV hoàn thiện.

---

## 15. QUY TẮC TRIỂN KHAI

Triển khai MODULE 8 sau khi các module ROS 2 chính đã có giao diện dữ liệu ổn định.

Viết từng phần:

1. Thiết kế Database.
2. Viết ROS 2 Monitoring Node.
3. Viết FastAPI Backend.
4. Viết WebSocket.
5. Viết Frontend Dashboard.
6. Tích hợp MySQL.
7. Tích hợp API điều khiển.
8. Kiểm thử toàn hệ thống.

Sau mỗi phần:

- Giải thích kiến trúc.
- Viết code hoàn chỉnh.
- Hướng dẫn chạy.
- Hướng dẫn test.
- Xác nhận kết quả trước khi sang phần tiếp theo.

Không viết toàn bộ trong một câu trả lời.

Không tự ý thay đổi tên ROS 2 Topic hoặc Message đã được thống nhất trong các module trước.

Không để Dashboard hoặc Database ảnh hưởng đến vòng điều khiển thời gian thực và cơ chế an toàn của AGV.

MỤC TIÊU CUỐI CÙNG:

Xây dựng một hệ thống AGV Forklift có Dashboard giám sát trực quan, cập nhật trạng thái theo thời gian thực, quản lý nhiệm vụ, lưu lịch sử hoạt động, thống kê lỗi và hỗ trợ người vận hành theo dõi robot từ xa trong mạng nội bộ.
