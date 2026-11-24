# Danh sách mua - Xe mecanum DATN (nâng hạ bằng step + vitme T8)

Giá tham khảo ở Điện Tử Tuyết Nga (ngày 27/10/2026), chỉ ghi những món đã xem được trên web.

## Đã chốt / đã có

| Món | Ghi chú |
|---|---|
| Vitme T8 **lead 2** (1 đầu ren), dài 150-200 mm, kèm đai ốc lead 2 | **Đổi** từ lead 8 (lead 8 tự tụt khi thả). Lead 2 tự hãm, 2 mm/vòng. Hỏi Tuyết Nga đổi được không |
| Động cơ bước **42BYGH60 (60mm)** - 240k | 0.7 Nm, 1.7 A, 2.8 Ω, 6.5 mH, 4 dây. Hỏi shop: trục Ø5, có kèm dây cáp không |
| Ray / trục dẫn hướng cho bàn nâng | Nếu là trục tròn Ø8 thì cần thêm bạc trượt LM8UU |

## 1. Nâng hạ (bắt buộc)

| Món | SL | Giá | Ghi chú |
|---|---|---|---|
| Driver **TMC2209** | 2 | 80k/cái | 1 dự phòng. Có tản nhiệt, ưu tiên bản V1.2/V1.3 (BTT, FYSETC) |
| ~~Khớp nối mềm 5-8mm~~ | 0 | | Không cần nữa vì đã chuyển sang dây đai GT2 |
| Gá động cơ 42 | 1 | 25k | Kèm 4 vít M3x8 hoặc M3x10 |
| Puly **GT2 20 hoặc 24 răng, lỗ 5 mm** | 1 | | Gắn trục động cơ |
| Puly **GT2 cùng số răng, lỗ 8 mm** | 1 | | Gắn vitme, đặt ngay trên ổ bi KFL08 |
| Dây đai **GT2 vòng kín, rộng 6 mm** | 1-2 | | Dài ≈ 2 × khoảng cách tâm + 40 mm (20 răng) / + 48 mm (24 răng) |
| Ổ bi **KFL08** hoặc **KP08** (lỗ 8 mm) | 1 | | Đặt ở đáy vitme để gánh 6 kg, không cho đè lên trục động cơ |
| Vòng chặn trục 8 mm (shaft collar) | 2 | | Kẹp vitme 2 mặt ổ bi. Không có thì mua thêm **2 đai ốc T8 lead 2** vặn hãm |
| Công tắc hành trình có cần gạt | 3 | | Đáy + đỉnh + 1 dự phòng |
| Tụ hóa 100 µF 35V | 2 | | Sát chân VM-GND của TMC2209 |
| Dây cáp động cơ bước 4 chân | 1 | | Xem động cơ có kèm dây chưa |

## 2. Bo mạch điều khiển

| Món | SL | Giá | Ghi chú |
|---|---|---|---|
| **WeAct Black Pill STM32F411CEU6** - bản **không** flash 8MB | 2 | 145k/cái | 1 dự phòng. Chip ghi **F411CEU6**, board có logo WeAct |
| Hàng rào chân đực + cái | vài thanh | | Nếu board không kèm |
| Dây Dupont đực-cái, cái-cái | 1 bó | | |
| MPU6050 (GY-521) | 1 | | Nếu chưa có |

Không mua: bản Flash 8MB (165k, chiếm PA4-PA7) và "Kit STM32F411CEU6 ARM-STM32F4" (230k, trang ghi lẫn F401CCU6).

## 3. Nguồn và bảo vệ

| Món | SL | Ghi chú |
|---|---|---|
| Cầu chì 5A + đế | 1 bộ | Ngay sau cực dương pin |
| Jack XT60 đực + cái | 2 cặp | Có rãnh định hướng, không cắm ngược được |
| Module chống ngược cực (MOSFET) hoặc diode Schottky SS54 | 1 | Ngay sau pin, bảo vệ cả xe |
| Mạch hạ áp 12V -> 5V (2-3A) | 1 | Thay TPS5430 đã hỏng. Chỉnh ra 5.0V và đo trước khi nối board |

## 4. Tùy chọn

| Món | Khi nào cần |
|---|---|
| Ổ bi 608ZZ | Vitme bị đảo ở đầu trên |
| Bạc trượt LM8UU | Ray nâng là trục tròn Ø8 chưa có bạc |
| Vít + đai ốc M3, M4 | Bắt ổ bi, công tắc, gá lên khung |

## 5. Cảm biến cho Pi (mua sau, khi F411 + nâng hạ đã chạy)

Cả nhóm này cắm vào Pi 4, **không tốn chân F411**.

| Món | SL | Giá tham khảo | Ghi chú |
|---|---|---|---|
| **Pi Camera OV5647 5MP, bản góc rộng** (cổng CSI) | 1 | ~100-200k | **Dò line** (chốt). Chúi xuống sàn, cao 15-25 cm. Mua kèm cáp CSI 30-50 cm |
| **Webcam Rapoo C260** (USB, 1080p, 95°, nét cố định) | 1 | ~400-500k | **Đọc QR trên hàng** (chốt). Hết hàng thì lấy webcam 1080p UVC bất kỳ <500k. QR mờ thì đổi sang module USB OV5640 AF |
| **Lidar 2D**: RPLidar C1/A1 hoặc LDRobot LD06/LD19 | 1 | ~1-3 triệu (hỏi shop) | **SLAM** khi mất line (ROS 2 + slam_toolbox). Xem nguồn 5V và kiểu cổng (USB hay UART) |
| Băng keo màu tương phản với sàn | 2-3 cuộn | | Làm line. Thử trên sàn thật trước |
| Dải LED nhỏ 5V chiếu xuống sàn | 1 | | Giữ ánh sáng ổn định cho camera dò line |
| Quạt + tản nhiệt cho Pi 4 | 1 bộ | | Bắt buộc khi chạy SLAM + camera |
| Mạch hạ áp **12V -> 5V 5A** (loại tốt, có cổng USB-C hoặc dây ra) | 1 | | Cấp cho Pi + lidar từ pin xe nếu sạc dự phòng Anker không gánh nổi (>3A) |
| In/dán mã QR hoặc ArUco | | | Dán trên hàng và ở các trạm |

Kiểm tra: Pi 4 bản **4 GB trở lên** cho SLAM. Nếu Pi lấy nguồn từ pin xe thì xem lại cầu chì 5A (Pi + lidar thêm khoảng 1.5 A ở 12V).

## Kiểm tra trước khi trả tiền

1. Vitme **lead 2**: dựng đứng, thả đai ốc ra thì nó **đứng yên**, không tự tụt. Vặn tay đai ốc thì trơn hết thanh. Ren chỉ có 1 đường xoắn.
2. Puly: lỗ đúng 5 mm (động cơ) và 8 mm (vitme), cả 2 cùng chuẩn GT2, cùng số răng.
3. Ổ bi KFL08/KP08 lồng vừa vitme.
4. Trục động cơ đo đúng **5 mm**.
5. Chip trên Black Pill ghi **STM32F411CEU6**.

## Sơ đồ nguồn (để nhớ khi lắp)

```
Pin 12V - XT60 - Cầu chì 5A - Chống ngược --+--> 4 x DRV8871 (VM)
                                            +--> TMC2209 (VM) + tụ 100uF
                                            +--> Hạ áp 5V --> Black Pill chân 5V
                                                               +--> 3V3: MPU6050, VIO TMC2209
Pi 4 <-- sạc dự phòng Anker (riêng), hoặc sau này: hạ áp 12V->5V 5A --> Pi 4 + lidar
          Pi 4 --> Pi Camera (CSI, dò line), Webcam USB (QR), Lidar (USB)
GND tất cả nối chung.
```
