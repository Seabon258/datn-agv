#!/usr/bin/env python3
"""Xem camera USB va doc ma QR. Bam q de thoat.
Chay: python3 cam_qr_test.py [so_thiet_bi]   (mac dinh 2 = Rapoo tren laptop)"""
import sys, time, cv2

dev = int(sys.argv[1]) if len(sys.argv) > 1 else 2
cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
if not cap.isOpened():
    sys.exit(f"Khong mo duoc /dev/video{dev}")

qr = cv2.QRCodeDetector()
# OpenCV cai bang apt (Ubuntu) thieu thu vien giai ma QR: thu giai 1 ma tu tao de biet
try:
    img = cv2.resize(cv2.QRCodeEncoder.create().encode("test"), (200, 200), interpolation=cv2.INTER_NEAREST)
    qr_ok = qr.detectAndDecode(cv2.copyMakeBorder(img, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255))[0] == "test"
except Exception:
    qr_ok = False
if not qr_ok:
    print("OpenCV nay khong giai ma QR duoc, chi hien hinh. Cai: pip3 install --user opencv-python")
last = ""
t0, n, fps = time.time(), 0, 0.0
while True:
    ok, f = cap.read()
    if not ok:
        break
    text, pts = qr.detectAndDecode(f)[:2] if qr_ok else ("", None)
    if pts is not None and text:
        cv2.polylines(f, [pts.astype(int).reshape(-1, 2)], True, (0, 255, 0), 3)
        cv2.putText(f, text, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        if text != last:
            print("QR:", text)
            last = text
    n += 1
    if time.time() - t0 >= 1:
        fps, n, t0 = n / (time.time() - t0), 0, time.time()
    cv2.putText(f, f"{fps:.0f} fps", (20, f.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
    cv2.imshow("Rapoo C260 - bam q de thoat", f)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
