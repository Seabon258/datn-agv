#!/usr/bin/env python3
"""Xem camera TRUC TIEP tren man hinh Pi (qua VNC), co ve ket qua nhan dien line.
Chay trong terminal tren desktop Pi:   cd ~/agv && python3 cam_view.py      (bam q de thoat)
Luu y: khi line_follow.py dang chay thi camera ban -> xem qua dashboard http://<ip-pi>:8080 thay the."""
import cv2
import yaml

from line_vision import LineDetector

cfg = yaml.safe_load(open('config.yaml'))
det = LineDetector(cfg['line'])
cap = cv2.VideoCapture(cfg['camera']['index'], cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg['camera']['width'])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg['camera']['height'])
if not cap.isOpened():
    raise SystemExit('Khong mo duoc camera (co the line_follow.py dang dung camera)')
cv2.namedWindow('AGV camera', cv2.WINDOW_NORMAL)
cv2.resizeWindow('AGV camera', 640, 480)
while True:
    ok, frame = cap.read()
    if not ok:
        print('mat hinh camera'); break
    r = det.process(frame)
    cv2.imshow('AGV camera', r['debug'])
    if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
        break
cap.release()
cv2.destroyAllWindows()
