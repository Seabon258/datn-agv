#!/usr/bin/env python3
"""Hieu chinh "hang anh -> khoang cach that tren san" cho camera (lam 1 LAN, khi doi vi tri/goc camera thi lam lai).

Chuan bi: dan 1 doan bang keo XANH nam NGANG (vuong goc huong xe) phia truoc xe, xe dat THANG.
(Dung duoc ca hinh chu T: than doc chay ve phia xe + thanh ngang o dau - code chi do thanh ngang.)
Do khoang cach tu TAM XE (giua 4 banh) toi GIUA be rong doan bang keo, vd 60 cm. Roi chay:

    python3 ground_calib.py 60 25         # 60 = khoang cach (cm) tam xe -> giua bang keo, 25 = chieu dai doan bang keo (cm)

Xe chay thang cham (gyro giu huong) ve phia bang keo, ghi (hang anh, khoang cach con lai tu encoder) moi khung,
fit duong cong va luu ground_calib.json. line_follow.py tu dung file nay de biet nga re con cach tam xe bao xa.
Dung som: tao file STOP. Xe tu dung khi bang keo ra khoi day anh + 3 cm, hoac di qua d0 + 5 cm, hoac 40 s.
"""
import json
import os
import sys
import time

import cv2
import numpy as np
import yaml

from line_vision import LineDetector
from stm32_link import Stm32Link

HERE = os.path.dirname(os.path.abspath(__file__))


def tape_row(det, frame, y_from):
    """(hang 0..1, be rong pixel hoac None neu 2 dau bang keo cham mep anh) cua doan bang keo ngang, hoac None."""
    h, w = frame.shape[:2]
    y0 = int(h * y_from)
    m = det.mask(frame[y0:, :])
    rows = (m > 0).sum(axis=1)
    mx = rows.max() if rows.size else 0
    if mx < w * 0.15:                                       # khong co hang nao du rong -> chua thay thanh ngang
        return None
    # chu T: thanh ngang 25 cm rong gap nhieu lan than doc -> lay DAI HANG LIEN NHAU quanh hang rong nhat
    nz = rows[rows > 0]
    if mx < 2.5 * float(np.median(nz)):                   # khong noi bat so voi line thuong -> chi la than doc
        return None
    k = int(np.argmax(rows))
    lo = hi = k
    while lo > 0 and rows[lo - 1] >= 0.5 * mx:
        lo -= 1
    while hi < len(rows) - 1 and rows[hi + 1] >= 0.5 * mx:
        hi += 1
    wide = np.arange(lo, hi + 1)
    if wide.size < 2:
        return None
    sub = m[wide.min(): wide.max() + 1]
    cols = np.nonzero(sub.sum(axis=0) >= 0.5 * len(wide))[0]   # cot thuoc thanh ngang (bo than doc mo nhat)
    width = None
    if cols.size and cols.min() > 2 and cols.max() < w - 3:   # thay du 2 dau -> do duoc ti le mm/pixel
        width = float(cols.max() - cols.min() + 1)
    return float((y0 + wide.mean()) / h), width


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    d0_mm = float(sys.argv[1]) * 10.0
    tape_mm = float(sys.argv[2]) * 10.0 if len(sys.argv) > 2 else None
    os.chdir(HERE)
    cfg = yaml.safe_load(open('config.yaml'))
    geo = cfg.get('geometry', {})
    mm_per_count = 3.14159 * geo.get('wheel_diam_mm', 80) / geo.get('enc_cpr', 3960)
    det = LineDetector(cfg['line'])
    cap = cv2.VideoCapture(cfg['camera']['index'], cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg['camera']['width'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg['camera']['height'])
    for _ in range(15):
        cap.read()
    if os.path.exists('STOP'):
        os.remove('STOP')
    link = Stm32Link(cfg['serial_port'])
    time.sleep(0.5)
    if not link.telem:
        sys.exit('Khong co telemetry STM32')
    enc0 = list(link.telem['enc'])
    os.makedirs('calib_dbg', exist_ok=True)
    for fn in os.listdir('calib_dbg'):
        os.remove(os.path.join('calib_dbg', fn))
    csv = open('calib_dbg/points.csv', 'w')
    csv.write('t,y,width_px,trav_mm,dist_mm\n')
    nfr = 0
    pts, seen_bottom, gone, t0 = [], False, 0, time.time()
    stop_at = None
    try:
        while time.time() - t0 < 40 and not os.path.exists('STOP'):
            ok, f = cap.read()
            if not ok:
                break
            trav = sum(a - b for a, b in zip(link.telem['enc'], enc0)) / 4 * mm_per_count
            tr = tape_row(det, f, 0.30)
            y = tr[0] if tr else None
            csv.write('%.2f,%s,%s,%.0f,%.0f\n' % (time.time() - t0, '%.3f' % y if y else '', tr[1] if tr and tr[1] else '',
                                                 trav, d0_mm - trav))
            if nfr % 10 == 0:
                cv2.imwrite('calib_dbg/f%03d_%dmm.jpg' % (nfr, d0_mm - trav), f)
            nfr += 1
            if y is not None:
                pts.append((y, d0_mm - trav, tr[1]))
                gone = 0
                seen_bottom = seen_bottom or y > 0.85
            else:
                gone += 1
            if seen_bottom and gone >= 5 and stop_at is None:
                stop_at = trav + 30.0                       # ra khoi day anh: chay them 3 cm cho chac roi dung
            if (stop_at is not None and trav >= stop_at) or trav >= d0_mm + 50:
                break
            link.drive(500, 0, 0, hold=True)
    finally:
        csv.close()
        link.stop()
        time.sleep(0.4)
        link.close()
        cap.release()

    if len(pts) < 15:
        sys.exit('Qua it diem (%d). Kiem tra bang keo nam ngang, nam trong anh, d0 du xa (>= 50 cm).' % len(pts))
    ys = np.array([p[0] for p in pts])
    ds = np.array([p[1] for p in pts])
    coef = np.polyfit(ys, ds, 3)                            # khoang cach (mm) theo hang anh
    resid = ds - np.polyval(coef, ys)
    # ti le ngang: mm tren san cho 1 pixel, theo hang anh (tu chieu dai bang keo da biet)
    sc = [(p[0], tape_mm / p[2]) for p in pts if tape_mm and p[2]]
    scale = None
    if len(sc) >= 8:
        scale = [float(c) for c in np.polyfit([s[0] for s in sc], [s[1] for s in sc], 2)]
    out = {'coef': [float(c) for c in coef], 'scale_coef': scale, 'scale_points': len(sc),
           'width': int(cfg['camera']['width']), 'y_min': float(ys.min()), 'y_max': float(ys.max()),
           'd_bottom_mm': float(np.polyval(coef, 1.0)), 'points': len(pts), 'rms_mm': float(np.sqrt((resid ** 2).mean())),
           'd0_mm': d0_mm, 'date': time.strftime('%Y-%m-%d %H:%M')}
    mono = np.polyval(coef, ys.min()) > np.polyval(coef, ys.max())
    if not mono or out['rms_mm'] > 25:
        json.dump(out, open('ground_calib_LOI.json', 'w'), indent=1)
        print('KHONG LUU: ket qua khong hop ly (giam dan: %s, sai so %.1f mm). Xem calib_dbg/' % (mono, out['rms_mm']))
    else:
        json.dump(out, open('ground_calib.json', 'w'), indent=1)
        print('DA LUU ground_calib.json: %d diem, sai so fit %.1f mm' % (out['points'], out['rms_mm']))
    for yy in (0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0):
        print('  hang anh %.1f -> cach tam xe %5.0f mm%s%s' % (yy, np.polyval(coef, yy),
              ('' if scale is None else ', 1 pixel = %.2f mm ngang' % np.polyval(scale, yy)),
              '' if out['y_min'] - 0.02 <= yy <= out['y_max'] + 0.02 else '  (ngoai khoang do, ngoai suy)'))


if __name__ == '__main__':
    main()
