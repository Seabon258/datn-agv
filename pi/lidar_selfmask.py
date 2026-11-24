#!/usr/bin/env python3
"""Hoc "mat na than xe" cho LiDAR: xe DUNG YEN, xung quanh SAT xe (trong self_mask_range_mm) TRONG.
Ghi lai khoang cach nho nhat moi 1 do trong vong self_mask_range_mm quanh LiDAR -> lidar_selfmask.json.
Luc chay, obstacle.py bo cac diem <= (mat na + self_mask_tol_mm) o cung goc (banh, khung, gia camera, tru nang...).
    python3 lidar_selfmask.py
Lam lai moi khi gan them / doi vi tri bo phan tren xe."""
import json
import os
import time

import yaml

from lidar import Lidar

os.chdir(os.path.dirname(os.path.abspath(__file__)))
cfg = yaml.safe_load(open('config.yaml'))
rng = cfg['obstacle'].get('self_mask_range_mm', 300)
lid = Lidar(cfg['lidar']['port'], cfg['lidar']['baud']).start()
if not lid.wait_ready(3, 3):
    raise SystemExit('LiDAR khong co du lieu')
mask, last, t0, n = {}, None, time.time(), 0
while time.time() - t0 < 4.0:
    if lid.scan_t != last:
        last = lid.scan_t
        n += 1
        for a, d in lid.scan:
            if d <= rng:
                k = int(round(a)) % 360
                mask[k] = max(mask.get(k, 0), d)        # lay xa nhat cua than xe o goc do (an toan khi loc)
    time.sleep(0.02)
lid.close()
json.dump({'range_mm': rng, 'bins': {str(k): v for k, v in sorted(mask.items())}, 'scans': n,
           'date': time.strftime('%Y-%m-%d %H:%M')}, open('lidar_selfmask.json', 'w'), indent=0)
print('Da luu lidar_selfmask.json: %d goc co bo phan xe trong vong %d mm (%d vong quet)' % (len(mask), rng, n))
