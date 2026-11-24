#!/usr/bin/env python3
"""Xac dinh huong LiDAR so voi xe (xe DUNG YEN, khong dieu khien dong co).
    python3 lidar_orient.py base     # ghi khung canh hien tai (CHUA dat vat) -> lidar_base.json
    python3 lidar_orient.py obj      # sau khi DAT VAT: so voi base -> goc + khoang cach cua vat (diem MOI xuat hien)
"""
import json
import math
import os
import sys
import time

import yaml

from lidar import Lidar

os.chdir(os.path.dirname(os.path.abspath(__file__)))
cfg = yaml.safe_load(open('config.yaml'))


def collect(sec=3.0):
    lid = Lidar(cfg['lidar']['port'], cfg['lidar']['baud']).start()
    t0 = time.time()
    while not lid.scan and time.time() - t0 < 6.0:          # cho LiDAR quay on dinh, co vong quet dau tien
        time.sleep(0.05)
    if not lid.scan:
        lid.close()
        sys.exit('LiDAR KHONG co vong quet sau 6 s (goi tot %d, loi %d) - kiem tra day / LiDAR co quay khong' % (lid.good_packets, lid.bad_packets))
    t1, last, pts, n = time.time(), lid.scan_t, [], 0
    while time.time() - t1 < sec:
        if lid.scan_t != last:
            last = lid.scan_t
            pts += lid.scan
            n += 1
        time.sleep(0.02)
    lid.close()
    print('(%d vong quet, cho LiDAR %.1f s)' % (n, t1 - t0))
    return pts


def bins(pts):
    """Khoang cach nho nhat moi 2 do."""
    b = {}
    for a, d in pts:
        k = int(a // 2) * 2
        b[k] = min(b.get(k, 9e9), d)
    return b


mode = sys.argv[1] if len(sys.argv) > 1 else 'base'
pts = collect()
b = bins(pts)
if mode == 'base':
    json.dump({str(k): v for k, v in b.items()}, open('lidar_base.json', 'w'))
    print('DA GHI MOC: %d diem, %d o goc co du lieu' % (len(pts), len(b)))
else:
    base = {int(k): v for k, v in json.load(open('lidar_base.json')).items()}
    new = [(k, d) for k, d in b.items() if d < base.get(k, 9e9) - 80 and 120 < d < 1500]   # gan hon moc >= 8 cm
    if not new:
        print('KHONG THAY VAT MOI (dat vat cach LiDAR 15-150 cm, cao toi mat quet LiDAR)')
        sys.exit()
    # gom cung lien tiep, lay cung co nhieu o nhat
    new.sort()
    groups, cur = [], [new[0]]
    for k, d in new[1:]:
        if k - cur[-1][0] <= 4:
            cur.append((k, d))
        else:
            groups.append(cur); cur = [(k, d)]
    groups.append(cur)
    if len(groups) > 1 and groups[0][0][0] == 0 and groups[-1][-1][0] >= 356:     # noi qua 0 do
        groups[0] = [(k - 360, d) for k, d in groups[-1]] + groups[0]; groups.pop()
    for grp in groups:
        a_c = sum(k + 1 for k, _ in grp) / len(grp) % 360
        print('  nhom: goc %5.0f do (cung %4d..%4d), gan nhat %4.0f mm, %2d o' % (a_c, grp[0][0], grp[-1][0] + 2, min(d for _, d in grp), len(grp)))
    g = max(groups, key=len)
    ang = sum(k + 1 for k, _ in g) / len(g) % 360
    dist = min(d for _, d in g)
    print('VAT MOI: goc LiDAR %.0f do (cung %d..%d), cach LiDAR %.0f mm, %d o goc. Cac nhom khac: %s' % (
        ang, g[0][0], g[-1][0] + 2, dist, len(g), [(x[0][0], len(x)) for x in groups if x is not g]))
