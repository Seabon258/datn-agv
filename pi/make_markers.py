#!/usr/bin/env python3
"""Tao cac trang A4 de IN toan bo ma ArUco trong station_map.yaml (schema v2).

    python3 make_markers.py          # -> markers_page1.png, markers_page2.png, ... (300 dpi)
In o ti le 100% (Actual size / Ty le thuc), KHONG "fit to page". In giay mo hoac can mo (san bong gay loa).
Sau khi in: do canh phan DEN bang thuoc, phai dung placement.marker_size_m (mac dinh 5 cm).
"""
import cv2
import numpy as np

from route_map import RouteMap

DPI = 300
MM = DPI / 25.4
rm = RouteMap('station_map.yaml')
size_mm = float(rm.raw.get('placement', {}).get('marker_size_m', 0.05)) * 1000
dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, rm.dictionary))


def label(i):
    kind, e = rm.kind(i)
    if kind == 'station':
        return 'TRAM %s' % e['name'], e['action']
    if kind == 'pre_station':
        return 'BAO TRUOC %s' % rm.station_by_id[int(e['station_id'])]['name'], 'dan truoc tram %.0f cm' % (e.get('pre_distance_m', 0.3) * 100)
    if kind == 'speed':
        return e['name'], '%.2f m/s' % e['speed']
    if kind == 'turn':
        return e['name'], 'dan truoc 30-40 cm'
    if kind == 'junction':
        return 'NGA RE %s' % e['name'], 'dan truoc 30-40 cm'
    return e['name'], ''


side = int(size_mm * MM)
quiet = int(10 * MM)                                         # vien trang >= 1 cm (bat buoc)
cell_w, cell_h = side + 2 * quiet, side + 2 * quiet + int(16 * MM)
W, H = int(210 * MM), int(297 * MM)
cols = max(1, (W - int(20 * MM)) // (cell_w + int(4 * MM)))
rows = max(1, (H - int(20 * MM)) // (cell_h + int(4 * MM)))
ids = sorted(rm.by_id)
pages = [ids[k:k + cols * rows] for k in range(0, len(ids), cols * rows)]
for pn, chunk in enumerate(pages, 1):
    page = np.full((H, W), 255, np.uint8)
    for k, i in enumerate(chunk):
        x = int(10 * MM) + (k % cols) * (cell_w + int(4 * MM))
        y = int(10 * MM) + (k // cols) * (cell_h + int(4 * MM))
        img = cv2.aruco.generateImageMarker(dic, i, side) if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(dic, i, side)
        page[y + quiet:y + quiet + side, x + quiet:x + quiet + side] = img
        cv2.rectangle(page, (x, y), (x + cell_w, y + side + 2 * quiet), 190, 2)       # duong cat (giu vien trang)
        t1, t2 = label(i)
        cv2.putText(page, 'ID %d  %s' % (i, t1), (x + 4, y + side + 2 * quiet + int(6 * MM)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2)
        cv2.putText(page, t2, (x + 4, y + side + 2 * quiet + int(12 * MM)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2)
    cv2.putText(page, 'In ti le 100%% - canh phan den = %d mm - trang %d/%d' % (size_mm, pn, len(pages)),
                (int(10 * MM), H - int(6 * MM)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2)
    cv2.imwrite('markers_page%d.png' % pn, page)
print('Da tao %d trang (markers_page1..%d.png): %d ma, canh %d mm, %d ma/trang. In ti le 100%%.'
      % (len(pages), len(pages), len(ids), size_mm, cols * rows))
