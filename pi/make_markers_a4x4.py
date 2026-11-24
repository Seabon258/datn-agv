#!/usr/bin/env python3
"""In ma ArUco: moi to A4 co 4 ma (luoi 2x2, cach deu), MAT SAU ghi tac dung cua tung ma.

    python3 make_markers_a4x4.py      # -> ../aruco/aruco_4ma_2mat.pdf (+ anh tung mat)

PDF xep: mat truoc 1, mat sau 1, mat truoc 2, mat sau 2, ...
In 2 mat, LAT THEO CANH DAI (long edge / "Lat theo canh dai"), ti le 100% (Actual size), KHONG "Fit to page".
Mat sau da dao trai/phai san, nen chu o mat sau nam dung sau ma tuong ung.
"""
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from route_map import RouteMap

DPI = 300
MM = DPI / 25.4
W, H = int(210 * MM), int(297 * MM)
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONTB = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'aruco')

rm = RouteMap(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'station_map.yaml'))
size_mm = float(rm.raw.get('placement', {}).get('marker_size_m', 0.05)) * 1000
dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, rm.dictionary))
ACT = {'HOME': 'Về vị trí đỗ / kết thúc', 'LIFT_UP': 'NÂNG hàng (lấy hàng)', 'LIFT_DOWN': 'HẠ hàng (trả hàng)'}


def info(i):
    """(tieu de, loai, [cac dong tac dung], cach dan)"""
    kind, e = rm.kind(i)
    if kind == 'station':
        lines = ['Xe DỪNG tại đây nếu đây là trạm', 'đích của nhiệm vụ, rồi:', '→ ' + ACT.get(e['action'], e['action'])]
        if e.get('cargo_type'):
            lines.append('Khu nhận hàng loại %s' % e['cargo_type'])
        if e.get('note'):
            lines.append('(%s)' % e['note'])
        lines.append('Không phải đích: xe đi qua.')
        return 'TRẠM %s' % e['name'], 'Trạm dừng (0–9)', lines, 'Dán ngay tại chỗ xe phải dừng.'
    if kind == 'pre_station':
        st = rm.station_by_id[int(e['station_id'])]['name']
        d = float(e.get('pre_distance_m', 0.3)) * 100
        return 'BÁO TRƯỚC %s' % st, 'Báo trước trạm (10–19)', [
            'Sắp tới trạm %s:' % st, 'xe GIẢM TỐC, chuẩn bị dừng.',
            'Không thấy mã trạm thì xe tự', 'dừng sau %.0f cm (encoder).' % d], 'Dán TRƯỚC trạm %s %.0f cm.' % (st, d)
    if kind == 'speed':
        lines = ['Đặt tốc độ %.2f m/s.' % e['speed']]
        if e.get('max_distance_m'):
            lines += ['Tự về NORMAL sau %.0f m' % e['max_distance_m'], 'hoặc khi gặp rẽ / trạm.']
        if e.get('note'):
            lines.append('(%s)' % e['note'])
        where = 'Đầu đoạn thẳng dài ≥ 1 m.' if e['name'] == 'SPEED_FAST' else 'Đầu đoạn cần đổi tốc độ.'
        return e['name'], 'Tốc độ (20–29)', lines, where
    if kind == 'turn':
        txt = {'left': 'RẼ TRÁI', 'right': 'RẼ PHẢI', 'uturn': 'QUAY ĐẦU 180°'}[e['direction']]
        lines = ['Phía trước có chỗ ' + txt + '.', 'Xe chậm lại và rẽ đúng hướng', 'này (ưu tiên hơn camera đoán).']
        if e['direction'] == 'uturn':
            lines = ['Cuối tuyến cụt: xe QUAY ĐẦU', '180° (gyro) rồi bám line về.']
        return e['name'], 'Rẽ (30–34)', lines, 'Dán TRƯỚC chỗ rẽ 30–40 cm.'
    if kind == 'junction':
        tr = {'left': 'trái', 'right': 'phải', 'straight': 'thẳng'}
        lines = ['Ngã ba/ngã tư: hướng đi theo', 'trạm đích hiện tại:'] + [
            '  %s → %s' % (k if k != 'default' else 'khác', tr[v]) for k, v in (e.get('routes') or {}).items()]
        return 'NGÃ RẼ %s' % e['name'], 'Ngã rẽ (35–39)', lines, 'Dán TRƯỚC ngã rẽ 30–40 cm.'
    if kind == 'rule':
        n = e['name']
        d = {'NO_DETOUR_ZONE': ['Vào LỐI HẸP: gặp vật cản thì', 'chỉ DỪNG CHỜ, không né.'],
             'DETOUR_ALLOWED': ['HẾT lối hẹp: được phép', 'né vật cản trở lại.'],
             'STOP_AND_WAIT': ['Giao cắt lối người đi:', 'DỪNG %.0f giây, kiểm tra LiDAR,' % e.get('wait_s', 2), 'rồi đi tiếp.'],
             'END_OF_LINE': ['CUỐI TUYẾN: xe dừng hẳn.', 'Báo LỖI nếu chưa tới trạm đích.'],
             'TEST_MARKER': ['Mã thử camera.', 'Không dán trên tuyến chạy.']}.get(n, [e.get('note', '')])
        where = {'NO_DETOUR_ZONE': 'Đầu lối hẹp.', 'DETOUR_ALLOWED': 'Cuối lối hẹp.', 'STOP_AND_WAIT': 'Trước chỗ giao cắt.',
                 'END_OF_LINE': 'Cuối tuyến.', 'TEST_MARKER': 'Chỉ cầm trước camera.'}.get(n, '')
        return n, 'Luật khu vực (40–49)', d, where
    return 'ID %d' % i, '?', [], ''


def cell_box(k):
    """O thu k (0..3) tren mat TRUOC: (x0, y0, x1, y1) pixel, luoi 2x2 chia deu trang."""
    col, row = k % 2, k // 2
    return col * W // 2, row * H // 2, (col + 1) * W // 2, (row + 1) * H // 2


def dashed(dr, a, b, color=150, dash=int(4 * MM)):
    (x0, y0), (x1, y1) = a, b
    n = int(max(abs(x1 - x0), abs(y1 - y0)) / dash)
    for s in range(0, n, 2):
        dr.line([(x0 + (x1 - x0) * s / n, y0 + (y1 - y0) * s / n),
                 (x0 + (x1 - x0) * (s + 1) / n, y0 + (y1 - y0) * (s + 1) / n)], fill=color, width=3)


def front(ids, pn, total):
    img = Image.new('L', (W, H), 255)
    dr = ImageDraw.Draw(img)
    f_id, f_sm = ImageFont.truetype(FONTB, int(5 * MM)), ImageFont.truetype(FONT, int(3 * MM))
    side = int(size_mm * MM)
    for k, i in enumerate(ids):
        x0, y0, x1, y1 = cell_box(k)
        cx, cy = (x0 + x1) // 2, (y0 + y1) // 2                 # ma nam CHINH GIUA moi phan tu (cat 4 manh deu)
        mk = cv2.aruco.generateImageMarker(dic, i, side) if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(dic, i, side)
        img.paste(Image.fromarray(mk), (cx - side // 2, cy - side // 2))
        q = int(10 * MM)                                             # vien trang toi thieu quanh ma (khong ve khung)
        title = info(i)[0]
        t = 'ID %d · %s' % (i, title)
        tw = dr.textlength(t, font=f_id)
        dr.text((cx - tw / 2, cy + side // 2 + q + int(4 * MM)), t, font=f_id, fill=0)
    dashed(dr, (W // 2, 0), (W // 2, H))
    dashed(dr, (0, H // 2), (W, H // 2))
    s = 'Mặt trước %d/%d · in 100%% · phần đen = %d mm · cắt theo 2 nét đứt thành 4 mảnh bằng nhau' % (pn, total, size_mm)
    dr.text(((W - dr.textlength(s, font=f_sm)) / 2, H - int(6 * MM)), s, font=f_sm, fill=110)
    return img


def back(ids, pn, total):
    img = Image.new('L', (W, H), 255)
    dr = ImageDraw.Draw(img)
    f_t, f_k, f_b, f_sm = (ImageFont.truetype(FONTB, int(6 * MM)), ImageFont.truetype(FONT, int(3.6 * MM)),
                           ImageFont.truetype(FONT, int(4.2 * MM)), ImageFont.truetype(FONT, int(3 * MM)))
    for k, i in enumerate(ids):
        # in 2 mat lat canh dai: o (cot, hang) mat truoc nam o (1-cot, hang) mat sau
        col, row = k % 2, k // 2
        x0, y0 = (1 - col) * W // 2, row * H // 2
        title, kind, lines, where = info(i)
        x, y = x0 + int(12 * MM), y0 + int(18 * MM)
        dr.text((x, y), 'ID %d' % i, font=ImageFont.truetype(FONTB, int(11 * MM)), fill=0)
        y += int(14 * MM)
        dr.text((x, y), title, font=f_t, fill=0)
        y += int(9 * MM)
        dr.text((x, y), kind, font=f_k, fill=90)
        y += int(9 * MM)
        for l in lines:
            dr.text((x, y), l, font=f_b, fill=0)
            y += int(6.2 * MM)
        if where:
            y += int(3 * MM)
            dr.text((x, y), 'Vị trí dán: ' + where, font=f_k, fill=60)
            y += int(5.5 * MM)
            dr.text((x, y), 'Luôn dán BÊN PHẢI line, cách mép 6 cm.', font=f_k, fill=60)
    dashed(dr, (W // 2, 0), (W // 2, H))
    dashed(dr, (0, H // 2), (W, H // 2))
    s = 'Mặt sau %d/%d · in 2 mặt, LẬT THEO CẠNH DÀI' % (pn, total)
    dr.text(((W - dr.textlength(s, font=f_sm)) / 2, H - int(6 * MM)), s, font=f_sm, fill=110)
    return img


if __name__ == '__main__':
    ids = sorted(rm.by_id)
    sheets = [ids[k:k + 4] for k in range(0, len(ids), 4)]
    os.makedirs(OUT, exist_ok=True)
    pages = []
    for n, chunk in enumerate(sheets, 1):
        f, b = front(chunk, n, len(sheets)), back(chunk, n, len(sheets))
        f.save(os.path.join(OUT, 'to%d_mat_truoc.png' % n))
        b.save(os.path.join(OUT, 'to%d_mat_sau.png' % n))
        pages += [f, b]
    pdf = os.path.join(OUT, 'aruco_4ma_2mat.pdf')
    pages[0].save(pdf, save_all=True, append_images=pages[1:], resolution=float(DPI))
    print('Da tao %s: %d to A4 (%d trang PDF), %d ma, 4 ma/to, canh den %d mm'
          % (os.path.abspath(pdf), len(sheets), len(pages), len(ids), size_mm))
