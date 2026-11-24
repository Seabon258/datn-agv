#!/usr/bin/env python3
"""Nhan dien line den / xanh duong bang OpenCV (khong can train model).

    det = LineDetector(cfg['line'])
    r = det.process(frame_bgr)   # r['found'], r['error'] (-1 trai .. +1 phai), r['cross'], r['debug']

Chay thu:  python3 line_vision.py [anh.jpg | so_camera]   -> in do lech, luu line_debug.jpg
"""
import sys

import cv2
import numpy as np


def ground_dist_mm(g, y):
    """Hang anh y (0..1) -> mm tu tam xe toi diem do tren san. Ngoai khoang da do: noi THANG theo do doc o bien
    (khong dung da thuc bac 3 ngoai suy - de cong sai)."""
    lo, hi = g['y_min'], g['y_max']
    yc = np.clip(y, lo, hi)
    d = np.polyval(g['coef'], yc)
    der = np.polyval(np.polyder(g['coef']), np.where(y > hi, hi, lo))
    return np.where((y > hi) | (y < lo), d + der * (y - yc), d)


def ground_scale(g, y):
    """mm tren san cho 1 pixel ngang o hang y. Ngoai khoang da do: giu gia tri o bien."""
    return np.polyval(g['scale_coef'], np.clip(y, g['y_min'], g['y_max']))


class LineDetector:
    def __init__(self, cfg, gcal=None):
        self.cfg = cfg
        # hieu chinh mat san (ground_calib.json): doi pixel -> mm that de co goc/lech THAT (khong meo phoi canh)
        self.g = gcal if gcal and gcal.get('scale_coef') else None
        self._since_corner, self._last_dir = 999, 0      # so khung tu lan cuoi thay nga re dang tin, huong do
        k = int(cfg.get('morph_kernel', 7))
        self.kernel = np.ones((k, k), np.uint8)

    def _blue(self, hsv):
        return cv2.inRange(hsv, np.array(self.cfg['blue_hsv_lo']), np.array(self.cfg['blue_hsv_hi']))

    def _black(self, hsv):
        """Line den = TOI va KHONG MAU (bao hoa thap). Nguong toi co dinh (black_v_max) hoac Otsu + kiem tra tuong phan."""
        v = hsv[:, :, 2]
        t = self.cfg.get('black_thresh', 0)
        if t > 0:
            _, m = cv2.threshold(v, t, 255, cv2.THRESH_BINARY_INV)
        else:
            otsu, m = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
            dark, light = v[m > 0], v[m == 0]
            # san tron (chi co nhieu): Otsu van chia doi -> doi line phai toi hon nen ro rang
            if dark.size == 0 or light.size == 0 or float(light.mean()) - float(dark.mean()) < self.cfg.get('min_contrast', 40):
                m[:] = 0
        m[hsv[:, :, 1] > self.cfg.get('black_max_sat', 90)] = 0       # co mau (xanh, vang go...) -> khong phai den
        return m

    def mask(self, roi):
        """color: blue | black | blue_black (ca 2 loai bang keo tren cung tuyen)."""
        hsv = cv2.cvtColor(cv2.GaussianBlur(roi, (5, 5), 0), cv2.COLOR_BGR2HSV)   # lam muot nhieu camera
        col = self.cfg.get('color', 'blue')
        if col == 'blue':
            m = self._blue(hsv)
        elif col == 'black':
            m = self._black(hsv)
        else:
            m = cv2.bitwise_or(self._blue(hsv), self._black(hsv))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.kernel)
        return cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.kernel)

    def process(self, frame):
        h, w = frame.shape[:2]
        y0 = int(h * self.cfg.get('roi_top', 0.55))
        y1 = int(h * self.cfg.get('roi_bottom', 1.0))
        roi = frame[y0:y1, :]
        m = self.mask(roi)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = [c for c in cnts if cv2.contourArea(c) >= self.cfg.get('min_area', 300)]
        out = {'found': False, 'error': 0.0, 'angle': 0.0, 'cross': False, 'area': 0, 'debug': frame.copy(),
               'corner': False, 'corner_y': 0.0, 'corner_dir': 0, 'corner_score': 0.0, 'near_error': None,
               'g_angle': None, 'g_offset_mm': None, 'g_dist_mm': None, 'g_pts': None}
        dbg = out['debug']
        cv2.rectangle(dbg, (0, y0), (w - 1, y1 - 1), (255, 255, 0), 1)
        if not cnts:
            cv2.putText(dbg, 'MAT LINE', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return out
        c = max(cnts, key=cv2.contourArea)
        mo = cv2.moments(c)
        cx = mo['m10'] / mo['m00']
        x, y, bw, bh = cv2.boundingRect(c)
        # goc line: so tam nua tren va nua duoi cua vung line trong ROI
        rh = roi.shape[0]
        top = m[: rh // 2, :]
        bot = m[rh // 2:, :]
        def col_center(part):
            xs = np.nonzero(part)[1]
            return xs.mean() if xs.size else cx
        angle = float(np.degrees(np.arctan2(col_center(top) - col_center(bot), rh / 2)))
        out.update(found=True, error=float((cx - w / 2) / (w / 2)) - self.cfg.get('center_offset', 0.0),
                   angle=angle, area=int(mo['m00']),
                   x0=float((x - w / 2) / (w / 2)), x1=float((x + bw - w / 2) / (w / 2)),
                   top_y=float((y0 + y) / h), roi_top=float(y0 / h),
                   cross=bw > w * self.cfg.get('cross_width_ratio', 0.6))
        # ----- nga cua: cac hang ma line rong bat thuong (nhanh ngang de len line doc) -----
        lm = np.zeros_like(m)
        cv2.drawContours(lm, [c], -1, 255, -1)          # chi xet vung line lon nhat
        lm &= m
        rows = (lm > 0).sum(axis=1)
        # be rong "than line" = trung vi be rong cac hang (than doc chiem da so hang)
        nz = rows[rows > 0]
        stem_w = float(np.median(nz)) if nz.size else 0.0
        # hang "nhanh ngang": rong hon han than line (khong chi rong do phoi canh / vet choi)
        wide = max(w * self.cfg.get('corner_row_ratio', 0.30), stem_w * self.cfg.get('corner_vs_stem', 2.2))
        arm = rows > wide
        stem_rows = [i for i in range(rh // 2, rh) if 0 < rows[i] and not arm[i]]
        if stem_rows:
            xs = np.nonzero(lm[stem_rows, :])[1]
            stem_x = float(xs.mean())
            out['near_error'] = (stem_x - w / 2) / (w / 2)
        else:
            stem_x = cx
        # ----- goc nhin tu tren xuong: fit line THAT tren san tu tam cac hang than line -----
        if self.g is not None:
            srows = [i for i in range(rh) if 0 < rows[i] and not arm[i]]
            if len(srows) >= 6:
                yn = np.array([(y0 + i) / h for i in srows])
                xc = np.array([np.nonzero(lm[i])[0].mean() for i in srows])
                gy = ground_dist_mm(self.g, yn)                              # mm phia truoc tam xe
                gx = (xc - w / 2) * ground_scale(self.g, yn)                 # mm sang phai
                a, b = np.polyfit(gy, gx, 1)                                 # x = a*y + b
                out.update(g_angle=float(np.degrees(np.arctan(a))), g_offset_mm=float(b),
                           g_dist_mm=float(gy.min()),
                           g_pts=[(float(p), float(-q)) for p, q in zip(gy, gx)])   # (mm toi, mm TRAI) tam line tren san
                cv2.putText(dbg, 'that: goc %+.0f lech %+.0fmm' % (out['g_angle'], b), (10, 58),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
        if arm.sum() >= self.cfg.get('corner_min_rows', 4):
            ai = np.nonzero(arm)[0]
            # nhanh phai VUON XA ra mot phia so voi than line (>= arm_min_ratio * be ngang anh)
            lefts, rights = [], []
            for i in ai:
                xr = np.nonzero(lm[i])[0]
                lefts.append(stem_x - xr.min()); rights.append(xr.max() - stem_x)
            le, ri = float(np.mean(lefts)), float(np.mean(rights))
            reach = w * self.cfg.get('arm_min_ratio', 0.20)
            if max(le, ri) - stem_w / 2 >= reach:
                d = 1 if le >= ri else -1
                out.update(corner=True, corner_y=float((y0 + ai.mean()) / h), corner_dir=d,
                           corner_score=float(len(ai)) * abs(le - ri) / w)
                cy = int(y0 + ai.mean())
                cv2.line(dbg, (0, cy), (w - 1, cy), (0, 0, 255), 2)
                cv2.putText(dbg, 'NGA CUA %s' % ('TRAI' if d > 0 else 'PHAI'), (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # ----- sat nga re: chi con thay NHANH NGANG (thanh rong), than doc da khuat duoi xe -----
        # Chi dung de GIU nga re khong bi mat; HUONG lay theo lan nhan nga re dang tin gan nhat.
        recent = self._since_corner <= self.cfg.get('bar_memory_frames', 20)
        if not out['corner'] and bw > w * self.cfg.get('bar_min_width', 0.40) and \
                (bh < rh * self.cfg.get('bar_max_height', 0.45) or recent):
            ref = self.cfg.get('_last_stem_x', w / 2)
            le, ri = ref - x, (x + bw) - ref
            if recent or (max(le, ri) >= w * self.cfg.get('arm_min_ratio', 0.20) and abs(le - ri) >= w * 0.10):
                d = self._last_dir if recent and self._last_dir else (1 if le > ri else -1)
                out.update(corner=True, corner_y=float((y0 + y + bh / 2) / h), corner_dir=d, corner_score=0.0)
                cv2.putText(dbg, 'NGA CUA %s (thanh ngang)' % ('TRAI' if d > 0 else 'PHAI'), (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        if out['near_error'] is not None and not out['corner']:
            self.cfg['_last_stem_x'] = stem_x                # nho vi tri than line gan nhat (de doan huong thanh ngang)
        # ----- nga cua kieu "chu T/L": than line ket thuc, ngay canh dau mut co line chay ngang ra mot phia -----
        if not out['corner']:
            top = y                                       # hang tren cung cua than line (trong ROI)
            if top > rh * 0.05:                           # dau mut nam trong ROI (khong cham mep tren)
                band = m[max(0, top - int(rh * 0.15)): min(rh, top + int(rh * 0.15)), :]
                half = max(stem_w, 8.0)
                lcols = np.nonzero(band[:, : max(0, int(stem_x - half))].any(axis=0))[0]
                rcols = np.nonzero(band[:, min(w, int(stem_x + half)):].any(axis=0))[0]
                le = (stem_x - half - lcols.min()) if lcols.size else 0.0
                ri = float(rcols.max()) if rcols.size else 0.0
                reach = w * self.cfg.get('arm_min_ratio', 0.20)
                if max(le, ri) >= reach and abs(le - ri) >= reach * 0.5:
                    d = 1 if le > ri else -1
                    out.update(corner=True, corner_y=float((y0 + top) / h), corner_dir=d,
                               corner_score=abs(le - ri) / w)
                    cv2.line(dbg, (0, y0 + top), (w - 1, y0 + top), (0, 0, 255), 2)
                    cv2.putText(dbg, 'NGA CUA %s (dau mut)' % ('TRAI' if d > 0 else 'PHAI'), (10, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        if out['corner'] and out['corner_score'] > 0:
            self._since_corner, self._last_dir = 0, out['corner_dir']
        else:
            self._since_corner += 1
        # ve vien da lam tron (bo rang cua do mep bang keo phan sang)
        cs = cv2.approxPolyDP(c, self.cfg.get('draw_eps', 3.0), True)
        cv2.drawContours(dbg[y0:y1], [cs], -1, (0, 255, 0), 2)
        cv2.line(dbg, (w // 2, y0), (w // 2, y1 - 1), (255, 0, 0), 2)          # xanh duong: huong xe (giua anh)
        cv2.line(dbg, (int(cx), y0), (int(cx), y1 - 1), (0, 0, 255), 2)        # do: tam line
        cv2.arrowedLine(dbg, (w // 2, y1 - 6), (int(cx), y1 - 6), (0, 0, 255), 2, tipLength=0.3)  # mui ten: huong can lai
        txt = 'lech %+.2f goc %+.0f' % (out['error'], angle) + (' DAU +' if out['cross'] else '')
        cv2.putText(dbg, txt, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        return out


def _load_cfg():
    import yaml
    with open('config.yaml') as f:
        return yaml.safe_load(f)


if __name__ == '__main__':
    cfg = _load_cfg()
    det = LineDetector(cfg['line'])
    src = sys.argv[1] if len(sys.argv) > 1 else str(cfg['camera']['index'])
    if src.isdigit() or src.startswith('/dev/'):
        cap = cv2.VideoCapture(int(src) if src.isdigit() else src, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg['camera']['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg['camera']['height'])
        for _ in range(10):                 # bo vai khung dau cho camera tu chinh sang
            ok, frame = cap.read()
        cap.release()
        if not ok:
            sys.exit('Khong doc duoc camera %s' % src)
    else:
        frame = cv2.imread(src)
    r = det.process(frame)
    print('thay line: %s | lech %+.2f | goc %+.1f do | dau +: %s | dien tich %d' %
          (r['found'], r['error'], r['angle'], r['cross'], r['area']))
    cv2.imwrite('line_debug.jpg', r['debug'])
    print('da luu line_debug.jpg')
