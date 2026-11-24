#!/usr/bin/env python3
"""Camera obstacle detector (bu cho LiDAR: vat DEN / BONG / THAP ma LiDAR 2D bo sot). Pure OpenCV, no ROS.

Idea: learn the FLOOR colour (Lab) from the first frames (outside the line). Any large region in the car's
corridor (car width projected with ground_calib) that is far from the floor colour and NOT the thin line is an
obstacle. Its nearest row -> real distance (ground_calib). Also: image suddenly dark/covered -> treat as blocked.

    co = CamObstacle(cfg['cam_obstacle'], gcal, car_width_mm, car_length_mm)
    st = co.update(frame_bgr, line_mask_full)   # {'zone': CLEAR|SLOW|STOP, 'front_mm', 'covered', 'ready'}
"""
import cv2
import numpy as np


class CamObstacle:
    def __init__(self, cfg, gcal, car_w_mm, car_l_mm):
        self.cfg, self.g = cfg, gcal
        self.half_w, self.front = car_w_mm / 2.0, car_l_mm / 2.0
        self.floor_mean = None
        self.floor_std = None
        self._learn = []
        self._hits = 0
        self.zone, self.front_mm = 'CLEAR', None
        self.last_mask = None

    @property
    def ready(self):
        return self.floor_mean is not None and self.g is not None

    def _dist(self, lab):
        d = (lab.astype(np.float32) - self.floor_mean) / self.floor_std
        w = np.array(self.cfg.get('lab_weights', [0.5, 1.0, 1.0]), np.float32)   # L it tin (bong do, phan chieu)
        return np.sqrt(((d * w) ** 2).sum(axis=2))

    def update(self, frame, line_mask, marker_corners=()):
        """line_mask: uint8 cung kich thuoc frame (255 = line). marker_corners: goc cac ma ArUco (bo qua, giay dan san)."""
        h, w = frame.shape[:2]
        y0 = int(h * self.cfg.get('roi_top', 0.40))
        lab = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2Lab)[y0:]
        ign = cv2.dilate(line_mask, np.ones((9, 9), np.uint8))
        for c in marker_corners:                      # ma ArUco + vien trang: giay phang tren san, xe chay qua duoc
            p = c.reshape(4, 2).astype(np.float32)
            ctr = p.mean(axis=0)
            cv2.fillConvexPoly(ign, ((p - ctr) * self.cfg.get('marker_pad', 2.0) + ctr).astype(np.int32), 255)
        lm = ign[y0:]
        out = {'zone': 'CLEAR', 'front_mm': None, 'covered': False, 'ready': self.ready}
        # ---- hoc mau san (vai giay dau, xe dung yen / dang o tren duong trong) ----
        if self.floor_mean is None:
            px = lab[lm == 0].reshape(-1, 3)
            if px.size:
                self._learn.append(px[:: max(1, len(px) // 4000)])
            if len(self._learn) >= self.cfg.get('learn_frames', 20):
                allpx = np.concatenate(self._learn).astype(np.float32)
                self.floor_mean = np.median(allpx, axis=0)
                mad = np.median(np.abs(allpx - self.floor_mean), axis=0) * 1.4826
                self.floor_std = np.maximum(mad, np.array(self.cfg.get('min_std', [8, 4, 4]), np.float32))
            return out
        if self.g is None:
            return out
        from line_vision import ground_dist_mm, ground_scale
        d = self._dist(lab)
        L = lab[:, :, 0].astype(np.float32)
        ab = lab[:, :, 1:].astype(np.float32)
        chroma = np.sqrt((((ab - self.floor_mean[1:]) / self.floor_std[1:]) ** 2).sum(axis=2))
        dark = L < self.floor_mean[0] - self.cfg.get('dark_delta', 45)            # toi ro ret (vat den: LiDAR hay sot)
        colored = (chroma > self.cfg.get('chroma_thresh', 7.0)) & (L < self.floor_mean[0] + self.cfg.get('bright_ignore', 40))
        # bo vung trang sang (giay, ma, phan chieu): vat trang dung thang thi LiDAR thay tot
        mask = ((dark | colored) & (lm == 0)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        rh = mask.shape[0]
        # hanh lang rong bang than xe (+le) theo tung hang, nho hieu chinh mat san
        yn = (y0 + np.arange(rh)) / h
        half_px = (self.half_w + self.cfg.get('corridor_margin_mm', 50)) / ground_scale(self.g, yn)
        cols = np.arange(w)[None, :]
        corridor = np.abs(cols - w / 2) <= half_px[:, None]
        hit_rows = ((mask > 0) & corridor).sum(axis=1) / np.maximum(1, corridor.sum(axis=1))
        rows = np.nonzero(hit_rows > self.cfg.get('row_fill', 0.35))[0]
        self.last_mask = mask
        covered = float(dark.mean()) > self.cfg.get('covered_frac', 0.85)          # gan nhu ca anh toi den -> bi che
        out['covered'] = covered
        if covered:
            out.update(zone='STOP', front_mm=0.0)
        else:
            # vat PHANG tren san (vet go, vet ban, bang keo...) chieu xuong san chi day vai cm; vat DUNG (hop, chai)
            # chieu xuong san thanh dai sau hang chuc cm va cang lai gan cang dai ra -> bo vung qua mong
            if len(rows):
                dd = ground_dist_mm(self.g, (y0 + np.array([rows.min(), rows.max()])) / h)
                out['depth_mm'] = float(dd[0] - dd[1])
                if out['depth_mm'] < self.cfg.get('min_depth_mm', 30):
                    rows = rows[:0]
        if not covered and len(rows) >= self.cfg.get('min_rows', 6):
            nearest = rows.max()                                 # hang thap nhat = gan xe nhat
            ys_mm = []
            for rr in rows:
                cs = np.nonzero(mask[rr] > 0)[0]
                if cs.size:
                    sc = float(ground_scale(self.g, np.array([(y0 + rr) / h]))[0])
                    ys_mm += [-(cs.min() - w / 2) * sc, -(cs.max() - w / 2) * sc]   # anh phai = xe phai (y am)
            if ys_mm:
                out['left_mm'], out['right_mm'] = max(ys_mm), min(ys_mm)
            dist_c = float(ground_dist_mm(self.g, np.array([(y0 + nearest) / h]))[0])
            fm = dist_c - self.front
            out['front_mm'] = fm
            out['zone'] = 'STOP' if fm < self.cfg.get('stop_mm', 300) else 'SLOW'
        # loc nhieu: can thay lien tiep vai khung moi bao (phan chieu tren san bong chop tat)
        # vat THAT tien lai gan tu xa (camera thay truoc ~1 m): lan dau thay ma da sat mui (< first_min_mm) -> vet san
        # (mang go sam/bong do vua lot vao day anh) -> bo qua cho toi khi camera thay trong tro lai
        if out['zone'] != 'CLEAR' and not covered:
            if self._hits == 0 and not getattr(self, '_ep_bad', False) and out['front_mm'] is not None \
                    and out['front_mm'] < self.cfg.get('first_min_mm', 100):
                self._ep_bad = True
            if getattr(self, '_ep_bad', False):
                out.update(zone='CLEAR', front_mm=None, flat=True)
        elif out['zone'] == 'CLEAR':
            self._ep_bad = False
        if out['zone'] != 'CLEAR':
            self._hits += 1
        else:
            self._hits = 0
        if self._hits < self.cfg.get('persist_frames', 4):
            out['zone'] = 'CLEAR' if not covered else out['zone']
        self.zone, self.front_mm = out['zone'], out['front_mm']
        return out
