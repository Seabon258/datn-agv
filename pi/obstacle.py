#!/usr/bin/env python3
"""LiDAR obstacle zones for the AGV — pure Python (no ROS, no hardware), testable with synthetic scans.

Frames: LIDAR frame = what lidar.py returns (angle_deg 0..360, clockwise, dist_mm).
        CAR frame   = x forward (mm), y LEFT (mm), origin = car centre (between the 4 wheels).
Mounting (config.yaml `obstacle`): yaw_offset_deg = lidar angle that points to the car FRONT,
        angle_sign = +1 if lidar angles grow towards the car's LEFT (counter-clockwise seen from above), -1 if clockwise,
        mount_x_mm / mount_y_mm = lidar position in the car frame.
These MUST be measured (box-in-front test). While any is null, ObstacleMonitor.ready is False and
line_follow.py refuses to move with obstacle checking enabled (never guess, kit docs/06).

    om = ObstacleMonitor(cfg['obstacle'], car_len_mm, car_wid_mm)
    st = om.update(scan, now)     # {'zone': CLEAR|SLOW|STOP, 'front_mm', 'left_free', 'right_free', 'points'}
    om.side_clear(side, now)      # PASS phase: no obstacle beside the car on that side anymore?
"""
import math

ZONES = ('CLEAR', 'SLOW', 'STOP')


def clock_of(x, y):
    """Diem trong khung XE (x truoc, y trai, mm) -> chuoi huong dong ho: '12 gio', '10 gio 30', '3 gio'...
    12 gio = thang truoc, 9 gio = trai, 3 gio = phai, 6 gio = sau. Lam tron 30 phut (15 do)."""
    ang = math.degrees(math.atan2(y, x))              # + = ve ben trai
    h = (12 - ang / 30.0) % 12                         # trai -> gio nho hon 12
    h = round(h * 2) / 2.0
    hh, half = int(h) % 12 or 12, (h % 1) >= 0.5
    return '%d gio%s' % (hh, ' 30' if half else '')


class ObstacleMonitor:
    REQUIRED = ('yaw_offset_deg', 'angle_sign', 'mount_x_mm', 'mount_y_mm')

    def __init__(self, cfg, car_len_mm, car_wid_mm, car_rear_mm=None):
        self.cfg = cfg
        self.L, self.W = float(car_len_mm), float(car_wid_mm)
        self.rear = float(car_rear_mm) if car_rear_mm else self.L / 2     # tu tam banh ra duoi xe (than xe lech truc doc)
        self.missing = [k for k in self.REQUIRED if cfg.get(k) is None]
        self.ready = not self.missing
        self.zone, self._cand, self._cand_n = 'CLEAR', 'CLEAR', 0
        self.last = {'zone': 'CLEAR', 'front_mm': None, 'left_free': False, 'right_free': False, 'points': []}
        self.seen = {}                    # bin 10 do (goc XE) -> so lan co diem NGOAI than xe (thay duoc ra ngoai)
        self.selfmask = {}                # goc LiDAR (do nguyen) -> khoang cach bo phan xe (lidar_selfmask.json)
        try:
            import json, os
            p = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lidar_selfmask.json')
            if os.path.exists(p):
                self.selfmask = {int(k): float(v) for k, v in json.load(open(p))['bins'].items()}
        except Exception:
            self.selfmask = {}
        self.blind = set()                # bin goc XE ma LiDAR bi than xe che / khong co tia ve -> KHONG BIET
        self.scan_t = 0.0

    # ---------- geometry ----------
    def to_car(self, scan):
        """LIDAR (angle, mm) -> CAR (x, y) mm, dropping blocked arcs and points on the car itself.
        Cap nhat self.blind: huong (goc XE, bin 10 do) khong co diem nao ra NGOAI than xe trong vong quet nay."""
        c = self.cfg
        out = []
        outside_bins = set()
        blocked = c.get('blocked_arcs_deg') or []
        m = c.get('self_margin_mm', 20)
        tol = c.get('self_mask_tol_mm', 40)
        for a, d in scan:
            if any(lo <= a <= hi if lo <= hi else (a >= lo or a <= hi) for lo, hi in blocked):
                continue
            k = int(round(a)) % 360
            if self.selfmask and any(d <= self.selfmask.get((k + j) % 360, -1) + tol for j in (-1, 0, 1)):
                continue                                          # bo phan cua chinh xe (mat na da hoc)
            th = math.radians(c['angle_sign'] * (a - c['yaw_offset_deg']))     # 0 = car front, + = towards LEFT
            x = c['mount_x_mm'] + d * math.cos(th)
            y = c['mount_y_mm'] + d * math.sin(th)
            front = max(self.L / 2, c.get('front_extent_mm', self.L / 2))   # than xe KHONG doi xung: phia truoc toi vo LiDAR
            if -self.rear - m <= x <= front + m and abs(y) <= self.W / 2 + m:   # part of the car / cargo / LiDAR mount
                continue
            outside_bins.add(int((math.degrees(th) % 360) // 10))
            out.append((x, y))
        for b in range(36):                                   # mu neu nhieu vong lien tiep khong thay gi ra ngoai
            self.seen[b] = 0 if b in outside_bins else self.seen.get(b, 0) + 1
        self.blind = {b for b, n in self.seen.items() if n >= c.get('blind_scans', 6)}
        return out

    def _corridor_ahead(self, pts):
        """Nearest point straight ahead within the car width (+margin), distance from the FRONT EDGE (mm)."""
        half = self.W / 2 + self.cfg.get('corridor_margin_mm', 50)
        front = max(self.L / 2, self.cfg.get('front_extent_mm', self.L / 2))     # vo LiDAR nho ra truoc than xe
        ds = [x - front for x, y in pts if x > front and abs(y) <= half]
        # sector rule (kit D-019): also anything within +-front_half_angle from the front edge centre
        ang = math.radians(self.cfg.get('front_half_angle_deg', 30))
        ds += [math.hypot(x - front, y) for x, y in pts if x > front and abs(math.atan2(y, x - front)) <= ang]
        return min(ds) if ds else None

    def _side_free(self, pts, side, width=None):
        """Room beside the car on one side: the band [car side, car side + width] along the car body must be empty.
        width=None -> max(detour_mm, side_clearance_mm) (kit 0.5 m, used for the CLEAR/flag in update())."""
        c = self.cfg
        if width is None:
            width = max(c.get('detour_mm', 350), c.get('side_clearance_mm', 500))
        x_lo, x_hi = -self.rear - 50, max(self.L / 2, c.get('front_extent_mm', self.L / 2)) + c.get('side_check_ahead_mm', 300)
        y_in = self.W / 2
        y_out = self.W / 2 + width
        s = 1 if side == 'left' else -1
        # KHONG THAY = KHONG BIET -> khong duoc coi la trong: huong nhin toi vung ben do ma LiDAR bi mu -> chan
        a_lo = math.degrees(math.atan2(s * y_in, x_hi - c.get('mount_x_mm', 0)))
        a_hi = math.degrees(math.atan2(s * y_in, x_lo - c.get('mount_x_mm', 0)))
        lo, hi = sorted(((a_lo % 360), (a_hi % 360)))
        mb_lo, mb_hi = c.get('mount_blind_deg', [115, 245])      # sau lung LiDAR (than xe che CO DINH): xe vua di qua vung nay
        bins = [b for b in range(int(lo // 10), int(hi // 10) + 1) if not (mb_lo <= b * 10 + 5 <= mb_hi)]
        blind_frac = sum(1 for b in bins if b in self.blind) / max(1, len(bins))
        if blind_frac > c.get('blind_max_frac', 0.2):
            return False
        return not any(x_lo <= x <= x_hi and y_in < s * y <= y_out for x, y in pts)

    # ---------- per scan ----------
    def update(self, scan, now):
        if not self.ready:
            return self.last
        pts = self.to_car(scan)
        f = self._corridor_ahead(pts)
        c = self.cfg
        raw = 'CLEAR'
        if f is not None and f < c.get('stop_mm', 300):
            raw = 'STOP'
        elif f is not None and f < c.get('slow_mm', 800):
            raw = 'SLOW'
        # persistence: worse zones need `persist_scans` in a row, better zones need `clear_scans` (hysteresis)
        if raw == self._cand:
            self._cand_n += 1
        else:
            self._cand, self._cand_n = raw, 1
        need = c.get('persist_scans', 2) if ZONES.index(raw) > ZONES.index(self.zone) else c.get('clear_scans', 3)
        if raw != self.zone and self._cand_n >= need:
            self.zone = raw
        self.scan_t = now
        self.last = {'zone': self.zone, 'raw': raw, 'front_mm': f, 'left_free': self._side_free(pts, 'left'),
                     'right_free': self._side_free(pts, 'right'), 'points': pts}
        return self.last

    def stale(self, now):
        return now - self.scan_t > self.cfg.get('scan_timeout_s', 0.5)

    def beside_clear(self, side_of_obstacle, pts=None, offset_mm=None):
        """PASS phase: after strafing away, is the obstacle no longer beside the car (on the line side)?
        offset_mm = xe dang lech khoi tuyen bao nhieu: chi xet DAI giua hong xe va vi tri tuyen cu (+ le),
        khong nhin toi tuong / do vat xa hon (truoc day tuong ben canh lam xe tuong chua vuot qua)."""
        pts = self.last['points'] if pts is None else pts
        s = 1 if side_of_obstacle == 'left' else -1
        band = self.cfg.get('beside_band_mm', 400)
        if offset_mm is not None:
            band = max(50.0, abs(offset_mm) + self.cfg.get('beside_extra_mm', 150) - self.W / 2)
        x_lo, x_hi = -self.rear - self.cfg.get('pass_margin_mm', 100), max(self.L / 2, self.cfg.get('front_extent_mm', self.L / 2)) + 100
        return not any(x_lo <= x <= x_hi and self.W / 2 < s * y <= self.W / 2 + band for x, y in pts)
