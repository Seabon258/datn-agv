#!/usr/bin/env python3
"""MO PHONG chay line_follow.py KHONG CAN XE: camera gia (ve line tu ground_calib), LiDAR gia (raycast), STM32 gia
(dong hoc mecanum + giu huong). Dung de bat loi logic truoc khi cho xe that chay.

    python3 sim_world.py <thu_muc_chay> <kich_ban> [--max-s 120]
    kich ban: straight_bottle | curve_bottle | gap | corner_bottle
Thu muc chay: ban sao thu muc pi (DB, video, logs rieng).
"""
import math
import os
import sys
import threading
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def scenario(name):
    """(line polyline mm ODOM, vat can [(x, y, r)], tuong [(x0,y0,x1,y1)])"""
    walls = [(-800, 2000, 6000, 2000), (-800, -1500, 6000, -1500), (6000, -1500, 6000, 2000)]
    if name == 'straight_bottle':
        return [(-200, 0), (3500, 0)], [(1700, 0, 40)], walls
    if name == 'curve_bottle':
        pts = [(-200, 0)] + [(x, 0) for x in range(0, 1600, 100)] + \
              [(1600 + 1200 * math.sin(a), 1200 - 1200 * math.cos(a)) for a in np.linspace(0, 0.9, 20)]
        return pts, [(1100, 0, 40)], walls
    if name == 'sudden':
        return [(-200, 0), (3200, 0)], [], walls
    if name in ('gap', 'glare'):
        return [(-200, 0), (3000, 0)], [], walls
    if name == 'trip62':                    # chai, line re trai ngay sau, THUNG nam tren duong doan thang
        return [(-200, 0), (1900, 0), (1900, 1300)], [(1100, 0, 40), (2450, 0, 160)], walls
    if name == 'corner':
        return [(-200, 0), (1400, 0), (1400, 1300)], [], walls
    if name == 'corner_bottle':
        return [(-200, 0), (1400, 0), (1400, 1300)], [(1250, 0, 40)], walls
    raise SystemExit('kich ban?')


class World:
    def __init__(self, cfg, name):
        self.cfg = cfg
        self.name = name
        self.line, self.obst, self.walls = scenario(name)
        self.gap = (1200, 1600) if name == 'gap' else None
        self.glare = (900, 1300) if name == 'glare' else None      # xe o doan nay: anh loa, KHONG thay line
        self.pose = [0.0, 0.0, 0.0]
        self.enc = [0, 0, 0, 0]
        self.cmd, self.cmd_t, self.hold, self.hold_th = (0, 0, 0), 0.0, False, None
        g = cfg['geometry']
        self.k = math.pi * g['wheel_diam_mm'] / g['enc_cpr']
        self.lxly = g.get('lxly_mm', 274)
        self.lock = threading.Lock()
        self.trace = []

    def step(self, dt):
        if getattr(self, 'name', '') == 'sudden' and not self.obst and self.pose[0] > 1000:
            self.obst.append((self.pose[0] + 320 + 200 + 40, 0.0, 40))     # vat roi xuong cach mui 20 cm
        with self.lock:
            vx, vy, wz = self.cmd if time.time() - self.cmd_t < 0.25 else (0, 0, 0)
            if self.hold and wz == 0 and (vx or vy):
                if self.hold_th is None:
                    self.hold_th = self.pose[2]
                wz = -2.0 * (self.pose[2] - self.hold_th) * self.lxly / self.k
            else:
                self.hold_th = None
            slip = 1.0 + (0.03 if vy else 0.0)               # ngang truot nhe 3 % (mecanum thuc te)
            fl, rl, rr, fr = vx - vy - wz, vx + vy - wz, vx - vy + wz, vx + vy + wz
            for i, w in enumerate((rr, rl, fr, fl)):
                self.enc[i] += int(round(w * dt))
            th = self.pose[2]
            f, l = vx * self.k * dt, vy * self.k * dt / slip
            self.pose[0] += f * math.cos(th) - l * math.sin(th)
            self.pose[1] += f * math.sin(th) + l * math.cos(th)
            self.pose[2] += wz * self.k / self.lxly * dt
            self.trace.append(tuple(self.pose))

    # ---------- camera ----------
    def render(self, gcal, w=640, h=480):
        from line_vision import ground_dist_mm, ground_scale
        img = np.empty((h, w, 3), np.uint8)
        img[:] = (90, 140, 180)
        rng = np.random.default_rng(int(time.time() * 1000) % 100000)
        img = np.clip(img.astype(np.int16) + rng.integers(-6, 7, img.shape), 0, 255).astype(np.uint8)
        ys = np.arange(int(h * 0.30), h)
        yn = ys / h
        dist = ground_dist_mm(gcal, yn)
        sc = ground_scale(gcal, yn)
        with self.lock:
            px, py, th = self.pose
        if self.glare and self.glare[0] <= px <= self.glare[1]:
            return img
        c, s = math.cos(th), math.sin(th)
        cols = np.arange(w)
        for k, y in enumerate(ys):
            fwd = dist[k]
            left = -(cols - w / 2) * sc[k]
            X = px + fwd * c - left * s
            Y = py + fwd * s + left * c
            on = self._on_line(X, Y)
            img[y, on] = (130, 60, 20)
            for ox, oy, r in self.obst:                       # vat (chan) tren san: mau xam trang
                inb = (X - ox) ** 2 + (Y - oy) ** 2 <= r * r
                img[y, inb] = (230, 230, 230)
        return img

    def _on_line(self, X, Y, half=24):
        best = np.full(X.shape, 1e9)
        for (ax, ay), (bx, by) in zip(self.line, self.line[1:]):
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            u = np.clip(((X - ax) * dx + (Y - ay) * dy) / L2, 0, 1)
            d = np.hypot(X - (ax + u * dx), Y - (ay + u * dy))
            if self.gap:
                d = np.where((ax + u * dx >= self.gap[0]) & (ax + u * dx <= self.gap[1]) & (abs(dy) < 1), 1e9, d)
            best = np.minimum(best, d)
        return best <= half

    # ---------- lidar ----------
    def scan(self):
        oc = self.cfg['obstacle']
        with self.lock:
            px, py, th = self.pose
        lx = px + oc['mount_x_mm'] * math.cos(th)
        ly = py + oc['mount_x_mm'] * math.sin(th)
        out = []
        for a in np.arange(0, 360, 0.8):
            thc = math.radians(oc['angle_sign'] * (a - oc['yaw_offset_deg']))      # goc khung XE
            if 115 < (math.degrees(thc) % 360) < 245:
                continue                                        # sau lung LiDAR bi than xe che
            ang = th + thc
            dx, dy = math.cos(ang), math.sin(ang)
            best = 8000.0
            for ox, oy, r in self.obst:
                fx, fy = lx - ox, ly - oy
                b = fx * dx + fy * dy
                cc = fx * fx + fy * fy - r * r
                disc = b * b - cc
                if disc >= 0:
                    t = -b - math.sqrt(disc)
                    if 0 < t < best:
                        best = t
            for x0, y0, x1, y1 in self.walls:
                ex, ey = x1 - x0, y1 - y0
                den = dx * ey - dy * ex
                if abs(den) < 1e-9:
                    continue
                t = ((x0 - lx) * ey - (y0 - ly) * ex) / den
                u = ((x0 - lx) * dy - (y0 - ly) * dx) / den
                if t > 0 and 0 <= u <= 1 and t < best:
                    best = t
            if best < 7900:
                out.append((float(a), float(best)))
        return out


def install(world, gcal):
    import cv2
    W = world

    class FakeCap:
        def __init__(self, *a, **k):
            self.t = time.time()

        def set(self, *a):
            return True

        def isOpened(self):
            return True

        def read(self):
            dt = 1 / 25.0 - (time.time() - self.t)
            if dt > 0:
                time.sleep(dt)
            self.t = time.time()
            return True, W.render(gcal)

        def release(self):
            pass
    cv2.VideoCapture = FakeCap

    class FakeLink:
        def __init__(self, *a, **k):
            self.on_telem = None
            self.telem = None
            self._run = True
            self._mk()
            threading.Thread(target=self._loop, daemon=True).start()

        def _mk(self):
            th = math.degrees(W.pose[2])
            th = (th + 180) % 360 - 180
            self.telem = {'enc': list(W.enc), 'heading_deg': round(th, 2), 'batt_v': 12.0, 'lift_pos': 0, 'speed': [0] * 4,
                          'imu_ok': True, 'lift_busy': False, 'lift_homed': True, 'limit_top': False, 'limit_bot': True,
                          'timeout': False, 'flags': 0, 'ms': int(time.time() * 1000)}
            self.telem_time = time.time()

        def _loop(self):
            last = time.time()
            while self._run:
                time.sleep(0.02)
                now = time.time()
                W.step(now - last)
                last = now
                self._mk()
                if self.on_telem:
                    self.on_telem(self.telem)

        def drive(self, vx, vy, wz, hold=True):
            with W.lock:
                W.cmd, W.cmd_t, W.hold = (vx, vy, wz), time.time(), hold

        def stop(self):
            with W.lock:
                W.cmd = (0, 0, 0)

        def close(self):
            self._run = False

        def __getattr__(self, n):
            return lambda *a, **k: None
    sm = types.ModuleType('stm32_link')
    sm.Stm32Link = FakeLink
    sys.modules['stm32_link'] = sm

    class FakeLidar:
        def __init__(self, *a, **k):
            self.scan, self.scan_t = [], 0.0
            self._run = True

        def start(self):
            threading.Thread(target=self._loop, daemon=True).start()
            return self

        def _loop(self):
            while self._run:
                sc = W.scan()
                self.scan, self.scan_t = sc, time.time()
                time.sleep(0.16)

        def wait_ready(self, *a, **k):
            time.sleep(0.3)
            return True

        def age(self):
            return time.time() - self.scan_t if self.scan_t else 9.0

        def close(self):
            self._run = False
    lm = types.ModuleType('lidar')
    lm.Lidar = FakeLidar
    sys.modules['lidar'] = lm


if __name__ == '__main__':
    import json
    import shutil
    import yaml
    run_dir, name = sys.argv[1], sys.argv[2]
    extra = sys.argv[3:]
    keep_maps = None
    if os.path.exists(os.path.join(run_dir, 'maps')):
        keep_maps = os.path.join(os.path.dirname(run_dir.rstrip('/')), '_maps_tmp')
        shutil.rmtree(keep_maps, ignore_errors=True)
        shutil.copytree(os.path.join(run_dir, 'maps'), keep_maps)
    if os.path.exists(run_dir):
        shutil.rmtree(run_dir)
    shutil.copytree(HERE, run_dir, ignore=shutil.ignore_patterns('videos', 'logs', '*.db', '__pycache__', 'calib_dbg', '*.avi'))
    os.makedirs(os.path.join(run_dir, 'videos', 'raw'), exist_ok=True)
    if keep_maps:
        shutil.copytree(keep_maps, os.path.join(run_dir, 'maps'), dirs_exist_ok=True)
    sys.path.insert(0, run_dir)
    os.chdir(run_dir)
    cfg = yaml.safe_load(open('config.yaml'))
    cfg['database'] = os.path.join(run_dir, 'sim.db')
    cfg['camera']['index'] = 0
    yaml.safe_dump(cfg, open('config.yaml', 'w'), allow_unicode=True, sort_keys=False)
    gcal = json.load(open('ground_calib.json'))
    world = World(cfg, name)
    install(world, gcal)
    import line_follow
    sys.argv = ['line_follow.py', '--no-lift'] + (extra or ['--max-s', '120'])
    try:
        line_follow.main()
    finally:
        import cv2
        tr = world.trace
        mx = max(abs(p[1]) for p in tr) if tr else 0
        print('SIM: cuoi x %.0f y %.0f th %.1f do | lech ngang max %.0f mm' % (world.pose[0], world.pose[1], math.degrees(world.pose[2]), mx))
        # va cham that (vi tri THAT cua xe, khong phai odom)
        g = cfg['geometry']
        half, front, rear = g['car_width_mm'] / 2, cfg['obstacle']['front_extent_mm'], g.get('car_rear_mm', 225)
        hit = 0
        for px, py, th in tr[::5]:
            for ox, oy, r in world.obst:
                dx, dy = ox - px, oy - py
                x = dx * math.cos(th) + dy * math.sin(th)
                y = -dx * math.sin(th) + dy * math.cos(th)
                if -rear - r <= x <= front + r and abs(y) <= half + r:
                    hit += 1
        print('SIM: so lan cham vat: %d' % hit)
        np.save(os.path.join(run_dir, 'trace.npy'), np.array(tr))
