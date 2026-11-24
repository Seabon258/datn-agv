#!/usr/bin/env python3
"""Bai chay thu tren san (Pi): toi/lui, ngang trai/phai, xoay 90 do. Ghi log 50 Hz ra CSV de phan tich.

    python3 demo_moves.py [/dev/ttyUSB0] [toc do xung/s]

Xe dat duoi san, trong ban kinh ~50 cm. Ctrl+C la dung ngay.
Dau xe = phia M3, M4. Vi tri banh: M4 truoc-trai, M3 truoc-phai, M2 sau-trai, M1 sau-phai (STM32 tu chia lenh).
"""
import csv
import sys
import math
import time

from stm32_link import FRAME_FLIP, Stm32Link

CPR = 3960.0                       # xung / vong banh (uoc tinh, dong co 111 vong/phut 1:90)
try:                                               # duong kinh banh da hieu chinh (config.yaml geometry.wheel_diam_mm)
    import yaml as _y
    _D = float(_y.safe_load(open(__import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), 'config.yaml')))['geometry']['wheel_diam_mm'])
except Exception:
    _D = 80.0
MM_PER_COUNT = 3.14159 * _D / CPR
ROT_STOP_EARLY = 1.5               # do: ngat som de bu quan tinh khi xoay (6.0 lam thieu ~4.7 do)


def wheel_motion(enc, enc0):
    """Quang duong (mm) theo truc xe tu 4 encoder. Cach cam 2026-11-20: M4 FL, M3 FR, M2 RL, M1 RR."""
    d = [(a - b) * MM_PER_COUNT for a, b in zip(enc, enc0)]
    rr, rl, fr, fl = d
    x = (fl + fr + rl + rr) / 4.0
    y = (-fl + fr + rl - rr) / 4.0          # sang trai +
    return (-x, y) if FRAME_FLIP else (x, y)


class Demo:
    def __init__(self, port, speed):
        self.link = Stm32Link(port, on_telem=self.on_telem)
        self.speed = speed
        self.rows = []
        self.step = ''
        self.hcum = None                   # huong cong don (khong nhay khi qua +-180 do)
        self._hlast = None
        self.f = open(f'demo_{time.strftime("%H%M%S")}.csv', 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'buoc', 'enc1', 'enc2', 'enc3', 'enc4', 'v1', 'v2', 'v3', 'v4', 'huong', 'gz'])
        time.sleep(0.5)
        if not self.link.telem:
            raise SystemExit('Khong nhan duoc telemetry tu STM32 (nguon 12V? day UART?)')

    def on_telem(self, t):
        h = t['heading_deg']
        if self._hlast is None:
            self.hcum = h
        else:
            d = h - self._hlast
            d -= 360.0 if d > 180.0 else 0.0
            d += 360.0 if d < -180.0 else 0.0
            self.hcum += d
        self._hlast = h
        self.w.writerow([f'{time.time():.3f}', self.step, *t['enc'], *t['speed'], f'{t["heading_deg"]:.2f}', t['gyro'][2]])

    def settle(self, s=0.8):
        end = time.time() + s
        while time.time() < end:
            self.link.stop()
            time.sleep(0.05)

    def move(self, name, vx, vy, dist_mm, ramp_s=0.0):
        """ramp_s > 0: Pi tang toc dan trong ramp_s giay (them vao ramp cua STM32) de banh nhe it truot."""
        self.step = name
        t = self.link.telem
        enc0, h0 = t['enc'], t['heading_deg']
        t_start = time.time()
        while True:
            t = self.link.telem
            x, y = wheel_motion(t['enc'], enc0)
            done = math.hypot(x, y) if (vx and vy) else (abs(x) if vx else abs(y))   # cheo: tinh quang theo duong thang
            limit = dist_mm / (max(abs(vx), abs(vy)) * MM_PER_COUNT) * 1.5 + 3   # 1.5 lan thoi gian ly thuyet
            if done >= dist_mm or time.time() - t_start > limit:
                break
            k = min(1.0, (time.time() - t_start) / ramp_s) if ramp_s > 0 else 1.0
            self.link.drive(round(vx * k), round(vy * k), 0, hold=True)
            time.sleep(0.02)
        self.settle()
        t = self.link.telem
        x, y = wheel_motion(t['enc'], enc0)
        print(f'{name:<12} encoder: doc {x / 10:+6.1f} cm, ngang {y / 10:+6.1f} cm | lech huong {t["heading_deg"] - h0:+5.1f} do '
              f'| {time.time() - t_start:4.1f} s', flush=True)

    def rotate(self, name, deg):
        self.step = name
        h0 = self.hcum
        w = 800 if deg > 0 else -800
        t_start = time.time()
        while abs(self.hcum - h0) < abs(deg) - ROT_STOP_EARLY and time.time() - t_start < abs(deg) / 90.0 * 10 + 3:
            self.link.drive(0, 0, w, hold=False)
            time.sleep(0.02)
        self.settle(1.0)
        got = self.hcum - h0
        print(f'{name:<12} xoay {got:+6.1f} do (muc tieu {deg:+.0f}, sai {got - deg:+.1f}) | {time.time() - t_start:4.1f} s', flush=True)

    def close(self):
        self.link.close()
        self.f.close()


def main():
    port = next((a for a in sys.argv[1:] if a.startswith('/dev/')), '/dev/ttyUSB0')
    speed = next((int(a) for a in sys.argv[1:] if a.isdigit()), 1200)
    d = Demo(port, speed)
    try:
        d.move('toi 30cm', speed, 0, 300)
        d.move('lui 30cm', -speed, 0, 300)
        d.move('trai 25cm', 0, speed, 250)
        d.move('phai 25cm', 0, -speed, 250)
        d.rotate('xoay trai 90', 90)
        d.rotate('xoay phai 90', -90)
    except KeyboardInterrupt:
        print('DUNG (Ctrl+C)')
    finally:
        d.close()
        print('log:', d.f.name)


if __name__ == '__main__':
    main()
