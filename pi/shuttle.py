#!/usr/bin/env python3
"""Chay tien - lui lap lai (theo encoder) de thu do on dinh. Chay doc lap SSH, dung khi co file STOP hoac het gio.
    nohup python3 shuttle.py <toc do xung/s> <cm moi chieu> <giay toi da> [giu huong 0/1] > shuttle.log 2>&1 &
Firmware tu 2026-11-19 18:37: dau banh dung, "tien" = ve phia M3 M4. Dung lenh ca xe (drive) + gyro giu huong."""
import os
import statistics as st
import sys
import time

from stm32_link import Stm32Link

v = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
cm = float(sys.argv[2]) if len(sys.argv) > 2 else 30
max_s = float(sys.argv[3]) if len(sys.argv) > 3 else 300
hold = (sys.argv[4] != '0') if len(sys.argv) > 4 else True
COUNTS = cm * 10 / (3.14159 * 80 / 3960)          # xung moi chieu

os.chdir(os.path.dirname(os.path.abspath(__file__)))
if os.path.exists('STOP'):
    os.remove('STOP')
link = Stm32Link('/dev/ttyUSB0')
time.sleep(0.4)
if not link.telem:
    raise SystemExit('KHONG CO TELEMETRY - STM chua co dien 12V?')

t_all = time.time()
leg = 0
h_start = link.telem['heading_deg']
print('luot chieu   cm   lech_huong  chenh_banh%  s', flush=True)
try:
    while time.time() - t_all < max_s and not os.path.exists('STOP'):
        sign = 1 if leg % 2 == 0 else -1
        e0, h0 = link.telem['enc'], link.telem['heading_deg']
        t0 = time.time(); sp = []
        while not os.path.exists('STOP'):
            d = [abs(a - b) for a, b in zip(link.telem['enc'], e0)]
            if sum(d) / 4 >= COUNTS or time.time() - t0 > COUNTS / v * 2 + 3:
                break
            link.drive(sign * v, 0, 0, hold=hold)
            time.sleep(0.05)
            sp.append([abs(x) for x in link.telem['speed']])
        t_end = time.time()
        while time.time() - t_end < 0.7:
            link.stop(); time.sleep(0.05)
        t = link.telem
        d = [abs(a - b) for a, b in zip(t['enc'], e0)]
        mid = sp[len(sp) // 3:] or sp
        avg = [st.mean(s[i] for s in mid) for i in range(4)] if mid else [0] * 4
        spread = (max(avg) - min(avg)) / max(1, st.mean(avg)) * 100
        dh = (t['heading_deg'] - h0 + 180) % 360 - 180
        print('%4d %5s %5.1f  %+6.1f do   %5.1f      %4.1f' % (leg // 2 + 1, 'tien' if sign > 0 else 'lui',
              sum(d) / 4 * 3.14159 * 80 / 3960 / 10, dh, spread, t_end - t0), flush=True)
        leg += 1
finally:
    link.stop()
    time.sleep(0.5)
    print('TONG: %d chieu trong %.0f s, huong lech so voi luc dau %+.1f do (giu huong: %s)' % (
        leg, time.time() - t_all, (link.telem['heading_deg'] - h_start + 180) % 360 - 180, 'co' if hold else 'khong'), flush=True)
    link.close()
