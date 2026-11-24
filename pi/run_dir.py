#!/usr/bin/env python3
"""Chay xe theo toc do 4 banh, doc lap voi SSH (Wi-Fi rot van chay). Dung khi co file STOP hoac het gio.
    nohup python3 run_dir.py <v1> <v2> <v3> <v4> <giay toi da> > run_dir.log 2>&1 &
"""
import os
import sys
import time

from stm32_link import Stm32Link

cmd = [int(x) for x in sys.argv[1:5]]
max_s = float(sys.argv[5]) if len(sys.argv) > 5 else 60
os.chdir(os.path.dirname(os.path.abspath(__file__)))
if os.path.exists('STOP'):
    os.remove('STOP')
link = Stm32Link('/dev/ttyUSB0')
time.sleep(0.4)
if not link.telem:
    raise SystemExit('KHONG CO TELEMETRY - STM chua co dien 12V?')
t0 = time.time()
nxt = 0
try:
    while time.time() - t0 < max_s and not os.path.exists('STOP'):
        link.set_speed(cmd)
        time.sleep(0.05)
        if time.time() - t0 >= nxt:
            print('%4.0f s  toc do M1..M4 %s' % (time.time() - t0, link.telem['speed']), flush=True)
            nxt += 5
finally:
    link.stop()
    time.sleep(0.5)
    link.close()
print('da dung sau %.0f s' % (time.time() - t0), flush=True)
