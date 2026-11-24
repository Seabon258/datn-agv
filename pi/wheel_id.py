#!/usr/bin/env python3
"""Quay lan luot tung kenh M1..M4 (lenh +800 xung/s, 5 s, nghi 3 s) de nhin vi tri va chieu banh."""
import time

from stm32_link import Stm32Link

link = Stm32Link('/dev/ttyUSB0')
time.sleep(0.4)
if not link.telem:
    raise SystemExit('KHONG CO TELEMETRY - STM chua co dien 12V?')
try:
    for ch in range(4):
        cmd = [0, 0, 0, 0]
        cmd[ch] = 800
        print('KENH M%d dang quay ...' % (ch + 1), flush=True)
        t0 = time.time()
        while time.time() - t0 < 5:
            link.set_speed(cmd)
            time.sleep(0.05)
        print('  toc do do duoc M1..M4:', link.telem['speed'], flush=True)
        t0 = time.time()
        while time.time() - t0 < 3:
            link.stop()
            time.sleep(0.05)
finally:
    link.stop()
    time.sleep(0.5)
    link.close()
print('XONG', flush=True)
