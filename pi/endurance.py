#!/usr/bin/env python3
"""Thu ben (BANH KE LEN): PWM co dinh chay lien tuc, ghi toc do moi 30 s. Dung: file STOP hoac het gio.
    nohup python3 endurance.py <duty -5000..5000> <giay> > endurance.log 2>&1 &"""
import os
import sys
import time

from stm32_link import Stm32Link

duty = int(sys.argv[1]) if len(sys.argv) > 1 else 5000
max_s = float(sys.argv[2]) if len(sys.argv) > 2 else 600
os.chdir(os.path.dirname(os.path.abspath(__file__)))
if os.path.exists('STOP'):
    os.remove('STOP')
link = Stm32Link('/dev/ttyUSB0')
time.sleep(0.4)
if not link.telem:
    raise SystemExit('KHONG CO TELEMETRY')
t0 = time.time(); nxt = 5
acc = [0.0] * 4; n = 0
try:
    while time.time() - t0 < max_s and not os.path.exists('STOP'):
        link.set_pwm([duty] * 4)
        time.sleep(0.05)
        s = link.telem['speed']
        acc = [acc[i] + abs(s[i]) for i in range(4)]; n += 1
        if time.time() - t0 >= nxt:
            avg = [round(a / max(1, n)) for a in acc]
            print('%4.0f s  toc do TB M1..M4 %s  (~%.0f vong/phut)' % (time.time() - t0, avg, sum(avg) / 4 / 3960 * 60), flush=True)
            acc = [0.0] * 4; n = 0
            nxt += 30
finally:
    link.stop(); time.sleep(0.4); link.close()
print('XONG sau %.0f s' % (time.time() - t0), flush=True)
