#!/usr/bin/env python3
"""Do toc do toi da (BANH KE LEN): PWM 100% tien 5 s, nghi, lui 5 s. In xung/s, vong/phut, m/s."""
import os
import time

from stm32_link import Stm32Link

CPR, CIRC = 3960.0, 3.14159 * 0.080
link = Stm32Link('/dev/ttyUSB0')
time.sleep(0.4)
if not link.telem:
    raise SystemExit('KHONG CO TELEMETRY')
try:
    for duty, name in ((5000, 'TIEN'), (-5000, 'LUI')):
        t0 = time.time(); top = [0] * 4
        while time.time() - t0 < 5 and not os.path.exists('STOP'):
            link.set_pwm([duty] * 4)
            time.sleep(0.05)
            top = [max(top[i], abs(link.telem['speed'][i])) for i in range(4)]
        avg = sum(top) / 4
        print('%s 100%%: toc do cao nhat M1..M4 %s xung/s | TB %.0f xung/s = %.0f vong/phut = %.2f m/s' % (
            name, top, avg, avg / CPR * 60, avg / CPR * CIRC), flush=True)
        t1 = time.time()
        while time.time() - t1 < 2:
            link.stop(); time.sleep(0.05)
finally:
    link.stop(); time.sleep(0.4); link.close()
print('XONG', flush=True)
