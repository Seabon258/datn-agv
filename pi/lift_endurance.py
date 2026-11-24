#!/usr/bin/env python3
"""Thu ben lift: nang N giay roi ha N giay lien tuc trong T giay.
    python3 lift_endurance.py [giay_moi_chieu=5] [tong_giay=300] [buoc/s=speed_sps]
Ghi moi chu ky vao lift_endurance.log. Dung khan: tao file STOP. Cuoi cung lift_pos phai ve dung gia tri ban dau."""
import os
import sys
import time

import yaml

from stm32_link import Stm32Link

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
cfg = yaml.safe_load(open('config.yaml'))
lc = cfg['lift']
sec = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
total = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0
sps = int(sys.argv[3]) if len(sys.argv) > 3 else int(lc.get('speed_sps', 800))
n = int(round(sec * sps))
sign = int(lc.get('dir_sign', 1))
if os.path.exists('STOP'):
    os.remove('STOP')
log = open('lift_endurance.log', 'w')


def say(s):
    print(s, flush=True)
    log.write(s + '\n')
    log.flush()


link = Stm32Link(cfg['serial_port'])
time.sleep(0.5)
if not link.telem:
    sys.exit('Khong co telemetry STM32')
p0 = link.telem['lift_pos']
say('BAT DAU: lift_pos %d | moi chieu %d buoc (%.1f s @ %d buoc/s) | tong %.0f s' % (p0, n, sec, sps, total))


def move(steps):
    t = link.telem
    start = t['lift_pos']
    link.lift_move(steps, sps)
    t0, seen = time.time(), False
    while time.time() - t0 < abs(steps) / sps * 1.5 + 3:
        time.sleep(0.05)
        t = link.telem
        seen = seen or t['lift_busy']
        if os.path.exists('STOP'):
            return None
        if (seen and not t['lift_busy']) or (not seen and time.time() - t0 > 1.0):
            break
    return time.time() - t0, t['lift_pos'] - start, seen, t


T0 = time.time()
k = 0
ok = True
try:
    while time.time() - T0 < total:
        k += 1
        for name, st in (('LEN', n * sign), ('XUONG', -n * sign)):
            r = move(st)
            if r is None:
                say('STOP -> dung')
                ok = False
                break
            dt, moved, seen, t = r
            bad = (not seen) or moved != st or t.get('timeout') or t.get('wdg_reset')
            say('chu ky %2d %-5s %.2f s | di %+d buoc | lift_pos %d | busy_thay=%s | pin %.2f V%s' % (
                k, name, dt, moved, t['lift_pos'], seen, t['batt_v'], '  <-- BAT THUONG' if bad else ''))
            if bad:
                ok = False
            time.sleep(0.5)
        if not ok:
            break
finally:
    link.stop()
    time.sleep(0.3)
    t = link.telem
    say('KET THUC sau %.0f s: %d chu ky | lift_pos cuoi %d (dau %d, lech %+d buoc) | %s' % (
        time.time() - T0, k, t['lift_pos'], p0, t['lift_pos'] - p0, 'OK' if ok else 'CO VAN DE - xem log'))
    say('XONG')
    link.close()
