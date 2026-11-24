#!/usr/bin/env python3
"""Thu lift (TMC2209 + vit me): python3 lift_test.py <mm | Ns> [buoc/s]     + = LEN, - = XUONG
    vd: python3 lift_test.py 5s       -> nang 5 GIAY;  python3 lift_test.py -5s -> ha 5 giay
    vd: python3 lift_test.py 5        -> nang 5 mm (800 buoc/mm theo config) roi do lai so buoc that
Gioi han 30 mm / lan (chua co cong tac hanh trinh). Dung khan: tao file STOP (hoac Ctrl+C)."""
import os
import sys
import time

import yaml

from stm32_link import Stm32Link

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
if len(sys.argv) < 2:
    sys.exit(__doc__)
arg = sys.argv[1]
cfg = yaml.safe_load(open('config.yaml'))
lc = cfg['lift']
sps = int(sys.argv[2]) if len(sys.argv) > 2 else int(lc.get('test_sps', 800))
# "5s" / "-5s" = chay theo THOI GIAN (giay) o toc do sps; so khong co "s" = mm
mm = float(arg[:-1]) * sps / lc['steps_per_mm'] if arg.endswith('s') else float(arg)
if abs(mm) > lc.get('test_max_mm', 30):
    sys.exit('Qua %d mm / lan: chua co cong tac hanh trinh, thu tung buoc nho' % lc.get('test_max_mm', 30))
steps = int(round(mm * lc['steps_per_mm'])) * int(lc.get('dir_sign', 1))   # dir_sign: + = LEN that
if os.path.exists('STOP'):
    os.remove('STOP')
link = Stm32Link(cfg['serial_port'])
time.sleep(0.5)
t = link.telem
if not t:
    sys.exit('Khong co telemetry STM32')
p0 = t['lift_pos']
print('lift_pos dau: %d | cong tac TREN %s, DUOI %s | lenh: %+.1f mm = %+d buoc @ %d buoc/s (%.1f mm/s)' % (
    p0, 'CHAM' if t['limit_top'] else 'ho', 'CHAM' if t['limit_bot'] else 'ho', mm, steps, sps, sps / lc['steps_per_mm']), flush=True)
link.lift_move(steps, sps)
t0 = time.time()
seen_busy = False
limit = abs(steps) / sps * 1.5 + 3
try:
    while time.time() - t0 < limit:
        time.sleep(0.05)
        t = link.telem
        seen_busy = seen_busy or t['lift_busy']
        if os.path.exists('STOP'):
            print('STOP -> dung', flush=True)
            break
        if (seen_busy and not t['lift_busy']) or (not seen_busy and time.time() - t0 > 1.0):
            break
finally:
    if t.get('lift_busy') or os.path.exists('STOP'):
        link.stop()
    time.sleep(0.3)
    t = link.telem
    d = (t['lift_pos'] - p0) * int(lc.get('dir_sign', 1))     # + = da di LEN
    print('xong sau %.1f s | lift_pos cuoi %d | da chay %+d buoc = %+.2f mm (theo %d buoc/mm) | busy=%s | cong tac TREN %s DUOI %s' % (
        time.time() - t0, t['lift_pos'], d, d / lc['steps_per_mm'], lc['steps_per_mm'], t['lift_busy'],
        'CHAM' if t['limit_top'] else 'ho', 'CHAM' if t['limit_bot'] else 'ho'), flush=True)
    if not seen_busy:
        print('CANH BAO: firmware khong bao lift chay (cham cong tac phia do? nguon 12 V tat?)', flush=True)
    link.close()
