#!/usr/bin/env python3
"""Doc log UART, bao moi khi banh nao BAT DAU hoac NGUNG quay (theo toc do V trong dong trang thai).
Chay: enc_watch.py <so giay>"""
import glob, os, re, sys, time
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 1200
MOVE_CPS = 300            # |toc do| lon hon nguong nay = dang quay
log = sorted(glob.glob('/home/bon/datn/logs/uart_*.log'))[-1]
f = open(log); f.seek(0, os.SEEK_END)
state, end = [False] * 4, time.time() + dur
print(time.strftime('%H:%M:%S'), 'BAT DAU THEO DOI 4 BANH', flush=True)
while time.time() < end:
    line = f.readline()
    if not line:
        time.sleep(0.1)
        newer = sorted(glob.glob('/home/bon/datn/logs/uart_*.log'))[-1]
        if newer != log:                       # qua ngay moi
            log = newer; f = open(log)
        continue
    m = re.search(r'\| V (-?\d+) (-?\d+) (-?\d+) (-?\d+)', line)
    if not m:
        continue
    v = [int(x) for x in m.groups()]
    for i in range(4):
        on = abs(v[i]) > MOVE_CPS
        if on != state[i]:
            state[i] = on
            print(time.strftime('%H:%M:%S'), f'M{i+1} {"QUAY" if on else "NGUNG"} ({v[i]:+d} xung/s)',
                  '| dang quay:', ' '.join(f'M{j+1}' for j in range(4) if state[j]) or 'khong', flush=True)
print(time.strftime('%H:%M:%S'), 'HET GIO THEO DOI', flush=True)
