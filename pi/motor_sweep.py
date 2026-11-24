#!/usr/bin/env python3
"""Quet toc do 1 dong co qua console chu (can uart_logger.py dang chay).
Chay: motor_sweep.py <so dong co 1-4> [buoc %] [+|-|both]   - BANH PHAI KE LEN"""
import glob, re, sys, time
ch = int(sys.argv[1]); step = int(sys.argv[2]) if len(sys.argv) > 2 else 10
dirs = {'+': (1,), '-': (-1,)}.get(sys.argv[3] if len(sys.argv) > 3 else 'both', (1, -1))
log = sorted(glob.glob('/home/bon/datn/logs/uart_*.log'))[-1]

def send(cmd):
    with open('/tmp/agv_cmd', 'w') as f:
        f.write(cmd + '\n')

def speeds_since(pos):
    with open(log) as f:
        f.seek(pos)
        return [int(m.group(ch)) for m in (re.search(r'\| V (-?\d+) (-?\d+) (-?\d+) (-?\d+)', l) for l in f) if m]

rows = []
for sign in dirs:
    for pct in range(step, 101, step):
        send(f'm {ch} {sign * pct}')
        time.sleep(1.2)                       # cho tang toc
        with open(log) as f:
            f.seek(0, 2); pos = f.tell()
        time.sleep(0.75)                      # lay toc do on dinh (~1-2 dong trang thai)
        v = speeds_since(pos)
        rows.append((sign * pct, sum(v) / len(v) if v else float('nan'), min(v, default=0), max(v, default=0)))
    send('s'); time.sleep(1.5)
send('s')
print(f'M{ch}  lenh%   toc do TB (xung/s)   min..max')
for pct, avg, lo, hi in rows:
    print(f'      {pct:+4d}   {avg:9.0f}          {lo}..{hi}')
