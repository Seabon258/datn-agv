#!/usr/bin/env python3
"""Cho xe chay toi-lui lap lai va danh gia (can uart_logger.py dang chay, firmware co lenh "b").

    fb_test.py <so luot> [toc do xung/s] [giu huong 0/1] ["lenh cai dat" ...]

Moi luot: "b +v 0 0 hold" 3 s roi "b -v 0 0 hold" 3 s. In cho moi chieu:
quang duong (cm, tu encoder), lech huong cuoi + lac toi da (do, tu gyro), banh khoi dong tre nhat (s),
chenh lech 4 banh (%). Xe phai dat duoi san, phia truoc/sau trong ~35 cm.
"""
import glob
import re
import sys
import time

FIFO = '/tmp/agv_cmd'
CPR = 3960.0                       # xung / vong banh (uoc tinh tu dong co 111 vong/phut, 1:90)
CIRC_CM = 3.14159 * 8.0            # banh mecanum 80 mm
ST = re.compile(r'^(\d\d:\d\d:\d\d) ENC .*\| V (-?\d+) (-?\d+) (-?\d+) (-?\d+) \|.*\| H (-?\d+) ')
DONE = re.compile(r'xong, xung encoder trong \d+ ms: (-?\d+) (-?\d+) (-?\d+) (-?\d+)')


def send(cmd):
    with open(FIFO, 'w') as f:
        f.write(cmd + '\n')
    time.sleep(0.15)


def log_path():
    return sorted(glob.glob('/home/bon/datn/logs/uart_*.log'))[-1]


def run(cmd, v):
    log = log_path()
    with open(log) as f:
        f.seek(0, 2)
        pos = f.tell()
    send(cmd)
    time.sleep(3.6)
    with open(log) as f:
        f.seek(pos)
        lines = f.read().splitlines()
    rows = [m for m in (ST.match(l) for l in lines) if m]
    done = next((m for m in (DONE.search(l) for l in lines) if m), None)
    if not rows or not done:
        return None
    cnt = [int(done.group(i)) for i in range(1, 5)]
    h = [int(m.group(6)) / 10.0 for m in rows]
    sp = [[int(m.group(2 + i)) for m in rows] for i in range(4)]
    dt = 3.0 / max(1, len(rows))
    starts = []
    for i in range(4):
        k = next((k for k, x in enumerate(sp[i]) if abs(x) > 0.15 * abs(v)), None)
        starts.append(k * dt if k is not None else 9.9)
    mean = sum(abs(c) for c in cnt) / 4
    return {
        'cm': mean / CPR * CIRC_CM,
        'drift': h[-1] - h[0],
        'wobble': max(abs(x - h[0]) for x in h),
        'late': max(starts) - min(starts),
        'late_wheel': 'M%d' % (starts.index(max(starts)) + 1),
        'spread': (max(abs(c) for c in cnt) - min(abs(c) for c in cnt)) / max(1, mean) * 100,
        'cnt': cnt,
    }


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    v = int(sys.argv[2]) if len(sys.argv) > 2 else 1500
    hold = sys.argv[3] if len(sys.argv) > 3 else '1'
    send('')
    for c in sys.argv[4:]:
        send(c)
    send('r 50')
    print(f'{"luot":>4} {"chieu":>5} {"cm":>6} {"lech":>6} {"lac":>6} {"tre(s)":>7} {"chenh%":>7}  xung M1..M4')
    for k in range(1, n + 1):
        for sign, name in ((1, 'toi'), (-1, 'lui')):
            r = run(f'b {sign * v} 0 0 {hold}', v)
            if r is None:
                print(f'{k:>4} {name:>5}   (khong co du lieu - mat UART / mach reset?)')
            else:
                print(f'{k:>4} {name:>5} {r["cm"]:6.1f} {r["drift"]:+6.1f} {r["wobble"]:6.1f} '
                      f'{r["late"]:5.2f}{r["late_wheel"]:>3} {r["spread"]:6.1f}   {r["cnt"]}', flush=True)
            time.sleep(1.0)
    send('r 500')


if __name__ == '__main__':
    main()
