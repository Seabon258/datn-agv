#!/usr/bin/env python3
"""Chay thu buoc nhay toc do 4 banh va cham diem PID (can uart_logger.py dang chay).

    pid_step.py <toc do xung/s> [KF KP KI KS]   (he so tinh theo phan nghin, bo trong = giu nguyen)

In ra cho tung banh: thoi gian len 90%, vot lo %, toc do on dinh TB, do rung (do lech chuan),
va do lac cua xe (gyro Z). BANH KE LEN hoac xe dat duoi san co cho trong.
"""
import glob
import re
import statistics as st
import sys
import time

FIFO = '/tmp/agv_cmd'
LINE = re.compile(r'^(\d\d):(\d\d):(\d\d) ENC .*\| V (-?\d+) (-?\d+) (-?\d+) (-?\d+) \|.*G (-?\d+) (-?\d+) (-?\d+)')


def send(cmd):
    with open(FIFO, 'w') as f:
        f.write(cmd + '\n')
    time.sleep(0.15)


def main():
    target = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
    log = sorted(glob.glob('/home/bon/datn/logs/uart_*.log'))[-1]
    send('')
    if len(sys.argv) >= 6:
        send('k ' + ' '.join(sys.argv[2:6]))
    send('r 20')                                   # 50 dong trang thai / giay
    time.sleep(0.5)
    with open(log) as f:
        f.seek(0, 2)
        start = f.tell()
    t0 = time.time()
    send(f'v {target} {target} {target} {target}')
    time.sleep(2.6)
    send('r 500')
    with open(log) as f:
        f.seek(start)
        rows = [LINE.match(l) for l in f]
    rows = [m for m in rows if m]
    if len(rows) < 20:
        print('Qua it du lieu (', len(rows), 'dong) - kiem tra mach / UART')
        return
    v = [[int(m.group(4 + i)) for m in rows] for i in range(4)]
    gz = [int(m.group(10)) for m in rows]
    moving = [k for k in range(len(rows)) if max(abs(v[i][k]) for i in range(4)) > 0.1 * abs(target)]
    if not moving:
        print('Banh khong quay - kiem tra 12V / day dong co')
        return
    k0, k1 = moving[0], moving[-1]
    dt = 2.0 / max(1, k1 - k0)                     # lenh "v" chay dung 2 s
    ks0, ks1 = k0 + int(1.0 / dt), k1 - int(0.15 / dt)   # vung on dinh: tu 1.0 s toi gan luc dung
    print(f'Muc tieu {target} xung/s, ~{dt * 1000:.0f} ms/mau, {k1 - k0} mau khi chay')
    print('banh  len90%(s)  vot_lo%  on_dinh_TB  rung(sd)')
    steady_all = []
    for i in range(4):
        s = v[i][k0:k1 + 1]
        k90 = next((k for k, x in enumerate(s) if abs(x) >= 0.9 * abs(target)), None)
        peak = max(s, key=abs)
        tail = v[i][ks0:ks1] or s
        steady_all.append(st.mean(tail))
        over = (abs(peak) - abs(target)) / abs(target) * 100
        print(f'M{i + 1}    {k90 * dt if k90 is not None else float("nan"):6.2f}    {over:6.1f}   '
              f'{st.mean(tail):8.0f}    {st.pstdev(tail):6.0f}')
    print(f'chenh lech giua 4 banh (on dinh): {max(steady_all) - min(steady_all):.0f} xung/s')
    g = gz[k0:k1 + 1]
    print(f'xe lac (gyro Z khi chay): sd {st.pstdev(g):.0f}, max {max(g, key=abs)}  (65.5 = 1 do/s)')


if __name__ == '__main__':
    main()
