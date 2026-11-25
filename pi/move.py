#!/usr/bin/env python3
"""Mot dong tac don: python3 move.py <tien|lui|trai|phai> <cm> [toc do xung/s]  |  move.py xoay <do> [toc do]
   move.py huong <gio> <cm> [toc do]   # chay theo HUONG DONG HO, than xe giu nguyen: 12 = tien, 9 = trai, 3 = phai, 6 = lui,
                                       # 10.5 = 10 gio 30 (cheo truoc-trai 45 do), 1.5 = 1 gio 30 (cheo truoc-phai)
Chay doc lap SSH duoc (nohup). Dung som: tao file STOP."""
import sys

import demo_moves as D

kind = sys.argv[1]
amount = float(sys.argv[2])
speed = int(sys.argv[3]) if len(sys.argv) > 3 and kind != 'huong' else 1200
vec = {'tien': (1, 0), 'lui': (-1, 0), 'trai': (0, 1), 'phai': (0, -1)}
d = D.Demo('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', speed)   # ten co dinh STM32 (ttyUSBx doi theo thu tu cam)
try:
    if kind == 'xoay':
        d.rotate('xoay %+.0f' % amount, amount)
    elif kind == 'huong':
        import math
        hour, cm = amount, float(sys.argv[3])
        speed = int(sys.argv[4]) if len(sys.argv) > 4 else 1200
        th = math.radians((12 - hour) * 30.0)          # 12 gio = 0 do; nguoc chieu kim (ve ben TRAI) = duong
        d.move('huong %gh %.0fcm' % (hour, cm), speed * math.cos(th), speed * math.sin(th), cm * 10, ramp_s=0.8)
    else:
        vx, vy = vec[kind]
        d.move('%s %.0fcm' % (kind, amount), vx * speed, vy * speed, amount * 10, ramp_s=0.8)
finally:
    d.close()
