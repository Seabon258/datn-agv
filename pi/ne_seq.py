#!/usr/bin/env python3
"""Thu QUY DAO NE (khong line, khong LiDAR): 10 gio 30 50 cm -> 12 gio 60 cm -> 1 gio 30 50 cm. Dung som: file STOP."""
import math, sys, time
import demo_moves as D

PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
steps = [(10.5, 50), (12, 60), (1.5, 50)]
sp = 1200
d = D.Demo(PORT, sp)
try:
    t = d.link.telem
    e0, h0 = list(t['enc']), t['heading_deg']
    for hour, cm in steps:
        th = math.radians((12 - hour) * 30.0)
        a = list(d.link.telem['enc'])
        d.move('huong %gh %dcm' % (hour, cm), sp * math.cos(th), sp * math.sin(th), cm * 10, ramp_s=0.8)
        x, y = D.wheel_motion(d.link.telem['enc'], a)
        print('    -> goc thuc te %.1f do (lenh %.0f), quang %.1f cm' % (math.degrees(math.atan2(y, x)), (12 - hour) * 30, math.hypot(x, y) / 10), flush=True)
    x, y = D.wheel_motion(d.link.telem['enc'], e0)
    print('TONG: tien %.1f cm, lech ngang %+.1f cm (muc tieu 0), huong than xe %+.1f do' % (x / 10, y / 10, d.link.telem['heading_deg'] - h0), flush=True)
finally:
    d.close()
