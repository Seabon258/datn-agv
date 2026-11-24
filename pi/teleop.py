#!/usr/bin/env python3
"""Lai thu xe mecanum bang ban phim tren Raspberry Pi (qua SSH cung duoc).

    python3 teleop.py [/dev/serial0] [--pwm]

Phim:  W/S toi-lui   A/D sang trai-phai   Q/E xoay trai-phai
       +/- tang/giam toc do   SPACE dung   U/J nang-ha 1 vong   H ve goc   X thoat
Nhan phim 1 lan la giu lenh do; SPACE de dung. Thoat hoac chuong trinh loi -> xe tu dung.

--pwm : gui PWM tho (khi PID chua chinh). Mac dinh gui toc do (xung/s) cho PID tren STM32.
BANH KE LEN o lan chay dau tien.
"""
import curses
import sys
import time

from stm32_link import Stm32Link

# Dau xe = phia M3, M4. Cach cam 2026-11-20: M4 truoc-trai, M3 truoc-phai, M2 sau-trai, M1 sau-phai.
# Dau tung banh da xu ly trong firmware (MOTOR_SIGN/ENC_SIGN): "+" = banh lan TOI.
# Neu chay W ma banh nao van quay nguoc thi doi dau no o day.
WHEEL_SIGN = [1, 1, 1, 1]

SPEED_STEPS_CPS = [800, 1500, 2500, 3500, 5000]     # che do PID, xung/s
SPEED_STEPS_PWM = [2500, 3000, 3500, 4000, 5000]    # che do --pwm, /5000 (lui can > ~40%)
LIFT_STEPS = 1600                                    # 1 vong dong co buoc


def mecanum(vx, vy, wz):
    """vx toi +, vy trai +, wz xoay trai + (moi cai -1..1) -> he so 4 banh M1..M4."""
    fl = vx - vy - wz
    rl = vx + vy - wz
    rr = vx - vy + wz
    fr = vx + vy + wz
    w = [rr, rl, fr, fl]                            # thu tu kenh M1..M4 (chi dung cho --pwm)
    m = max(1.0, max(abs(x) for x in w))           # giu ti le khi vuot 1
    return [WHEEL_SIGN[i] * w[i] / m for i in range(4)]


KEYS = {
    'w': (1, 0, 0), 's': (-1, 0, 0),
    'a': (0, 1, 0), 'd': (0, -1, 0),
    'q': (0, 0, 1), 'e': (0, 0, -1),
}


def main(scr, port, use_pwm):
    curses.curs_set(0)
    scr.nodelay(True)
    link = Stm32Link(port)
    steps = SPEED_STEPS_PWM if use_pwm else SPEED_STEPS_CPS
    level, motion, note = 1, (0, 0, 0), ''
    try:
        while True:
            k = scr.getch()
            if k != -1:
                c = chr(k).lower() if 0 <= k < 256 else ''
                if c in KEYS:
                    motion = KEYS[c]
                elif c == ' ':
                    motion = (0, 0, 0)
                elif c in '+=':
                    level = min(level + 1, len(steps) - 1)
                elif c in '-_':
                    level = max(level - 1, 0)
                elif c == 'u':
                    link.lift_move(LIFT_STEPS); note = 'nang 1 vong'
                elif c == 'j':
                    link.lift_move(-LIFT_STEPS); note = 'ha 1 vong'
                elif c == 'h':
                    link.lift_home(); note = 've goc'
                elif c == 'x':
                    break

            cmd = [round(x * steps[level]) for x in mecanum(*motion)]
            if motion == (0, 0, 0):
                link.stop() if link._cmd is not None else None
            elif use_pwm:
                link.set_pwm(cmd)
            else:                                       # STM32 tu chia 4 banh + giu huong bang gyro
                vx, vy, wz = (round(m * steps[level]) for m in motion)
                link.drive(vx, vy, wz, hold=(wz == 0))

            t = link.telem
            scr.erase()
            scr.addstr(0, 0, f"AGV teleop  {'PWM' if use_pwm else 'PID'}  muc {level + 1}/{len(steps)} = {steps[level]}")
            scr.addstr(1, 0, "W/S toi-lui  A/D ngang  Q/E xoay  +/- toc do  SPACE dung  U/J nang-ha  H goc  X thoat")
            scr.addstr(3, 0, f"lenh banh M1..M4: {cmd}")
            if t and time.time() - link.telem_time < 0.5:
                scr.addstr(4, 0, f"toc do do duoc : {t['speed']}")
                scr.addstr(5, 0, f"pin {t['batt_v']:.2f} V   nang {t['lift_pos']} buoc   IMU {'OK' if t['imu_ok'] else 'LOI'}"
                                 f"   {'MAT LIEN KET!' if t['timeout'] else ''}")
            else:
                scr.addstr(4, 0, "!!! KHONG NHAN DUOC TELEMETRY TU STM32 - kiem tra day UART / nguon")
            scr.addstr(7, 0, note)
            scr.refresh()
            time.sleep(0.05)                            # 20 Hz
    finally:
        link.close()                                    # gui STOP


if __name__ == '__main__':
    port = next((a for a in sys.argv[1:] if a.startswith('/dev/')), '/dev/serial0')
    curses.wrapper(main, port, '--pwm' in sys.argv)
