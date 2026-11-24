#!/usr/bin/env python3
"""
Doc dong CSV tu STM32 (USART3, TEST_STREAM=1) va ve do thi encoder truc tiep.

Dong du lieu firmware gui:  D,t_ms,motor,dir,E1,E2,E3,E4

Cach dung (chay trong terminal VS Code):
    python3 tools/enc_plot.py                    # tu tim /dev/ttyUSB* hoac /dev/ttyACM*
    python3 tools/enc_plot.py -p /dev/ttyUSB0
    python3 tools/enc_plot.py --replay logs/enc_20261025_120000.csv   # xem lai file da luu
    python3 tools/enc_plot.py --replay a.csv b.csv                     # so sanh nhieu lan chay

Moi lan chay live se luu CSV vao tools/logs/ de so sanh ve sau.
"""
import argparse
import csv
import glob
import os
import sys
import termios
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

ENC_CPR = 3960          # phai giong ENC_CPR trong main.h (count / vong banh, x4)
N_MOTOR = 4
WINDOW_S = 30.0         # so giay hien thi tren do thi live
RPM_AVG = 5             # so mau lay trung binh khi tinh RPM (5 x 20 ms = 100 ms)
COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]
HEADER = ["t_ms", "motor", "dir", "E1", "E2", "E3", "E4"]


class Serial:
    """Mo cong serial bang termios (khong can pyserial)."""

    def __init__(self, port, baud=115200):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attr = termios.tcgetattr(self.fd)
        speed = getattr(termios, f"B{baud}")
        attr[0] = 0                                      # iflag
        attr[1] = 0                                      # oflag
        attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attr[3] = 0                                      # lflag: raw
        attr[4] = attr[5] = speed
        termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.buf = b""

    def lines(self):
        try:
            self.buf += os.read(self.fd, 4096)
        except BlockingIOError:
            pass
        *done, self.buf = self.buf.split(b"\n")
        return [l.decode(errors="ignore").strip() for l in done]


def find_port():
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("Khong thay cong serial nao (/dev/ttyUSB*, /dev/ttyACM*). Cam mach USB-UART vao PD8 chua?")
    return ports[0]


def parse(line):
    """'D,t,m,dir,e1,e2,e3,e4' -> [t, m, dir, e1..e4] hoac None."""
    if not line.startswith("D,"):
        return None
    try:
        v = [int(x) for x in line[2:].split(",")]
    except ValueError:
        return None
    return v if len(v) == 3 + N_MOTOR else None


def rpm_series(t_s, counts):
    """RPM tu chenh lech count qua RPM_AVG mau."""
    out = []
    for i in range(len(t_s)):
        j = max(0, i - RPM_AVG)
        dt = t_s[i] - t_s[j]
        out.append((counts[i] - counts[j]) / ENC_CPR / dt * 60.0 if dt > 0 else 0.0)
    return out


def make_axes(title):
    fig, (ax_rev, ax_rpm) = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
    fig.suptitle(title)
    ax_rev.set_ylabel("So vong banh")
    ax_rpm.set_ylabel("Toc do (RPM)")
    ax_rpm.set_xlabel("Thoi gian (s)")
    for ax in (ax_rev, ax_rpm):
        ax.grid(True, alpha=0.3)
    return fig, ax_rev, ax_rpm


def live(port, motors):
    ser = Serial(port)
    os.makedirs(os.path.join(os.path.dirname(__file__), "logs"), exist_ok=True)
    log_path = os.path.join(os.path.dirname(__file__), "logs",
                            time.strftime("enc_%Y%m%d_%H%M%S.csv"))
    log_f = open(log_path, "w", newline="")
    log = csv.writer(log_f)
    log.writerow(HEADER)
    print(f"Doc {port} @115200, luu vao {log_path}. Dong cua so de dung.")

    maxlen = int(WINDOW_S * 50) + 10
    t = deque(maxlen=maxlen)
    cnt = [deque(maxlen=maxlen) for _ in range(N_MOTOR)]
    state = {"t0": None, "c0": None, "motor": 0, "dir": 0}

    fig, ax_rev, ax_rpm = make_axes(f"Encoder live - {port}")
    l_rev = {m: ax_rev.plot([], [], color=COLORS[m], label=f"M{m+1}")[0] for m in motors}
    l_rpm = {m: ax_rpm.plot([], [], color=COLORS[m], label=f"M{m+1}")[0] for m in motors}
    ax_rev.legend(loc="upper left")
    info = ax_rev.text(0.99, 0.95, "", transform=ax_rev.transAxes, ha="right", va="top",
                       family="monospace", fontsize=9,
                       bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"))

    def update(_):
        for line in ser.lines():
            v = parse(line)
            if v is None:
                if line:
                    print(line)            # in cac dong text khac (banner, loi...)
                continue
            log.writerow(v)
            tm, state["motor"], state["dir"], *c = v
            if state["t0"] is None:
                state["t0"], state["c0"] = tm, c
            t.append((tm - state["t0"]) / 1000.0)
            for i in range(N_MOTOR):
                cnt[i].append(c[i] - state["c0"][i])
        if len(t) < 2:
            return []
        ts = list(t)
        txt = [f"Motor dang chay: M{state['motor']}  dir={state['dir']:+d}"]
        for m in motors:
            c = list(cnt[m])
            rpm = rpm_series(ts, c)
            l_rev[m].set_data(ts, [x / ENC_CPR for x in c])
            l_rpm[m].set_data(ts, rpm)
            txt.append(f"M{m+1}: {c[-1] / ENC_CPR:8.2f} vong  {rpm[-1]:7.1f} rpm")
        info.set_text("\n".join(txt))
        for ax in (ax_rev, ax_rpm):
            ax.relim()
            ax.autoscale_view()
        ax_rpm.set_xlim(max(0.0, ts[-1] - WINDOW_S), max(ts[-1], 1.0))
        return []

    _anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    plt.show()
    log_f.close()
    os.close(ser.fd)
    print(f"Da luu {log_path}")


def replay(paths, motors):
    fig, ax_rev, ax_rpm = make_axes("So sanh: " + ", ".join(os.path.basename(p) for p in paths))
    styles = ["-", "--", ":", "-."]
    for k, path in enumerate(paths):
        with open(path) as f:
            rows = [[int(x) for x in r] for r in csv.reader(f) if r and r[0].lstrip("-").isdigit()]
        if not rows:
            print(f"{path}: khong co du lieu")
            continue
        t0 = rows[0][0]
        ts = [(r[0] - t0) / 1000.0 for r in rows]
        name = os.path.splitext(os.path.basename(path))[0]
        print(f"\n{path}  ({ts[-1]:.1f} s)")
        for m in motors:
            c = [r[3 + m] - rows[0][3 + m] for r in rows]
            fwd = sum(max(0, b - a) for a, b in zip(c, c[1:])) / ENC_CPR
            rev = sum(min(0, b - a) for a, b in zip(c, c[1:])) / ENC_CPR
            rpm = rpm_series(ts, c)
            print(f"  M{m+1}: tong {c[-1]/ENC_CPR:+8.2f} vong | toi {fwd:7.2f} | lui {rev:7.2f}"
                  f" | max {max(rpm):6.1f} rpm, min {min(rpm):6.1f} rpm")
            lbl = f"M{m+1}" + (f" ({name})" if len(paths) > 1 else "")
            ax_rev.plot(ts, [x / ENC_CPR for x in c], styles[k % 4], color=COLORS[m], label=lbl)
            ax_rpm.plot(ts, rpm, styles[k % 4], color=COLORS[m], label=lbl)
    ax_rev.legend(loc="upper left", fontsize=8)
    plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="cong serial, mac dinh tu tim")
    ap.add_argument("-m", "--motors", default="1,2,3,4", help="motor can ve, vd 1 hoac 1,3 (mac dinh 1,2,3,4)")
    ap.add_argument("--replay", nargs="+", metavar="CSV", help="ve lai file CSV da luu")
    a = ap.parse_args()
    motors = [int(x) - 1 for x in a.motors.split(",")]
    if a.replay:
        replay(a.replay, motors)
    else:
        live(a.port or find_port(), motors)


if __name__ == "__main__":
    main()
