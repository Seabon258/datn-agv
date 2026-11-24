#!/usr/bin/env python3
"""
Danh gia chat luong motor + chuyen dong tu file CSV cua enc_plot.py.

    python3 tools/enc_analyze.py tools/logs/enc_xxx.csv            # tu nhan motor dang chay (cot motor = mask)
    python3 tools/enc_analyze.py tools/logs/a.csv tools/logs/b.csv # so sanh nhieu file (vd M1..M4)
    python3 tools/enc_analyze.py file.csv --png out.png            # luu hinh

Chi so (moi doan chay toi / lui, toc do on dinh lay trong 2 s cuoi doan):
  rpm TB, do dao dong (std, %), p2p     -> do em / do on dinh toc do
  t90                                   -> thoi gian dat 90 % toc do on dinh tu luc bat dau
  quan tinh                             -> so vong + thoi gian troi sau khi PWM = 0
  lech toi/lui (%)                      -> do bat doi xung cua motor/hop so
  lap lai (std rpm giua cac chu ky)     -> do on dinh giua cac lan chay
  tan so dao dong chinh vs tan so quay  -> dao dong 1 lan/vong = banh/truc lech tam
  loi encoder                           -> mau nhay bat thuong, dem nguoc chieu
  ket                                   -> doan co lenh chay ma motor khong quay
"""
import argparse
import csv
import os
import statistics as st

import matplotlib.pyplot as plt

from enc_plot import ENC_CPR, N_MOTOR, COLORS

STEADY_S = 2.0      # toc do on dinh lay trong STEADY_S giay cuoi moi doan chay
STALL_RPM = 5.0     # doan chay co rpm TB duoi muc nay -> motor ket / mat nguon
AVG = 5             # cua so tinh rpm (5 mau x 20 ms)


def load(path):
    with open(path) as f:
        rows = [[int(x) for x in r] for r in csv.reader(f) if r and r[0].lstrip("-").isdigit()]
    t0 = rows[0][0]
    t = [(r[0] - t0) / 1000.0 for r in rows]
    return t, rows


def rpm_at(t, c, i, n=AVG):
    j = max(0, i - n)
    return (c[i] - c[j]) / ENC_CPR / (t[i] - t[j]) * 60.0 if t[i] > t[j] else 0.0


def segments(rows):
    """Tach thanh cac doan lien tiep cung dir: [(dir, i_start, i_end)]."""
    seg, s = [], 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i][2] != rows[s][2]:
            seg.append((rows[s][2], s, i - 1))
            s = i
    return seg


def dominant_freq(x, fs):
    """Tan so co bien do lon nhat (DFT don gian, bo DC)."""
    n = len(x)
    m = sum(x) / n
    x = [v - m for v in x]
    best = (0.0, 0.0)
    import cmath
    for k in range(1, n // 2):
        a = abs(sum(x[i] * cmath.exp(-2j * cmath.pi * k * i / n) for i in range(n))) * 2 / n
        if a > best[1]:
            best = (k * fs / n, a)
    return best


def motors_in(rows):
    """Cot motor la mask: bit0=M1 ... bit3=M4 (log cu: 1 = M1, 2 = M2 van dung)."""
    mask = rows[-1][1]
    return [m + 1 for m in range(N_MOTOR) if mask & (1 << m)]


def analyze(path, motor):
    t, rows = load(path)
    m = motor - 1
    c = [r[3 + m] - rows[0][3 + m] for r in rows]
    rpm = [rpm_at(t, c, i) for i in range(len(t))]
    seg = segments(rows)
    dt = st.median(b - a for a, b in zip(t, t[1:]))

    res = {"path": path, "motor": motor, "t": t, "rpm": rpm, "rev": [x / ENC_CPR for x in c],
           "run": {1: [], -1: []}, "coast": [], "stall": [], "reversed": False}

    # Loi encoder: buoc nhay > 3x trung binh doan, hoac dem nguoc chieu khi dang chay on dinh
    glitch = 0
    for k, (d, a, b) in enumerate(seg):
        if d != 0:
            if a == 0 or b == len(t) - 1 or t[b] - t[a] < STEADY_S + 1.0:
                continue                      # doan bi cat o dau/cuoi file
            s = next(i for i in range(a, b + 1) if t[b] - t[i] <= STEADY_S)
            v = rpm[s:b + 1]
            raw = [(c[i] - c[i - 1]) / ENC_CPR / (t[i] - t[i - 1]) * 60 for i in range(s + 1, b + 1)]
            mean = st.mean(v)
            if abs(mean) < STALL_RPM:
                res["stall"].append((d, t[a], t[b]))
                continue
            if mean * d < 0:
                res["reversed"] = True
            for x in raw:
                if abs(x) > 3 * abs(mean) or x * mean < 0:
                    glitch += 1
            t90 = next((t[i] - t[a] for i in range(a, b + 1) if abs(rpm[i]) >= 0.9 * abs(mean)), float("nan"))
            f, amp = dominant_freq(raw, 1.0 / dt)
            res["run"][d].append(dict(mean=mean, std=st.pstdev(v), p2p=max(v) - min(v), t90=t90,
                                      f=f, amp=amp, frot=abs(mean) / 60.0,
                                      rev=(c[b] - c[a]) / ENC_CPR, dur=t[b] - t[a]))
        elif k > 0 and seg[k - 1][0] != 0 and b != len(t) - 1:
            stop = next((i for i in range(a + AVG, b + 1) if abs(rpm[i]) < 1.0), b)
            res["coast"].append(dict(dir=seg[k - 1][0], rev=(c[b] - c[a]) / ENC_CPR, t=t[stop] - t[a]))
    res["glitch"] = glitch
    return res


def report(r):
    print(f"\n=== {os.path.basename(r['path'])}  -  Motor M{r['motor']} ({r['t'][-1]:.0f} s du lieu) ===")
    summ = {}
    for d, name in ((1, "Toi"), (-1, "Lui")):
        runs = r["run"][d]
        if not runs:
            continue
        means = [abs(x["mean"]) for x in runs]
        mu = st.mean(means)
        rip = st.mean(x["std"] for x in runs)
        summ[d] = mu
        print(f"  {name}: {len(runs)} doan | rpm on dinh {mu:6.1f} | dao dong std {rip:4.2f} rpm"
              f" ({rip / mu * 100:4.1f} %) | p2p {st.mean(x['p2p'] for x in runs):5.1f} rpm"
              f" | t90 {st.mean(x['t90'] for x in runs):4.2f} s"
              f" | lap lai +-{(st.pstdev(means) if len(means) > 1 else 0):4.2f} rpm")
        f = st.mean(x["f"] for x in runs)
        fr = st.mean(x["frot"] for x in runs)
        print(f"        dao dong chinh {f:4.2f} Hz (bien do {st.mean(x['amp'] for x in runs):4.1f} rpm),"
              f" tan so quay banh {fr:4.2f} Hz -> ti le {f / fr:4.2f} x/vong")
    for d, name in ((1, "sau toi"), (-1, "sau lui")):
        co = [x for x in r["coast"] if x["dir"] == d]
        if co:
            print(f"  Quan tinh {name}: {st.mean(abs(x['rev']) for x in co):4.2f} vong trong"
                  f" {st.mean(x['t'] for x in co):4.2f} s")
    if 1 in summ and -1 in summ:
        print(f"  Lech toi/lui: {(summ[-1] - summ[1]) / ((summ[1] + summ[-1]) / 2) * 100:+5.1f} %"
              f" (duong = lui nhanh hon)")
    for d, ta, tb in r["stall"]:
        print(f"  !! KET: doan {'toi' if d > 0 else 'lui'} {ta:.1f}-{tb:.1f} s co lenh chay ma motor khong quay")
    if r["reversed"]:
        print("  !! Encoder dem NGUOC chieu lenh PWM (lenh toi -> count giam): dao C1/C2 hoac dao dau trong code")
    print(f"  Loi encoder (mau nhay / dem nguoc giua chung): {r['glitch']}")
    return summ


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--png", help="luu hinh thay vi hien cua so")
    a = ap.parse_args()

    results = [analyze(p, m) for p in a.csv for m in motors_in(load(p)[1])]
    summ = [report(r) for r in results]
    if len(results) > 1:
        print("\n=== So sanh ===")
        ref = summ[0]
        for r, s in zip(results, summ):
            diff = "".join(f"  ({'toi' if d > 0 else 'lui'} {(s[d] - ref[d]) / ref[d] * 100:+5.1f} % so voi M{results[0]['motor']})"
                           for d in (1, -1) if r is not results[0] and d in s and d in ref)
            print(f"  M{r['motor']}: toi {s.get(1, 0):6.1f} rpm, lui {s.get(-1, 0):6.1f} rpm{diff}")

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
    for r in results:
        col = COLORS[(r["motor"] - 1) % N_MOTOR]
        lbl = f"M{r['motor']} ({os.path.basename(r['path'])})"
        ax1.plot(r["t"], r["rev"], color=col, label=lbl)
        ax2.plot(r["t"], r["rpm"], color=col, label=lbl)
    ax1.set_ylabel("So vong banh")
    ax2.set_ylabel("Toc do (RPM)")
    ax2.set_xlabel("Thoi gian (s)")
    for ax in (ax1, ax2):
        ax.grid(True, alpha=0.3)
    ax1.legend(fontsize=8)
    fig.tight_layout()
    if a.png:
        fig.savefig(a.png, dpi=110)
        print(f"\nDa luu hinh {a.png}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
