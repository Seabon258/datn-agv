#!/usr/bin/env python3
"""
Ve 2 do thi so sanh toc do M1 vs M2: o 50 % duty va 100 % duty.

    python3 tools/compare_plot.py logs/enc_M1M2_xxx.csv logs/enc_M1M2_100pct_xxx.csv [--png out.png]

Toc do da doi dau theo chieu lenh (lenh toi = duong) de 2 motor so sanh truc tiep,
du encoder cua motor nao dem nguoc.
"""
import argparse
import statistics as st

import matplotlib.pyplot as plt

from enc_analyze import load, rpm_at, segments, STEADY_S
from enc_plot import ENC_CPR

SERIES = {1: ("M1", "#2a78d6"), 2: ("M2", "#eb6834")}
SHOW_S = 28.0           # ve 2 chu ky dau cho de nhin


def speed(path, motor):
    t, rows = load(path)
    m = motor - 1
    c = [r[3 + m] for r in rows]
    rpm = [rpm_at(t, c, i) for i in range(len(t))]
    # Dau encoder: so voi lenh trong cac doan chay (lenh toi -> rpm duong)
    seg = [(d, a, b) for d, a, b in segments(rows) if d != 0]
    sign = 1 if sum(d * st.mean(rpm[a:b + 1]) for d, a, b in seg) >= 0 else -1
    rpm = [sign * v for v in rpm]
    # Toc do on dinh: 2 s cuoi moi doan chay day du
    steady = {1: [], -1: []}
    for d, a, b in seg:
        if a > 0 and b < len(t) - 1:
            s = next(i for i in range(a, b + 1) if t[b] - t[i] <= STEADY_S)
            steady[d].append(abs(st.mean(rpm[s:b + 1])))
    return t, rpm, {d: st.mean(v) for d, v in steady.items() if v}


def panel(ax, path, title):
    res = {m: speed(path, m) for m in SERIES}
    for m, (name, col) in SERIES.items():
        t, rpm, _ = res[m]
        n = sum(1 for x in t if x <= SHOW_S)
        ax.plot(t[:n], rpm[:n], color=col, lw=2, label=name)
    s1, s2 = res[1][2], res[2][2]
    lines = [f"{'Toi' if d > 0 else 'Lui'}:  M1 {s1[d]:5.1f}  M2 {s2[d]:5.1f} rpm"
             f"  ({(s2[d] - s1[d]) / s1[d] * 100:+.1f} %)" for d in (1, -1) if d in s1 and d in s2]
    ax.text(0.99, 0.04, "\n".join(lines), transform=ax.transAxes, ha="right", va="bottom",
            family="monospace", fontsize=9, color="#0b0b0b",
            bbox=dict(facecolor="#fcfcfb", edgecolor="#d9d8d3", boxstyle="round,pad=0.4"))
    ax.set_title(title, loc="left", fontsize=12, color="#0b0b0b")
    ax.set_ylabel("Toc do banh (RPM)", color="#52514e")
    ax.axhline(0, color="#a8a7a0", lw=0.8)
    ax.grid(True, color="#e8e7e2", lw=0.8)
    ax.tick_params(colors="#52514e")
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.legend(loc="upper right", frameon=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv50")
    ap.add_argument("csv100")
    ap.add_argument("--png")
    a = ap.parse_args()

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), sharey=True)
    fig.patch.set_facecolor("#fcfcfb")
    for ax in (ax1, ax2):
        ax.set_facecolor("#fcfcfb")
    panel(ax1, a.csv50, "Duty 50 %  -  nguon 12 V")
    panel(ax2, a.csv100, "Duty 100 %  -  nguon 12 V")
    ax2.set_xlabel("Thoi gian (s)", color="#52514e")
    fig.suptitle(f"So sanh toc do M1 vs M2 (encoder, {ENC_CPR} count/vong, lenh toi = duong)",
                 fontsize=13, color="#0b0b0b")
    fig.tight_layout()
    if a.png:
        fig.savefig(a.png, dpi=110, facecolor=fig.get_facecolor())
        print(f"Da luu {a.png}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
