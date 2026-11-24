#!/usr/bin/env python3
"""LiDAR 2D (Camsense-style, packet type 0x02, 8 points x 2 bytes) on a USB serial port, pure Python.

Packet (28 bytes, little-endian), decoded from the real device 2026-11-22 at 115200 baud:
    55 AA | type(0x02) | count(0x08) | speed u16 (rpm*64) | start_angle u16 | count * dist u16 (mm) |
    end_angle u16 | check u16 (not verified yet -> packets are validated by structure + angle continuity)
    angle_deg = (raw - 0xA000) / 64

    lid = Lidar(port, baud); lid.start()
    lid.scan      -> latest complete 360 deg scan: list of (angle_deg, dist_mm), angle in the LIDAR frame
    lid.scan_hz, lid.rpm, lid.bad_packets

Run alone:  python3 lidar.py [port]   -> prints nearest distance per sector, saves lidar_scan.png every 2 s
"""
import math
import struct
import threading
import time

import serial

HDR = b'\x55\xaa'
PKT_LEN = 28
N_PTS = 8


def _angle(raw):
    return ((raw - 0xA000) / 64.0) % 360.0


class Lidar:
    def __init__(self, port, baud=115200, min_mm=50, max_mm=8000):
        self.port, self.baud = port, baud
        self.min_mm, self.max_mm = min_mm, max_mm
        self.scan, self.scan_t, self.scan_hz, self.rpm = [], 0.0, 0.0, 0.0
        self.bad_packets = self.good_packets = 0
        self._cur, self._last_a = [], None
        self._times = []
        self._stop = threading.Event()
        self._ser = None
        self._th = None

    def start(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()
        return self

    def close(self):
        self._stop.set()
        if self._th:
            self._th.join(1.0)
        if self._ser:
            self._ser.close()

    def wait_ready(self, timeout=3.0, retries=3, log=print):
        """Cho vong quet dau tien; khong co thi DONG/MO LAI cong (toi da `retries` lan). True neu co du lieu."""
        for k in range(retries):
            t0 = time.time()
            while not self.scan and time.time() - t0 < timeout:
                time.sleep(0.05)
            if self.scan:
                return True
            log('LiDAR lan %d: chua co vong quet sau %.0f s (goi tot %d, loi %d, luong doc %s) -> mo lai cong' % (
                k + 1, timeout, self.good_packets, self.bad_packets, 'song' if self._th and self._th.is_alive() else 'CHET'))
            self.close()
            time.sleep(0.5)
            self._stop.clear()
            self.scan, self.scan_t, self._cur, self._last_a = [], 0.0, [], None
            self.good_packets = self.bad_packets = 0
            self.start()
        return bool(self.scan)

    def age(self):
        """Seconds since the last complete scan (stale check: treat as no data if large)."""
        return time.time() - self.scan_t if self.scan_t else float('inf')

    def _run(self):
        buf = b''
        while not self._stop.is_set():
            try:
                buf += self._ser.read(512)
            except serial.SerialException:
                time.sleep(0.2)
                continue
            while True:
                i = buf.find(HDR)
                if i < 0:
                    buf = buf[-1:]
                    break
                if len(buf) - i < PKT_LEN:
                    buf = buf[i:]
                    break
                pkt = buf[i:i + PKT_LEN]
                if pkt[2] != 0x02 or pkt[3] != N_PTS:
                    self.bad_packets += 1
                    buf = buf[i + 1:]                       # resync: drop one byte, search again
                    continue
                self._handle(pkt)
                buf = buf[i + PKT_LEN:]

    def _handle(self, pkt):
        speed, a0 = struct.unpack_from('<HH', pkt, 4)
        dists = struct.unpack_from('<%dH' % N_PTS, pkt, 8)
        a1 = struct.unpack_from('<H', pkt, 8 + 2 * N_PTS)[0]
        s, e = _angle(a0), _angle(a1)
        span = (e - s) % 360.0
        if span > 30.0:                                      # one packet covers a few degrees only
            self.bad_packets += 1
            return
        self.good_packets += 1
        self.rpm = speed / 64.0
        if self._last_a is not None and s < self._last_a - 180.0:     # wrapped past 360 -> scan complete
            now = time.time()
            if len(self._cur) >= 30:                         # bo vong qua it diem (wrap gia do goi hong)
                self._times.append(now)
                self._times = [t for t in self._times if now - t <= 2.0]
                if len(self._times) >= 2:
                    self.scan_hz = (len(self._times) - 1) / (self._times[-1] - self._times[0])
                self.scan, self.scan_t = self._cur, now
            self._cur = []
        self._last_a = s
        for k, d in enumerate(dists):
            if self.min_mm <= d <= self.max_mm:
                self._cur.append(((s + span * k / (N_PTS - 1)) % 360.0, d))


def sector_min(scan, center_deg, half_deg):
    """Nearest distance (mm) in [center-half, center+half] (LIDAR frame), or None."""
    best = None
    for a, d in scan:
        if abs((a - center_deg + 180.0) % 360.0 - 180.0) <= half_deg and (best is None or d < best):
            best = d
    return best


def save_plot(scan, path, max_mm=3000):
    import cv2
    import numpy as np
    S = 500
    img = np.full((S, S, 3), 255, np.uint8)
    c = S // 2
    for r in range(500, max_mm + 1, 500):
        cv2.circle(img, (c, c), int(r / max_mm * c), (220, 220, 220), 1)
        cv2.putText(img, '%.1fm' % (r / 1000), (c + 3, c - int(r / max_mm * c) + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
    cv2.line(img, (c, c), (c, 0), (200, 200, 255), 1)
    cv2.putText(img, '0 do (huong goc lidar)', (c + 4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 200), 1)
    for a, d in scan:
        if d > max_mm:
            continue
        rr = d / max_mm * c
        x = int(c + rr * math.sin(math.radians(a)))           # 0 deg = len tren, goc tang theo chieu kim dong ho
        y = int(c - rr * math.cos(math.radians(a)))
        cv2.circle(img, (x, y), 2, (0, 0, 0), -1)
    cv2.circle(img, (c, c), 4, (0, 0, 255), -1)
    cv2.imwrite(path, img)


if __name__ == '__main__':
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    lid = Lidar(port).start()
    t0 = time.time()
    last_png = 0.0
    try:
        while time.time() - t0 < float(sys.argv[2]) if len(sys.argv) > 2 else True:
            time.sleep(0.5)
            sc = lid.scan
            if not sc:
                print('chua co vong quet (goi tot %d, loi %d)' % (lid.good_packets, lid.bad_packets), flush=True)
                continue
            f = ['%4s: %5s' % (n, ('%d' % m) if m else '-') for n, m in
                 (('0', sector_min(sc, 0, 30)), ('90', sector_min(sc, 90, 30)), ('180', sector_min(sc, 180, 30)), ('270', sector_min(sc, 270, 30)))]
            print('%.1f vong/s %3.0f rpm | %3d diem/vong | gan nhat (mm) theo huong lidar: %s | goi loi %d' % (
                lid.scan_hz, lid.rpm, len(sc), '  '.join(f), lid.bad_packets), flush=True)
            if time.time() - last_png > 2.0:
                save_plot(sc, 'lidar_scan.png')
                last_png = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        lid.close()
