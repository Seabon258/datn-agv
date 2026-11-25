#!/usr/bin/env python3
"""Bo nho duong di (LineMemory) + bam theo bo nho + ne vat can theo quy dao - Python thuan, test duoc khong can xe.

Khung toa do (deu don vi mm, rad):
    XE   : x toi truoc, y sang TRAI, goc 0 tai tam xe (giua 4 banh)  (= khung ObstacleMonitor / ground_calib)
    ODOM : khung co dinh luc khoi dong chuong trinh, x/y mm, th rad (+ = xoay trai, nguoc chieu kim)

    odo = Odometry(mm_per_count); odo.update(telem, t)   # goi moi goi telemetry STM32 (encoder 4 banh + heading gyro)
    pose = odo.pose_at(t_anh)                            # NOI SUY theo thoi diem chup anh (khong dung pose moi nhat)
    mem = LineMemory(cfg)
    mem.observe(pts_xe, pose, t)                          # diem tam line (mm, khung XE) -> gop vao polyline ODOM
    mem.cross_track(pose) / path_ahead(...) / known_until(...) / straight_before(...)

Pha thiet ke: docs/feature_line_memory.md (P1 bo nho, P2 bam bo nho, P3 ne + nhap lai, P4 chong troi, P5 luu tuyen).
"""
import bisect
import json
import math
import threading
from collections import deque


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def to_odom(pose, x, y):
    px, py, th = pose
    c, s = math.cos(th), math.sin(th)
    return px + x * c - y * s, py + x * s + y * c


def to_car(pose, X, Y):
    px, py, th = pose
    c, s = math.cos(th), math.sin(th)
    dx, dy = X - px, Y - py
    return dx * c + dy * s, -dx * s + dy * c


def smoothstep5(u):
    u = 0.0 if u < 0 else 1.0 if u > 1 else u
    return u * u * u * (u * (u * 6 - 15) + 10)


# ======================================================================================= ODOMETRY
class Odometry:
    """Tich phan encoder mecanum (toi + ngang) theo goc gyro. Luu lich su (t, x, y, th) de noi suy theo thoi diem."""

    def __init__(self, mm_per_count, hist=400):
        self.k = mm_per_count
        self.x = self.y = 0.0
        self.th = None
        self._e = None
        self._hraw = None
        self._hunw = 0.0
        self.hist = deque(maxlen=hist)
        self.lock = threading.Lock()
        self.corr = [0.0, 0.0, 0.0]          # tong hieu chinh da ap (mm, mm, rad) - de bao cao do troi

    @staticmethod
    def body(de):
        """de = [M1 rr, M2 rl, M3 fr, M4 fl] (xung) -> (toi, trai) xung."""
        return sum(de) / 4.0, (-de[3] + de[1] - de[0] + de[2]) / 4.0

    def update(self, telem, t):
        e = telem['enc']
        h = math.radians(telem['heading_deg'])
        with self.lock:
            if self._e is None:
                self._e, self._hraw, self.th = list(e), h, 0.0
                self._hunw = 0.0
                self.hist.append((t, self.x, self.y, self.th))
                return
            self._hunw += wrap(h - self._hraw)               # telemetry goc bi cat +-180 -> mo cuon
            self._hraw = h
            de = [a - b for a, b in zip(e, self._e)]
            self._e = list(e)
            f, l = self.body(de)
            th_new = self._hunw + self.corr[2]
            thm = 0.5 * (self.th + th_new)                   # goc giua buoc
            f, l = f * self.k, l * self.k
            self.x += f * math.cos(thm) - l * math.sin(thm)
            self.y += f * math.sin(thm) + l * math.cos(thm)
            self.th = th_new
            self.hist.append((t, self.x, self.y, self.th))

    @property
    def pose(self):
        with self.lock:
            return (self.x, self.y, self.th or 0.0)

    def pose_at(self, t):
        with self.lock:
            h = list(self.hist)
        if not h:
            return (0.0, 0.0, 0.0)
        if t <= h[0][0]:
            return h[0][1:]
        if t >= h[-1][0]:
            return h[-1][1:]
        i = bisect.bisect_left([q[0] for q in h], t)
        a, b = h[i - 1], h[i]
        u = (t - a[0]) / max(1e-9, b[0] - a[0])
        return (a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2]), a[3] + u * wrap(b[3] - a[3]))

    def correct(self, dx, dy, dth=0.0):
        """Hieu chinh pose (tu line that): dich + xoay quanh vi tri xe hien tai. Ap ca vao lich su de noi suy nhat quan."""
        with self.lock:
            self.x += dx
            self.y += dy
            self.corr[0] += dx
            self.corr[1] += dy
            self.corr[2] += dth
            if self.th is not None:
                self.th += dth
            self.hist = deque(((t, x + dx, y + dy, th + dth) for t, x, y, th in self.hist), maxlen=self.hist.maxlen)


# ======================================================================================= LINE MEMORY
class LineMemory:
    """Polyline line trong ODOM, lay mau deu `res` mm. Moi nut: [x, y, conf, n_obs, t_first, t_last]."""

    def __init__(self, cfg=None):
        c = cfg or {}
        self.res = c.get('memory_resolution_m', 0.02) * 1000
        self.max_back = c.get('memory_max_length_m', 5.0) * 1000
        self.merge_r = c.get('memory_merge_mm', self.res * 0.75)
        self.gap_max = c.get('memory_gap_max_mm', 250.0)
        self.gain = c.get('drift_correction_gain', 0.3)
        self.corr_max = c.get('drift_correction_max_mm', 25.0)
        self.min_obs_corr = c.get('drift_min_obs', 2)
        self.min_age = c.get('drift_min_age_s', 0.4)
        self.bridge_max = c.get('memory_bridge_max_mm', 800.0)
        self.bridge_frames = c.get('memory_bridge_frames', 5)
        self._rej = 0                                         # so khung lien tiep line thay duoc KHONG noi duoc vao bo nho
        self.nodes = []
        self.drift_log = []                                   # (t, lech_ngang_mm, lech_goc_do) truoc khi hieu chinh

    # ---------- dung bo nho ----------
    def _nearest(self, X, Y, lo=0, hi=None):
        hi = len(self.nodes) if hi is None else hi
        best, bi = None, -1
        for i in range(max(0, lo), min(hi, len(self.nodes))):
            n = self.nodes[i]
            d = (n[0] - X) ** 2 + (n[1] - Y) ** 2
            if best is None or d < best:
                best, bi = d, i
        return bi, (math.sqrt(best) if best is not None else float('inf'))

    @staticmethod
    def _resample(pts, step):
        """Diem (sap xep gan -> xa) -> diem cach deu step mm tren duong gap khuc noi chung."""
        if len(pts) < 2:
            return list(pts)
        out = [pts[0]]
        acc = 0.0
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            while seg > 1e-9 and acc + seg >= step:
                u = (step - acc) / seg
                x0, y0 = x0 + u * (x1 - x0), y0 + u * (y1 - y0)
                out.append((x0, y0))
                seg = math.hypot(x1 - x0, y1 - y0)
                acc = 0.0
            acc += seg
        return out

    def observe(self, pts_car, pose, t, conf=1.0, odo=None):
        """pts_car: [(x toi, y trai)] mm khung XE (tu ground_calib) cua TAM line. Tra ve dict thong ke.
        odo != None -> (P4) so line thay duoc voi bo nho cu -> hieu chinh troi odometry truoc khi gop."""
        st = {'added': 0, 'merged': 0, 'corr_mm': 0.0}
        if len(pts_car) < 3:
            return st
        pts_car = sorted(pts_car, key=lambda p: p[0])
        if odo is not None and len(self.nodes) > 10:
            e, dth, m = self._drift(pts_car, pose, t)
            if m >= 5:
                self.drift_log.append((t, e, math.degrees(dth)))
                # dich xe theo phap tuyen duong (khong sua theo doc tuyen: camera khong do duoc)
                nx, ny = self._normal_at(pose)
                k = max(-self.corr_max, min(self.corr_max, -self.gain * e))
                odo.correct(k * nx, k * ny, 0.0)
                pose = (pose[0] + k * nx, pose[1] + k * ny, pose[2])
                st['corr_mm'] = k
        P = [to_odom(pose, x, y) for x, y in pts_car]
        P = self._resample(P, self.res)
        # khoang trong lon (sau khi ne / xoay o goc / doan line gay da di qua): camera thay line on dinh nhieu khung
        # ma khong noi duoc -> cho phep NOI CAU tu cuoi bo nho toi diem gan nhat cua khung nay
        bridge = False
        if self.nodes and P:
            last = self.nodes[-1]
            dmin = min(math.hypot(X - last[0], Y - last[1]) for X, Y in P)
            near_any = min(self._nearest(X, Y)[1] for X, Y in P[:3])
            if dmin > self.gap_max and near_any > self.merge_r:
                self._rej += 1
                ex, ey = to_car(pose, last[0], last[1])        # cuoi bo nho da o SAU / ngang xe (xe di qua roi) -> noi bat ke xa
                if self._rej >= self.bridge_frames and (dmin <= self.bridge_max or ex < 0):
                    bridge = True
                    P = sorted(P, key=lambda q: math.hypot(q[0] - last[0], q[1] - last[1]))
            else:
                self._rej = 0
        for X, Y in P:
            i, d = self._nearest(X, Y)
            if i >= 0 and d < self.merge_r:
                n = self.nodes[i]
                w = n[2] * min(n[3], 10)
                n[0] = (n[0] * w + X * conf) / (w + conf)
                n[1] = (n[1] * w + Y * conf) / (w + conf)
                n[2] = min(1.0, max(n[2], conf))
                n[3] += 1
                n[5] = t
                st['merged'] += 1
                continue
            if self.nodes:
                last = self.nodes[-1]
                dl = math.hypot(X - last[0], Y - last[1])
                if (dl > self.gap_max and not bridge) or dl < self.res * 0.5:
                    continue                                   # xa bat thuong (nhieu / line khac) hoac trung
                if bridge and dl > self.gap_max:
                    st['bridged'] = round(dl)
                    bridge = False                             # chi 1 cau / khung, cac diem sau noi binh thuong
                # chi noi them o DUOI (phia truoc): diem phai o phia truoc nut cuoi theo huong duong
                elif len(self.nodes) >= 2:
                    tx, ty = self.tangent(len(self.nodes) - 1)
                    if (X - last[0]) * tx + (Y - last[1]) * ty < -self.res * 0.5:
                        continue
            self.nodes.append([X, Y, conf, 1, t, t])
            st['added'] += 1
            self._rej = 0
        self._trim(pose)
        return st

    def _drift(self, pts_car, pose, t):
        """Lech ngang TB (mm, + = line that nam ben TRAI nut nho) giua diem thay duoc va nut da on dinh (cu, thay >= 2 lan)."""
        errs, angs = [], []
        for x, y in pts_car:
            X, Y = to_odom(pose, x, y)
            i, d = self._nearest(X, Y)
            if i < 0 or d > 80:
                continue
            n = self.nodes[i]
            if n[3] < self.min_obs_corr or t - n[4] < self.min_age:
                continue
            tx, ty = self.tangent(i)
            errs.append((X - n[0]) * -ty + (Y - n[1]) * tx)
        if not errs:
            return 0.0, 0.0, 0
        errs.sort()
        return errs[len(errs) // 2], 0.0, len(errs)

    def _normal_at(self, pose):
        i, _ = self._nearest(pose[0], pose[1])
        tx, ty = self.tangent(i) if i >= 0 else (math.cos(pose[2]), math.sin(pose[2]))
        return -ty, tx

    def _trim(self, pose):
        if len(self.nodes) < 3:
            return
        i, _ = self._nearest(pose[0], pose[1])
        s = 0.0
        for j in range(i, 0, -1):
            s += math.hypot(self.nodes[j][0] - self.nodes[j - 1][0], self.nodes[j][1] - self.nodes[j - 1][1])
            if s > self.max_back:
                del self.nodes[:j]
                return

    # ---------- truy van ----------
    def tangent(self, i, span=3):
        n = self.nodes
        a, b = n[max(0, i - span)], n[min(len(n) - 1, i + span)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        return (dx / L, dy / L) if L > 1e-6 else (1.0, 0.0)

    def project_many(self, XY):
        """Chieu NHIEU diem (numpy, nhanh tren Pi) -> list (s, c) giong project(); dau/cuoi ngoai suy thang."""
        import numpy as np
        n = self.nodes
        if len(n) < 2 or not len(XY):
            return []
        A = np.array([[q[0], q[1]] for q in n], float)
        d = A[1:] - A[:-1]
        L = np.hypot(d[:, 0], d[:, 1])
        L = np.where(L < 1e-9, 1e-9, L)
        cum = np.concatenate([[0.0], np.cumsum(L)])
        P = np.asarray(XY, float)
        rel = P[:, None, :] - A[None, :-1, :]                                   # (p, seg, 2)
        u = np.clip((rel * d[None]).sum(axis=2) / (L * L)[None], 0.0, 1.0)
        foot = A[None, :-1, :] + u[..., None] * d[None]
        dist = np.hypot(P[:, None, 0] - foot[..., 0], P[:, None, 1] - foot[..., 1])
        i = dist.argmin(axis=1)
        k = np.arange(len(P))
        s = cum[i] + u[k, i] * L[i]
        side = (d[i, 0] * rel[k, i, 1] - d[i, 1] * rel[k, i, 0]) / L[i]
        tot = cum[-1]
        out = []
        t0, te = self.tangent(0), self.tangent(len(n) - 1)
        for j in range(len(P)):
            sj, cj = float(s[j]), float(side[j])
            if sj >= tot - 1e-6:
                al = (P[j, 0] - A[-1, 0]) * te[0] + (P[j, 1] - A[-1, 1]) * te[1]
                if al > 0:
                    sj, cj = tot + al, (P[j, 0] - A[-1, 0]) * -te[1] + (P[j, 1] - A[-1, 1]) * te[0]
            elif sj <= 1e-6:
                al = (P[j, 0] - A[0, 0]) * t0[0] + (P[j, 1] - A[0, 1]) * t0[1]
                if al < 0:
                    sj, cj = al, (P[j, 0] - A[0, 0]) * -t0[1] + (P[j, 1] - A[0, 1]) * t0[0]
            out.append((sj, cj))
        return out

    def project(self, X, Y):
        """Chieu diem len polyline -> (chi so doan, s mm tu dau bo nho, lech ngang mm + = diem ben TRAI duong)."""
        n = self.nodes
        if len(n) < 2:
            return None
        best = None
        s_acc = 0.0
        for i in range(len(n) - 1):
            ax, ay, bx, by = n[i][0], n[i][1], n[i + 1][0], n[i + 1][1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            L = math.sqrt(L2) if L2 > 0 else 1e-9
            u = max(0.0, min(1.0, ((X - ax) * dx + (Y - ay) * dy) / L2)) if L2 > 0 else 0.0
            px, py = ax + u * dx, ay + u * dy
            d = math.hypot(X - px, Y - py)
            if best is None or d < best[0]:
                side = (dx * (Y - ay) - dy * (X - ax)) / L
                best = (d, i, s_acc + u * L, side)
            s_acc += L
        i, s, side = best[1], best[2], best[3]
        # qua 2 dau bo nho: tinh theo duong NGOAI SUY thang (de bam / ne qua doan chua biet)
        if s >= s_acc - 1e-6 or s <= 1e-6:
            Xe, Ye, tx, ty = self.point_at(s_acc if s > 1e-6 else 0.0)
            along = (X - Xe) * tx + (Y - Ye) * ty
            if (s > 1e-6 and along > 0) or (s <= 1e-6 and along < 0):
                s = (s_acc if s > 1e-6 else 0.0) + along
                side = (X - Xe) * -ty + (Y - Ye) * tx
        return i, s, side

    def length(self):
        n = self.nodes
        return sum(math.hypot(n[i + 1][0] - n[i][0], n[i + 1][1] - n[i][1]) for i in range(len(n) - 1))

    def point_at(self, s):
        """(X, Y, tx, ty) tai hoanh do cong s (mm); ngoai dau/cuoi -> ngoai suy thang theo tiep tuyen."""
        n = self.nodes
        if s <= 0:
            tx, ty = self.tangent(0)
            return n[0][0] + s * tx, n[0][1] + s * ty, tx, ty
        acc = 0.0
        for i in range(len(n) - 1):
            L = math.hypot(n[i + 1][0] - n[i][0], n[i + 1][1] - n[i][1])
            if acc + L >= s and L > 1e-9:
                u = (s - acc) / L
                tx, ty = (n[i + 1][0] - n[i][0]) / L, (n[i + 1][1] - n[i][1]) / L
                return n[i][0] + u * (n[i + 1][0] - n[i][0]), n[i][1] + u * (n[i + 1][1] - n[i][1]), tx, ty
            acc += L
        tx, ty = self.tangent(len(n) - 1)
        return n[-1][0] + (s - acc) * tx, n[-1][1] + (s - acc) * ty, tx, ty

    def cross_track(self, pose):
        p = self.project(pose[0], pose[1])
        return None if p is None else p[2]                    # + = XE lech trai so voi duong

    def known_until(self):
        return self.length()

    def straight_before(self, s_end, length, tol_mm):
        """Doan [s_end - length, s_end] co THANG khong (lech toi da khoi day cung < tol_mm)."""
        if s_end - length < 0:
            return False
        a = self.point_at(s_end - length)
        b = self.point_at(s_end)
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        if L < 0.8 * length:
            return False
        worst = 0.0
        for k in range(1, 10):
            p = self.point_at(s_end - length + length * k / 10.0)
            worst = max(worst, abs((p[0] - a[0]) * dy - (p[1] - a[1]) * dx) / L)
        return worst < tol_mm

    def snapshot(self):
        """Ban sao DONG BANG (de lam duong tham chieu khi ne / bam bo nho: bo nho song van cap nhat ma khong lam rung quy dao)."""
        c = LineMemory.__new__(LineMemory)
        c.__dict__.update(self.__dict__)
        c.nodes = [n[:] for n in self.nodes]
        c.drift_log = []
        return c

    def turn_between(self, s_a, s_b, step=40.0):
        """Goc doi huong lon nhat (do) cua duong trong [s_a, s_b] (chi phan DA BIET, khong tinh ngoai suy)."""
        s_b = min(s_b, self.length())
        if len(self.nodes) < 3 or s_b <= s_a:
            return 0.0
        h0, worst, s = None, 0.0, max(0.0, s_a)
        while s <= s_b:
            _, _, tx, ty = self.point_at(s)
            h = math.atan2(ty, tx)
            h0 = h if h0 is None else h0
            worst = max(worst, abs(wrap(h - h0)))
            s += step
        return math.degrees(worst)

    def sharpest_turn(self, s_a, s_b, window=250.0, step=25.0):
        """Goc doi huong LON NHAT (do) trong cua so `window` mm bat ky nam trong [s_a, s_b] (ke ca ngoai suy):
        GOC GAT (vuong / nhon) -> lon; duong CONG THOAI dai -> nho."""
        if len(self.nodes) < 3 or s_b <= s_a:
            return 0.0
        hs, s = [], s_a
        while s <= s_b:
            _, _, tx, ty = self.point_at(s)
            hs.append((s, math.atan2(ty, tx)))
            s += step
        worst = 0.0
        for i in range(len(hs)):
            for j in range(i + 1, len(hs)):
                if hs[j][0] - hs[i][0] > window:
                    break
                worst = max(worst, abs(wrap(hs[j][1] - hs[i][1])))
        return math.degrees(worst)

    # ---------- luu / nap tuyen (P5) ----------
    def save(self, path, meta=None):
        json.dump({'nodes': [[round(v, 1) for v in n[:2]] for n in self.nodes], 'meta': meta or {}}, open(path, 'w'))

    def load(self, path, t=0.0):
        d = json.load(open(path))
        self.nodes = [[x, y, 0.6, 3, t - 100.0, t - 100.0] for x, y in d['nodes']]
        return len(self.nodes)


# ======================================================================================= QUY DAO NE
class DetourPlan:
    """Duong tham chieu (bo nho / ngoai suy) + do lech ngang offset(s) dang smoothstep bac 5: ra -> song song -> ve."""

    def __init__(self, mem, s_a, L_out, s_b, L_back, D, s_end_known, c0=0.0):
        self.mem, self.s_a, self.L_out, self.s_b, self.L_back, self.D = mem, s_a, L_out, s_b, L_back, D
        self.s_end_known = s_end_known
        self.c0 = c0                                         # do lech LUC BAT DAU (xe co the da lech san, vd lap lai quy dao giua chung)

    def offset(self, s):
        if s < self.s_a:
            return self.c0
        if s < self.s_a + self.L_out:
            return self.c0 + (self.D - self.c0) * smoothstep5((s - self.s_a) / self.L_out)
        if s < self.s_b:
            return self.D
        return self.D * (1.0 - smoothstep5((s - self.s_b) / self.L_back))

    def end_s(self):
        return self.s_b + self.L_back

    def point(self, s):
        X, Y, tx, ty = self.mem.point_at(s)
        o = self.offset(s)
        return X - ty * o, Y + tx * o, tx, ty

    def poses(self, s0, s1, step=30.0):
        out, s = [], s0
        while s <= s1:
            X, Y, tx, ty = self.point(s)
            out.append((X, Y, math.atan2(ty, tx)))
            s += step
        return out


def footprint_hit(pose, pts_odom, front, rear, half_w, margin):
    """Co diem nao (ODOM) nam trong hinh chu nhat than xe tai pose (+ margin) khong."""
    for X, Y in pts_odom:
        x, y = to_car(pose, X, Y)
        if -rear - margin <= x <= front + margin and abs(y) <= half_w + margin:
            return True
    return False


def obstacle_cluster(mem, pts_odom, s_car, s_max, band, link_mm=80.0, max_depth=600.0, max_width=600.0):
    """Cum diem vat can NAM TREN duong tham chieu: hat giong = diem cach duong <= band va o phia truoc;
    mo rong theo diem lan can (<= link_mm) -> (s_min, s_max, c_min, c_max, so diem) hoac None. c + = ben trai."""
    proj = [(X, Y, s, c) for (X, Y), (s, c) in zip(pts_odom, mem.project_many(pts_odom))]
    seed = [q for q in proj if s_car < q[2] <= s_max and abs(q[3]) <= band]
    if not seed:
        return None
    # chi lay vat GAN NHAT phia truoc (khong noi sang tuong / do vat khac o xa hon)
    s_first = min(q[2] for q in seed)
    seed = [q for q in seed if q[2] <= s_first + max_depth]
    clus = list(seed)
    c_lo, c_hi = min(q[3] for q in seed), max(q[3] for q in seed)
    # mo rong ra 2 ben de do be rong vat, nhung GIOI HAN sau/rong: tuong chay doc ben canh khong bi tinh la vat
    rest = [q for q in proj if q not in seed and s_first - 50 <= q[2] <= s_first + max_depth
            and c_lo - max_width / 2 <= q[3] <= c_hi + max_width / 2]
    grown = True
    while grown and rest:
        grown = False
        keep = []
        for q in rest:
            if any((q[0] - c[0]) ** 2 + (q[1] - c[1]) ** 2 <= link_mm * link_mm for c in clus):
                clus.append(q)
                grown = True
            else:
                keep.append(q)
        rest = keep
    ss, cs = [q[2] for q in clus], [q[3] for q in clus]
    return min(ss), max(ss), min(cs), max(cs), len(clus)



def _try_plan(mem, pose, pts_odom, geo, cfg, s_car, s0, s1, D, L, lon, known):
    """Mot phuong an (do lech D, doan ra/ve dai L) + kiem tra vung than xe quet. Tra ve (plan, diem_cham | None)."""
    s_out_end = s0 - geo['front'] - lon                 # mui xe phai lech du D truoc khi toi mat truoc vat
    s_a = max(s_car, s_out_end - L)
    L_out = max(1.0, s_out_end - s_a)                    # vat gan: doan ra ngan -> doc hon (toi muc truot ngang tai cho)
    s_b = s1 + geo['rear'] + lon                         # duoi xe qua mat sau vat (+ le) moi ve
    c0 = mem.cross_track(pose) or 0.0
    c0 = c0 if c0 * D > 0 and abs(c0) <= abs(D) else 0.0     # chi giu do lech san neu cung phia va chua vuot qua D
    plan = DetourPlan(mem, s_a, L_out, s_b, L, D, known, c0)
    # kiem tra ca vung than xe quet qua (tu vi tri hien tai -> het quy dao) voi diem LiDAR
    clear = cfg.get('sweep_margin_mm', 40)
    hit = None
    # vung quet: tu VI TRI HIEN TAI (ke ca doan truot ngang khi doan ra ngan) -> het quy dao, noi suy moi 20 mm
    # khe ho `clear` chi ap cho doan xe LECH khoi line; doan nam tren line = nhu bam line thuong (margin 0)
    # khe ho `clear` chi bat buoc o doan XE NGANG VAT; doan ra / ve chi can khong cham (doan ve xe bam
    # line THAT camera thay + an toan luc chay; duong ngoai suy tu bo nho ngan co the lech vai do)
    s_near0, s_near1 = s0 - geo['front'] - lon, s1 + geo['rear'] + lon
    seq = [tuple(pose) + (s_car,)]
    # xe da NGANG vat ma con thieu do lech: TRUOT NGANG TAI CHO toi offset dau quy dao roi moi tien
    X0, Y0, tx0, ty0 = plan.point(s_car)
    if math.hypot(X0 - pose[0], Y0 - pose[1]) > 15:
        seq.append((X0, Y0, pose[2], s_car))
    s = s_car + 20
    while s <= plan.end_s():
        X, Y, tx, ty = plan.point(s)
        seq.append((X, Y, math.atan2(ty, tx), s))
        s += 25.0
    for a, b in zip(seq, seq[1:]):
        if a[3] > max(known, s_near1):
            break                                          # doan ve NGOAI SUY sau vat: luc chay xe bam line THAT (camera) + an toan luc chay
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) // 20))
        mg = clear if (b[3] >= s_near0 and a[3] <= s_near1) else 0.0
        for k in range(n):
            u = k / n
            q = (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), a[2] + u * wrap(b[2] - a[2]))
            if footprint_hit(q, pts_odom, geo['front'], geo['rear'], geo['half_w'], mg):
                hit = q
                break
        if hit:
            break
    return plan, hit


def plan_detour(mem, pose, pts_odom, geo, cfg, prefer=1):
    """Lap quy dao ne. Tra ve (plan, side, ly_do). plan None -> khong ne (dung cho).
    geo: front, rear, half_w (mm). cfg: tham so detour_* (mm)."""
    mem = mem.snapshot()                                  # duong tham chieu CO DINH cho ca lan ne
    p = mem.project(pose[0], pose[1])
    if p is None:
        return None, 0, 'chua co bo nho duong'
    s_car = p[1]
    band = geo['half_w'] + cfg.get('path_band_mm', 60)
    cl = obstacle_cluster(mem, pts_odom, s_car + geo['front'] - 50, s_car + cfg.get('detect_ahead_mm', 1500), band,
                          max_depth=cfg.get('object_depth_max_mm', 600), max_width=cfg.get('object_width_max_mm', 600))
    if cl is None:
        return None, 0, 'khong thay vat tren duong'
    s0, s1, cmin, cmax, n = cl
    depth_min = cfg.get('object_depth_min_mm', 150)        # LiDAR chi thay MAT TRUOC vat -> gia su vat sau toi thieu
    s1 = max(s1, s0 + depth_min)
    margin = cfg.get('detour_margin_mm', 100)
    L = cfg.get('detour_curve_length_mm', 500)
    L_min = cfg.get('detour_curve_min_mm', 150)
    lon = cfg.get('detour_long_margin_mm', 80)
    rejoin = cfg.get('rejoin_search_distance_mm', 400)
    # doan duong sau vat: bo nho da biet toi dau; khong du -> ngoai suy thang neu doan cuoi THANG
    need_until = s1 + geo['rear'] + lon + L + rejoin * 0.5
    known = mem.length()
    if known < need_until:
        if not mem.straight_before(known, cfg.get('extrapolate_min_straight_mm', 400), cfg.get('straight_tol_mm', 25)):
            return None, 0, 'doan line truoc vat KHONG thang (gan goc re) -> khong doan duong sau vat, dung cho'
    # duong tham chieu trong doan ne phai GAN THANG / cong nhe: quy dao lech song song khong di vong duoc goc vuong
    s_lo, s_hi = max(s_car + cfg.get('turn_check_skip_mm', 120), s0 - geo['front'] - lon - L), s1 + geo['rear'] + lon + L   # bo goc VUA xoay xong duoi xe
    # chi tu choi GOC GAT (doi huong nhieu trong doan ngan); duong CONG THOAI van ne theo duong cong duoc
    sharp = mem.sharpest_turn(s_lo, s_hi, cfg.get('sharp_window_mm', 250))
    if sharp > cfg.get('detour_max_turn_deg', 35):
        return None, 0, 'vat gan GOC RE GAT (doi huong %.0f do trong %d mm) -> khong ne, dung cho' % (sharp, cfg.get('sharp_window_mm', 250))
    # thu 2 ben
    cands = []
    hits = {}
    for side in ((prefer, -prefer)):
        D = (cmax + geo['half_w'] + margin) if side > 0 else -(-cmin + geo['half_w'] + margin)
        if abs(D) > cfg.get('detour_max_mm', 700):
            continue
        ok_plan = None
        for widen in range(cfg.get('detour_widen_tries', 3) + 1):
            for frac in cfg.get('detour_curve_fracs', [1.0, 0.75, 0.5, 0.3]):
                Lt = max(L_min, L * frac)
                plan, hit = _try_plan(mem, pose, pts_odom, geo, cfg, s_car, s0, s1, D, Lt, lon, known)
                if hit is None:
                    ok_plan = plan
                    break
            if ok_plan is not None or hit is None:
                break
            # quy dao cham vat (phan vat khong duoc gom vao cum, vd vat hinh L / 2 vat sat nhau): NOI RONG do lech
            # vua du vuot qua cac diem dang cham, roi thu lai (khong qua detour_max_mm)
            mg = cfg.get('sweep_margin_mm', 20)
            inside = [p for p in pts_odom if (lambda q: -geo['rear'] - mg <= q[0] <= geo['front'] + mg and abs(q[1]) <= geo['half_w'] + mg)(to_car(hit, *p))]
            cs = [c for _, c in mem.project_many(inside)]
            if not cs:
                break
            D_new = (max(cs) + geo['half_w'] + margin) if side > 0 else -(-min(cs) + geo['half_w'] + margin)
            if abs(D_new) <= abs(D) + 5 or abs(D_new) > cfg.get('detour_max_mm', 700):
                break
            D = D_new
        if ok_plan is not None:
            cands.append((abs(D), side, ok_plan))
        else:
            hits[side] = [round(v, 1) for v in hit]
    if not cands:
        plan_detour.last_hits = hits
        return None, 0, 'hai ben khong du cho (can lech <= %d mm, ca vung than xe quet phai trong)' % cfg.get('detour_max_mm', 700)
    cands.sort(key=lambda c: (round(c[0] / 30.0), c[1] != prefer))   # chenh < 3 cm: uu tien ben mac dinh
    _, side, plan = cands[0]
    plan.obstacle = (s0, s1, cmin, cmax, n)
    return plan, side, 'ok'


# ======================================================================================= PURE PURSUIT
def pursuit_cmd(pose, path_fn, s_car, lookahead, speed, k_head, lxly, mm_per_count, max_wz):
    """Bam duong cho xe MECANUM: van toc than (vx, vy) huong ve diem cach `lookahead` phia truoc tren duong,
    wz giu huong than theo tiep tuyen duong. path_fn(s) -> (X, Y, tx, ty). Tra ve (vx, vy, wz) xung/s."""
    X, Y, tx, ty = path_fn(s_car + lookahead)
    fx, fy = to_car(pose, X, Y)
    L = math.hypot(fx, fy)
    if L < 1e-6:
        return 0.0, 0.0, 0.0
    v = speed / mm_per_count                                # mm/s -> xung/s
    vx, vy = v * fx / L, v * fy / L
    _, _, ttx, tty = path_fn(s_car)
    err = wrap(math.atan2(tty, ttx) - pose[2])
    w = k_head * err                                         # rad/s
    wz = w * lxly / mm_per_count
    wz = max(-max_wz, min(max_wz, wz))
    return vx, vy, wz


# ======================================================================================= MAY TRANG THAI
# Bang chuyen trang thai TUONG MINH cho cac che do moi (che do cu cua line_follow giu nguyen).
TRANSITIONS = {
    ('FOLLOW', 'line_lost_known'): 'MEMORY',       # mat line tam thoi, bo nho con duong phia truoc
    ('MEMORY', 'line_back'): 'FOLLOW',
    ('MEMORY', 'memory_exhausted'): 'SEARCH',      # het duong da biet / qua memory_follow_max -> tim (cu)
    ('FOLLOW', 'obstacle_on_path'): 'BLOCKED',
    ('MEMORY', 'obstacle_on_path'): 'BLOCKED',
    ('BLOCKED', 'obstacle_gone'): 'FOLLOW',
    ('BLOCKED', 'wait_done'): 'DPLAN',
    ('DPLAN', 'plan_ok'): 'DEXEC',
    ('DPLAN', 'plan_fail'): 'BLOCKED',             # khong du thong tin duong sau vat / khong du cho -> cho tiep
    ('DEXEC', 'path_blocked'): 'BLOCKED',          # vat moi xuat hien tren quy dao -> dung
    ('DEXEC', 'detour_done'): 'REJOIN',
    ('REJOIN', 'line_confirmed'): 'FOLLOW',
    ('REJOIN', 'path_blocked'): 'BLOCKED',
    ('REJOIN', 'rejoin_failed'): 'ERROR',
    ('BLOCKED', 'timeout'): 'ERROR',
    ('DEXEC', 'timeout'): 'ERROR',
    ('MEMORY', 'timeout'): 'ERROR',
}


def next_state(state, event):
    """Chuyen trang thai theo bang; su kien khong hop le -> ValueError (khong de lot loi logic)."""
    try:
        return TRANSITIONS[(state, event)]
    except KeyError:
        raise ValueError('chuyen trang thai khong hop le: %s + %s' % (state, event))


# ======================================================================================= VE HINH (bao cao)
def draw_png(path, mem, traj=(), plans=(), obst=(), rejoins=(), title=''):
    """Ve bo nho duong (xanh), quy dao xe theo odometry (den), quy dao ne (cam), diem vat can (do) -> PNG."""
    import cv2
    import numpy as np
    traj = [(p[2], p[3]) for p in traj]                    # pose_log: (khung, t, x, y, th, mode)
    pts = [(n[0], n[1]) for n in mem.nodes] + list(traj) + [p for p in obst]
    for pl in plans:
        pts += [(q[0], q[1]) for q in pl.poses(pl.s_a, pl.end_s(), 50)]
    if not pts:
        return False
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    x0, x1, y0, y1 = min(xs) - 300, max(xs) + 300, min(ys) - 300, max(ys) + 300
    S = 1000.0
    sc = S / max(x1 - x0, y1 - y0)
    W, H = int((y1 - y0) * sc) + 40, int((x1 - x0) * sc) + 60
    img = np.full((H, W, 3), 255, np.uint8)

    def P(X, Y):                                           # x odom huong LEN, y odom sang TRAI
        return int(20 + (y1 - Y) * sc), int(40 + (x1 - X) * sc)
    g = 500
    for gx in range(int(x0 // g) * g, int(x1) + g, g):
        cv2.line(img, P(gx, y0), P(gx, y1), (235, 235, 235), 1)
    for gy in range(int(y0 // g) * g, int(y1) + g, g):
        cv2.line(img, P(x0, gy), P(x1, gy), (235, 235, 235), 1)
    for X, Y in obst:
        cv2.circle(img, P(X, Y), 2, (0, 0, 220), -1)
    for pl in plans:
        q = pl.poses(pl.s_a, pl.end_s(), 20)
        for a, b in zip(q, q[1:]):
            cv2.line(img, P(a[0], a[1]), P(b[0], b[1]), (0, 140, 255), 2)
    for a, b in zip(mem.nodes, mem.nodes[1:]):
        cv2.line(img, P(a[0], a[1]), P(b[0], b[1]), (200, 120, 0), 3)
    for a, b in zip(traj, traj[1:]):
        cv2.line(img, P(a[0], a[1]), P(b[0], b[1]), (0, 0, 0), 1)
    for X, Y, txt in rejoins:
        cv2.circle(img, P(X, Y), 6, (0, 160, 0), 2)
        cv2.putText(img, txt, P(X + 40, Y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 120, 0), 1)
    cv2.putText(img, title, (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    cv2.putText(img, 'xanh: bo nho line | den: xe (odom) | cam: quy dao ne | do: vat can | luoi 0.5 m', (10, H - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)
    cv2.imwrite(path, img)
    return True
