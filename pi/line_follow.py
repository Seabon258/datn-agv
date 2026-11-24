#!/usr/bin/env python3
"""Xe chay theo line (den/xanh) bang camera + PD, ghi chuyen chay / su co vao SQLite, cap nhat dashboard.

    python3 line_follow.py                 # chay that (xe + camera)
    python3 line_follow.py --no-robot      # chi thu camera + nhan dien, khong dieu khien xe
    python3 line_follow.py --item A01 --src "Tram 1" --dst "Kho A"
    python3 line_follow.py --cargo 2 --item A01          # van chuyen hang loai 2: PICKUP -> ZONE_B (bang station_map.yaml v2)
    python3 line_follow.py --dest ZONE_C --return-home    # -> PICKUP -> ZONE_C -> HOME

Dung: Ctrl+C, file STOP, nut DUNG KHAN tren dashboard, hoac mat line qua lau. Xe luon tu dung neu
chuong trinh chet (STM32 dung sau 0.3 s khong co lenh).
"""
import argparse
import json
import os
import signal
import time

import cv2
import yaml

from db import AgvDB, battery_percent
from line_vision import LineDetector
from markers import MarkerDetector
from route_map import RouteMap, RouteNavigator
from obstacle import ObstacleMonitor, clock_of
import memory_nav as MN
from cam_obstacle import CamObstacle

HERE = os.path.dirname(os.path.abspath(__file__))


def _sigterm(signum, frame):
    raise KeyboardInterrupt          # pkill / systemd stop -> chay khoi finally: dung xe + ghi ket thuc chuyen


def main():
    signal.signal(signal.SIGTERM, _sigterm)
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-robot', action='store_true')
    ap.add_argument('--item')
    ap.add_argument('--src', default='Tram xuat phat')
    ap.add_argument('--dst')
    ap.add_argument('--max-s', type=float, default=600)
    ap.add_argument('--calib', action='store_true', help='dat xe DUNG GIUA va THANG line, do do lech camera')
    ap.add_argument('--cargo', type=int, help='loai hang -> khu theo cargo_routing trong station_map.yaml')
    ap.add_argument('--dest', help='ten tram dich (vd ZONE_C, HOME)')
    ap.add_argument('--no-pickup', action='store_true', help='khong ghe PICKUP truoc')
    ap.add_argument('--return-home', action='store_true', help='xong thi ve HOME')
    ap.add_argument('--route', help='P5 Teach & Repeat: nap/luu tuyen maps/route_<ten>.json (biet truoc duong sau vat can)')
    ap.add_argument('--no-obstacle', action='store_true', help='TAT kiem tra vat can LiDAR (chi de thu, khong dung khi demo)')
    ap.add_argument('--no-lift', action='store_true',
                    help='KHONG chay stepper: toi tram chi dung station.sim_lift_s giay (test lo trinh khi chua chot lift)')
    args = ap.parse_args()

    os.chdir(HERE)
    cfg = yaml.safe_load(open('config.yaml'))
    fc, lc = cfg['follow'], cfg['line']
    db = AgvDB(cfg['database'])
    gcal0 = json.load(open('ground_calib.json')) if os.path.exists('ground_calib.json') else None
    det = LineDetector(lc, gcal0)

    cap = cv2.VideoCapture(cfg['camera']['index'], cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg['camera']['width'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg['camera']['height'])
    cap.set(cv2.CAP_PROP_FPS, cfg['camera']['fps'])
    if not cap.isOpened():
        db.event('error', 'khong mo duoc camera %s' % cfg['camera']['index'])
        raise SystemExit('Khong mo duoc camera')
    # camera vua mo: vai khung dau den/toi (dang tu chinh sang) -> bo di, cho anh sang on dinh
    t_cam, prev_m, stable = time.time(), None, 0
    while time.time() - t_cam < 4.0:                     # cho camera tu chinh sang xong: do sang on dinh 8 khung
        ok, f = cap.read()
        if not ok:
            continue
        m = float(f.mean())
        stable = stable + 1 if (m > 30 and prev_m is not None and abs(m - prev_m) < 2.0) else 0
        prev_m = m
        if stable >= 8:
            break

    if args.calib:
        lc['center_offset'] = 0.0
        det = LineDetector(lc)
        es, an = [], []
        for _ in range(60):
            ok, f = cap.read()
            if not ok:
                continue
            rr = det.process(f)
            if rr['found']:
                es.append(rr['error']); an.append(rr['angle'])
        cap.release()
        if not es:
            raise SystemExit('Khong thay line de hieu chinh')
        import statistics
        print('HIEU CHINH: do lech TB %+.3f (dat center_offset: %.3f) | goc TB %+.1f do | %d khung'
              % (statistics.mean(es), statistics.mean(es), statistics.mean(an), len(es)))
        return

    link = None
    if not args.no_robot:
        from stm32_link import Stm32Link
        link = Stm32Link(cfg['serial_port'])
        time.sleep(0.5)
        if not link.telem:
            db.event('link_lost', 'khong nhan duoc telemetry STM32 luc khoi dong')
            raise SystemExit('Khong nhan duoc telemetry tu STM32')

    # ----- BO NHO DUONG DI (docs/feature_line_memory.md): odometry Pi + polyline line tren san -----
    mcfg = cfg.get('memory', {})
    odo = mem = None
    geo_m = cfg.get('geometry', {})
    GEOM = {'front': float(cfg.get('obstacle', {}).get('front_extent_mm', geo_m.get('car_length_mm', 450) / 2)),
            'rear': float(geo_m.get('car_rear_mm') or geo_m.get('car_length_mm', 450) / 2),
            'half_w': geo_m.get('car_width_mm', 340) / 2.0}
    if link and mcfg.get('enabled', False) and gcal0 and gcal0.get('scale_coef'):
        odo = MN.Odometry(3.14159 * geo_m.get('wheel_diam_mm', 80) / geo_m.get('enc_cpr', 3960))
        link.on_telem = lambda t: odo.update(t, time.time())
        mem = MN.LineMemory(mcfg)
        if args.route and os.path.exists(os.path.join('maps', 'route_%s.json' % args.route)):
            print('nap tuyen %s: %d diem' % (args.route, mem.load(os.path.join('maps', 'route_%s.json' % args.route), time.time())), flush=True)
    elif mcfg.get('enabled', False):
        print('bo nho duong: TAT (can ground_calib.json co scale_coef)', flush=True)

    # ----- LiDAR vat can (kit Module 5): bat -> phai co du lieu + da cau hinh huong lap, khong thi khong chay -----
    oc = cfg.get('obstacle', {})
    lid = om = None
    if link and oc.get('enabled') and not args.no_obstacle:
        from lidar import Lidar
        geo_ = cfg.get('geometry', {})
        om = ObstacleMonitor(oc, geo_.get('car_length_mm', 450), geo_.get('car_width_mm', 340), geo_.get('car_rear_mm'))
        if not om.ready:
            raise SystemExit('obstacle: chua do thong so lap LiDAR: %s (config.yaml)' % ', '.join(om.missing))
        lid = Lidar(cfg['lidar']['port'], cfg['lidar']['baud']).start()
        if not lid.wait_ready(oc.get('lidar_wait_s', 3.0), oc.get('lidar_retries', 3), log=lambda m: print(m, flush=True)):
            lid.close()
            db.event('error', 'LiDAR khong co du lieu luc khoi dong (da thu mo lai %d lan) -> khong chay' % oc.get('lidar_retries', 3))
            raise SystemExit('LiDAR khong co vong quet - kiem tra day / LiDAR co quay khong')

    # ----- camera phat hien vat can (bu cho LiDAR: vat den / bong / thap). Can hieu chinh mat san -----
    coc = cfg.get('cam_obstacle', {})
    co = None
    if coc.get('enabled') and not args.no_obstacle and gcal0:
        geo_ = cfg.get('geometry', {})
        co = CamObstacle(coc, gcal0, geo_.get('car_width_mm', 340), geo_.get('car_length_mm', 450))
        co.front = max(co.front, oc.get('front_extent_mm', co.front))   # khoang cach vat can tinh tu vo LiDAR (nho ra truoc)

    # ----- nhiem vu van chuyen (TRANSPORT) -----
    try:
        rm = RouteMap('station_map.yaml')                  # tu kiem tra loi bang ma, sai -> khong chay
    except ValueError as e:
        raise SystemExit(str(e))
    sc, lift_cfg = cfg.get('station', {}), cfg.get('lift', {})
    geo = cfg.get('geometry', {})
    cps_per_m_s = geo.get('enc_cpr', 3960) / (3.14159 * geo.get('wheel_diam_mm', 80) / 1000.0)
    plan = []
    if args.cargo is not None or args.dest:
        try:
            plan = rm.plan(cargo_type=args.cargo, dest=args.dest, pickup=not args.no_pickup, return_home=args.return_home)
        except ValueError as e:
            raise SystemExit(str(e))
    nav = RouteNavigator(rm, plan, lambda v: v * cps_per_m_s) if plan else None
    stations = rm.station_by_id
    steps = [(int(rm.stations[n]['id']), rm.stations[n]['action']) for n in plan]
    if any(a in ('LIFT_UP', 'LIFT_DOWN') for _, a in steps) and not (lift_cfg.get('up_mm') or lift_cfg.get('up_s')) and not args.no_lift:
        raise SystemExit('Nhiem vu co nang/ha nhung lift.up_mm trong config.yaml CHUA DAT (null). '
                         'Do hanh trinh cang nang can thiet (mm) roi dien vao - khong doan.')
    mk = MarkerDetector(cfg['aruco'])

    dst = args.dst
    if args.item and not dst:
        it = db.item(args.item)
        dst = it['warehouse'] if it else None
    if steps and not dst:
        dst = stations[steps[-1][0]]['name']
    if steps and args.src == 'Tram xuat phat':
        args.src = stations[steps[0][0]]['name']
    trip = db.start_trip(args.item, args.src, dst)
    if steps:
        db.event('info', 'nhiem vu%s: ' % ((' hang loai %d' % args.cargo) if args.cargo is not None else '') +
                 ' -> '.join('%s(%d) %s' % (stations[i]['name'], i, a) for i, a in steps), trip)
    if os.path.exists('STOP'):
        os.remove('STOP')

    # ----- tu ghi video moi chuyen -----
    vw, vraw, vc = None, None, cfg.get('video', {})
    if vc.get('record', True):
        os.makedirs(vc.get('dir', 'videos'), exist_ok=True)
        old = sorted(f for f in os.listdir(vc.get('dir', 'videos')) if f.endswith('.avi'))
        for f in old[:max(0, len(old) - vc.get('keep', 20) + 1)]:
            os.remove(os.path.join(vc.get('dir', 'videos'), f))
        vpath = os.path.join(vc.get('dir', 'videos'), 'trip_%04d.avi' % trip)
        vw = cv2.VideoWriter(vpath, cv2.VideoWriter_fourcc(*'MJPG'), 30,
                             (cfg['camera']['width'], cfg['camera']['height']))
        os.makedirs(os.path.join(vc.get('dir', 'videos'), 'raw'), exist_ok=True)
        vraw = cv2.VideoWriter(os.path.join(vc.get('dir', 'videos'), 'raw', 'trip_%04d_raw.avi' % trip), cv2.VideoWriter_fourcc(*'MJPG'), 30,
                               (cfg['camera']['width'], cfg['camera']['height'])) if vc.get('record_raw', True) else None
        print('ghi video: %s' % vpath, flush=True)

    state, result = 'FOLLOW_LINE', 'ok'
    last_err, last_t, lost_since = 0.0, time.time(), None
    d_f, trend, turn_dir, confirm = 0.0, 0.0, 1, 0
    last_dbg = None
    mode, t_mode = 'START', time.time()            # START | FOLLOW | APPROACH | TURN | SEARCH
    corner_gone, corner_max_y, h_start, h_prev, turned = 0, 0.0, None, None, 0.0
    enc_c0 = enc_gone = None
    # bang "hang anh -> khoang cach that toi tam xe" (ground_calib.py). Co -> dung nga re dung tam theo do dac
    gcal = json.load(open('ground_calib.json')) if os.path.exists('ground_calib.json') else None
    target_cnt = None                              # encoder TB (xung) tai do tam xe nam tren nga re

    def enc_mean():
        return sum(link.telem['enc']) / 4.0

    def gated_target(old, y):
        """Cap nhat diem dung theo lan thay moi CHI KHI gan uoc luong cu (< corner_gate_mm). Mot line ngang
        nhay ra xa hon han (vd canh doi dien cua duong hinh chu nhat) la line KHAC -> bo qua."""
        new = corner_target(y)
        if old is None or abs(new - old) * mm_per_count <= fc.get('corner_gate_mm', 120):
            return new, True
        return old, False

    def corner_target(y):
        """Diem dich (xung encoder) = vi tri hien tai + khoang cach tu tam xe toi nga re do bang camera."""
        from line_vision import ground_dist_mm
        d = float(ground_dist_mm(gcal, min(y, 1.0)))
        return enc_mean() + max(0.0, d) / mm_per_count
    i_err = 0.0
    votes = 0.0                                    # bau chon huong re qua nhieu khung (+ trai / - phai)
    step_i, loaded, st_id, st_max_y, st_gone, st_cmd_t, st_busy_seen = 0, False, None, 0.0, 0, None, False
    speed_cap = nav.speed_cps if nav else None    # gioi han toc do tu ma ArUco (FAST/NORMAL/SLOW)
    turn_hint, hint_until_m = 0, 0.0              # ma re / nga re bao truoc huong (+1 trai, -1 phai, 2 quay dau)
    straight_until_m = -1.0                       # nga re "di thang": bo qua nga cua trong quang nay
    wait_until = 0.0
    ignored_far = False
    start_ok, s_start = 0, 0.0
    rot_blocked, rot_hold, t_rot = False, False, 0.0                     # xuat phat: so khung lien tiep thay line; quang da di luc bat dau bam
    ob = {'zone': 'CLEAR', 'front_mm': None, 'left_free': False, 'right_free': False}
    ob_scan_t, ob_prev_mode, ob_side, ob_lat0, ob_pass0, ob_clear_n, ob_stale_logged = 0.0, 'FOLLOW', 0, 0.0, None, 0, False
    ob_need = oc.get('detour_mm', 350)
    cst = {'zone': 'CLEAR', 'front_mm': None}
    ob_by_cam = False                              # vat chi CAMERA thay (LiDAR khong thay) -> vuot theo quang an toan
    free_n = {1: 0, -1: 0}                         # so vong quet LIEN TIEP ben trai(+1)/phai(-1) trong (loc nhay)
    ob_last_scan_free = 0.0
    blocked_n = 0                                  # dang ne: so khung lien tiep ben ne bi chan
    ob_noroom_logged = False
    ob_base_lat = None                             # vi tri ngang (mm) cua tuyen luc gap vat; None = dang tren tuyen
    ob_gone_at = None                              # quang (mm) luc vat khuat khoi tam LiDAR khi dang vuot
    ob_theta = 90.0                                # goc ne: 45 = cheo 10 gio 30, 90 = ngang 9 gio
    # ---- bo nho duong ----
    pose_log, frame_i = [], 0                      # (khung, t, x, y, th, mode) -> csv + hinh bao cao
    dplan, dplans, plan_try_t, plan_why = None, [], 0.0, ''
    mem_s0, rj_s0, ob_path_t, ob_path_n, force_block, ob_on_path = 0.0, 0.0, 0.0, 0, False, False
    rejoin_marks, drift_seen = [], []
    PURSUIT = dict(lookahead=mcfg.get('pursuit_lookahead_mm', 150), k_head=mcfg.get('heading_k', 1.5),
                   lxly=cfg.get('geometry', {}).get('lxly_mm', 274))

    def pursue(path_fn, s_c, speed_mm_s):
        return MN.pursuit_cmd(odo.pose, path_fn, s_c, PURSUIT['lookahead'], speed_mm_s, PURSUIT['k_head'],
                              PURSUIT['lxly'], mm_per_count, fc['max_wz'])

    def mem_s(ref=None):
        ref = ref if ref is not None else mem
        pr = ref.project(*odo.pose[:2]) if (ref is not None and len(ref.nodes) >= 2) else None
        return None if pr is None else pr[1]
    mem_ref = None                                 # ban dong bang cua bo nho khi bam bo nho (MEMORY)
    end_resume = 0

    def path_blocked(vx_, vy_, wz_, extra_pts=()):
        """An toan khi ne/nhap lai: du doan vi tri than xe sau exec_predict_s; co diem LiDAR (hien tai + vat da ghi) -> chan."""
        import math
        if om is None:
            return False
        if lid.age() > oc.get('scan_timeout_s', 0.5):
            return True
        pc = odo.pose
        T = mcfg.get('exec_predict_s', 0.4)
        k = mm_per_count * T
        c, s_ = math.cos(pc[2]), math.sin(pc[2])
        pred = (pc[0] + (vx_ * c - vy_ * s_) * k, pc[1] + (vx_ * s_ + vy_ * c) * k, pc[2] + wz_ * mm_per_count / PURSUIT['lxly'] * T)
        ps = odo.pose_at(ob_scan_t)
        pts = [MN.to_odom(ps, x, y) for x, y in ob.get('points', []) if abs(x) < 1500 and abs(y) < 1200] + list(extra_pts)
        mg = mcfg.get('exec_margin_mm', 25)
        return MN.footprint_hit(pred, pts, GEOM['front'], GEOM['rear'], GEOM['half_w'], mg)

    def clock_txt(side, theta):
        h = (12 - theta / 30.0) if side > 0 else (theta / 30.0)
        hh, half = int(h + 1e-6) % 12 or 12, (h % 1) >= 0.25
        return '%d gio%s' % (hh, ' 30' if half else '')
    # LiDAR (mui xe) mat dau vat khi vat con ngang ~ x = gone_x_mm (vung mu sau lung LiDAR) -> con chay them toi khi qua duoi xe
    tail_clear_mm = cfg.get('geometry', {}).get('car_rear_mm', 225) + oc.get('gone_x_mm', 150) + oc.get('tail_margin_mm', 100)
    cv = {}                                        # ne CONG: s0, D (mm, + trai), S1 (ra), S3 (ve), pha

    def diag_theta(front_mm, need):
        """Goc ne (do, tinh tu huong 12 gio): 45 = 10 gio 30 (cheo), 90 = 9 gio (ngang). Cang nhieu cho phia truoc -> cang cheo."""
        import math
        room = (front_mm if front_mm is not None else 0.0) - oc.get('diag_front_margin_mm', 80)
        if room <= 0:
            return 90.0
        return max(oc.get('diag_min_deg', 45), min(90.0, math.degrees(math.atan2(need, room))))

    def pick_side():
        """Chon ben ne (trai/phai) du cho + quang lech can. Tra ve (side, need) hoac (0, None)."""
        best_sd, best = 0, None
        for sd in ((1, -1) if oc.get('prefer_side', 'left') == 'left' else (-1, 1)):
            need = detour_need(sd)
            if need > oc.get('detour_max_mm', 700) or not om:
                continue
            free = om._side_free(ob.get('points', []), 'left' if sd > 0 else 'right', need + oc.get('side_margin_mm', 100))
            if free and free_n[sd] >= oc.get('free_scans', 12) and (best is None or need < best):
                best_sd, best = sd, need
        return best_sd, best

    def detour_need(side):
        """Vat chua biet kich thuoc: do mep vat (LiDAR) phia se ne -> quang truot can = mep vat + nua than xe + le."""
        half_w = cfg.get('geometry', {}).get('car_width_mm', 340) / 2.0
        front = cfg.get('geometry', {}).get('car_length_mm', 450) / 2.0
        ys = [y for x, y in ob.get('points', []) if front < x < front + oc.get('object_depth_mm', 700) and abs(y) < oc.get('detour_max_mm', 700) + half_w]
        if cst.get('zone') != 'CLEAR' and cst.get('left_mm') is not None:    # be rong vat do bang camera
            ys += [cst['left_mm'], cst['right_mm']]
        if not ys:
            return oc.get('detour_mm', 350)
        edge = max(ys) if side > 0 else -min(ys)           # vat lan sang phia ne bao xa (mm tu truc xe)
        return max(oc.get('detour_mm', 350), edge + half_w + oc.get('detour_margin_mm', 80))
    MOVING = ('FOLLOW', 'APPROACH', 'CREEP', 'END', 'STATION', 'ST_CREEP', 'ALIGN', 'PASS', 'REACQUIRE', 'MEMORY')

    def lateral_mm():
        """Quang truot ngang (mm, + = trai) tu encoder mecanum: vy = (-fl + rl - rr + fr)/4; enc = [M1 rr, M2 rl, M3 fr, M4 fl]."""
        e = link.telem['enc']
        return (-e[3] + e[1] - e[0] + e[2]) / 4.0 * mm_per_count

    def fwd_mm():
        return sum(link.telem['enc']) / 4.0 * mm_per_count
    uturn = False                                 # dang quay dau
    corner_seen_t, corner_seen_dir = -9.0, 0      # lan cuoi thay nga re (luoi an toan: mat line ngay sau do = re, khong phai cuoi duong) (ma 32): goc xoay toi thieu/toi da khac nga re
    e_f = None

    v_cmd, w_cmd, rem_mm = 0.0, 0.0, None         # lenh sau khi qua bo tang/giam toc

    def ramp(cur, target, up, down):
        """Gioi han toc do thay doi: tang toi da up*dt, giam toi da down*dt."""
        if abs(target) >= abs(cur) and (target >= 0) == (cur >= 0):
            step = up * dt
        else:
            step = down * dt
        return cur + max(-step, min(step, target - cur))

    def brake_mm(v):
        """Quang truot khi dung tu toc do v (xung/s): firmware ham theo gia toc wheel_accel."""
        return v * v / (2.0 * fc.get('wheel_accel', 2500)) * mm_per_count
    end_seen_t = 0.0                               # lan cuoi thay dau mut line trong anh (cuoi duong)
    mm_per_count = 3.14159 * cfg.get('geometry', {}).get('wheel_diam_mm', 80) / cfg.get('geometry', {}).get('enc_cpr', 3960)
    t0 = time.time()
    enc0 = link.telem['enc'] if link else None
    last_bat, frames, fps = 0.0, 0, 0.0
    fps_t = time.time()
    try:
        while True:
            # ----- lenh dung -----
            cmds = db.pop_commands()
            for cm in cmds:                                  # nguoi van hanh chon ben ne theo huong dong ho (xe VAN tu kiem tra an toan)
                if cm in ('ne_9', 'ne_3'):
                    oc['prefer_side'] = 'left' if cm == 'ne_9' else 'right'
                    oc['prefer_only'] = True
                    db.event('obstacle', 'nguoi van hanh chon ne ve %s' % ('9 gio (TRAI)' if cm == 'ne_9' else '3 gio (PHAI)'), trip)
            if 'estop' in cmds or os.path.exists('STOP'):
                state, result = 'ESTOP', 'aborted'
                db.event('estop', 'dung khan tu dashboard / file STOP', trip)
                break
            if time.time() - t0 > args.max_s:
                state, result = 'TIMEOUT', 'aborted'
                db.event('info', 'het thoi gian toi da %.0f s' % args.max_s, trip)
                break

            ok, frame = cap.read()
            if not ok:
                state, result = 'CAMERA_ERROR', 'failed'
                db.event('error', 'mat hinh camera', trip)
                break
            t_frame = time.time() - mcfg.get('cam_latency_s', 0.08)
            if vraw is not None:
                vraw.write(frame)                           # anh goc, chua ve gi (de thu lai thuat toan)
            r = det.process(frame)
            frame_i += 1
            if odo is not None:
                pose_f = odo.pose_at(t_frame)
                pose_log.append((frame_i, round(t_frame - t0, 3), round(pose_f[0], 1), round(pose_f[1], 1), round(pose_f[2], 4), mode))
                if r['found'] and r.get('g_pts') and mode not in ('TURN', 'SEARCH', 'START', 'BLOCKED', 'DEXEC', 'MEMORY'):
                    # khong ghi khi dung canh vat / dang ne (line bi vat che, nhin xien) - chi ghi khi dang tren line
                    use_corr = mcfg.get('drift_correction', True) and mode in ('FOLLOW', 'MEMORY', 'REJOIN') and not r['corner']
                    mst = mem.observe(r['g_pts'], pose_f, t_frame, conf=1.0, odo=odo if use_corr else None)
                    if mst['corr_mm']:
                        drift_seen.append(mst['corr_mm'])
            confirmed = mk.update(frame)
            mk.draw(r['debug'])
            if r['found']:
                # loc do lech (giam giat do vien line nhay tung khung)
                e_f = r['error'] if e_f is None else e_f + fc.get('err_lpf', 0.5) * (r['error'] - e_f)
                r['error'] = e_f
            else:
                e_f = None
            now = time.time()
            dt = max(1e-3, now - last_t)
            last_t = now

            vx = wz = vy = 0
            force_block = False
            if om or co:
                stale = False
                if om:
                    if lid.scan_t != ob_scan_t:
                        ob_scan_t = lid.scan_t
                        ob = om.update(lid.scan, now)
                    stale = lid.age() > oc.get('scan_timeout_s', 0.5)
                    if stale and not ob_stale_logged:
                        db.event('error', 'LiDAR mat du lieu %.1f s -> coi nhu co vat can, xe dung' % lid.age(), trip)
                    ob_stale_logged = stale
                lid_zone = ob['zone']
                if om and ob_scan_t != ob_last_scan_free:
                    ob_last_scan_free = ob_scan_t
                    for sd in (1, -1):
                        wneed = detour_need(sd) + oc.get('side_margin_mm', 100)
                        ok_sd = om._side_free(ob.get('points', []), 'left' if sd > 0 else 'right', wneed)
                        free_n[sd] = free_n[sd] + 1 if ok_sd else 0
                if co:
                    cst = co.update(frame, det.mask(frame), mk.last_corners)
                    # gop: canh bao NANG hon thang; khoang cach lay gan hon
                    order = ('CLEAR', 'SLOW', 'STOP')
                    if order.index(cst['zone']) > order.index(ob['zone']):
                        ob = dict(ob, zone=cst['zone'])
                    if cst['front_mm'] is not None and (ob['front_mm'] is None or cst['front_mm'] < ob['front_mm']):
                        ob = dict(ob, front_mm=cst['front_mm'])
                near_corner = r['corner'] or now - corner_seen_t < fc.get('corner_memory_s', 1.5) + 0.5
                if not near_corner and mem is not None and len(mem.nodes) > 5 and mode in ('FOLLOW', 'MEMORY'):
                    s_here = mem_s()
                    if s_here is not None and mem.turn_between(s_here, s_here + GEOM['front'] + mcfg.get('detour_trigger_mm', 700)) \
                            > mcfg.get('detour_max_turn_deg', 35):
                        near_corner = True                    # bo nho biet line RE phia truoc: logic goc re xu ly, khong "ne" do vat sau goc
                if near_corner and mode in ('FOLLOW', 'MEMORY'):
                    ob_path_n = 0                                 # sap re: duong khong di thang -> bo phat hien xa (vung dung 30 cm + kiem tra xoay van chay)
                elif mem is not None and om and mode in ('FOLLOW', 'MEMORY', 'BLOCKED') and ob_scan_t != ob_path_t and len(mem.nodes) > 5:
                    ob_path_t = ob_scan_t
                    pose_s = odo.pose_at(ob_scan_t)
                    trig = mcfg.get('detour_trigger_mm', 700)
                    near_pts = [MN.to_odom(pose_s, x, y) for x, y in ob.get('points', [])
                                if 0 < x < GEOM['front'] + trig + 300 and abs(y) < 900]
                    pr = mem.project(pose_s[0], pose_s[1])
                    s_lim = (pr[1] + GEOM['front'] + trig) if pr else 0.0
                    if pr and now - end_seen_t < 1.0:
                        s_lim = min(s_lim, mem.length() + 100)   # thay CUOI line / NGA RE: khong ngoai suy thang xuyen qua (tuong sau goc)
                    cl = MN.obstacle_cluster(mem, near_pts, pr[1], s_lim,
                                             GEOM['half_w'] + mcfg.get('path_band_mm', 60)) if (pr and near_pts) else None
                    ob_path_n = ob_path_n + 1 if cl else 0
                    ob_on_path = bool(cl)
                if mem is not None and mode in ('FOLLOW', 'MEMORY') and ob_path_n >= oc.get('persist_scans', 2):
                    force_block = True
                    ob_path_n = 0
                if (mode == 'CREEP' and now - t_mode > 0.2 and ob['zone'] == 'STOP' and not stale and rem_mm is not None
                        and rem_mm <= fc.get('corner_stop_tol_mm', 120)):
                    # gan toi tam goc (con <= 12 cm) ma phia TRUOC co tuong/do vat: KHONG can di thang nua -> xoay luon
                    # (xoay co kiem tra cho trong rieng: ban kinh than xe quanh tam)
                    db.event('info', 'toi gan tam goc (con %.0f mm), phia truoc co vat cach %.0f mm -> xoay %s luon' % (
                        rem_mm, ob['front_mm'] or -1, 'TRAI' if (votes or turn_dir) > 0 else 'PHAI'), trip)
                    mode, t_mode, confirm = 'TURN', now, 0
                    if votes:
                        turn_dir = 1 if votes > 0 else -1
                    h_start = h_prev = None
                    turned = 0.0
                elif (ob['zone'] == 'STOP' or stale or force_block) and mode in MOVING and mode not in ('PASS',):
                    if ob_base_lat is None and link and mem is None:
                        ob_base_lat = lateral_mm()            # vi tri ngang cua TUYEN: moi cach thoat deu phai truot ve day
                    ob_by_cam = lid_zone != 'STOP' and not stale and not force_block
                    ob_prev_mode = mode if mode not in ('REACQUIRE',) else 'FOLLOW'
                    mode, t_mode, ob_clear_n, ob_noroom_logged = 'BLOCKED', now, 0, False
                    hw = cfg.get('geometry', {}).get('car_width_mm', 340) / 2 + oc.get('corridor_margin_mm', 50)
                    pts_f = [(x, y) for x, y in ob.get('points', []) if x > 0 and abs(y) <= hw]   # vat TRONG duong di
                    nx, ny = min(pts_f, key=lambda q: q[0] ** 2 + q[1] ** 2) if pts_f else (1.0, 0.0)
                    where = ('o %s, ' % clock_of(nx, ny)) if (pts_f and not ob_by_cam) else ('o 12 gio, ' if ob_by_cam else '')
                    db.event('obstacle', 'VAT CAN %scach mui xe %s mm (%s) -> dung cho' % (
                        where, '%.0f' % ob['front_mm'] if ob['front_mm'] is not None else '?',
                        'camera thay, LiDAR khong' if ob_by_cam else 'LiDAR'), trip)
            if (om and link and mem is None and mode == 'FOLLOW' and ob['zone'] == 'SLOW' and ob['front_mm'] is not None
                    and ob['front_mm'] >= oc.get('curve_trigger_mm', 350) and (oc.get('curve_enabled', True) or oc.get('diag_enabled', True))
                    and oc.get('detour_enabled', True) and not loaded and (nav.detour_allowed if nav else True)):
                sd, need = pick_side()
                if sd and oc.get('diag_enabled', True):
                    ob_base_lat = lateral_mm()
                    ob_theta = diag_theta(ob['front_mm'], need)
                    mode, t_mode, ob_side, ob_need, ob_lat0, blocked_n = 'DETOUR', now, sd, need, lateral_mm(), 0
                    db.event('obstacle', 'VAT CAN o 12 gio cach %.0f mm -> NE CHEO ve %s (lech %.0f mm, goc %.0f do)' % (
                        ob['front_mm'], clock_txt(sd, ob_theta), need, ob_theta), trip)
                elif sd:
                    s_out = max(oc.get('curve_out_min_mm', 250), min(oc.get('curve_out_max_mm', 600),
                                ob['front_mm'] - oc.get('curve_out_margin_mm', 100)))
                    ob_base_lat = lateral_mm()
                    cv = dict(s0=fwd_mm(), D=sd * need, S1=s_out, phase='OUT', s_hold=None)
                    mode, t_mode, ob_side, ob_need = 'CURVE', now, sd, need
                    db.event('obstacle', 'vat can o xa %.0f mm -> NE CONG ve %s (%s), lech %.0f mm trong %.0f mm' % (
                        ob['front_mm'], '10 gio 30' if sd > 0 else '1 gio 30', 'TRAI' if sd > 0 else 'PHAI', need, s_out), trip)
            if mode == 'START':
                # xuat phat: dung yen cho thay line (khong xoay tim) toi da start_wait_s
                start_ok = start_ok + 1 if r['found'] else 0
                if start_ok >= fc.get('start_confirm', 8):
                    mode, last_err = 'FOLLOW', r['error']
                    s_start = _distance_m(link.telem['enc'], enc0) if link else 0.0
                elif now - t_mode > fc.get('start_wait_s', 3.0):
                    result, state = 'failed', 'NO_LINE_AT_START'
                    db.event('line_lost', 'luc xuat phat khong thay line duoi camera, xe khong chay', trip)
                    break
            # ===== may trang thai: FOLLOW -> (thay nga cua) APPROACH -> TURN -> FOLLOW ; mat line -> SEARCH =====
            trav_m = _distance_m(link.telem['enc'], enc0) if link else 0.0
            directives = [nav.on_marker(m['id']) for m in confirmed] if nav else \
                [{'event': 'thay ma %d (khong co nhiem vu)' % m['id']} for m in confirmed]
            srcs = list(confirmed)
            if nav:
                d_exp = nav.on_distance(trav_m)
                if d_exp:
                    directives.append(d_exp)
                    srcs.append(None)
            for m_d, dv in zip(srcs, directives):
                if dv.get('event'):
                    db.event('marker', ('ID %d: ' % m_d['id'] if m_d else '') + dv['event'], trip)
                if 'speed_cps' in dv:
                    speed_cap = dv['speed_cps']
                if dv.get('turn_hint'):
                    turn_hint, hint_until_m = dv['turn_hint'], trav_m + sc.get('hint_valid_m', 1.0)
                if dv.get('straight_through'):
                    straight_until_m = trav_m + sc.get('straight_through_m', 0.8)
                if dv.get('wait_s') and mode in ('FOLLOW', 'STATION'):
                    mode, wait_until = 'WAIT', now + dv['wait_s']
                if dv.get('end_of_line'):
                    state, result = ('ARRIVED', 'ok') if not dv.get('error') else ('END_NOT_TARGET', 'failed')
                    mode = 'STOP_NOW'
                if dv.get('stop_at_station') is not None and mode in ('FOLLOW', 'STATION', 'WAIT'):
                    sid = dv['stop_at_station']
                    if m_d is not None and m_d['id'] == sid:          # thay ma tram: toi gan roi dung theo marker
                        mode, st_id, st_max_y, st_gone, t_mode = 'STATION', sid, m_d['cy'], 0, now
                    else:                                              # het quang bao truoc: dung ngay tai day
                        mode, st_id, t_mode, st_cmd_t, st_busy_seen = 'ST_ACTION', sid, now, None, False
            if mode == 'STOP_NOW':
                db.event('arrived' if result == 'ok' else 'error', 'dung theo ma END_OF_LINE', trip)
                break
            if mode == 'WAIT' and now >= wait_until:
                mode = 'FOLLOW'
            if r['found'] and r['corner']:
                corner_seen_t, corner_seen_dir = now, r['corner_dir']
            if mode == 'FOLLOW':
                go_straight = trav_m < straight_until_m           # ma nga re bao di thang: khong re o nga nay
                if r['found'] and r['corner'] and not go_straight and not (r['cross'] and fc.get('stop_at_cross', False)):
                    mode, turn_dir, corner_gone, corner_max_y = 'APPROACH', r['corner_dir'], 0, r['corner_y']
                    votes = r['corner_dir'] * max(r.get('corner_score', 0.0), 0.05)
                    if turn_hint in (1, -1) and trav_m < hint_until_m:
                        turn_dir, votes = turn_hint, turn_hint * 1000.0   # ma ArUco thang moi phieu bau tu anh
                    target_cnt = corner_target(r['corner_y']) if (gcal and link) else None
                    db.event('info', 'thay nga cua -> se re %s' % ('TRAI' if turn_dir > 0 else 'PHAI'), trip)
                elif r['found']:
                    d_raw = (r['error'] - last_err) / dt
                    d_f += fc.get('d_lpf', 0.3) * (d_raw - d_f)
                    last_err = r['error']
                    trend += 0.2 * ((r['error'] + r['angle'] / 45.0) - trend)
                    last_dbg = r['debug']
                    if r['top_y'] > r['roi_top'] + fc.get('end_top_margin', 0.06):
                        end_seen_t = now                    # line ket thuc trong anh -> sap het duong
                    # bam tam line: line lech phai (error +) -> xoay phai (wz -). PID: I xoa lech deu 1 phia
                    i_err = max(-fc.get('i_max', 0.3), min(fc.get('i_max', 0.3), i_err + r['error'] * dt))
                    wz = -(fc['kp'] * r['error'] + fc['kd'] * d_f + fc.get('ki', 0.0) * i_err)
                    slow = abs(r['error']) > 0.5 or abs(r['angle']) > fc.get('corner_angle', 25)
                    vx = fc['slow_speed'] if slow else fc['speed']
                elif link and not go_straight and now - corner_seen_t < fc.get('corner_memory_s', 1.5):
                    # vua thay nga re roi mat line: do la goc cua (than doc khuat duoi xe) -> chay toi tam roi xoay
                    turn_dir = turn_hint if (turn_hint in (1, -1) and trav_m < hint_until_m) else corner_seen_dir
                    votes = turn_dir * 1.0
                    mode, enc_c0, t_mode, target_cnt = 'CREEP', list(link.telem['enc']), now, None
                    db.event('info', 'mat line ngay sau nga re -> coi la goc cua, xoay %s' % ('TRAI' if turn_dir > 0 else 'PHAI'), trip)
                elif fc.get('end_on_lost', True) and now - end_seen_t < 1.0 and link:
                    mode, enc_c0, t_mode = 'END', list(link.telem['enc']), now
                    db.event('info', 'thay cuoi line: chay them %d mm roi dung' % fc.get('corner_extra_mm', 240), trip)
                elif mem is not None and mem_s() is not None and mem.length() - mem_s() >= mcfg.get('memory_min_ahead_mm', 350) \
                        and _distance_m(link.telem['enc'], enc0) - s_start >= fc.get('search_min_travel_m', 0.15):
                    mem_ref = mem.snapshot()
                    mode, t_mode, confirm, mem_s0 = 'MEMORY', now, 0, mem_s(mem_ref)
                    db.event('info', 'mat line -> bam theo BO NHO (con %.0f mm duong da biet phia truoc)' % (mem.length() - mem_s0), trip)
                else:
                    if link and _distance_m(link.telem['enc'], enc0) - s_start < fc.get('search_min_travel_m', 0.15):
                        mode, t_mode, start_ok = 'START', now, 0     # chua di duoc bao xa: dung cho line on dinh lai, khong xoay lung tung
                        db.event('info', 'mat line ngay sau xuat phat -> dung cho, khong xoay tim', trip)
                    else:
                        mode, t_mode, confirm = 'SEARCH', now, 0
                        turn_dir = 1 if trend <= 0 else -1
                        db.event('info', 'mat line, xoay %s tim lai' % ('trai' if turn_dir > 0 else 'phai'), trip)
                        if last_dbg is not None:
                            cv2.imwrite('lost_%d.jpg' % trip, last_dbg)

            if mode == 'APPROACH':
                # chay cham toi nga cua, lai theo than line doc gan xe (khong bi nhanh ngang keo lech)
                near = r['near_error'] if r['found'] else None
                if r['found'] and r['corner']:
                    corner_gone, corner_max_y = 0, max(corner_max_y, r['corner_y'])
                else:
                    corner_gone += 1
                    if corner_gone == 1 and link:
                        enc_gone = list(link.telem['enc'])  # tinh quang "chay them" tu khung DAU TIEN mat nga cua
                gone = corner_gone >= fc.get('corner_lost_frames', 6)
                # "toi" = nga cua da troi ra khoi DAY anh (truoc do da xuong gan day), khong dung som nua
                reached = gone and corner_max_y >= fc.get('corner_near_y', 0.60)
                if gone and not reached:
                    mode = 'FOLLOW'                         # nga cua o xa roi bien mat: bao nham, bam line tiep
                    db.event('info', 'nga cua bien mat khi con xa (y max %.2f): bo qua' % corner_max_y, trip)
                elif reached and link and fc.get('corner_extra_mm', 0) > 0:
                    mode, enc_c0, t_mode, ignored_far = 'CREEP', enc_gone, now, False
                    if gcal and target_cnt is not None:
                        db.event('info', 'nga re ra khoi anh: con %.0f mm (do bang camera) toi tam xe' % (
                            (target_cnt - enc_mean()) * mm_per_count), trip)
                    else:
                        db.event('info', 'nga cua ra khoi anh: chay thang them %d mm (chua hieu chinh ground_calib)' % fc['corner_extra_mm'], trip)
                elif reached:
                    h_start = h_prev = None
                    turned = 0.0
                    mode, t_mode, confirm = 'TURN', now, 0
                    db.event('info', 'toi nga cua: dung, xoay %s' % ('TRAI' if turn_dir > 0 else 'PHAI'), trip)
                else:
                    if r['found'] and r['corner']:
                        votes += r['corner_dir'] * max(r.get('corner_score', 0.0), 0.05) * (r['corner_y'] < fc.get('corner_vote_max_y', 0.85))
                        if gcal and link:
                            target_cnt, _ = gated_target(target_cnt, r['corner_y'])
                    vx = fc.get('approach_speed', 500)
                    wz = -fc['kp'] * near if near is not None else 0

            if mode == 'END':
                done_mm = sum(a - b for a, b in zip(link.telem['enc'], enc_c0)) / 4 * mm_per_count
                rem_mm = fc.get('corner_extra_mm', 240) - done_mm
                # line GAY dai hon tam nhin camera trong giong cuoi line: dang chay them ma thay line lai PHIA TRUOC -> bam tiep
                end_resume = end_resume + 1 if (r['found'] and abs(r['error']) < fc.get('gap_resume_err', 0.4) and done_mm > 30) else 0
                if end_resume >= fc.get('search_confirm', 3):
                    mode, d_f, last_err, i_err, end_resume = 'FOLLOW', 0.0, r['error'], 0.0, 0
                    end_seen_t = 0.0
                    db.event('info', 'line GAY (khong phai cuoi line): chay them %.0f mm thi thay line lai -> bam tiep' % done_mm, trip)
                elif rem_mm <= brake_mm(v_cmd) or now - t_mode > 15:
                    if turn_hint == 2 and trav_m < hint_until_m + sc.get('uturn_extra_m', 1.0):
                        # ma U_TURN_AHEAD: cuoi tuyen cut -> quay dau 180 do (theo gyro) roi bam line ve
                        mode, t_mode, confirm, turn_dir, uturn = 'TURN', now, 0, 1, True
                        h_start = h_prev = None
                        turned, turn_hint = 0.0, 0
                        db.event('marker', 'cuoi tuyen cut: quay dau', trip)
                    else:
                        state, result = 'ARRIVED', 'ok'
                        if nav and not nav.done:
                            state, result = 'END_NOT_TARGET', 'failed'
                            db.event('error', 'het line nhung chua toi dich %s' % nav.target, trip)
                        else:
                            db.event('arrived', 'toi cuoi line (chay them %.0f mm)' % done_mm, trip)
                        break
                vx = fc.get('approach_speed', 500)

            if mode == 'CREEP':
                # chay thang (STM32 giu huong bang gyro) toi khi tam xe nam tren nga cua
                if r['found'] and r['corner']:
                    votes += r['corner_dir'] * max(r.get('corner_score', 0.0), 0.05) * (r['corner_y'] < fc.get('corner_vote_max_y', 0.85))   # bau chon huong
                    if gcal:
                        target_cnt, used = gated_target(target_cnt, r['corner_y'])   # khoang cach that, co loc nhay
                        if not used and not ignored_far:
                            ignored_far = True
                            db.event('info', 'bo qua line ngang o xa (khong phai nga re dang toi)', trip)
                done_mm = sum(a - b for a, b in zip(link.telem['enc'], enc_c0)) / 4 * mm_per_count
                if gcal and target_cnt is not None:
                    rem_mm = (target_cnt - enc_mean()) * mm_per_count
                else:
                    rem_mm = fc.get('corner_extra_mm', 240) - done_mm
                if rem_mm <= brake_mm(v_cmd) or now - t_mode > 15:
                    mode, t_mode, confirm = 'TURN', now, 0
                    if votes:
                        turn_dir = 1 if votes > 0 else -1   # chot huong theo da so cac khung
                    h_start = h_prev = None
                    turned = 0.0
                    db.event('info', 'tam xe toi nga cua (chay them %.0f mm): dung, xoay %s (phieu %+.2f)' % (
                        done_mm, 'TRAI' if turn_dir > 0 else 'PHAI', votes), trip)
                else:
                    vx = fc.get('approach_speed', 500)

            if mode == 'TURN':
                el = now - t_mode
                if el < fc.get('corner_pause_s', 0.4):
                    pass                                    # dung yen hẳn truoc khi xoay
                elif el > fc.get('turn_max_s', 8.0):
                    result, state = 'failed', 'TURN_FAILED'
                    db.event('line_lost', 'xoay o nga cua %.1f s khong thay line moi, xe dung' % el, trip)
                    break
                else:
                    w_t = fc.get('turn_wz', 800) if turned < fc.get('turn_slow_deg', 60) else fc.get('turn_wz_min', 900)
                    wz = w_t * turn_dir
                    hd = link.telem.get('heading_deg') if link and link.telem else None
                    if hd is not None:
                        if h_prev is not None:
                            turned += abs(((hd - h_prev) + 180) % 360 - 180)   # cong don, chiu duoc qua +-180
                        h_prev = hd
                        min_ok = turned >= (sc.get('uturn_min_deg', 150) if uturn else fc.get('turn_min_deg', 50))
                        if turned >= (sc.get('uturn_max_deg', 230) if uturn else fc.get('turn_max_deg', 150)):
                            result, state = 'failed', 'TURN_FAILED'
                            db.event('line_lost', 'xoay %.0f do o nga cua khong thay line moi, xe dung' % turned, trip)
                            break
                    else:
                        min_ok = el > fc.get('corner_pause_s', 0.4) + fc.get('turn_min_s', 0.8)
                    # line moi: than line GAN XE nam giua anh. Cho phep thay nga cua tiep theo o xa
                    # (duong co nhieu nga re lien tiep, vd chu U). Goc chi xet khi khong co nga cua (goc bi nhanh ngang lam sai)
                    e_near = r['near_error'] if r['near_error'] is not None else r['error']
                    if r['g_angle'] is not None:
                        # goc THAT (mat san): line moi song song huong xe -> xong xoay; lech ngang de ALIGN lo
                        ok_line = (r['found'] and min_ok and abs(r['g_angle']) < fc.get('turn_accept_deg', 8)
                                   and abs(r['g_offset_mm']) < fc.get('align_max_mm', 200))
                    else:
                        ok_line = (r['found'] and abs(e_near) < fc.get('turn_accept', 0.25) and min_ok
                                   and (r['corner'] or abs(r['angle']) < fc.get('turn_accept_angle', 90)))
                    confirm = confirm + 1 if ok_line else 0
                    if confirm >= fc.get('search_confirm', 3):
                        mode, d_f, last_err, trend, i_err = 'FOLLOW', 0.0, r['error'], 0.0, 0.0
                        uturn = False
                        if r['g_offset_mm'] is not None and abs(r['g_offset_mm']) > fc.get('align_ok_mm', 15):
                            mode, t_mode = 'ALIGN', now              # line song song nhung lech ngang: truot ve
                        wz = 0
                        db.event('info', 'qua nga cua xong (xoay %.0f do, %.1f s)' % (turned, el), trip)

            if mode == 'STATION':
                # da xac nhan tram can toi: van bam line, cham lai, theo doi marker xuong gan day anh
                vis = mk.visible.get(st_id)
                if vis:
                    st_max_y, st_gone = max(st_max_y, vis['cy']), 0
                elif mk.n % max(1, int(cfg['aruco'].get('process_every_n_frames', 2))) == 0:
                    st_gone += 1
                trig = (vis and vis['cy'] >= sc.get('marker_trigger_y', 0.8)) or \
                       (st_gone >= sc.get('marker_gone_frames', 6) and st_max_y >= sc.get('marker_near_y', 0.6))
                if trig and link:
                    mode, enc_c0, t_mode = 'ST_CREEP', list(link.telem['enc']), now
                elif st_gone >= sc.get('marker_lost_frames', 30) or now - t_mode > sc.get('approach_timeout_s', 20):
                    mode = 'FOLLOW'
                    db.event('station', 'mat dau marker tram ID %d truoc khi toi (y max %.2f): bo qua' % (st_id, st_max_y), trip)
                elif r['found']:
                    wz = -(fc['kp'] * r['error'] + fc['kd'] * d_f)
                    vx = sc.get('approach_speed', 1000)

            if mode == 'ST_CREEP':
                off = stations[st_id].get('stop_offset_mm') or sc.get('stop_offset_mm', 250)
                done_mm = sum(a - b for a, b in zip(link.telem['enc'], enc_c0)) / 4 * mm_per_count
                rem_mm = off - done_mm
                if rem_mm <= brake_mm(v_cmd) or now - t_mode > sc.get('creep_timeout_s', 15):
                    mode, t_mode, st_cmd_t, st_busy_seen = 'ST_ACTION', now, None, False
                    db.event('station', 'dung tai tram %s (chay them %.0f mm)' % (stations[st_id]['name'], done_mm), trip)
                else:
                    vx = sc.get('approach_speed', 1000)

            if mode == 'ST_ACTION':
                # KHOA CHUYEN DONG trong luc nang/ha (interlock, kit H8): vx = vy = wz = 0 suot buoc nay
                act = stations[st_id]['action']
                el = now - t_mode
                t = link.telem if link else {}
                if el < sc.get('pause_s', 0.5):
                    pass
                elif st_cmd_t is None:
                    if act in ('LIFT_UP', 'LIFT_DOWN') and args.no_lift:
                        db.event('station', '%s GIA LAP (--no-lift): dung %.0f s, khong chay stepper' % (act, sc.get('sim_lift_s', 2)), trip)
                    elif act in ('LIFT_UP', 'LIFT_DOWN') and link:
                        sps_l = lift_cfg.get('speed_sps', 800)
                        if lift_cfg.get('up_s'):
                            n = int(round(lift_cfg['up_s'] * sps_l))          # nang theo THOI GIAN (chua chot buoc vit me)
                            what = '%.1f s' % lift_cfg['up_s']
                        else:
                            n = int(round(lift_cfg['up_mm'] * lift_cfg.get('steps_per_mm', 800)))
                            what = '%.0f mm' % lift_cfg['up_mm']
                        link.lift_move((n if act == 'LIFT_UP' else -n) * int(lift_cfg.get('dir_sign', 1)), sps_l)
                        db.event('station', '%s %s (%d buoc)' % (act, what, n), trip)
                    st_cmd_t = now
                else:
                    st_busy_seen = st_busy_seen or bool(t.get('lift_busy'))
                    done = act not in ('LIFT_UP', 'LIFT_DOWN') or (st_busy_seen and not t.get('lift_busy')) or \
                        (not t.get('lift_busy') and now - st_cmd_t > 1.0)
                    if args.no_lift and act in ('LIFT_UP', 'LIFT_DOWN'):
                        done = now - st_cmd_t >= sc.get('sim_lift_s', 2)
                    if now - st_cmd_t > sc.get('action_timeout_s', 30):
                        result, state = 'failed', 'LIFT_TIMEOUT'
                        db.event('error', '%s qua %d s chua xong -> dung nhiem vu' % (act, sc.get('action_timeout_s', 30)), trip)
                        break
                    if done:
                        if act == 'LIFT_UP':
                            loaded = True
                        elif act == 'LIFT_DOWN':
                            loaded = False
                        db.event('station', 'xong %s tai %s (vi tri lift %s)' % (act, stations[st_id]['name'], t.get('lift_pos')), trip)
                        step_i += 1
                        mk.lock(st_id)
                        if nav:
                            nav.station_done()
                            speed_cap = nav.speed_cps
                        if step_i >= len(steps):
                            state, result = 'MISSION_DONE', 'ok'
                            db.event('arrived', 'hoan thanh nhiem vu (%d buoc)' % len(steps), trip)
                            break
                        mode, d_f, i_err = 'FOLLOW', 0.0, 0.0

            if mode == 'ALIGN':
                # mecanum truot ngang ve tam line + giu goc that ~0 (chi khi co hieu chinh mat san)
                if not r['found'] or r['g_offset_mm'] is None or now - t_mode > fc.get('align_timeout_s', 8):
                    mode = 'FOLLOW'
                elif abs(r['g_offset_mm']) <= fc.get('align_ok_mm', 15):
                    mode = 'FOLLOW'
                    db.event('info', 'can ngang xong (con lech %.0f mm)' % r['g_offset_mm'], trip)
                else:
                    vy = max(-fc.get('align_vy_max', 1500), min(fc.get('align_vy_max', 1500),
                             -fc.get('align_k', 8.0) * r['g_offset_mm']))     # line ben phai (+) -> truot phai (vy -)
                    wz = -fc.get('align_kw', 30.0) * r['g_angle']
                    vx = fc.get('align_vx', 300)

            if mode == 'CURVE':
                # quy dao lech ngang hinh sin theo quang di toi: ra (S1) -> giu (vuot qua) -> ve (S1). Than xe giu huong (gyro).
                import math
                s = fwd_mm() - cv['s0']
                lat = lateral_mm() - ob_base_lat
                vxc = oc.get('curve_speed', 1000)
                if cv['phase'] == 'OUT':
                    u = min(1.0, s / cv['S1'])
                    tgt = cv['D'] * (1 - math.cos(math.pi * u)) / 2
                    dtgt = cv['D'] * math.pi / (2 * cv['S1']) * math.sin(math.pi * u)
                    if u >= 1.0:
                        cv.update(phase='HOLD', s_hold=s)
                elif cv['phase'] == 'HOLD':
                    tgt, dtgt = cv['D'], 0.0
                    lean = 'right' if cv['D'] > 0 else 'left'
                    g_now = om.beside_clear(lean, offset_mm=lat)
                    if g_now and cv.get('gone_at') is None:
                        cv['gone_at'] = s
                    elif not g_now:
                        cv['gone_at'] = None
                    passed = (cv.get('gone_at') is not None and s - cv['gone_at'] >= tail_clear_mm
                              and s - cv['s_hold'] >= (oc.get('pass_min_cam_mm', 850) if ob_by_cam else oc.get('pass_min_mm', 450)))
                    if passed:
                        cv.update(phase='BACK', s_back=s)
                        db.event('obstacle', 'da vuot vat -> cong ve tuyen', trip)
                    elif s - cv['s_hold'] > oc.get('pass_max_mm', 1500):
                        result, state = 'failed', 'PASS_TOO_LONG'
                        db.event('obstacle', 'ne cong %.0f mm van chua vuot duoc -> dung' % (s - cv['s_hold']), trip)
                        break
                if cv['phase'] == 'BACK':
                    u = min(1.0, (s - cv['s_back']) / cv['S1'])
                    tgt = cv['D'] * (1 + math.cos(math.pi * u)) / 2
                    dtgt = -cv['D'] * math.pi / (2 * cv['S1']) * math.sin(math.pi * u)
                    if u >= 1.0 and abs(lat) <= oc.get('return_tol_mm', 20) * 2:
                        mode, t_mode, confirm = 'REACQUIRE', now, 0
                        ob_base_lat = None
                        db.event('obstacle', 'ne cong xong (lech con %.0f mm) -> tim lai line' % lat, trip)
                if mode == 'CURVE':
                    vx = vxc
                    vy = vxc * dtgt + oc.get('curve_k', 3.0) * (tgt - lat)         # bam quy dao + sua sai
                    vy = max(-oc.get('strafe_speed', 1200), min(oc.get('strafe_speed', 1200), vy))
                    if ob['zone'] == 'STOP' or lid.age() > oc.get('scan_timeout_s', 0.5):   # an toan: gap vat gan / mat LiDAR -> dung
                        mode, t_mode, ob_clear_n, ob_noroom_logged = 'BLOCKED', now, 0, False
                        vx = vy = 0
                        db.event('obstacle', 'dang ne cong gap vat < %d mm -> dung, chuyen truot ngang' % oc.get('stop_mm', 300), trip)

            if mode == 'BLOCKED':
                # dung cho. Vat di -> chay tiep. Van con sau wait_s -> ne neu duoc phep va du cho, khong thi cho tiep.
                stale = om and lid.age() > oc.get('scan_timeout_s', 0.5)
                ob_clear_n = ob_clear_n + 1 if (ob['zone'] != 'STOP' and not stale and not (mem is not None and ob_on_path)) else 0
                if ob_clear_n >= oc.get('clear_frames', 15):
                    off = abs(lateral_mm() - ob_base_lat) if (ob_base_lat is not None and link) else 0.0
                    if off >= ob_need - oc.get('return_tol_mm', 20) * 2:
                        # da lech DU quang can (theo be rong vat): vuot qua roi ve tuyen
                        mode, ob_pass0, t_mode, ob_gone_at = 'PASS', fwd_mm(), now, None
                        db.event('obstacle', 'da lech du %.0f mm -> chay song song vuot qua' % off, trip)
                    elif off > oc.get('return_tol_mm', 20) * 2:
                        # moi lech MOT PHAN: phia truoc "trong" co the chi vi vat ra khoi tam camera -> chua duoc vuot.
                        # tiep tuc truot khi ben do trong lai; khong thi o lai cho (an toan)
                        if free_n[ob_side] >= oc.get('free_scans', 12):
                            mode, t_mode, blocked_n = 'DETOUR', now, 0
                            db.event('obstacle', 'moi lech %.0f/%.0f mm, ben ne trong lai -> truot tiep' % (off, ob_need), trip)
                        else:
                            ob_clear_n = 0
                    else:
                        mode = ob_prev_mode if ob_prev_mode in ('FOLLOW', 'APPROACH', 'STATION', 'ALIGN', 'MEMORY', 'DEXEC', 'REJOIN', 'CREEP') else 'FOLLOW'
                        if mode in ('DEXEC', 'REJOIN') and dplan is None:
                            mode = 'FOLLOW'
                        ob_base_lat = None
                        db.event('obstacle', 'vat can da di -> chay tiep', trip)
                elif mem is not None and now - t_mode > oc.get('wait_s', 3.0) and not stale:
                    if now - plan_try_t > 1.0:
                        plan_try_t = now
                        allowed = oc.get('detour_enabled', True) and (not loaded or mcfg.get('detour_when_loaded', False)) \
                            and (nav.detour_allowed if nav else True)
                        ps = odo.pose_at(ob_scan_t)
                        pts_o = [MN.to_odom(ps, x, y) for x, y in ob.get('points', []) if abs(x) < 2500 and abs(y) < 1500]
                        if dplan is not None:
                            pts_o += getattr(dplan, 'pts', [])          # vat da thay truoc (nay o vung mu sau LiDAR)
                        pref = 1 if oc.get('prefer_side', 'left') == 'left' else -1
                        if allowed:
                            pl, sd, why = MN.plan_detour(mem, odo.pose, pts_o, GEOM, mcfg, pref)
                            if pl is not None and oc.get('prefer_only') and sd != pref:
                                pl, why = None, 'nguoi van hanh chi cho ne ve %s, ben do khong du cho' % ('9 gio' if pref > 0 else '3 gio')
                        else:
                            pl, sd, why = None, 0, 'khong duoc ne (dang cho hang / vung cam ne)'
                        if pl is not None and len(dplans) >= mcfg.get('max_replans', 4):
                            result, state = 'failed', 'DETOUR_RETRY'
                            db.event('obstacle', 'da lap quy dao ne %d lan van bi chan -> dung (ERROR)' % len(dplans), trip)
                            break
                        if pl is not None:
                            pl.pts = pts_o
                            dplan = pl
                            dplans.append(pl)
                            mode, t_mode, confirm = 'DEXEC', now, 0
                            s0_, s1_, cmin_, cmax_, n_ = pl.obstacle
                            db.event('obstacle', 'QUY DAO NE ve %s (%s): lech %.0f mm, vat rong %.0f mm sau %.0f mm, ra %.0f mm -> song song -> ve %.0f mm%s' % (
                                '10 gio 30' if sd > 0 else '1 gio 30', 'TRAI' if sd > 0 else 'PHAI', abs(pl.D), cmax_ - cmin_, s1_ - s0_,
                                pl.L_out, pl.L_back, '' if pl.end_s() <= pl.s_end_known else ' (duong sau vat: NGOAI SUY THANG)'), trip)
                        elif why != plan_why:
                            plan_why = why
                            db.event('obstacle', 'khong ne: %s -> dung cho' % why, trip)
                            try:                                     # luu du lieu de phan tich lai (bo nho, pose, LiDAR)
                                os.makedirs('logs', exist_ok=True)
                                json.dump({'why': why, 'pose': odo.pose, 'nodes': [n[:2] for n in mem.nodes], 'pts': pts_o,
                                           'hits': {str(k): v for k, v in getattr(MN.plan_detour, 'last_hits', {}).items()}},
                                          open(os.path.join('logs', 'planfail_%04d.json' % trip), 'w'))
                            except Exception:
                                pass
                    if mode == 'BLOCKED' and now - t_mode > oc.get('block_timeout_s', 60):
                        result, state = 'failed', 'BLOCKED_TIMEOUT'
                        db.event('obstacle', 'vat can chan %.0f s, khong ne duoc (%s) -> dung nhiem vu' % (now - t_mode, plan_why), trip)
                        break
                elif now - t_mode > oc.get('wait_s', 3.0) and not stale:
                    allowed = oc.get('detour_enabled', True) and not loaded and (nav.detour_allowed if nav else True) and om is not None
                    side, best = 0, None
                    order = (1, -1) if oc.get('prefer_side', 'left') == 'left' else (-1, 1)
                    for sd in (order[:1] if oc.get('prefer_only') else order):
                        need = detour_need(sd)
                        if need > oc.get('detour_max_mm', 700):
                            continue                                  # vat qua rong phia nay
                        free = om._side_free(ob.get('points', []), 'left' if sd > 0 else 'right', need + oc.get('side_margin_mm', 100))
                        free = free and free_n[sd] >= oc.get('free_scans', 6)   # phai trong lien tuc ~1 s
                        if free and (best is None or need < best):
                            side, best = sd, need
                    ob_need = best or oc.get('detour_mm', 350)
                    if allowed and side:
                        mode, ob_side, ob_lat0, t_mode, blocked_n = 'DETOUR', side, lateral_mm(), now, 0
                        ob_theta = diag_theta(ob['front_mm'], ob_need) if (oc.get('diag_enabled', True) and not ob_by_cam) else 90.0
                        db.event('obstacle', 'vat can van con -> NE ve %s (%s) %.0f mm, goc %.0f do' % (
                            clock_txt(side, ob_theta), 'TRAI' if side > 0 else 'PHAI', ob_need, ob_theta), trip)
                    elif allowed and not ob_noroom_logged:
                        ob_noroom_logged = True
                        db.event('obstacle', 'khong ben nao du cho de ne (can %.0f mm trong) -> dung cho vat di' % (
                            min(detour_need(1), detour_need(-1)) + oc.get('side_clearance_mm', 500) - oc.get('detour_mm', 350)), trip)
                    elif now - t_mode > oc.get('block_timeout_s', 60):
                        result, state = 'failed', 'BLOCKED_TIMEOUT'
                        db.event('obstacle', 'vat can chan %.0f s, khong ne duoc (%s) -> dung nhiem vu' % (
                            now - t_mode, 'dang cho hang' if loaded else ('khu cam ne' if not allowed else 'hai ben khong du cho')), trip)
                        break

            if mode == 'DETOUR':
                # truot ngang (mecanum, giu huong gyro) ra khoi tuyen. Ben do bi chan LIEN TUC -> dung lai cho.
                d = abs(lateral_mm() - (ob_base_lat if ob_base_lat is not None else ob_lat0))
                remain = max(0.0, ob_need - d)
                side_ok = om._side_free(ob.get('points', []), 'left' if ob_side > 0 else 'right', remain + oc.get('side_margin_mm', 100))
                blocked_n = 0 if side_ok else blocked_n + 1
                free = blocked_n < oc.get('block_scans', 3) * 5          # ~3 vong quet (camera 30 khung/s, LiDAR 6 vong/s)
                if d >= ob_need:
                    mode, ob_pass0, t_mode, ob_gone_at = 'PASS', fwd_mm(), now, None
                    db.event('obstacle', 'da ne ngang %.0f mm -> chay song song' % d, trip)
                elif not free and d < ob_need - 50:
                    mode, t_mode = 'BLOCKED', now
                    db.event('obstacle', 'ben dang ne bi chan -> dung cho', trip)
                else:
                    import math
                    sp = oc.get('strafe_speed', 1200)
                    vy = ob_side * sp * math.sin(math.radians(ob_theta))
                    vx = sp * math.cos(math.radians(ob_theta))
                    if vx > 0 and ob['front_mm'] is not None and ob['front_mm'] < oc.get('diag_front_margin_mm', 80) + 20:
                        vx = 0.0                                   # sat vat phia truoc (< 10 cm) -> chi truot ngang

            if mode == 'MEMORY':
                # P2: mat line tam thoi (gay / mo / bi che) -> bam theo duong DA NHO bang pure pursuit
                if mem_ref is None:
                    mem_ref = mem.snapshot()
                s_c = mem_s(mem_ref)
                confirm = confirm + 1 if r['found'] else 0
                if confirm >= fc.get('search_confirm', 3):
                    mode, d_f, last_err, i_err = 'FOLLOW', 0.0, r['error'], 0.0
                    db.event('info', 'thay lai line sau khi bam bo nho %.0f mm' % ((s_c or mem_s0) - mem_s0), trip)
                elif s_c is None or s_c - mem_s0 > mcfg.get('memory_follow_max_mm', 600) or s_c > mem_ref.length() + 20:
                    mode, t_mode, confirm = 'SEARCH', now, 0
                    turn_dir = 1 if trend <= 0 else -1
                    db.event('line_lost', 'bam bo nho %.0f mm van khong thay line -> xoay tim' % ((s_c or mem_s0) - mem_s0), trip)
                elif now - t_mode > mcfg.get('memory_timeout_s', 15):
                    result, state = 'failed', 'MEMORY_TIMEOUT'
                    db.event('error', 'bam bo nho qua %.0f s -> dung' % (now - t_mode), trip)
                    break
                else:
                    vx, vy, wz = pursue(mem_ref.point_at, s_c, mcfg.get('memory_follow_speed', 60))

            if mode == 'DEXEC':
                # P3: bam quy dao ne (bo nho + lech ngang smoothstep) - giu huong than theo duong
                s_c = mem_s(dplan.mem)
                if not r['found']:
                    confirm = 0
                if s_c is None:
                    mode, t_mode = 'BLOCKED', now
                elif (s_c >= dplan.obstacle[1] + GEOM['rear'] + mcfg.get('detour_long_margin_mm', 80)
                      and r['found'] and r['g_offset_mm'] is not None and (confirm := confirm + 1) >= mcfg.get('rejoin_confirm_frames', 5)):
                    # da qua vat va camera THAY line that (line cong / re ve phia xe): bo phan quy dao doan, bam line luon
                    pc = odo.pose
                    rejoin_marks.append((pc[0], pc[1], 'nhap som %+.0f mm' % r['g_offset_mm']))
                    mode, d_f, last_err, i_err, dplan = 'FOLLOW', 0.0, r['error'], 0.0, None
                    if abs(r['g_offset_mm']) > fc.get('align_ok_mm', 15):
                        mode, t_mode = 'ALIGN', now
                    db.event('obstacle', 'da qua vat, camera thay line that (lech %+.0f mm) -> bam line, bo doan quy dao con lai' % r['g_offset_mm'], trip)
                elif s_c >= dplan.end_s():
                    mode, t_mode, confirm, rj_s0 = 'REJOIN', now, 0, s_c
                    db.event('obstacle', 'het quy dao ne -> nhap lai line (lech ngang odom %.0f mm)' % (dplan.mem.cross_track(odo.pose) or 0), trip)
                elif now - t_mode > mcfg.get('detour_timeout_s', 60):
                    result, state = 'failed', 'DETOUR_TIMEOUT'
                    db.event('error', 'ne qua %.0f s -> dung' % (now - t_mode), trip)
                    break
                else:
                    vx, vy, wz = pursue(dplan.point, s_c, mcfg.get('detour_speed', 80))
                    if vx > 0 and (ob['zone'] == 'STOP' or (ob['front_mm'] is not None and ob['front_mm'] < oc.get('stop_mm', 300))) \
                            and abs(dplan.mem.cross_track(odo.pose) or 0.0) < abs(dplan.D) - 30:
                        # vat THANG TRUOC mui trong vung dung ma xe CHUA lech du khoi no -> khong duoc tien (chi truot ngang)
                        vx = 0.0
                    if vx > 0 and ob['front_mm'] is not None and ob['front_mm'] < oc.get('stop_mm', 300) \
                            and abs(dplan.mem.cross_track(odo.pose) or 0.0) >= abs(dplan.D) - 30 and s_c > dplan.s_a + dplan.L_out:
                        # da lech du ma VAN co vat thang truoc trong vung dung: khong phai chai -> vat MOI chan duong
                        ob_prev_mode, mode, t_mode, ob_clear_n = 'DEXEC', 'BLOCKED', now, 0
                        vx = vy = wz = 0
                        db.event('obstacle', 'dang ne: vat THANG TRUOC cach mui %.0f mm -> dung' % ob['front_mm'], trip)
                    if mode == 'DEXEC' and path_blocked(vx, vy, wz) and vx > 0:
                        # vat sat mui: chua lech du thi CHUA tien - truot ngang truoc (cung toc do)
                        lat_need = dplan.offset(s_c + PURSUIT['lookahead']) - (dplan.mem.cross_track(odo.pose) or 0.0)
                        sp = mcfg.get('detour_speed', 80) / mm_per_count
                        vx, vy = 0.0, (sp if lat_need > 0 else -sp) if abs(lat_need) > 10 else 0.0
                    if mode == 'DEXEC' and path_blocked(vx, vy, wz):
                        ob_prev_mode, mode, t_mode, ob_clear_n = 'DEXEC', 'BLOCKED', now, 0
                        vx = vy = wz = 0
                        db.event('obstacle', 'co vat MOI tren quy dao ne -> dung', trip)

            if mode == 'REJOIN':
                # P3: ve lai duong tham chieu, CAMERA phai thay line that N khung moi tin
                s_c = mem_s(dplan.mem)
                ok_line = r['found'] and (r['g_offset_mm'] is None or abs(r['g_offset_mm']) < mcfg.get('rejoin_max_offset_mm', 120))
                confirm = confirm + 1 if ok_line else 0
                if confirm >= mcfg.get('rejoin_confirm_frames', 5):
                    off_mm = r['g_offset_mm']
                    pc = odo.pose
                    rejoin_marks.append((pc[0], pc[1], 'nhap lai %+.0f mm' % (off_mm or 0)))
                    mode, d_f, last_err, i_err, dplan = 'FOLLOW', 0.0, r['error'], 0.0, None
                    if off_mm is not None and abs(off_mm) > fc.get('align_ok_mm', 15):
                        mode, t_mode = 'ALIGN', now
                    db.event('obstacle', 'NHAP LAI LINE: camera thay line, lech %s mm -> bam line tiep' % ('%+.0f' % off_mm if off_mm is not None else '?'), trip)
                elif s_c is None or s_c - rj_s0 > mcfg.get('rejoin_search_distance_mm', 400):
                    result, state = 'failed', 'REJOIN_FAILED'
                    db.event('line_lost', 'ne xong di them %.0f mm khong thay line -> dung (ERROR)' % ((s_c or rj_s0) - rj_s0), trip)
                    break
                else:
                    vx, vy, wz = pursue(dplan.mem.point_at, s_c, mcfg.get('rejoin_speed', 50))
                    if ob['front_mm'] is not None and ob['front_mm'] < oc.get('stop_mm', 300) and vx > 0:
                        ob_prev_mode, mode, t_mode, ob_clear_n = 'REJOIN', 'BLOCKED', now, 0
                        vx = vy = wz = 0
                        db.event('obstacle', 'nhap lai line: vat thang truoc cach mui %.0f mm -> dung' % ob['front_mm'], trip)
                    elif path_blocked(vx, vy, wz):
                        ob_prev_mode, mode, t_mode, ob_clear_n = 'REJOIN', 'BLOCKED', now, 0
                        vx = vy = wz = 0
                        db.event('obstacle', 'co vat tren duong nhap lai -> dung', trip)

            if mode == 'PASS':
                # chay thang song song tuyen toi khi vat da o SAU than xe (lidar) + them pass_clearance
                lean = (lateral_mm() - ob_base_lat) if (ob_base_lat is not None and link) else ob_side
                gone = (not om) or om.beside_clear('right' if lean > 0 else 'left', offset_mm=lean if ob_base_lat is not None else ob_need)
                done_mm = fwd_mm() - ob_pass0
                need_pass = oc.get('pass_min_cam_mm', 850) if ob_by_cam else oc.get('pass_min_mm', 450)
                if ob['zone'] == 'STOP':
                    mode, t_mode = 'BLOCKED', now
                    db.event('obstacle', 'gap vat can khi dang vuot -> dung', trip)
                if gone and ob_gone_at is None:
                    ob_gone_at = done_mm                  # vat vua khuat khoi tam LiDAR (mui xe): duoi xe van con ngang vat
                elif not gone:
                    ob_gone_at = None
                tail_ok = ob_gone_at is not None and done_mm - ob_gone_at >= tail_clear_mm
                if ob['zone'] == 'STOP':
                    pass
                elif gone and tail_ok and r['found'] and done_mm >= need_pass:
                    # duong line cong/re ve phia xe: da qua vat va THAY lai line -> bam line luon, khong chay thang tiep
                    mode, t_mode, confirm, ob_base_lat = 'REACQUIRE', now, 0, None
                    db.event('obstacle', 'da vuot vat (%.0f mm) va thay lai line -> bam line' % done_mm, trip)
                elif gone and tail_ok and done_mm >= need_pass:
                    mode, ob_lat0, t_mode = 'RETURN', lateral_mm(), now
                    db.event('obstacle', 'da vuot vat can (%.0f mm) -> quay ve tuyen' % done_mm, trip)
                elif done_mm > oc.get('pass_max_mm', 1500):
                    result, state = 'failed', 'PASS_TOO_LONG'
                    db.event('obstacle', 'chay song song %.0f mm van chua vuot duoc -> dung' % done_mm, trip)
                    break
                else:
                    vx = oc.get('pass_speed', 1000)

            if mode == 'RETURN':
                off = lateral_mm() - (ob_base_lat if ob_base_lat is not None else ob_lat0 - ob_side * ob_need)
                if abs(off) <= oc.get('return_tol_mm', 20):
                    mode, t_mode, confirm = 'REACQUIRE', now, 0
                    ob_base_lat = None
                    db.event('obstacle', 'da truot ve tuyen (con lech %.0f mm) -> tim lai line' % off, trip)
                else:
                    vy = -oc.get('strafe_speed', 1200) if off > 0 else oc.get('strafe_speed', 1200)
                    if abs(off) < 60:
                        vy *= 0.5                                  # cham lai khi gan ve tuyen
                    if oc.get('diag_enabled', True) and ob['zone'] == 'CLEAR':
                        import math
                        vx = abs(vy) / math.tan(math.radians(oc.get('return_deg', 45)))   # ve cheo (1 gio 30 / 10 gio 30)

            if mode == 'REACQUIRE':
                confirm = confirm + 1 if r['found'] else 0
                if confirm >= fc.get('search_confirm', 3):
                    mode, d_f, last_err, i_err = 'FOLLOW', 0.0, r['error'], 0.0
                    if r['g_offset_mm'] is not None and abs(r['g_offset_mm']) > fc.get('align_ok_mm', 15):
                        mode, t_mode = 'ALIGN', now
                    db.event('obstacle', 'tim lai duoc line sau khi ne -> bam line tiep', trip)
                elif now - t_mode > oc.get('reacquire_s', 5.0):
                    result, state = 'failed', 'REACQUIRE_FAILED'
                    db.event('line_lost', 'ne xong %.0f s khong thay lai line -> dung' % (now - t_mode), trip)
                    break
                else:
                    vx = oc.get('reacquire_speed', 400)

            if mode in ('SEARCH', 'TURN') and om and ob.get('points') is not None:
                import math
                r_rot = math.hypot(cfg['geometry'].get('car_length_mm', 450) / 2, cfg['geometry'].get('car_width_mm', 340) / 2) \
                    + oc.get('rotation_margin_mm', 60)
                near = [math.hypot(x, y) for x, y in ob['points'] if math.hypot(x, y) < r_rot]
                stale_r = lid.age() > oc.get('scan_timeout_s', 0.5)
                if near or stale_r:
                    if not rot_blocked:
                        rot_blocked, t_rot = True, now
                        db.event('obstacle', 'KHONG XOAY: co vat cach tam xe %.0f mm (goc xe quet toi %.0f mm) -> dung' % (
                            min(near) if near else -1, r_rot), trip)
                    if now - t_rot > oc.get('rotation_wait_s', 15):
                        result, state = 'failed', 'ROTATION_BLOCKED'
                        db.event('obstacle', 'vat can sat xe %.0f s, khong xoay duoc -> dung' % (now - t_rot), trip)
                        break
                    t_mode += dt                               # dong bang thoi gian cua trang thai xoay trong luc cho
                    vx = vy = wz = 0
                    w_cmd = 0.0
                    rot_hold = True
                else:
                    rot_blocked, rot_hold = False, False
            else:
                rot_hold = False
            if mode == 'SEARCH' and not rot_hold:
                el = now - t_mode
                if el > fc.get('search_s', 8.0):
                    result, state = 'failed', 'LINE_LOST'
                    db.event('line_lost', 'mat line, xoay tim %.1f s khong thay, xe dung' % el, trip)
                    break
                wz = fc.get('search_wz', 800) * turn_dir
                # chi nhan line hien ve phia dang xoay (bo doan line cu troi ra mep ben kia)
                ok_line = r['found'] and r['error'] * turn_dir <= fc.get('search_accept', 0.3)
                confirm = confirm + 1 if ok_line else 0
                if confirm >= fc.get('search_confirm', 3):
                    mode, d_f, last_err = 'FOLLOW', 0.0, r['error']
                    wz = 0
                    db.event('info', 'tim lai duoc line sau %.1f s' % el, trip)

            # ===== DIEU TOC THICH NGHI (ghi de vx theo ngu canh, roi gioi han tang/giam toc) =====
            vmax, vmin, vpt = fc['speed'], fc['slow_speed'], fc.get('point_speed', 600)
            if speed_cap:                                    # ma toc do ArUco (FAST/NORMAL/SLOW)
                vmax = min(vmax, speed_cap)
                vmin, vpt = min(vmin, vmax), min(vpt, vmax)
            if loaded:                                       # dang mang hang: cham lai toan bo (kit H10)
                ks = sc.get('loaded_speed_scale', 0.6)
                vmax, vmin, vpt = vmax * ks, vmin * ks, vpt * ks
            if mode == 'FOLLOW' and vx > 0:
                # line cang lech / nghieng -> cang cham (noi suy muot, khong nhay 2 muc nhu truoc)
                k = min(1.0, max(abs(r['error']) / fc.get('err_full', 0.5), abs(r['angle']) / fc.get('angle_full', 30)))
                vx = vmax - (vmax - vmin) * k
                # thay cuoi line trong anh -> ham dan theo vi tri dau mut (cang gan day anh cang cham)
                m0 = r['roi_top'] + fc.get('end_top_margin', 0.12)
                if r['found'] and r['top_y'] > m0:
                    f = max(0.0, (1.0 - r['top_y']) / (1.0 - m0))
                    vx = min(vx, vpt + (vmax - vpt) * f)
            elif mode == 'APPROACH' and vx > 0:
                # nga cua cang xuong gan day anh -> cang cham, toi day thi con point_speed
                f = max(0.0, min(1.0, (0.95 - corner_max_y) / 0.5))
                vx = vpt + (vmax - vpt) * f
            elif mode in ('CREEP', 'END', 'ST_CREEP') and vx > 0 and rem_mm is not None:
                # ham theo quang con lai: v = sqrt(2 a d), dung dung cho ma khong giat
                v_d = (2.0 * fc.get('creep_decel', 2000) * max(0.0, rem_mm) / mm_per_count) ** 0.5
                vx = max(fc.get('creep_min_speed', 300), min(vpt if mode != 'ST_CREEP' else sc.get('approach_speed', 1000), v_d))
            if (om or co) and ob['zone'] == 'SLOW' and mode in ('FOLLOW', 'APPROACH', 'STATION', 'ALIGN'):
                vx = min(vx, oc.get('slow_speed', 800))
                if mode == 'FOLLOW' and r['near_error'] is not None:
                    wz = -fc['kp'] * r['near_error']          # bo phan line o xa (co the la vat can mau xanh)
            if mode == 'ST_ACTION':
                vx = vy = wz = 0                            # interlock: tuyet doi khong chay khi nang/ha
            if loaded and vx > vmax:
                vx = vmax
            if mode in ('DETOUR', 'RETURN', 'MEMORY', 'DEXEC', 'REJOIN'):
                v_cmd = vx                                  # ne / ve tuyen CHEO: giu thanh phan tien da tinh (truoc day bi ep = 0 -> chi chay 9 gio)
            elif mode in ('START', 'TURN', 'SEARCH', 'ST_ACTION', 'WAIT', 'BLOCKED'):
                v_cmd = 0.0                                 # dung tai cho (xoay / cho / nang ha)
            else:
                v_cmd = ramp(v_cmd, vx, fc.get('accel_up', 2500), fc.get('accel_down', 2500))
            vx = v_cmd
            if mode == 'TURN':
                w_cmd = ramp(w_cmd, wz, fc.get('turn_accel', 4000), fc.get('turn_accel', 4000))
                wz = w_cmd
            else:
                w_cmd = 0.0
            wz = max(-fc['max_wz'], min(fc['max_wz'], wz))
            state = {'MEMORY': 'BAM_BO_NHO', 'DEXEC': 'NE_THEO_QD', 'REJOIN': 'NHAP_LAI_LINE', 'CURVE': 'NE_CONG', 'BLOCKED': 'VAT_CAN', 'DETOUR': 'NE_NGANG', 'PASS': 'VUOT_QUA', 'RETURN': 'VE_TUYEN',
                     'REACQUIRE': 'TIM_LAI_LINE', 'WAIT': 'DUNG_CHO', 'STATION': 'TOI_TRAM', 'ST_CREEP': 'VAO_TRAM', 'ST_ACTION': 'NANG_HA',
                     'START': 'START', 'ALIGN': 'CAN_NGANG', 'END': 'CUOI_LINE', 'CREEP': 'CHAY_THEM', 'FOLLOW': 'FOLLOW_LINE', 'APPROACH': 'TOI_NGA_CUA', 'TURN': 'XOAY_NGA_CUA', 'SEARCH': 'SEARCH'}[mode]
            if vw is not None:
                f = r['debug']
                hd = link.telem.get('heading_deg') if link and link.telem else None
                txt = '%5.1fs %s vx%d wz%+d' % (now - t0, state, vx, wz) + (' h%+.0f' % hd if hd is not None else '')
                if mode == 'TURN':
                    txt += ' xoay%.0f' % turned
                cv2.putText(f, txt, (5, f.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3)
                cv2.putText(f, txt, (5, f.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
                vw.write(f)

            if link and (mode == 'ST_ACTION' or (link.telem and link.telem.get('lift_busy'))):
                # dang NANG / HA: KHONG gui STOP (firmware STOP = dung ca lift). Gui van toc 0 -> banh dung yen, lien ket con song
                link.drive(0, 0, 0, hold=False)
            elif link:
                if vx or wz or vy:
                    link.drive(int(vx), int(vy), int(wz), hold=(mode in ('CREEP', 'END', 'ST_CREEP', 'DETOUR', 'PASS', 'RETURN', 'REACQUIRE', 'CURVE')))
                else:
                    link.stop()

            # ----- dashboard: anh debug + trang thai (~5 lan/s) -----
            frames += 1
            if now - fps_t >= 1.0:
                fps, frames, fps_t = frames / (now - fps_t), 0, now
                print('%5.1f s %-12s fps %4.1f | line %s lech %+.2f nga cua %s | vx %4d vy %+5d wz %+5d%s' % (
                    now - t0, state, fps, 'CO ' if r['found'] else 'MAT', r['error'],
                    ('%s y%.2f' % ('T' if r['corner_dir'] > 0 else 'P', r['corner_y'])) if r['corner'] else '-  ', vx, vy, wz,
                    (' | vat can %s %s mm (cam %s) T%s P%s' % (ob['zone'], '%.0f' % ob['front_mm'] if ob['front_mm'] is not None else '-',
                     cst['zone'], 'trong' if ob['left_free'] else 'chan', 'trong' if ob['right_free'] else 'chan')) if (om or co) else ''), flush=True)
            if frames % 6 == 0:
                cv2.imwrite('cam_tmp.jpg', r['debug'])
                os.replace('cam_tmp.jpg', 'cam.jpg')
                t = link.telem if link else None
                dist = _distance_m(t['enc'], enc0) if t else 0.0
                db.set_status(state=state, trip_id=trip, item=args.item, dst=dst, line_found=r['found'],
                              loaded=loaded, mission_step=('%d/%d' % (step_i, len(steps))) if steps else None,
                              target=nav.target if nav else None, plan=plan, speed_cap=round(speed_cap) if speed_cap else None,
                              lift_pos=t['lift_pos'] if t else None,
                              line_error=round(r['error'], 3), fps=round(fps, 1), distance_m=round(dist, 2),
                              battery_v=t['batt_v'] if t else None, heading=t['heading_deg'] if t else None,
                              speed=t['speed'] if t else None, imu_ok=t['imu_ok'] if t else None,
                              obstacle=ob['zone'] if (om or co) else None, obstacle_mm=round(ob['front_mm']) if (om or co) and ob['front_mm'] is not None else None)
                if t and t['batt_v'] >= cfg['battery'].get('sensor_min_v', 6.0) and now - last_bat > cfg['battery']['log_every_s']:
                    last_bat = now
                    db.log_battery(t['batt_v'])
                    if t['batt_v'] < cfg['battery']['low_v'] and battery_percent(t['batt_v']) < 10:
                        db.event('low_battery', 'pin %.2f V' % t['batt_v'], trip)
    except KeyboardInterrupt:
        state, result = 'STOPPED', 'aborted'
    finally:
        if lid:
            lid.close()
        if link:
            link.stop()
            time.sleep(0.3)
            dist = _distance_m(link.telem['enc'], enc0) if link.telem else None
            link.close()
        else:
            dist = None
        cap.release()
        if vw is not None:
            vw.release()
        if vraw is not None:
            vraw.release()
        if mem is not None:
            try:
                os.makedirs('logs', exist_ok=True)
                with open(os.path.join('logs', 'pose_%04d.csv' % trip), 'w') as fpl:
                    fpl.write('khung,t,x_mm,y_mm,th_rad,mode\n')
                    for q in pose_log:
                        fpl.write(','.join(str(v) for v in q) + '\n')
                obst = [p for pl in dplans for p in getattr(pl, 'pts', [])]
                dl = mem.drift_log
                ttl = 'chuyen %d | bo nho %.2f m, %d nut | hieu chinh troi: %d lan, TB %.1f mm, max %.1f mm, tong %.0f/%.0f mm' % (
                    trip, mem.length() / 1000, len(mem.nodes), len(drift_seen),
                    (sum(abs(v) for v in drift_seen) / len(drift_seen)) if drift_seen else 0, max([abs(v) for v in drift_seen] or [0]),
                    odo.corr[0], odo.corr[1])
                MN.draw_png(os.path.join('logs', 'mem_%04d.png' % trip), mem, pose_log, dplans, obst, rejoin_marks, ttl)
                mem.save(os.path.join('logs', 'mem_%04d.json' % trip), {'trip': trip, 'drift': drift_seen[-200:]})
                with open(os.path.join('logs', 'drift_%04d.csv' % trip), 'w') as fd:
                    fd.write('t,lech_ngang_mm,lech_goc_do\n')
                    for q in dl:
                        fd.write('%.2f,%.1f,%.2f\n' % (q[0] - t0, q[1], q[2]))
                print(ttl, flush=True)
                if args.route and result == 'ok':
                    os.makedirs('maps', exist_ok=True)
                    mem.save(os.path.join('maps', 'route_%s.json' % args.route), {'trip': trip})
                    print('da luu tuyen maps/route_%s.json' % args.route, flush=True)
            except Exception as ex:                     # log loi, khong chan viec dung xe
                print('luu bo nho loi: %r' % ex, flush=True)
        db.end_trip(trip, result, dist)
        db.set_status(state=state, trip_id=None, line_found=False)
        print('Ket thuc: %s (%s)' % (state, result))


def _distance_m(enc, enc0):
    """Quang duong tien trung binh 4 banh (m). ~3960 xung / vong banh 80 mm."""
    d = sum(a - b for a, b in zip(enc, enc0)) / 4
    return abs(d) / 3960 * 3.14159 * 0.080


if __name__ == '__main__':
    main()
