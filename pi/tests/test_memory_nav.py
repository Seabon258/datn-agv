"""pytest cho memory_nav: chay khong can xe.  python3 -m pytest -q tests/"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import memory_nav as M  # noqa: E402

GEO = {'front': 320.0, 'rear': 210.0, 'half_w': 215.0}
CFG = {'detour_margin_mm': 100, 'detour_curve_length_mm': 500, 'detour_max_mm': 700, 'extrapolate_min_straight_mm': 400,
       'straight_tol_mm': 25, 'rejoin_search_distance_mm': 400, 'detour_long_margin_mm': 80, 'sweep_margin_mm': 40}


def drive_and_observe(mem, path_pts, view=(300, 900), step=20.0, noise=0.0, odo_err=None, seed=1):
    """Xe chay doc path (danh sach diem ODOM that), moi buoc camera thay doan line [300..900] mm phia truoc tam xe."""
    rnd = random.Random(seed)
    s_tot = [0.0]
    for a, b in zip(path_pts, path_pts[1:]):
        s_tot.append(s_tot[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))

    def at(s):
        for i in range(len(path_pts) - 1):
            if s_tot[i + 1] >= s:
                u = (s - s_tot[i]) / max(1e-9, s_tot[i + 1] - s_tot[i])
                a, b = path_pts[i], path_pts[i + 1]
                return a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), math.atan2(b[1] - a[1], b[0] - a[0])
        a, b = path_pts[-2], path_pts[-1]
        return b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0])
    s, t = 0.0, 0.0
    while s < s_tot[-1]:
        x, y, th = at(s)
        pose_true = (x, y, th)
        pose_est = pose_true if odo_err is None else odo_err(pose_true, s)
        pts = []
        for d in range(int(view[0]), int(view[1]), 20):
            if s + d > s_tot[-1]:
                break
            X, Y, _ = at(s + d)
            cx, cy = M.to_car(pose_true, X, Y)
            if cx < view[0] or cx > view[1]:
                continue
            pts.append((cx + rnd.gauss(0, noise), cy + rnd.gauss(0, noise)))
        mem.observe(pts, pose_est, t)
        s += step
        t += 0.1
    return mem


def max_err_to(mem, truth_fn):
    return max(truth_fn(n[0], n[1]) for n in mem.nodes)


def test_odometry_straight_and_strafe():
    k = 0.0648
    o = M.Odometry(k)
    o.update({'enc': [0, 0, 0, 0], 'heading_deg': 0.0}, 0.0)
    o.update({'enc': [1000, 1000, 1000, 1000], 'heading_deg': 0.0}, 1.0)        # toi 64.8 mm
    assert abs(o.pose[0] - 1000 * k) < 1e-6 and abs(o.pose[1]) < 1e-6
    # truot trai: vy = (-fl + rl - rr + fr)/4 ; enc = [rr, rl, fr, fl] -> rr -, rl +, fr +, fl -
    o.update({'enc': [0, 2000, 2000, 0], 'heading_deg': 0.0}, 2.0)
    assert abs(o.pose[1] - 1000 * k) < 1e-6
    # noi suy theo thoi gian
    p = o.pose_at(0.5)
    assert abs(p[0] - 500 * k) < 1e-6


def test_odometry_heading_unwrap():
    o = M.Odometry(1.0)
    o.update({'enc': [0] * 4, 'heading_deg': 170.0}, 0)
    o.update({'enc': [0] * 4, 'heading_deg': -170.0}, 1)                        # qua +-180: chi xoay 20 do
    assert abs(math.degrees(o.pose[2]) - 20.0) < 1e-6


def test_memory_straight_line():
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (3000, 0)], noise=2.0)
    assert len(mem.nodes) > 100
    assert max_err_to(mem, lambda x, y: abs(y)) < 8
    assert abs(mem.length() - 2600) < 150                                     # thay tu 300 toi het duong
    # khong co nut trung: cac nut cach nhau >= ~res/2
    d = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(mem.nodes, mem.nodes[1:])]
    assert min(d) > 8


def test_memory_right_angle_corner_kept_sharp():
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (1500, 0), (1500, 1200)], noise=1.0)
    # goc vuong: co nut gan dinh goc (1500, 0), khong bi lam tron xa hon 40 mm
    _, dmin = mem._nearest(1500, 0)
    assert dmin < 40
    err = lambda x, y: min(abs(y) if x <= 1500 else 1e9, abs(x - 1500) if y >= 0 else 1e9)
    assert max_err_to(mem, err) < 15


def test_memory_gap_line():
    mem = M.LineMemory({})
    # line gay 100 mm: hai doan thang thang hang -> bo nho noi duoc (gap 100 < gap_max 250)
    pts = [(0, 0), (1000, 0)]
    drive_and_observe(mem, pts)
    n1 = len(mem.nodes)
    mem.observe([(x, 0.0) for x in range(300, 700, 20)], (800.0, 0.0, 0.0), 99.0)   # doan sau cho gay
    assert len(mem.nodes) > n1
    assert mem.straight_before(mem.length(), 400, 25)


def test_cross_track_and_projection():
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (3000, 0)])
    assert abs(mem.cross_track((1000.0, 50.0, 0.0)) - 50.0) < 3                 # xe lech TRAI 5 cm
    assert abs(mem.cross_track((1000.0, -30.0, 0.0)) + 30.0) < 3


def test_drift_correction_reduces_error():
    """Odometry troi ngang 0.05 mm moi mm di (truot mecanum): co hieu chinh -> loi ve gan 0."""
    class FakeOdo:
        def __init__(self):
            self.dy = 0.0

        def correct(self, dx, dy, dth=0.0):
            self.dy += dy

    def run(correct):
        mem = M.LineMemory({'drift_correction_gain': 0.3})
        odo = FakeOdo()
        errs = []
        s, t = 0.0, 0.0
        while s < 3000:
            drift = 0.05 * s + odo.dy                       # pose uoc luong lech trai so voi that
            pose_true = (s, 0.0, 0.0)
            pose_est = (s, drift, 0.0)
            pts = [(d, 0.0) for d in range(300, 900, 20) if s + d < 3300]
            mem.observe(pts, pose_est, t, odo=odo if correct else None)
            errs.append(abs(0.05 * s + odo.dy))
            s += 20
            t += 0.1
        return errs[-1]
    assert run(True) < 0.3 * run(False)


def test_detour_plan_clears_obstacle_and_is_smooth():
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (1200, 0)])
    pose = (300.0, 0.0, 0.0)
    # chai d=80 mm tai x 1000 tren line
    box = [(1000 + 40 * math.cos(a), 40 * math.sin(a)) for a in [i * 0.2 for i in range(32)] if math.cos(a) < 0.3]
    plan, side, why = M.plan_detour(mem, pose, box, GEO, CFG, prefer=1)
    assert plan is not None, why
    assert side == 1 and plan.D > GEO['half_w'] + 40
    # than xe khong cham vat o bat ky diem nao cua quy dao (vat that: ca hinh tron)
    full = [(1000 + 40 * math.cos(a), 40 * math.sin(a)) for a in [i * 0.1 for i in range(63)]]
    for q in plan.poses(300, plan.end_s(), 10):
        assert not M.footprint_hit(q, full, GEO['front'], GEO['rear'], GEO['half_w'], 20)
    # dau / cuoi tron: offset va dao ham ~0 o 2 dau
    assert plan.offset(plan.s_a) == 0 and abs(plan.offset(plan.end_s())) < 1e-6
    e = 1.0
    assert abs(plan.offset(plan.s_a + e) - plan.offset(plan.s_a)) < 0.01
    assert abs(plan.offset(plan.end_s()) - plan.offset(plan.end_s() - e)) < 0.01


def test_detour_picks_right_when_left_blocked():
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (1200, 0)])
    box = [(1000, y) for y in range(-40, 41, 10)]
    wall_left = [(x, 450) for x in range(0, 2500, 30)]
    plan, side, why = M.plan_detour(mem, (300.0, 0.0, 0.0), box + wall_left, GEO, CFG, prefer=1)
    assert plan is not None and side == -1, why


def test_detour_refuses_near_corner():
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (500, 0), (500, 700)])
    # bo nho chi biet toi gan goc; doan cuoi (sau goc) qua ngan -> khong thang -> khong ne
    mem.nodes = [n for n in mem.nodes if n[1] < 300]
    box = [(500, y) for y in range(150, 260, 10)]
    plan, side, why = M.plan_detour(mem, (100.0, 0.0, 0.0), [(520, 200)], GEO, CFG)
    assert plan is None


def test_detour_refuses_when_both_sides_blocked():
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (1200, 0)])
    box = [(1000, y) for y in range(-40, 41, 10)]
    walls = [(x, 450) for x in range(0, 2500, 30)] + [(x, -450) for x in range(0, 2500, 30)]
    plan, side, why = M.plan_detour(mem, (300.0, 0.0, 0.0), box + walls, GEO, CFG)
    assert plan is None


def test_pursuit_heads_to_path():
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (2000, 0)])
    # xe lech trai 60 mm -> vy phai am (sang phai), vx duong
    vx, vy, wz = M.pursuit_cmd((800.0, 60.0, 0.0), mem.point_at, 800 - 300, 150, 60, 1.5, 274, 0.0648, 2600)
    assert vx > 0 and vy < 0
    # xe quay lech trai 10 do -> wz am (xoay phai ve huong duong)
    _, _, wz = M.pursuit_cmd((800.0, 0.0, math.radians(10)), mem.point_at, 500, 150, 60, 1.5, 274, 0.0648, 2600)
    assert wz < 0


def test_simulated_detour_execution_no_collision():
    """Mo phong xe bam quy dao ne bang pure pursuit (dong hoc ly tuong) -> khong cham vat, ve lai line."""
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (1300, 0)])
    full = [(1000 + 40 * math.cos(a), 40 * math.sin(a)) for a in [i * 0.1 for i in range(63)]]
    pose = [300.0, 0.0, 0.0]
    plan, side, _ = M.plan_detour(mem, tuple(pose), [p for p in full if p[0] < 1000], GEO, CFG)
    k, dt = 0.0648, 0.05
    for _ in range(4000):
        s = mem.project(pose[0], pose[1])[1]
        if s > plan.end_s() + 300:
            break
        vx, vy, wz = M.pursuit_cmd(tuple(pose), plan.point, s, 150, 60, 1.5, 274, k, 2600)
        c, sn = math.cos(pose[2]), math.sin(pose[2])
        pose[0] += (vx * c - vy * sn) * k * dt
        pose[1] += (vx * sn + vy * c) * k * dt
        pose[2] += wz * k / 274 * dt
        assert not M.footprint_hit(tuple(pose), full, GEO['front'], GEO['rear'], GEO['half_w'], 10), pose
    assert abs(pose[1]) < 15 and abs(math.degrees(pose[2])) < 3


def test_state_table():
    assert M.next_state('FOLLOW', 'obstacle_on_path') == 'BLOCKED'
    assert M.next_state('DPLAN', 'plan_fail') == 'BLOCKED'
    assert M.next_state('REJOIN', 'rejoin_failed') == 'ERROR'
    try:
        M.next_state('FOLLOW', 'detour_done')
        assert False
    except ValueError:
        pass


def test_memory_bridges_after_corner_turn():
    """Sau xoay tai goc (than nhanh moi bat dau cach cuoi bo nho ~30 cm) -> bo nho van noi tiep."""
    mem = M.LineMemory({})
    for k in range(40):                                  # di thang toi goc (1000, 0); camera chi thay toi goc
        x = k * 20.0
        mem.observe([(d, 0.0) for d in range(300, 900, 20) if x + d <= 1000], (x, 0.0, 0.0), k * 0.1)
    n0 = len(mem.nodes)
    pose = (1000.0, 0.0, math.pi / 2)                     # da xoay trai 90 do tai goc
    for k in range(8):
        mem.observe([(d, 0.0) for d in range(300, 900, 20)], (1000.0, k * 10.0, math.pi / 2), 10 + k * 0.1)
    assert len(mem.nodes) > n0 + 20
    assert mem.nodes[-1][1] > 800 and abs(mem.nodes[-1][0] - 1000) < 5


def test_detour_refuses_corner_even_when_route_known():
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (1400, 0), (1400, 1300)])
    box = [(1250 + 40 * math.cos(a), 40 * math.sin(a)) for a in [i * 0.2 for i in range(32)] if math.cos(a) < 0.3]
    plan, side, why = M.plan_detour(mem, (300.0, 0.0, 0.0), box, GEO, CFG)
    assert plan is None and 'GOC RE' in why


def test_project_many_matches_project():
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (1000, 0), (1500, 600)])
    rnd = random.Random(3)
    pts = [(rnd.uniform(-200, 2000), rnd.uniform(-500, 900)) for _ in range(60)]
    many = mem.project_many(pts)
    for (X, Y), (s, c) in zip(pts, many):
        _, s1, c1 = mem.project(X, Y)
        assert abs(s - s1) < 1e-3 and abs(c - c1) < 1e-3


def test_wall_beside_line_not_merged_and_no_strafe_into_wall():
    """Tinh huong chuyen 52: tuong chay doc ben PHAI cach line 30 cm + chai tren line cach mui 50 cm."""
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (520, 0)], view=(300, 520))         # camera that chi thay ~22 cm line
    wall = [(x, -300) for x in range(-300, 2200, 15)]
    bottle = [(1150 + 40 * math.cos(a), 40 * math.sin(a)) for a in [i * 0.2 for i in range(32)] if math.cos(a) < 0.3]
    cfg = dict(CFG, extrapolate_min_straight_mm=150, path_band_mm=20)
    cl = M.obstacle_cluster(mem, bottle + wall, 0 + 320 - 50, 1500, 215 + 20)
    assert cl is not None and cl[1] - cl[0] < 200 and cl[3] - cl[2] < 120        # chi la chai
    plan, side, why = M.plan_detour(mem, (0.0, 0.0, 0.0), bottle + wall, GEO, cfg)
    assert plan is not None and side == 1, why                                    # ne TRAI, xa tuong
    full = [(1150 + 40 * math.cos(a), 40 * math.sin(a)) for a in [i * 0.1 for i in range(63)]]
    for q in plan.poses(0, plan.end_s(), 10):
        assert not M.footprint_hit(q, full + wall, GEO['front'], GEO['rear'], GEO['half_w'], 20)


def test_sweep_catches_immediate_strafe_into_wall():
    """Vat sat mui + tuong ben trai sat hong: khong duoc lap quy dao truot ngang xuyen tuong."""
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (800, 0)])
    box = [(560, y) for y in range(-40, 41, 10)]                               # sat mui (mui x=320)
    wall_l = [(x, 300) for x in range(-300, 1500, 15)]
    wall_r = [(x, -800) for x in range(-300, 1500, 15)]
    plan, side, why = M.plan_detour(mem, (0.0, 0.0, 0.0), box + wall_l + wall_r, GEO, CFG)
    assert plan is None or side == -1
    if plan is not None:
        assert abs(plan.D) < 800 - 215


def test_turn_between_sees_corner_ahead():
    mem = M.LineMemory({})
    drive_and_observe(mem, [(0, 0), (900, 0), (900, 800)])
    assert mem.turn_between(0, 500) < 10
    assert mem.turn_between(300, 1500) > 60
