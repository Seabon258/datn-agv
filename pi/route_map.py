#!/usr/bin/env python3
"""ArUco route map (station_map.yaml schema_version 2) + marker navigator. Pure Python, no ROS, no hardware.

    rm = RouteMap('station_map.yaml')            # raises ValueError listing every problem in the table
    plan = rm.plan(cargo_type=2)                  # ['PICKUP', 'ZONE_B']  (or rm.plan(dest='ZONE_C'), return_home=True)
    nav = RouteNavigator(rm, plan, speed_to_cps)  # speed_to_cps: m/s -> wheel counts/s
    d = nav.on_marker(marker_id)                  # what the car should do now (see Directive below)
    nav.on_distance(travelled_m)                  # call every loop: expires FAST zone / pre-station fallback stop
    nav.target, nav.done

Directive keys (all optional): speed_cps (new speed cap), turn_hint (+1 left, -1 right, 0 none, 2 u-turn),
straight_through (ignore the junction/corner just ahead), slow_for_station (station id), stop_at_station (id,
do its action), stop_after_m (fallback stop distance), wait_s, end_of_line, detour_allowed, event (text for DB).
"""
import yaml

KIND_RANGES = {'station': range(0, 10), 'pre_station': range(10, 20), 'speed': range(20, 30),
               'turn': range(30, 35), 'junction': range(35, 40), 'rule': range(40, 50)}
DIR = {'left': 1, 'right': -1, 'straight': 0, 'uturn': 2}


class RouteMap:
    def __init__(self, path='station_map.yaml'):
        d = yaml.safe_load(open(path))
        if d.get('schema_version') != 2:
            raise ValueError('%s: can schema_version 2' % path)
        self.raw = d
        self.dictionary = d.get('dictionary', 'DICT_4X4_50')
        self.by_id = {}
        problems = []

        def add(kind, entry):
            i = int(entry['id'])
            if i in self.by_id:
                problems.append('ID %d bi trung (%s va %s)' % (i, self.by_id[i][0], kind))
            if not 0 <= i <= 49:
                problems.append('ID %d ngoai DICT_4X4_50 (0-49)' % i)
            self.by_id[i] = (kind, entry)

        for s in d.get('stations', []):
            add('station', s)
        self.stations = {s['name']: s for s in d.get('stations', [])}
        self.station_by_id = {int(s['id']): s for s in d.get('stations', [])}
        for key, k in (('pre_station', 'pre_station'), ('speed', 'speed'), ('rules', 'rule')):
            for e in d.get(key, []) or []:
                add(k, e)
        known = {'schema_version', 'dictionary', 'stations', 'pre_station', 'speed', 'turns', 'junctions', 'rules',
                 'cargo_routing', 'placement'}
        for key in d:
            if key not in known:
                problems.append('khoa "%s" khong biet (go sai ten?) - cac khoa hop le: %s' % (key, ', '.join(sorted(known))))
        for e in d.get('turns', []) or []:
            add('turn', e)
        for e in d.get('junctions', []) or []:
            add('junction', e)
        for i, (kind, e) in self.by_id.items():
            if i not in KIND_RANGES[kind]:
                problems.append('ID %d loai %s nam sai khoang %s' % (i, kind, KIND_RANGES[kind]))
            if kind == 'pre_station' and int(e['station_id']) not in self.station_by_id:
                problems.append('ma bao truoc %d tro toi tram %s khong ton tai' % (i, e['station_id']))
            if kind == 'turn' and e.get('direction') not in ('left', 'right', 'uturn'):
                problems.append('ma re %d: direction phai la left/right/uturn' % i)
            if kind == 'junction':
                for name, dr in (e.get('routes') or {}).items():
                    if name != 'default' and name not in self.stations:
                        problems.append('nga re %d: tram "%s" khong co trong stations' % (i, name))
                    if dr not in ('left', 'right', 'straight'):
                        problems.append('nga re %d: huong "%s" khong hop le' % (i, dr))
        self.cargo_routing = {int(k): v for k, v in (d.get('cargo_routing') or {}).items()}
        for c, name in self.cargo_routing.items():
            if name not in self.stations:
                problems.append('cargo_routing %d -> "%s" khong co trong stations' % (c, name))
        if problems:
            raise ValueError('Bang ma ArUco co loi:\n  - ' + '\n  - '.join(problems))

    def kind(self, marker_id):
        return self.by_id.get(int(marker_id), (None, None))

    def plan(self, cargo_type=None, dest=None, pickup=True, return_home=False):
        """Ordered target station names for a TRANSPORT mission."""
        if cargo_type is not None:
            if int(cargo_type) not in self.cargo_routing:
                raise ValueError('loai hang %s khong co trong cargo_routing' % cargo_type)
            dest = self.cargo_routing[int(cargo_type)]
        if dest not in self.stations:
            raise ValueError('tram dich "%s" khong co trong stations' % dest)
        p = (['PICKUP'] if pickup and 'PICKUP' in self.stations and dest != 'PICKUP' else []) + [dest]
        if return_home and 'HOME' in self.stations and dest != 'HOME':
            p.append('HOME')
        return p

    def junction_dir(self, jid, target_name):
        routes = self.by_id[int(jid)][1].get('routes') or {}
        return routes.get(target_name, routes.get('default', 'straight'))


class RouteNavigator:
    """Turns confirmed marker IDs into driving directives for the current target station."""

    def __init__(self, rm, plan, speed_to_cps, normal_speed_m_s=None):
        self.rm, self.plan, self.i = rm, list(plan), 0
        self.to_cps = speed_to_cps
        normal = normal_speed_m_s or next((e['speed'] for k, e in rm.by_id.values()
                                           if k == 'speed' and e['name'] == 'SPEED_NORMAL'), 0.15)
        self.normal_cps = speed_to_cps(normal)
        self.speed_cps = self.normal_cps
        self.fast_until_m = None
        self.pre_stop_at_m = None
        self.detour_allowed = True
        self.travelled_m = 0.0

    @property
    def target(self):
        return self.plan[self.i] if self.i < len(self.plan) else None

    @property
    def target_id(self):
        t = self.target
        return int(self.rm.stations[t]['id']) if t else None

    @property
    def done(self):
        return self.i >= len(self.plan)

    def station_done(self):
        """Call after the action at the current target finished."""
        self.i += 1
        self.pre_stop_at_m = None
        self.reset_speed()

    def reset_speed(self):
        self.speed_cps, self.fast_until_m = self.normal_cps, None

    def on_distance(self, travelled_m):
        """Every loop. Returns a directive when something expires (FAST zone end, pre-station fallback stop)."""
        self.travelled_m = travelled_m
        if self.fast_until_m is not None and travelled_m >= self.fast_until_m:
            self.reset_speed()
            return {'speed_cps': self.speed_cps, 'event': 'het doan FAST -> NORMAL'}
        if self.pre_stop_at_m is not None and travelled_m >= self.pre_stop_at_m:
            self.pre_stop_at_m = None
            return {'stop_at_station': self.target_id,
                    'event': 'khong thay ma tram %s sau quang bao truoc -> dung theo odometry' % self.target}
        return None

    def on_marker(self, mid):
        kind, e = self.rm.kind(mid)
        tgt = self.target
        if kind is None:
            return {'event': 'ma %d khong co trong bang -> bo qua' % mid}
        if kind == 'station':
            if e['name'] == tgt:
                self.pre_stop_at_m = None
                return {'stop_at_station': int(e['id']), 'event': 'toi tram dich %s (%s)' % (tgt, e['action'])}
            return {'event': 'di qua tram %s (dich hien tai %s)' % (e['name'], tgt)}
        if kind == 'pre_station':
            st = self.rm.station_by_id[int(e['station_id'])]
            if st['name'] == tgt:
                self.reset_speed()
                self.pre_stop_at_m = self.travelled_m + float(e.get('pre_distance_m', 0.3))
                return {'slow_for_station': int(st['id']), 'stop_after_m': float(e.get('pre_distance_m', 0.3)),
                        'speed_cps': self.speed_cps, 'event': 'bao truoc tram %s: cham lai' % tgt}
            return {'event': 'bao truoc tram %s (khong phai dich) -> bo qua' % st['name']}
        if kind == 'speed':
            self.speed_cps = self.to_cps(float(e['speed']))
            self.fast_until_m = (self.travelled_m + float(e['max_distance_m'])) if e.get('max_distance_m') else None
            return {'speed_cps': self.speed_cps, 'event': '%s %.2f m/s' % (e['name'], e['speed'])}
        if kind == 'turn':
            self.reset_speed()
            return {'turn_hint': DIR[e['direction']], 'speed_cps': self.speed_cps,
                    'event': '%s: chuan bi %s' % (e['name'], e['direction'])}
        if kind == 'junction':
            self.reset_speed()
            dr = self.rm.junction_dir(mid, tgt)
            return {'turn_hint': DIR[dr], 'straight_through': dr == 'straight', 'speed_cps': self.speed_cps,
                    'event': 'nga re %s: dich %s -> %s' % (e['name'], tgt, dr)}
        if kind == 'rule':
            n = e['name']
            if n == 'NO_DETOUR_ZONE':
                self.detour_allowed = False
                return {'detour_allowed': False, 'event': 'vao loi hep: cam ne'}
            if n == 'DETOUR_ALLOWED':
                self.detour_allowed = True
                return {'detour_allowed': True, 'event': 'het loi hep: cho phep ne'}
            if n == 'STOP_AND_WAIT':
                return {'wait_s': float(e.get('wait_s', 2.0)), 'event': 'giao cat: dung cho %.1f s' % e.get('wait_s', 2.0)}
            if n == 'END_OF_LINE':
                return {'end_of_line': True, 'error': not self.done,
                        'event': 'cuoi tuyen' + ('' if self.done else ' NHUNG chua toi dich %s' % tgt)}
            return {'event': '%s (khong lam gi)' % n}
        return {}


if __name__ == '__main__':
    import sys
    rm = RouteMap(sys.argv[1] if len(sys.argv) > 1 else 'station_map.yaml')
    print('Bang ma hop le: %d ma. Tram: %s' % (len(rm.by_id), ', '.join('%s(%d)' % (n, s['id']) for n, s in rm.stations.items())))
    for c, z in sorted(rm.cargo_routing.items()):
        print('  hang loai %d -> %s' % (c, ' -> '.join(rm.plan(cargo_type=c))))
