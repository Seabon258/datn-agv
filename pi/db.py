#!/usr/bin/env python3
"""Co so du lieu SQLite cho AGV: hang hoa, chuyen chay, su co, pin, trang thai hien tai.

    db = AgvDB('agv.db')
    trip = db.start_trip(item_code='A01', src='Tram 1', dst='Kho A')
    db.event('line_lost', 'mat line 0.6 s tai doan 3', trip)
    db.end_trip(trip, 'ok')
    db.set_status(state='FOLLOW_LINE', battery_v=12.3, ...)

Moi tien trinh (line_follow.py, web.py) mo ket noi rieng; SQLite WAL cho phep doc/ghi dong thoi.
"""
import json
import sqlite3
import time

SCHEMA = """
CREATE TABLE IF NOT EXISTS items (
    code        TEXT PRIMARY KEY,          -- noi dung ma QR, vd A01
    name        TEXT NOT NULL,
    warehouse   TEXT NOT NULL,             -- kho dich
    note        TEXT DEFAULT ''
);
CREATE TABLE IF NOT EXISTS trips (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    start_ts    REAL NOT NULL,
    end_ts      REAL,
    item_code   TEXT,
    src         TEXT,
    dst         TEXT,
    result      TEXT DEFAULT 'running',    -- running | ok | failed | aborted
    distance_m  REAL DEFAULT 0
);
CREATE TABLE IF NOT EXISTS events (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          REAL NOT NULL,
    kind        TEXT NOT NULL,             -- line_lost | obstacle | low_battery | estop | link_lost | info ...
    detail      TEXT DEFAULT '',
    trip_id     INTEGER
);
CREATE TABLE IF NOT EXISTS battery_log (
    ts          REAL NOT NULL,
    volt        REAL NOT NULL,
    percent     REAL
);
CREATE TABLE IF NOT EXISTS status (
    id          INTEGER PRIMARY KEY CHECK (id = 1),
    ts          REAL,
    data        TEXT                       -- JSON trang thai hien tai cho dashboard
);
CREATE TABLE IF NOT EXISTS commands (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          REAL NOT NULL,
    cmd         TEXT NOT NULL,             -- estop | start | resume ... (dashboard -> robot)
    done        INTEGER DEFAULT 0
);
CREATE INDEX IF NOT EXISTS ev_ts ON events(ts);
CREATE INDEX IF NOT EXISTS bat_ts ON battery_log(ts);
"""

# Acquy chi 12V khi nghi: dien ap -> % (AGV_HARDWARE.md muc 6)
SOC_TABLE = [(11.30, 0), (11.51, 10), (11.66, 20), (11.81, 30), (11.96, 40), (12.10, 50),
             (12.24, 60), (12.37, 70), (12.50, 80), (12.62, 90), (12.73, 100)]


def battery_percent(v):
    if v <= SOC_TABLE[0][0]:
        return 0.0
    for (v0, p0), (v1, p1) in zip(SOC_TABLE, SOC_TABLE[1:]):
        if v <= v1:
            return p0 + (p1 - p0) * (v - v0) / (v1 - v0)
    return 100.0


class AgvDB:
    def __init__(self, path='agv.db'):
        self.con = sqlite3.connect(path, timeout=5, check_same_thread=False)
        self.con.row_factory = sqlite3.Row
        self.con.execute('PRAGMA journal_mode=WAL')
        self.con.executescript(SCHEMA)
        self.con.commit()

    # ----- hang hoa -----
    def upsert_item(self, code, name, warehouse, note=''):
        self.con.execute('INSERT INTO items(code,name,warehouse,note) VALUES(?,?,?,?) '
                         'ON CONFLICT(code) DO UPDATE SET name=excluded.name, warehouse=excluded.warehouse, '
                         'note=excluded.note', (code, name, warehouse, note))
        self.con.commit()

    def delete_item(self, code):
        self.con.execute('DELETE FROM items WHERE code=?', (code,))
        self.con.commit()

    def item(self, code):
        r = self.con.execute('SELECT * FROM items WHERE code=?', (code,)).fetchone()
        return dict(r) if r else None

    def items(self):
        return [dict(r) for r in self.con.execute('SELECT * FROM items ORDER BY code')]

    # ----- chuyen chay -----
    def start_trip(self, item_code=None, src=None, dst=None):
        cur = self.con.execute('INSERT INTO trips(start_ts,item_code,src,dst) VALUES(?,?,?,?)',
                               (time.time(), item_code, src, dst))
        self.con.commit()
        self.event('trip_start', '%s: %s -> %s' % (item_code or '-', src or '?', dst or '?'), cur.lastrowid)
        return cur.lastrowid

    def end_trip(self, trip_id, result='ok', distance_m=None):
        self.con.execute('UPDATE trips SET end_ts=?, result=?, distance_m=COALESCE(?, distance_m) WHERE id=?',
                         (time.time(), result, distance_m, trip_id))
        self.con.commit()
        self.event('trip_end', result, trip_id)

    def trips(self, limit=50):
        return [dict(r) for r in self.con.execute('SELECT * FROM trips ORDER BY id DESC LIMIT ?', (limit,))]

    # ----- su co / nhat ky -----
    def event(self, kind, detail='', trip_id=None):
        self.con.execute('INSERT INTO events(ts,kind,detail,trip_id) VALUES(?,?,?,?)',
                         (time.time(), kind, detail, trip_id))
        self.con.commit()

    def events(self, limit=100, kind=None):
        if kind:
            q = self.con.execute('SELECT * FROM events WHERE kind=? ORDER BY id DESC LIMIT ?', (kind, limit))
        else:
            q = self.con.execute('SELECT * FROM events ORDER BY id DESC LIMIT ?', (limit,))
        return [dict(r) for r in q]

    # ----- pin -----
    def log_battery(self, volt):
        self.con.execute('INSERT INTO battery_log(ts,volt,percent) VALUES(?,?,?)',
                         (time.time(), volt, battery_percent(volt)))
        self.con.commit()

    def battery(self, hours=24):
        q = self.con.execute('SELECT ts,volt,percent FROM battery_log WHERE ts>? ORDER BY ts',
                             (time.time() - hours * 3600,))
        return [dict(r) for r in q]

    # ----- trang thai hien tai (robot ghi, dashboard doc) -----
    def set_status(self, **data):
        self.con.execute('INSERT INTO status(id,ts,data) VALUES(1,?,?) '
                         'ON CONFLICT(id) DO UPDATE SET ts=excluded.ts, data=excluded.data',
                         (time.time(), json.dumps(data)))
        self.con.commit()

    def status(self):
        r = self.con.execute('SELECT ts,data FROM status WHERE id=1').fetchone()
        if not r:
            return {'ts': None}
        d = json.loads(r['data'])
        d['ts'] = r['ts']
        d['age_s'] = time.time() - r['ts']
        return d

    # ----- lenh tu dashboard -----
    def push_command(self, cmd):
        self.con.execute('INSERT INTO commands(ts,cmd) VALUES(?,?)', (time.time(), cmd))
        self.con.commit()

    def pop_commands(self):
        rows = [dict(r) for r in self.con.execute('SELECT * FROM commands WHERE done=0 ORDER BY id')]
        if rows:
            self.con.execute('UPDATE commands SET done=1 WHERE id<=?', (rows[-1]['id'],))
            self.con.commit()
        return [r['cmd'] for r in rows]

    def summary(self):
        c = self.con
        day = time.time() - 86400
        return {
            'trips_total': c.execute('SELECT COUNT(*) FROM trips').fetchone()[0],
            'trips_ok': c.execute("SELECT COUNT(*) FROM trips WHERE result='ok'").fetchone()[0],
            'trips_24h': c.execute('SELECT COUNT(*) FROM trips WHERE start_ts>?', (day,)).fetchone()[0],
            'incidents_24h': c.execute("SELECT COUNT(*) FROM events WHERE ts>? AND kind NOT IN "
                                       "('trip_start','trip_end','info')", (day,)).fetchone()[0],
            'distance_m': c.execute('SELECT COALESCE(SUM(distance_m),0) FROM trips').fetchone()[0],
        }
