#!/usr/bin/env python3
"""ArUco station markers (DICT_4X4_50 by default) — pure OpenCV, no ROS.

    det = MarkerDetector(cfg['aruco'])
    events = det.update(frame_bgr)      # list of confirmed markers THIS frame: {'id', 'cx', 'cy', 'size'}
    det.visible                         # dict id -> last seen {'cx','cy','size'} (normalised 0..1)
    det.lock(marker_id)                 # ignore this id until it has left the view for unlock_frames

Rules (from the forklift kit spec §8.2): a marker is confirmed only after it is seen in `confirm_frames`
consecutive processed frames; a confirmed/locked id is not reported again until it disappears for
`unlock_frames` processed frames (hysteresis, no double triggering).

Run alone:  python3 markers.py [image.jpg | camera_index]   -> prints markers, saves markers_debug.jpg
"""
import sys

import cv2
import numpy as np


def _dictionary(name):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


class MarkerDetector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.dict = _dictionary(cfg.get('dictionary', 'DICT_4X4_50'))
        if hasattr(cv2.aruco, 'ArucoDetector'):                       # OpenCV >= 4.7
            self._det = cv2.aruco.ArucoDetector(self.dict, cv2.aruco.DetectorParameters())
            self._detect = lambda g: self._det.detectMarkers(g)
        else:                                                         # OpenCV 4.5/4.6
            params = cv2.aruco.DetectorParameters_create()
            self._detect = lambda g: cv2.aruco.detectMarkers(g, self.dict, parameters=params)
        self.n = 0
        self.streak = {}       # id -> consecutive processed frames seen
        self.absent = {}       # id -> consecutive processed frames NOT seen (for locked ids)
        self.locked = set()
        self.visible = {}
        self.last_corners = []

    def detect(self, frame):
        """Raw detection on one frame: list of {'id','cx','cy','size'} (cx, cy, size normalised to frame)."""
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)
        self.last_corners = corners if ids is not None else []
        out = []
        if ids is None:
            return out
        for c, i in zip(corners, ids.flatten()):
            p = c.reshape(4, 2)
            side = float(np.mean([np.linalg.norm(p[k] - p[(k + 1) % 4]) for k in range(4)]))
            if side < self.cfg.get('min_side_px', 12):
                continue
            out.append({'id': int(i), 'cx': float(p[:, 0].mean() / w), 'cy': float(p[:, 1].mean() / h),
                        'size': side / w})
        return out

    def update(self, frame):
        """Call every camera frame. Processes every `process_every_n_frames`. Returns newly CONFIRMED markers."""
        self.n += 1
        if self.n % max(1, int(self.cfg.get('process_every_n_frames', 2))) != 0:
            return []
        seen = {m['id']: m for m in self.detect(frame)}
        self.visible = seen
        confirmed = []
        for i in list(self.streak):
            if i not in seen:
                del self.streak[i]
        for i, m in seen.items():
            self.streak[i] = self.streak.get(i, 0) + 1
            if i in self.locked:
                self.absent[i] = 0
                continue
            if self.streak[i] == int(self.cfg.get('confirm_frames', 3)):
                confirmed.append(m)
                self.lock(i)
        for i in list(self.locked):
            if i not in seen:
                self.absent[i] = self.absent.get(i, 0) + 1
                if self.absent[i] >= int(self.cfg.get('unlock_frames', 15)):
                    self.locked.discard(i)
                    self.absent.pop(i, None)
        return confirmed

    def lock(self, marker_id):
        self.locked.add(marker_id)
        self.absent[marker_id] = 0

    def draw(self, img):
        if len(self.last_corners):
            for c in self.last_corners:
                cv2.polylines(img, [c.reshape(4, 2).astype(np.int32)], True, (255, 0, 255), 2)
        for i, m in self.visible.items():
            h, w = img.shape[:2]
            cv2.putText(img, 'ID%d%s' % (i, '*' if i in self.locked else ''), (int(m['cx'] * w) - 15, int(m['cy'] * h)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        return img


def load_station_map(path='station_map.yaml'):
    """id -> {'name','default_action','stop_offset_mm'}"""
    import yaml
    data = yaml.safe_load(open(path))
    return {int(s['id']): s for s in data['stations']}


if __name__ == '__main__':
    import yaml
    cfg = yaml.safe_load(open('config.yaml'))
    det = MarkerDetector(cfg['aruco'])
    src = sys.argv[1] if len(sys.argv) > 1 else str(cfg['camera']['index'])
    if src.isdigit() or src.startswith('/dev/'):
        cap = cv2.VideoCapture(int(src) if src.isdigit() else src, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg['camera']['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg['camera']['height'])
        for _ in range(10):
            ok, frame = cap.read()
        cap.release()
        if not ok:
            sys.exit('Khong doc duoc camera')
    else:
        frame = cv2.imread(src)
    ms = det.detect(frame)
    print('marker:', ms if ms else 'khong thay')
    cv2.imwrite('markers_debug.jpg', det.draw(frame.copy()))
    print('da luu markers_debug.jpg')
