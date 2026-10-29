"""(10/23 보스, 11/3 시연) bowls in the basket quadrants -> which quadrant is empty, seen by the fixed D455 + YOLOE.

The perception process owns the camera and shares its frame (/dev/shm/fixed_latest.npz, every 0.5 s).  This process
runs YOLOE ('bowl', 'plate', 'dish drying rack') on every new shared frame and
  - (with a window) draws the boxes and the four quadrants (green = empty, red = bowl, yellow = chosen) on the fixed
    view for the spectators, until the pipeline that started it ends (--until_pid; Q / ESC only without it);
  - writes the state (per quadrant: bowl points, empty; the bowls as boxes for the collision model) to --state.
The basket pose (for the quadrants) comes from the pipeline's basket_vision.json (--basket), read when it appears.

    python calibration/demo_watch.py --T_base_cam X.json --basket run/basket_vision.json --state /dev/shm/demo_quadrants.json
    python calibration/demo_watch.py --frame saved.npz --T_base_cam X.json --basket res.json --png out.png   (offline check)
Opt-in (pipeline --quadrant auto / --demo_view); nothing else reads it."""
import argparse
import json
import os
import time
import zipfile
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
PROMPTS = ['bowl', 'ceramic plate', 'red ceramic plate', 'dish drying rack']
NAME_ALIASES = {'ceramic plate': 'plate', 'red ceramic plate': 'plate', 'white ceramic bowl': 'bowl'}
COLOURS = {'bowl': (255, 140, 0), 'plate': (60, 200, 255), 'dish drying rack': (180, 180, 180)}   # RGB
# basket frame (slot frame, z = rim): x robot side -> far side, y left seen from the robot (plate_plans.basket_gaps)
HALF = (0.265, 0.26)
SPLIT = (0.035, 0.0095)                            # x: the clip between near / far, y: between pin pairs A (+0.129) / B (-0.11)
QUADRANTS = {'Q1': ('far', 'right'), 'Q2': ('far', 'left'), 'Q3': ('near', 'left'), 'Q4': ('near', 'right')}


BOWL_PROMPT = 'white ceramic bowl'                # (10/26 real frame: 'bowl' 0.23 / missed, this on the zoomed basket 0.31-0.35)


class Detector:
    """YOLOE on the whole frame for the other prompts; 'bowl' in the prompts = a second pass on the basket box (+ margin)
    zoomed `zoom` x with BOWL_PROMPT alone (bowls are ~40 px there and lose to 'plate' / the rack in one pass)."""

    def __init__(self, prompts=PROMPTS, zoom=2.0, imgsz=640, half=False):
        from ultralytics import YOLOE
        # (보스 10/29: bowls on the table came as 'plate' or nothing) 'bowl' also runs BOWL_PROMPT on the whole frame
        # (10/29 frame: the blue bowl 0.43 vs plate 0.18, the white one 0.36 vs missed)
        self.names = [BOWL_PROMPT if p == 'bowl' else p for p in prompts]
        self.zoom = zoom
        self.output_names = list(dict.fromkeys(NAME_ALIASES.get(p, p) for p in self.names))
        self.imgsz, self.half = int(imgsz), bool(half)
        self.m = YOLOE(str(ROOT / 'yoloe-v8l-seg.pt'))
        self.m.set_classes(self.names, self.m.get_text_pe(self.names))
        self.bowl = None
        if 'bowl' in prompts:
            self.bowl = YOLOE(str(ROOT / 'yoloe-v8l-seg.pt'))
            self.bowl.set_classes([BOWL_PROMPT, 'dish drying rack'], self.bowl.get_text_pe([BOWL_PROMPT, 'dish drying rack']))

    def _predict(self, model, names, rgb, conf):
        r = model.predict(rgb[:, :, ::-1].copy(), conf=conf, verbose=False, retina_masks=True,
                          imgsz=self.imgsz, half=self.half)[0]
        if r.boxes is None or not len(r.boxes):
            return []
        masks = r.masks.data.cpu().numpy() > 0.5 if r.masks is not None else [None] * len(r.boxes)
        return [(NAME_ALIASES.get(names[int(k)], names[int(k)]), float(c), tuple(int(v) for v in b), m) for k, c, b, m in
                zip(r.boxes.cls.cpu().numpy(), r.boxes.conf.cpu().numpy(), r.boxes.xyxy.cpu().numpy(), masks)]

    def __call__(self, rgb, conf=0.15, margin=30):
        """-> [(name, conf, (x0, y0, x1, y1), mask bool HxW), ...]"""
        dets = self._predict(self.m, self.names, rgb, conf)
        racks = sorted((d for d in dets if d[0] == 'dish drying rack'), key=lambda d: -d[1])
        dets = [d for d in dets if d[0] != 'dish drying rack'] + racks[:1]   # (10/26) one basket in the cell: the cloth under
        if self.bowl is None:                                                # it came as a 2nd rack (0.50 vs 0.93)
            return dets
        h, w = rgb.shape[:2]
        racks = [d for d in dets if d[0] == 'dish drying rack']
        x0, y0, x1, y1 = max(racks, key=lambda d: d[1])[2] if racks else (0, 0, w, h)
        x0, y0, x1, y1 = max(0, x0 - margin), max(0, y0 - margin), min(w, x1 + margin), min(h, y1 + margin)
        crop = cv2.resize(np.ascontiguousarray(rgb[y0:y1, x0:x1]), None, fx=self.zoom, fy=self.zoom, interpolation=cv2.INTER_CUBIC)
        for name, c, b, m in self._predict(self.bowl, ['bowl', 'rack'], crop, conf):
            if name != 'bowl':
                continue
            full = np.zeros((h, w), bool)
            if m is not None:
                full[y0:y1, x0:x1] = cv2.resize(m.astype(np.uint8), (x1 - x0, y1 - y0), interpolation=cv2.INTER_NEAREST) > 0
            dets.append(('bowl', c, tuple(int(v / self.zoom) + o for v, o in zip(b, (x0, y0, x0, y0))), full))
        return dets


def quadrant_of(xy):
    """Basket-frame xy -> 'Q1'..'Q4', or None outside the basket."""
    x, y = xy
    if abs(x) > HALF[0] or abs(y) > HALF[1]:
        return None
    fn, lr = 'near' if x < SPLIT[0] else 'far', 'left' if y > SPLIT[1] else 'right'
    return next(q for q, v in QUADRANTS.items() if v == (fn, lr))


def quadrant_corners(q):
    """Corners (basket frame, rim height) of a quadrant."""
    fn, lr = QUADRANTS[q]
    x = (-HALF[0], SPLIT[0]) if fn == 'near' else (SPLIT[0], HALF[0])
    y = (SPLIT[1], HALF[1]) if lr == 'left' else (-HALF[1], SPLIT[1])
    return np.array([[x[0], y[0], 0], [x[1], y[0], 0], [x[1], y[1], 0], [x[0], y[1], 0]])


def bowl_quadrants(dets, depth, K, Tbc, Tb, min_conf=0.2, min_points=80, share=0.25, name='bowl'):
    """Bowls (or `name`: 'plate' = plates already in the basket) -> {quadrant: their points there}; one counts in every
    quadrant holding >= `share` of its points."""
    from basket_vision import cloud
    out = {q: 0 for q in QUADRANTS}
    Tbi = np.linalg.inv(Tb)
    for n_, c, _, mk in dets:
        if n_ != name or c < min_conf or mk is None:
            continue
        P, _ = cloud(cv2.erode(mk.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool), depth, K, Tbc)
        P = P[P[:, 2] > 0.0]
        if len(P) < min_points:
            continue
        L = P @ Tbi[:3, :3].T + Tbi[:3, 3]
        qs = [quadrant_of(p[:2]) for p in L]
        for q in QUADRANTS:
            n = sum(1 for v in qs if v == q)
            if n >= share * len(qs):
                out[q] += n
    return out


DEPTH_BAND = (-0.05, 0.05)  # (10/29) basket frame z around the rim: a standing plate / a bowl fills it, the tines hardly
DEPTH_MIN = 120             # points (stride 2) in a quadrant's band = taken (10/22 red plate Q3 263, empty Q3 0, Q4 tines 56)


def depth_occupancy(depth, K, Tbc, Tb, posj=None):
    """(보스 10/29: a plate put in the basket is an obstacle for the next one, seen in real time) basket quadrants taken,
    from depth alone (YOLOE gives a plate standing in the rack 0.10-0.16): points in the rim band per quadrant.  -> {q:
    dict(n, kind 'plate' (thin along the pin row) / 'bowl', pin (plate: nearest gap of the measured basket), pts (base))}."""
    from basket_vision import arm_points, cloud
    import plate_plans as pp
    P, _ = cloud(np.ones(depth.shape, bool), depth, K, Tbc, stride=2)
    if posj is not None and len(P):
        from scipy.spatial import cKDTree
        d, _ = cKDTree(arm_points(posj)).query(P)
        P = P[d > 0.04]
    Ti = np.linalg.inv(Tb)
    L = P @ Ti[:3, :3].T + Ti[:3, 3]
    sel = (np.abs(L[:, 0]) < HALF[0] - 0.015) & (np.abs(L[:, 1]) < HALF[1] - 0.015) & (L[:, 2] > DEPTH_BAND[0]) & (L[:, 2] < DEPTH_BAND[1])
    P, L = P[sel], L[sel]
    qs = np.array([quadrant_of(l[:2]) or '' for l in L])
    rq = {m: r for r, m in pp.robot_quadrants(Tb).items()}       # basket model quadrant -> robot quadrant name
    out = {}
    for q in QUADRANTS:
        m = qs == q
        if m.sum() < DEPTH_MIN:
            continue
        x, y = L[m, 0], L[m, 1]
        sx, sy = (float(np.percentile(v, 90) - np.percentile(v, 10)) for v in (x, y))
        # a standing plate is long across the pin row, short along it (10/22 Q3: 5 x 13 cm); a bowl the other way
        # round from the camera (Q1 / Q2: 15 x 7 cm)
        info = dict(n=int(m.sum()), kind='plate' if sy > 1.5 * sx else 'bowl', pts=P[m])
        if info['kind'] == 'plate':
            gaps = pp.basket_gaps()[q]
            n_ = len(gaps)
            xm = float(np.median(x))
            info['pin'] = min(range(1, n_ + 1), key=lambda pin: abs(gaps[pp.pin_gap(Tb, q, pin)][0] - xm))
        out[rq.get(q, q)] = info
    return out


def bowl_boxes(dets, depth, K, Tbc, posj=None):
    """Bowls -> oriented boxes (name, centre, size, R, conf, n) in base, like basket_vision.obstacle_boxes."""
    from types import SimpleNamespace
    from basket_vision import obstacle_boxes
    det = SimpleNamespace(plate_masks=[(m, c) for n, c, _, m in dets if n == 'bowl' and m is not None])
    return [(f'bowl_{i}',) + tuple(b[1:]) for i, b in enumerate(obstacle_boxes(det, None, depth, K, Tbc, posj, min_conf=0.2, min_points=60))]


def choose(counts, allowed=('Q3', 'Q4'), min_points=150):
    """The empty quadrants in `allowed` order -> the first one, or None."""
    empty = [q for q in allowed if counts.get(q, 0) < min_points]
    return (empty[0] if empty else None), empty


def occupied(counts, plates, min_points=150):
    """(10/26 보스: Q3에 물체가 있는데 empty) quadrant -> 'bowl' / 'plate' (already inserted) / None = empty."""
    return {q: 'bowl' if counts.get(q, 0) >= min_points else 'plate' if plates.get(q, 0) >= min_points else None
            for q in QUADRANTS}


def draw(rgb, dets, K, Tbc, Tb, counts, chosen, note='', show_conf=False, plates=None):
    img = rgb.copy()
    occ = occupied(counts, plates or {})
    for name, c, (x0, y0, x1, y1), _ in dets:
        col = COLOURS.get(name, (255, 60, 200))
        cv2.rectangle(img, (x0, y0), (x1, y1), col, 2)
        label = f'{name} {c:.2f}' if show_conf else ('basket' if name == 'dish drying rack' else name)   # (10/26 보스) spectators:
        cv2.putText(img, label, (x0, max(14, y0 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)              # no jittering numbers
    if Tb is not None:
        Tcb = np.linalg.inv(Tbc) @ Tb
        for q in QUADRANTS:
            P = quadrant_corners(q) @ Tcb[:3, :3].T + Tcb[:3, 3]
            uv = (P @ K.T)[:, :2] / P[:, 2:3]
            col = (255, 220, 0) if q == chosen else (230, 40, 40) if occ[q] else (40, 210, 60)
            cv2.polylines(img, [uv.astype(np.int32)], True, col, 4 if q == chosen else 2)
            for i, s in enumerate((q, occ[q] or 'empty')):
                (w, h), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                u, v = uv.mean(0).astype(int)
                cv2.putText(img, s, (u - w // 2, v - 4 + i * (h + 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
    lines = ([f'empty: {" ".join(q for q in QUADRANTS if not occ[q]) or "-"}', f'target: {chosen or "-"}']
             if Tb is not None or note else []) + ([note] if note else [])
    for i, s in enumerate(lines):
        cv2.putText(img, s, (10, 26 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
    return img


def _box_iou(a, b):
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    aa, bb = max(1, (a[2] - a[0]) * (a[3] - a[1])), max(1, (b[2] - b[0]) * (b[3] - b[1]))
    return inter / (aa + bb - inter)


def audience_objects(dets, overlap=0.45):
    """One spectator overlay per object.  The zoomed bowl pass wins over the whole-frame plate duplicate."""
    candidates = sorted((d for d in dets if d[0] in ('bowl', 'plate')),
                        key=lambda d: (d[0] == 'bowl', d[1]), reverse=True)
    kept = []
    for d in candidates:
        if not any(_box_iou(d[2], old[2]) >= overlap for old in kept):
            kept.append(d)
    return kept


def _basket_image_polygon(K, Tbc, Tb):
    if K is None or Tbc is None or Tb is None:
        return None
    local = np.array([[-HALF[0], -HALF[1], 0], [HALF[0], -HALF[1], 0],
                      [HALF[0], HALF[1], 0], [-HALF[0], HALF[1], 0]], float)
    Tcb = np.linalg.inv(Tbc) @ Tb
    P = local @ Tcb[:3, :3].T + Tcb[:3, 3]
    return ((P @ K.T)[:, :2] / P[:, 2:3]).astype(np.int32)


def _in_basket(det, polygon, shape):
    if polygon is None:
        return False
    _, _, (x0, y0, x1, y1), mask = det
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    if cv2.pointPolygonTest(polygon, (cx, cy), False) >= 0:
        return True
    if mask is None or mask.shape != shape:
        return False
    basket = np.zeros(shape, np.uint8)
    cv2.fillConvexPoly(basket, polygon, 1)
    return np.count_nonzero(mask & (basket > 0)) >= 0.2 * max(1, np.count_nonzero(mask))


def draw_audience(rgb, dets, K, Tbc, Tb, chosen=None):
    """Clean audience view: the pick plate is `target`; any detected object in the basket is `obstacle`."""
    img = rgb.copy()
    polygon = _basket_image_polygon(K, Tbc, Tb)
    for det in audience_objects(dets):
        name, _, (x0, y0, x1, y1), mask = det
        role = 'obstacle' if _in_basket(det, polygon, img.shape[:2]) else f'target: {name}'   # (보스 10/29) bowls on the table too
        col = np.asarray((245, 80, 70) if role == 'obstacle' else (40, 210, 255), np.uint8)
        if mask is not None and mask.shape == img.shape[:2]:
            img[mask] = (0.72 * img[mask] + 0.28 * col).astype(np.uint8)
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, contours, -1, tuple(int(v) for v in col), 2)
        else:
            cv2.rectangle(img, (x0, y0), (x1, y1), tuple(int(v) for v in col), 2)
        cv2.putText(img, role, (x0, max(18, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    tuple(int(v) for v in col), 2)
    return img


def draw_depth_obstacles(img, occ, K, Tbc):
    """Plates standing in the basket (depth_occupancy; YOLOE misses them) drawn as 'obstacle', as the bowls are."""
    Tcb = np.linalg.inv(Tbc)
    col = (245, 80, 70)
    for q, info in occ.items():
        if info['kind'] != 'plate':
            continue
        C = info['pts'] @ Tcb[:3, :3].T + Tcb[:3, 3]
        uv = ((C @ K.T)[:, :2] / C[:, 2:3]).astype(np.int32)
        hull = cv2.convexHull(uv)
        over = img.copy()
        cv2.fillConvexPoly(over, hull, col)
        img = cv2.addWeighted(over, 0.28, img, 0.72, 0)
        cv2.polylines(img, [hull], True, col, 2, cv2.LINE_AA)
        x0, y0 = hull[:, 0, 0].min(), hull[:, 0, 1].min()
        cv2.putText(img, 'obstacle', (int(x0), max(18, int(y0) - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    return img


def load_basket(path):
    try:
        r = json.loads(Path(path).read_text())
        return np.asarray(r['T'], float) if r.get('ok') else None
    except (OSError, ValueError, KeyError, TypeError):
        return None


def write_state(path, state):
    tmp = Path(str(path) + '.tmp')
    tmp.write_text(json.dumps(state))
    os.replace(tmp, path)


def write_rgbd_share(path, frame, timestamp=None):
    """Atomically publish one fixed-camera RGB-D frame for the other perception tools."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    timestamp = time.time() if timestamp is None else float(timestamp)
    tmp = path.with_name(path.stem + '.tmp.npz')
    np.savez(tmp, rgb=frame.rgb, depth=frame.depth, K=frame.K, t=timestamp)
    os.replace(tmp, path)


def detect_live(a):
    """(10/26 보스: YOLOE만 띄워서 검출 확인) boxes + masks only, camera opened here (nothing else may hold it) at its
    own rate, or the perception share.  Prompts / conf from the command line; fps and YOLOE ms in the terminal."""
    import sys
    det = Detector(a.prompts, zoom=a.zoom, imgsz=a.imgsz, half=a.half)
    Tbc = np.asarray(json.loads(Path(a.T_base_cam).read_text())['T_base_cam'], float) if a.T_base_cam else None
    Tb = load_basket(a.basket) if a.basket else None
    cam = None
    if a.camera:
        sys.path.insert(0, str(ROOT / 'real_time_project'))
        from real_time_utils import D455Source
        serial = a.camera
        if serial == 'fixed':
            sys.path.insert(0, str(ROOT / 'calibration'))
            import plate_plans as pp
            serial = str(pp.C['cameras']['fixed_serial'])
        cam = D455Source(serial=serial)
    win, last, t_prev, n_frames = 'YOLOE detect', 0.0, time.perf_counter(), 0
    occ_cache, depth = {}, None
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 960)
    try:
        while True:
            if cam is not None:
                fr = cam.read()
                rgb, K, depth = fr.rgb, np.asarray(fr.K, float), fr.depth
                if a.write_share:
                    write_rgbd_share(a.share, fr)
            else:
                try:
                    z = np.load(a.share)
                    if float(z['t']) <= last:
                        if (cv2.waitKey(20) & 0xff) in (ord('q'), 27):
                            break
                        continue
                    last, rgb = float(z['t']), z['rgb']
                    K = np.asarray(z['K'], float) if 'K' in z else None
                    depth = z['depth'] if 'depth' in z else None
                except (OSError, ValueError, KeyError):
                    time.sleep(0.05)
                    continue
            t0 = time.perf_counter()
            if Tb is None and a.basket and n_frames % 15 == 0:     # (10/29) the basket file may be written after the start
                Tb = load_basket(a.basket)
            dets = det(rgb, a.conf)
            ms = (time.perf_counter() - t0) * 1000
            if a.audience:
                img = draw_audience(rgb, dets, K, Tbc, Tb)
                if Tb is not None and Tbc is not None and K is not None and depth is not None:
                    if n_frames % 3 == 0:                     # (보스 10/29) a plate put in the basket = obstacle, live
                        occ_cache = depth_occupancy(depth, K, Tbc, Tb)
                    img = draw_depth_obstacles(img, occ_cache, K, Tbc)
            else:
                img = rgb.copy()
                for name, c, _, mk in dets:                    # masks lightly filled so a missed bowl is obvious
                    if mk is not None:
                        col = np.array(COLOURS.get(name, (255, 60, 200)), np.uint8)
                        img[mk] = (0.6 * img[mk] + 0.4 * col).astype(np.uint8)
                img = draw(img, dets, None, None, None, {}, None, show_conf=True)
            n_frames += 1                                      # (10/26 보스) no changing numbers on the image: terminal, every 5 s
            if time.perf_counter() - t_prev > 5.0:
                print(f'{n_frames / (time.perf_counter() - t_prev):.1f} fps, YOLOE {ms:.0f} ms, conf >= {a.conf}: '
                      + ', '.join(f'{n} {sum(d[0] == n for d in dets)}'
                                  for n in det.output_names + (['bowl'] if det.bowl else [])), flush=True)
                t_prev, n_frames = time.perf_counter(), 0
            cv2.imshow(win, img[:, :, ::-1])
            if (cv2.waitKey(1) & 0xff) in (ord('q'), 27):
                break
    finally:
        if cam is not None:
            cam.close()
        cv2.destroyAllWindows()


def main():
    a = argparse.ArgumentParser()
    a.add_argument('--share', default='/dev/shm/fixed_latest.npz')
    a.add_argument('--frame', help='offline: one saved frame (npz rgb/depth/K) instead of the share')
    a.add_argument('--T_base_cam', help='required unless --detect_only')
    a.add_argument('--basket', help="basket_vision.json (T); quadrants drawn once it exists")
    a.add_argument('--posj', type=float, nargs=6, help='joints deg (the arm is cut out of the bowl boxes)')
    a.add_argument('--state', default='/dev/shm/demo_quadrants.json')
    a.add_argument('--allowed', nargs='+', default=['Q3', 'Q4'])
    a.add_argument('--chosen_file', help='the pipeline writes the quadrant it took here (drawn yellow)')
    a.add_argument('--until_pid', type=int, help='exit when this process ends (the pipeline)')
    a.add_argument('--no_window', action='store_true')
    a.add_argument('--show_full', nargs='+', default=None, help='(10/26) draw these quadrants as holding a bowl regardless')
    a.add_argument('--show_plate', nargs='+', default=None, help='(10/26) quadrants the pipeline already put a plate in')
    a.add_argument('--hold', type=int, default=3, help='a bowl / plate seen in one of the last HOLD frames still counts (no flicker)')
    a.add_argument('--png', help='offline: save the overlay here')
    a.add_argument('--detect_only', action='store_true', help='(10/26) YOLOE boxes only (no basket, quadrants, state); q quits')
    a.add_argument('--camera', help="--detect_only: open this D455 here ('fixed' = the config's fixed camera) instead of the share")
    a.add_argument('--write_share', action='store_true',
                   help='--detect_only --camera: atomically publish every camera frame to --share '
                        '(rgb/depth metres/K/t) for the other perception tools')
    a.add_argument('--prompts', nargs='+', default=PROMPTS, help='--detect_only: YOLOE prompts')
    a.add_argument('--conf', type=float, default=0.15, help='--detect_only: YOLOE confidence threshold')
    a.add_argument('--imgsz', type=int, default=None, help='YOLOE inference size (audience: 512, otherwise: 640)')
    a.add_argument('--half', action=argparse.BooleanOptionalAction, default=None,
                   help='YOLOE FP16 inference on CUDA (audience: on, otherwise: off)')
    a.add_argument('--zoom', type=float, default=2.0, help='bowl crop scale')
    a.add_argument('--audience', action='store_true', help='clean spectator overlay: object masks/labels and thin quadrant guides; hides basket detection')
    a = a.parse_args()
    a.imgsz = a.imgsz or (512 if a.audience else 640)
    a.half = a.audience if a.half is None else a.half
    if a.detect_only:
        return detect_live(a)
    Tbc = np.asarray(json.loads(Path(a.T_base_cam).read_text())['T_base_cam'], float)
    det, n, recent = Detector(zoom=a.zoom, imgsz=a.imgsz, half=a.half), 0, []
    last = 0.0 if a.frame else time.time() - 1.0     # (10/26 보스) not the previous run's frame left in the share
    win = 'demo: fixed camera'
    if not a.no_window and not a.png:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1280, 960)
    while True:
        if a.until_pid and not Path(f'/proc/{a.until_pid}').exists():
            break
        try:                                            # (10/26) the whole frame read here: one bad read (Bad CRC on
            with np.load(a.frame or a.share) as z:      # 'depth.npy', the view died) skips that frame
                t = float(z['t']) if 't' in z else 0.0
                if not a.frame and t <= last:
                    z = None
                else:
                    rgb, depth, K = z['rgb'], z['depth'], z['K'].astype(float)
        except (OSError, ValueError, KeyError, EOFError, zipfile.BadZipFile) as exc:
            if not isinstance(exc, FileNotFoundError):
                print(f'shared frame skipped: {exc!r}', flush=True)
            time.sleep(0.05)
            continue
        if z is None:
            if not a.no_window and (cv2.waitKey(30) & 0xff) in (ord('q'), 27) and not a.until_pid:
                break
            continue
        last = t
        dets = det(rgb)
        Tb = load_basket(a.basket) if a.basket else None
        counts = bowl_quadrants(dets, depth, K, Tbc, Tb) if Tb is not None else {}
        plates = bowl_quadrants(dets, depth, K, Tbc, Tb, min_conf=0.15, name='plate') if Tb is not None else {}
        now = occupied(counts, plates)                  # this frame alone (the pipeline votes over frames)
        recent = (recent + [(counts, plates)])[-max(1, a.hold):]       # the largest of the last frames: a bowl / plate
        counts = {q: max(c.get(q, 0) for c, _ in recent) for q in QUADRANTS}   # missed in one frame stays on the view
        plates = {q: max(p_.get(q, 0) for _, p_ in recent) for q in QUADRANTS}
        for q in a.show_full or []:                     # (10/26 보스 최후의 수) shown as full whatever is seen there
            counts[q] = max(counts.get(q, 0), 150)
        for q in a.show_plate or []:
            plates[q] = max(plates.get(q, 0), 150)
        occ = occupied(counts, plates)
        chosen_by_rule, empty = choose({q: 150 if occ[q] else 0 for q in QUADRANTS}, a.allowed) if Tb is not None else (None, [])
        taken = Path(a.chosen_file).read_text().strip().split(':')[0] if a.chosen_file and Path(a.chosen_file).exists() else None
        n += 1
        write_state(a.state, dict(t=t, frame=n, basket=None if Tb is None else Tb.tolist(), counts=counts, plates=plates, empty=empty,
                                  empty_now=[q for q in a.allowed if not now[q] and q not in (a.show_full or []) + (a.show_plate or [])],
                                  chosen=chosen_by_rule, bowls=bowl_boxes(dets, depth, K, Tbc, a.posj),
                                  dets=[(nm, round(c, 3), b) for nm, c, b, _ in dets]))
        if a.audience:
            img = draw_audience(rgb, dets, K, Tbc, Tb, taken or chosen_by_rule)
        else:
            img = draw(rgb, dets, K, Tbc, Tb, counts, taken or chosen_by_rule,
                       '' if Tb is not None else 'basket: measuring ...', plates=plates)
        if a.png:
            cv2.imwrite(a.png, img[:, :, ::-1])
            print(json.dumps(dict(counts=counts, empty=empty, chosen=chosen_by_rule, dets=len(dets))), flush=True)
            break
        if not a.no_window:
            cv2.imshow(win, img[:, :, ::-1])
            if (cv2.waitKey(1) & 0xff) in (ord('q'), 27) and not a.until_pid:   # the pipeline's view stays to the end
                break
        if a.frame:
            break
    if not a.no_window and not a.png:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
