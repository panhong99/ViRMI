"""(10/20 보스: 바구니에 아루코가 없다고 가정) basket pose from the fixed D455 RGB-D alone, no markers.

YOLOE prompt mask ('dish drying rack', plates cut out) + depth -> base-frame points, the arm (FK at the capture joints)
cut out -> top-view correlation of the basket model's upper structure over every yaw -> 3-DOF ICP (x, y, yaw; z =
config basket.fixed_z_m) -> the blue handle (basket -y) picks between the fit and its 180-degree turn.  A one-time offset
maps the rack structure to the slot frame.  Checked offline against the marker pose of 56 real frames
(experiments/basket_free/fit.py): 10/20 test 23/24 accepted, xy median 3.4 mm, max 7.3 mm, yaw max 2.1 deg.

    python calibration/basket_vision.py --share /dev/shm/fixed_latest.npz --T_base_cam X.json --posj j1 .. j6 --out res.json
        n fresh frames from the perception process's share -> the median pose, refused if the frames disagree.
Opt-in (config basket.source: vision); the marker path is unchanged."""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
PROMPTS = ['dish drying rack', 'plate']             # rack mask; plates standing in it are cut out
CUT = []                                            # (10/23 11/3 시연, opt-in --cut bowl) more prompts cut out of the rack mask
MAJORITY = [0]                                      # (10/29, opt-in --majority K) keep the >= K frames of one 180-degree turn
MIN_HANDLE, MIN_POINTS = 20, 6000                   # below: refuse (look again with the arm out of the view)
# One-time offset of the rack structure (rl/assets/basket_full.obj walls/rails/handle) from the frame the slots are
# defined in: median of 25 marker frames on 10/19 17:00-19:30 (experiments/basket_free model_v5): fit = marker + (-3.0, -24.4) mm.
STRUCT_TO_FRAME = np.eye(4)
STRUCT_TO_FRAME[:2, 3] = [0.0030, 0.0244]


class Detector:
    def __init__(self):
        from ultralytics import YOLOE
        self.m = YOLOE(str(ROOT / 'yoloe-v8l-seg.pt'))
        self.m.set_classes(PROMPTS + CUT, self.m.get_text_pe(PROMPTS + CUT))

    def masks(self, rgb, conf=0.05):             # (10/21: basket turned -12 deg at x 0.85 -> rack only 0.10; was 0.1)
        """(rack mask or None, plate mask union, rack confidence); self.plate_masks = [(mask, conf), ...] each plate."""
        r = self.m.predict(rgb[:, :, ::-1].copy(), conf=conf, verbose=False, retina_masks=True)[0]
        h, w = rgb.shape[:2]
        rack, best, plates = None, 0.0, np.zeros((h, w), bool)
        self.plate_masks = []
        if r.masks is None:
            return rack, plates, best
        for mk, c, k in zip(r.masks.data.cpu().numpy() > 0.5, r.boxes.conf.cpu().numpy(), r.boxes.cls.cpu().numpy()):
            if int(k) == 0 and c > best:
                rack, best = mk, float(c)
            elif int(k) == 1:
                plates |= mk
                self.plate_masks.append((mk, float(c)))
            elif int(k) >= 2:                                   # (--cut) bowls etc.: not the rack, not an obstacle here
                plates |= mk
        return rack, plates, best


def obstacle_boxes(det, rack, depth, K, Tbc, posj=None, min_conf=0.25, min_points=150, pad=0.01):
    """(10/22 보스 장애물 데모) plates seen OUTSIDE the basket (YOLOE 'plate' masks, < 30 % in the rack mask) -> oriented
    boxes (name, centre, size, R) in base: depth points (table and arm cut off), PCA axes, 2-98 % extents + pad,
    >= 3 cm thick.  The plate to pick is one of them: the pipeline drops the one at the grasp target."""
    from scipy.spatial import cKDTree
    out = []
    for i, (mk, c) in enumerate(getattr(det, 'plate_masks', [])):
        if c < min_conf or (rack is not None and (mk & rack).sum() > 0.3 * mk.sum()):
            continue
        P, _ = cloud(cv2.erode(mk.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool), depth, K, Tbc)
        P = P[P[:, 2] > 0.0]                                        # table top ~ -0.02
        if posj is not None and len(P):
            d, _ = cKDTree(arm_points(posj)).query(P)
            P = P[d > 0.04]
        if len(P) < min_points:
            continue
        m = np.median(P, axis=0)
        P = P[np.linalg.norm(P - m, axis=1) < 0.25]                 # depth bleeding off the rim
        if len(P) < min_points:
            continue
        mu = P.mean(0)
        _, _, Vt = np.linalg.svd(P - mu, full_matrices=False)
        R = Vt.T * np.sign(np.linalg.det(Vt.T))
        L = (P - mu) @ R
        lo, hi = np.percentile(L, 2, axis=0), np.percentile(L, 98, axis=0)
        size = np.maximum(hi - lo + 2 * pad, [0.03, 0.03, 0.03])
        centre = mu + R @ ((lo + hi) / 2)
        out.append((f'obstacle_plate_{i}', np.round(centre, 4).tolist(), np.round(size, 4).tolist(), np.round(R, 5).tolist(),
                    round(c, 3), len(P)))
    return out


def cloud(mask, depth, K, Tbc, stride=1):
    v, u = np.nonzero(mask[::stride, ::stride])
    v, u = v * stride, u * stride
    d = depth[v, u].astype(float)
    ok = (d > 0.2) & (d < 2.5)
    u, v, d = u[ok], v[ok], d[ok]
    P = np.c_[(u - K[0, 2]) / K[0, 0] * d, (v - K[1, 2]) / K[1, 1] * d, d]
    return P @ Tbc[:3, :3].T + Tbc[:3, 3], np.c_[u, v]


def blue(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    # The handle is often only a few pixels wide and its saturation drops under
    # glare/exposure changes.  Keep this cue conservative in hue but less brittle
    # in saturation/value; the expanded YOLOE ROI below limits false positives.
    return (hsv[..., 0] > 88) & (hsv[..., 0] < 135) & (hsv[..., 1] > 70) & (hsv[..., 2] > 45)


PLATE_COLOURS = ''                                  # (10/22 보스: 꽂힌 접시) 'red' / 'red_white': drop those pixels from the rack fit
                                                    # (10/22 real 63 frames: red_white 58 -> 33 accepted, the lit rails read white)


MIN_COVER = None                                    # (10/22, opt-in 0.68) refuse when the rack mask covers less of the fitted
                                                    # basket's image footprint: a person / the arm over it (real 63 frames:
                                                    # the 3 worst accepted fits, 21-26 mm, had 0.54-0.67; good ones >= 0.67)


def rack_cover(rack, T, K, Tbc, half=(0.265, 0.26), depth_m=0.14):
    """Share of the fitted basket's image footprint (rim and floor corners, convex hull) inside the YOLOE rack mask."""
    C = np.array([[sx * half[0], sy * half[1], z, 1.0] for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, -depth_m)])
    p = (np.linalg.inv(Tbc) @ np.asarray(T) @ np.linalg.inv(STRUCT_TO_FRAME) @ C.T)[:3]
    if (p[2] <= 0.05).any():
        return 0.0
    uv = (K @ (p / p[2]))[:2].T.astype(np.int32)
    poly = np.zeros(rack.shape, np.uint8)
    cv2.fillConvexPoly(poly, cv2.convexHull(uv), 1)
    poly = poly.astype(bool)
    return float((rack & poly).sum() / max(poly.sum(), 1))


REFIT_MM = None                                     # (10/22) after the fit: drop points farther than this from the model, refit


def refit(P, M, yaw, t, z_rim, mm, trim=0.9):
    """(10/22) plates standing in the rack sit between the tines, ~1 cm off any model point: keep the points within
    `mm` of the fitted model and fit again (they pulled the pose ~1 cm and made the frames disagree)."""
    from scipy.spatial import cKDTree
    c, s = np.cos(yaw), np.sin(yaw)
    loc = np.c_[(P[:, :2] - t) @ np.array([[c, -s], [s, c]]), P[:, 2] - z_rim]
    d, _ = cKDTree(M).query(loc)
    keep = d <= mm / 1000
    if keep.sum() < 0.3 * len(P):
        return yaw, t, None
    return icp_xy_yaw(P[keep], M, yaw, t, z_rim, trim=trim)


def plate_colour(rgb, white=True, blob_px=11):
    """(10/22) plates standing in the rack (YOLOE keeps them in the rack mask): red or white pixels; the rack wires
    are dark grey, the blue handle is kept."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s_, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = ((h < 10) | (h > 165)) & (s_ > 80) & (v > 50)
    wh = (s_ < 45) & (v > 150) if white else np.zeros_like(red)
    if white == 'blob':                             # only large white areas (a plate face), not the lit rails
        wh = cv2.morphologyEx(wh.astype(np.uint8), cv2.MORPH_OPEN, np.ones((blob_px, blob_px), np.uint8)).astype(bool)
    return cv2.dilate((red | wh).astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)


def handle_roi(rack):
    """Return an image ROI around the YOLOE rack extent, including an external handle.

    The basket handle can lie completely outside the segmentation mask.  Search
    a scale-aware margin around the detected rack instead of intersecting the
    colour cue with the rack mask itself.
    """
    ys, xs = np.nonzero(rack)
    if len(xs) == 0:
        return np.zeros_like(rack, dtype=bool)
    h, w = rack.shape
    pad_x = max(24, int((xs.max() - xs.min() + 1) * 0.20))
    pad_y = max(24, int((ys.max() - ys.min() + 1) * 0.20))
    x0, x1 = max(0, int(xs.min()) - pad_x), min(w, int(xs.max()) + pad_x + 1)
    y0, y1 = max(0, int(ys.min()) - pad_y), min(h, int(ys.max()) + pad_y + 1)
    roi = np.zeros_like(rack, dtype=bool)
    roi[y0:y1, x0:x1] = True
    return roi


def model_points(n=20000, z_min=-0.13):
    """Basket model surface (rl/assets/basket_full.obj, basket frame), floor grid left out (it touches the table)."""
    import trimesh
    m = trimesh.load(str(ROOT / 'rl/assets/basket_full.obj'), force='mesh')
    p, _ = trimesh.sample.sample_surface(m, n, seed=0)
    return p[p[:, 2] > z_min]


def arm_points(posj_deg):
    """Robot links + the gripper cylinder in base at the capture joints (scene points near them are the arm)."""
    import sys
    sys.path.insert(0, str(ROOT / 'calibration'))
    from robot_model import CollisionChecker, Kinematics
    global _CH
    if '_CH' not in globals():
        _CH = CollisionChecker(Kinematics(), [], bounds=((0, 0.1), (0, 0.1), (0, 0.1)))
        _CH.set_tool(0.165, 0.06)
    return _CH.robot_points(np.radians(posj_deg))


def model_tree(M):
    """KD-tree of the basket model, cached per model array."""
    from scipy.spatial import cKDTree
    global _TREE
    if '_TREE' not in globals() or _TREE[0] is not M:
        _TREE = (M, cKDTree(M))
    return _TREE[1]


def icp_xy_yaw(S, M, yaw0, t0, z_rim, iters=30, trim=0.7):
    """Scene points S (base) onto the model M (basket frame) in x, y, yaw at a fixed height: trimmed point-to-point."""
    tree = model_tree(M)                                          # (10/23) built once: was 30x per call, 6 s a frame
    yaw, t = yaw0, np.asarray(t0, float)
    for _ in range(iters):
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        loc = np.c_[(S[:, :2] - t) @ R, S[:, 2] - z_rim]          # scene in the basket frame
        d, j = tree.query(loc, workers=-1)
        keep = d <= np.quantile(d, trim)
        a, b = loc[keep, :2], M[j[keep], :2]                       # loc -> model: small 2D rigid update
        ca, cb = a.mean(0), b.mean(0)
        H = (a - ca).T @ (b - cb)
        dth = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        # model = Rd (loc - ca) + cb  ->  basket frame turns by -dth
        Rd = np.array([[np.cos(dth), -np.sin(dth)], [np.sin(dth), np.cos(dth)]])
        # new scene->basket map: Rd R^T (x - t) - Rd ca + cb  =  R'^T (x - t')
        Rn = R @ Rd.T
        tn = t + R @ (ca - Rd.T @ cb)
        step = np.linalg.norm(tn - t)
        yaw, t = np.arctan2(Rn[1, 0], Rn[0, 0]), tn
        if abs(dth) < 1e-6 and step < 1e-6:                       # (point-to-point creeps along wires: many small steps)
            break
    c, s = np.cos(yaw), np.sin(yaw)                               # (a Powell polish on the truncated distance was
    loc = np.c_[(S[:, :2] - t) @ np.array([[c, -s], [s, c]]), S[:, 2] - z_rim]   # tried: 10/20 xy median 3.4 -> 5.8 mm)
    d, _ = tree.query(loc, workers=-1)
    return yaw, t, float(np.mean(np.sort(d)[:int(trim * len(d))]))


def raster(xy, lo, n, cell):
    img = np.zeros((n, n))
    ij = np.floor((xy - lo) / cell).astype(int)
    ok = (ij >= 0).all(1) & (ij < n).all(1)
    np.add.at(img, (ij[ok, 0], ij[ok, 1]), 1.0)
    return img


def model_rasters(M, Mu, n, half, cell, step_deg):
    """(10/23) the blurred, flipped top-view raster of the model at every yaw: the same each frame and run, so cached."""
    from scipy.ndimage import gaussian_filter
    global _RAST
    key = (id(M), len(Mu), n, half, cell, step_deg)          # Mu: the band of M (a new array per call)
    if '_RAST' not in globals() or _RAST[0] != key:
        yaws, out = np.radians(np.arange(0.0, 360.0, step_deg)), []
        for a in yaws:
            R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
            Mi = gaussian_filter(np.minimum(raster(Mu[:, :2] @ R.T, np.array([-half, -half]), n, cell), 3.0), 1.5)
            out.append((Mi / (np.linalg.norm(Mi) + 1e-9))[::-1, ::-1])
        _RAST = (key, yaws, np.array(out))
    return _RAST[1], _RAST[2]


def global_search(P, M, z_rim, cell=0.005, half=0.5, step_deg=1.0, keep=6, band=(-0.07, 0.03), chunk=40):
    """Top-view correlation of the upper structure (rails, handle, wall tops, tine tops) over every yaw and shift:
    -> the `keep` best (yaw, centre) seeds for the ICP.  Rewards overlap both ways, so a turned fit that leaves the
    model's rails or tine rows without scene points loses (the ICP alone sits in +-20 deg wire-pitch minima).
    (10/23) = fftconvolve(scene, model[::-1, ::-1], 'same') per yaw, batched over yaws on all cores (6 s -> <1 s)."""
    from scipy.ndimage import gaussian_filter
    from scipy import fft
    S = P[(P[:, 2] - z_rim > band[0]) & (P[:, 2] - z_rim < band[1])]
    Mu = M[(M[:, 2] > band[0]) & (M[:, 2] < band[1])]
    ctr = np.median(S[:, :2], axis=0)
    n = int(2 * half / cell)
    lo = ctr - half
    Si = gaussian_filter(np.minimum(raster(S[:, :2], lo, n, cell), 3.0), 1.5)
    Si -= Si.mean()
    yaws, Mr = model_rasters(M, Mu, n, half, cell, step_deg)
    shape = [fft.next_fast_len(2 * n - 1, True)] * 2
    Sf, a0 = fft.rfft2(Si, shape, workers=-1), (n - 1) // 2
    out = []
    for i in range(0, len(yaws), chunk):
        c = fft.irfft2(Sf * fft.rfft2(Mr[i:i + chunk], shape, workers=-1), shape, workers=-1)[:, a0:a0 + n, a0:a0 + n]
        for a, ci in zip(yaws[i:i + chunk], c):
            k = np.unravel_index(np.argmax(ci), ci.shape)
            out.append((ci[k], a, ctr + (np.array(k) - n // 2) * cell))
    out.sort(key=lambda r: -r[0])
    seeds = []
    for sc, a, t in out:                                           # distinct seeds (yaw > 5 deg apart)
        if all(abs((a - b + np.pi) % (2 * np.pi) - np.pi) > np.radians(5) for b, _ in seeds):
            seeds.append((a, t))
        if len(seeds) >= keep:
            break
    return [(a, t) for a, t in seeds]


def estimate(det, rgb, depth, K, Tbc, z_rim, posj=None, M=None):
    """Whole-model fit: YOLOE rack mask - plates - arm -> points -> top-view correlation seeds -> ICP; the handle picks
    the 180-degree turn.  -> dict(T basket frame in base, ..., refused = '' or why) or None (no rack found)."""
    from scipy.spatial import cKDTree
    rack, plates, conf = det.masks(rgb)
    if rack is None:
        return None
    m = cv2.dilate(rack.astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool) & ~plates
    if PLATE_COLOURS:
        m &= ~plate_colour(rgb, white={'red': False, 'red_white': True, 'red_white_blob': 'blob'}[PLATE_COLOURS])
    P, _ = cloud(m, depth, K, Tbc)
    P = P[(P[:, 2] > 0.015) & (P[:, 2] < z_rim + 0.03)]
    if posj is not None:
        d, _ = cKDTree(arm_points(posj)).query(P)
        P = P[d > 0.04]
    if len(P) < 100:
        return None
    seeds = global_search(P, M, z_rim)
    fits = sorted((icp_xy_yaw(P, M, y0, t0, z_rim) for y0, t0 in seeds), key=lambda r: r[2])
    yaw, t, cost = fits[0]
    # Do not require the cue to be inside the segmentation mask: the handle is
    # an external part of the basket and is commonly outside YOLOE's rack mask.
    handle_pixels = handle_roi(rack) & blue(rgb) & ~plates
    Hb, _ = cloud(handle_pixels, depth, K, Tbc)                    # the blue handle is basket -y: pick the turn of the
    Hb = Hb[(Hb[:, 2] > 0.03) & (Hb[:, 2] < z_rim + 0.03)]         # good fits (cost within 30 %) that puts it there
    n_handle = len(Hb)
    if n_handle >= 20:
        def handle_y(r):
            c, s = np.cos(r[0]), np.sin(r[0])
            return np.median((Hb[:, :2] - r[1]) @ np.array([-s, c]))
        flip = icp_xy_yaw(P, M, yaw + np.pi, t, z_rim)             # the rim is nearly 180-deg symmetric: the handle decides
        yaw, t, cost = min((fits[0], flip), key=handle_y)
    if REFIT_MM:
        yaw, t, _ = refit(P, M, yaw, t, z_rim, REFIT_MM)
    T = np.eye(4)
    T[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    T[:2, 3], T[2, 3] = t, z_rim
    T = T @ STRUCT_TO_FRAME                                          # rack structure -> the slot (marker) frame
    cover = rack_cover(rack, T, K, Tbc)
    why = [w for w, bad in (('handle not seen', n_handle < MIN_HANDLE), ('few points', len(P) < MIN_POINTS),
                            (f'basket occluded (cover {cover:.2f})', MIN_COVER is not None and cover < MIN_COVER)) if bad]
    return dict(T=T, conf=conf, cost=cost, n_top=len(P), n_handle=n_handle, cover=cover, refused=', '.join(why),
                obstacles=obstacle_boxes(det, rack, depth, K, Tbc, posj))


def live(share, Tbc, posj, z_rim, n=5, gap_s=0.6, timeout_s=40.0, max_spread_mm=6.0, max_spread_deg=1.5, stale_s=5.0):
    """n fresh frames of the fixed camera share -> median pose; refused if any frame is refused or they disagree."""
    det, M = Detector(), model_points()
    t0, last, res = time.time(), 0.0, []                           # frames newer than the loaded models only
    while len(res) < n:
        if time.time() - t0 > timeout_s:
            raise RuntimeError(f'basket vision: {len(res)}/{n} frames in {timeout_s:.0f} s')
        try:                                                        # (10/26) whole frame read here (Bad CRC once)
            with np.load(share) as z_:
                z = {k: z_[k] for k in ('t', 'rgb', 'depth', 'K')}
            t = float(z['t'])
        except Exception:
            time.sleep(0.1)
            continue
        if time.time() - t > stale_s:                               # (10/26: FP died of CUDA OOM, the share froze)
            raise RuntimeError(f'fixed camera frames stopped {time.time() - t:.0f} s ago: the perception process '
                               'is not sharing (stopped? see perception.log)')
        if t <= last or t < t0:
            time.sleep(0.1)
            continue
        last = t
        e = estimate(det, z['rgb'], z['depth'], z['K'].astype(float), Tbc, z_rim, posj, M)
        res.append(e if e is not None else dict(refused='rack not found'))
        time.sleep(gap_s)
    return combine(res, z_rim, max_spread_mm, max_spread_deg)


def camera(serial, Tbc, posj, z_rim, n=5, gap_s=0.6, save=None, max_spread_mm=6.0, max_spread_deg=1.5):
    """(10/21 보스, look4) the same from the fixed D455 opened here (no perception process running): n frames ->
    median pose; the first frame is saved to `save` (npz rgb/depth/K/t, the format of the perception share)."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'real_time_project'))
    from real_time_utils import D455Source
    det, M = Detector(), model_points()
    cam = D455Source(serial=serial)
    res = []
    try:
        for _ in range(15):                                         # auto exposure settles
            cam.read()
        for i in range(n):
            fr = cam.read()
            if i == 0 and save:
                np.savez_compressed(save, rgb=fr.rgb, depth=fr.depth, K=fr.K, t=time.time())
            e = estimate(det, fr.rgb, fr.depth, np.asarray(fr.K, float), Tbc, z_rim, posj, M)
            res.append(e if e is not None else dict(refused='rack not found'))
            time.sleep(gap_s)
    finally:
        cam.pipeline.stop()
    return combine(res, z_rim, max_spread_mm, max_spread_deg)


def combine(res, z_rim, max_spread_mm, max_spread_deg):
    """Per-frame estimates -> median pose; refused if any frame is refused or they disagree."""
    why = sorted({r['refused'] for r in res if r.get('refused')})
    if why:
        return dict(ok=False, why='; '.join(why), frames=len(res))
    dropped = 0
    if MAJORITY[0]:                                  # (10/29 real: 1 of 5 frames picked the handle's 180-degree turn, the
        yaws = np.array([np.arctan2(r['T'][1, 0], r['T'][0, 0]) for r in res])   # other 4 agreed within 1 mm / 0.2 deg)
        same = [np.abs((yaws - y0 + np.pi) % (2 * np.pi) - np.pi) < np.pi / 2 for y0 in yaws]
        best = max(same, key=lambda m: m.sum())
        if best.sum() < MAJORITY[0]:
            return dict(ok=False, why=f'180-degree turn undecided: {int(best.sum())}/{len(res)} frames agree', frames=len(res))
        dropped, res = int(len(res) - best.sum()), [r for r, k in zip(res, best) if k]
    xy = np.array([r['T'][:2, 3] for r in res])
    yaw = np.array([np.arctan2(r['T'][1, 0], r['T'][0, 0]) for r in res])
    yaw = yaw[0] + (yaw - yaw[0] + np.pi) % (2 * np.pi) - np.pi
    mxy, myaw = np.median(xy, 0), float(np.median(yaw))
    spread_mm = float(np.abs(xy - mxy).max() * 1000)
    spread_deg = float(np.degrees(np.abs(yaw - myaw).max()))
    T = np.eye(4)
    T[:2, :2] = [[np.cos(myaw), -np.sin(myaw)], [np.sin(myaw), np.cos(myaw)]]
    T[:2, 3], T[2, 3] = mxy, z_rim
    ok = spread_mm <= max_spread_mm and spread_deg <= max_spread_deg
    return dict(ok=ok, why='' if ok else f'frames disagree: {spread_mm:.1f} mm, {spread_deg:.1f} deg', T=T.tolist(),
                spread_mm=spread_mm, spread_deg=spread_deg, frames=len(res), dropped_turned=dropped,
                conf=float(np.median([r['conf'] for r in res])), n_points=int(np.median([r['n_top'] for r in res])),
                obstacles=max((r.get('obstacles', []) for r in res), key=len))   # (10/22) plates outside the basket


def main():
    a = argparse.ArgumentParser()
    a.add_argument('--share', default='/dev/shm/fixed_latest.npz')
    a.add_argument('--T_base_cam', help='json with T_base_cam (required unless --camera fixed)')
    a.add_argument('--posj', type=float, nargs=6, help='joints deg at the capture (the arm is cut out)')
    a.add_argument('--z_rim', type=float)
    a.add_argument('--n', type=int, default=5)
    a.add_argument('--out', required=True)
    a.add_argument('--camera', help='fixed D455 serial, or "fixed" = config camera/calibration/home joints: open the camera here instead of --share (10/21 look4); "share" = those defaults with the --share frames')
    a.add_argument('--save', help='--camera: save the first frame here (npz)')
    a.add_argument('--cut', nargs='+', default=[], help="(10/23, opt-in) more YOLOE prompts cut out of the rack mask, e.g. bowl")
    a.add_argument('--majority', type=int, default=0, help='(10/29, opt-in) drop the frames that picked the other 180-degree '
                                                           'turn when >= this many agree (e.g. 4 of --n 5)')
    a = a.parse_args()
    CUT[:] = a.cut
    MAJORITY[0] = a.majority
    if a.camera in ('fixed', 'share'):              # (look4) the config's camera, calibration, home joints, rim height;
        import calib_core as cc                     # (10/29) 'share': the same, the frames from --share (a viewer owns
        import plate_plans as pp                    # the camera)
        a.camera, a.T_base_cam = (None if a.camera == 'share' else str(pp.C['cameras']['fixed_serial'])), a.T_base_cam or str(cc.latest_t_base_cam())
        a.posj = a.posj or list(pp.HOME_JOINTS_DEG)
        a.z_rim = a.z_rim if a.z_rim is not None else pp.C['basket']['fixed_z_m']
    Tbc = np.asarray(json.loads(Path(a.T_base_cam).read_text())['T_base_cam'], float)
    try:
        r = (camera(a.camera, Tbc, a.posj, a.z_rim, a.n, save=a.save) if a.camera else
             live(a.share, Tbc, a.posj, a.z_rim, a.n))
    except Exception as exc:
        r = dict(ok=False, why=f'{type(exc).__name__}: {exc}')
    Path(a.out).write_text(json.dumps(r))
    print(json.dumps({k: v for k, v in r.items() if k != 'T'}), flush=True)


if __name__ == '__main__':
    main()
