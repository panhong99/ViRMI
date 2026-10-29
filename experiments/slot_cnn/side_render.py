"""(보스 10/29 night) slot CNN data for the side-wall D455 (338122303684): renders of the basket from around its measured
pose (calibration/data/side_cam/T_base_side.json), cropped to ONE quadrant's two tine rows and resized to 1280 x 720 exactly
as the real frames are (crop_box()), labels = only that quadrant's tines (the other quadrants' tines in the crop are
negatives, as the clips, wheels and frame).  Sometimes a plate is held over / partly in a gap (the docking view).
    ~/miniconda3/envs/foundationpose/bin/python experiments/slot_cnn/side_render.py --n 8000 --out calibration/data/slot_cnn/sim_side_v1
Same labels.json format as data.py (network pixels), so train.py / eval.py take it as is."""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import data as D  # noqa: E402

ROOT = D.ROOT
SIDE = ROOT / 'calibration/data/side_cam/T_base_side.json'
BASKET = ROOT / 'tesollo/demo_safe/basket_fixed.json'


def side_pose():
    """T_basket_cam of the side camera (base pose / basket_fixed)."""
    Tbc = np.array(json.loads(SIDE.read_text())['T_base_cam'])
    Tb = np.array(json.loads(BASKET.read_text())['T'])
    return np.linalg.inv(Tb) @ Tbc


def crop_box(tines, q, T_cam_basket, K=D.K_WRIST, margin=0.35, aspect=16 / 9):
    """16:9 pixel box (x0, y0, w, h) around quadrant q's tine roots and tips, + margin of its size."""
    P = []
    for t in tines:
        if t['q'] == q:
            P += [t['base'], t['top']]
    uv, _ = D.project(np.array(P), T_cam_basket, K)
    (u0, v0), (u1, v1) = uv.min(0), uv.max(0)
    cu, cv_ = (u0 + u1) / 2, (v0 + v1) / 2
    w = max((u1 - u0) * (1 + margin), (v1 - v0) * (1 + margin) * aspect)
    h = w / aspect
    x0, y0 = int(round(cu - w / 2)), int(round(cv_ - h / 2))
    w, h = int(round(w)), int(round(h))
    x0, y0 = max(0, min(D.W - w, x0)), max(0, min(D.H - h, y0))
    return x0, y0, w, h


def held_plate(m, rng, ts, q, B):
    """A plate over / partly in a random gap of quadrant q, its face along the tines (the docking view)."""
    f = B['full']
    row = [t for t in ts if t['q'] == q and t['row'] == 0]
    xs = sorted(t['base'][0] for t in row)
    if len(xs) < 3:
        return
    i = rng.integers(0, len(xs) - 1)
    x = (xs[i] + xs[i + 1]) / 2 + rng.normal(0, 0.006)
    y = np.mean([t['base'][1] for t in ts if t['q'] == q])
    lean = np.radians(12.5 if q in ('Q3', 'Q4') else -12.5) + rng.normal(0, np.radians(8))
    n = np.array([np.cos(lean), 0, np.sin(lean)])
    low = f['floor_z'] + rng.uniform(0.02, 0.20)                 # lowest rim point: in the gap .. 15 cm over the rim
    R, th = 0.134, rng.uniform(0.008, 0.02)
    u = np.array([0, 0, 1.0]) - n[2] * n
    u /= np.linalg.norm(u)
    ctr = np.array([x, y + rng.normal(0, 0.01), low]) + R * u
    Rp = D._frame_z(n)
    colour = np.array([0.6, 0.05, 0.06]) * rng.uniform(0.7, 1.3) if rng.random() < 0.5 else np.full(3, rng.uniform(0.75, 0.98))
    v1, n1, f1 = D._unit_cyl(48)
    m.add((v1 * [R, R, th] - [0, 0, th / 2]) @ Rp.T + ctr, n1 @ Rp.T, f1, np.clip(colour, 0, 1))
    cap = np.r_[[[0, 0, 0]], np.c_[np.cos(np.linspace(0, 2 * np.pi, 48, endpoint=False)),
                                  np.sin(np.linspace(0, 2 * np.pi, 48, endpoint=False)), np.zeros(48)]]
    for zz in (-th / 2, th / 2):
        fc = np.array([[0, 1 + k, 1 + (k + 1) % 48] for k in range(48)])
        m.add((cap * [R, R, 1] + [0, 0, zz]) @ Rp.T + ctr, np.tile(Rp[:, 2] * np.sign(zz), (49, 1)), fc, np.clip(colour * 0.95, 0, 1))


def main():
    from scipy.spatial.transform import Rotation
    a = argparse.ArgumentParser()
    a.add_argument('--n', type=int, default=8000)
    a.add_argument('--out', required=True)
    a.add_argument('--seed', type=int, default=0)
    a.add_argument('--quadrants', nargs='+', default=['Q3', 'Q4'])
    a = a.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    parts, B = D.basket_parts(False)
    tines, rail = D.tine_table(parts, B), B['pins']['rail_z']
    T0 = side_pose()
    rd = D.Renderer()
    rng = np.random.default_rng(a.seed)
    meta = []
    for i in range(a.n):
        q = a.quadrants[i % len(a.quadrants)]
        m, ts, inserted = D.build_scene(rng, parts, B, tines, 'v5_cross')
        if rng.random() < 0.5:
            held_plate(m, rng, ts, q, B)
        T = T0.copy()                                             # the camera may be re-mounted a little
        T[:3, 3] += rng.normal(0, [0.03, 0.03, 0.03])
        T[:3, :3] = Rotation.from_rotvec(rng.normal(0, np.radians(2.5), 3)).as_matrix() @ T[:3, :3]
        if i % 20 == 0:
            tex = D.table_texture(rng)
        Tcw = np.linalg.inv(T)
        img, depth = rd.render(m, Tcw, tex, rng)
        x0, y0, w, h = crop_box(ts, q, Tcw)
        j = rng.normal(0, 0.04, 3)                                # the real crop comes from a measured pose: jitter it
        x0, y0 = int(x0 + j[0] * w), int(y0 + j[1] * h)
        w = int(w * (1 + j[2]))
        h = int(w * 9 / 16)
        x0, y0 = max(0, min(D.W - w, x0)), max(0, min(D.H - h, y0))
        crop = cv2.resize(img[y0:y0 + h, x0:x0 + w], (D.W, D.H), interpolation=cv2.INTER_CUBIC)
        cv2.imwrite(str(out / f'{i:06d}.jpg'), D.post(crop, rng), [cv2.IMWRITE_JPEG_QUALITY, 95])
        lab = []
        for rec in D.labels([t for t in ts if t['q'] == q], rail, Tcw, depth):
            for k in ('root', 'tip'):
                if rec[k] is not None:                            # full-frame network px -> crop -> network px
                    u, v = rec[k][0] / D.SX, rec[k][1] / D.SY
                    u, v = (u - x0) * D.W / w, (v - y0) * D.H / h
                    rec[k] = [float(u * D.SX), float(v * D.SY)] if 0 <= u < D.W and 0 <= v < D.H else None
                    rec[k + '_vis'] = bool(rec[k + '_vis'] and rec[k] is not None)
            lab.append(rec)
        meta.append(dict(i=i, profile='side_v1', quadrant=q, crop=[x0, y0, w, h], inserted_plates=inserted, tines=lab))
        if i % 500 == 0:
            print(f'{i}/{a.n}', flush=True)
    (out / 'labels.json').write_text(json.dumps(dict(size=[D.OW, D.OH], scale=[D.SX, D.SY], profile='side_v1', samples=meta)))


if __name__ == '__main__':
    main()
