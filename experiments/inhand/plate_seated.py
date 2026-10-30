"""(보스 10/30) the target: a plate 보스 seated by hand in a slot, measured by the same fixed D455 as plate_inhand.py
(so the camera's own pose error is in both).  Read-only.
    ~/miniconda3/envs/foundationpose/bin/python experiments/inhand/plate_seated.py --quadrant Q3 --pin 6
Plate = the largest plane of the depth points within 0.25 m of where a seated plate's centre would be (the gap, 13 cm
up), 3 cm over the basket floor (not the floor / cloth); centre = the 13.5 cm circle as in plate_inhand.  Prints the
plate in the basket frame: lowest rim vs the floor (0 if the camera were exact), its x vs the gap, tilt and yaw; saves
outputs/inhand_20261030/seated_<Q>_<pin>_<time>.json (+ frame)."""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plate_inhand as PI  # noqa: E402


def main():
    a = argparse.ArgumentParser()
    a.add_argument('--quadrant', default='Q3')
    a.add_argument('--pin', type=int, default=6)
    a.add_argument('--frame', default='', help='a saved .npz (rgb, depth, K) instead of the camera')
    a = a.parse_args()
    import calib_core as cc
    import plate_plans as pp
    if a.frame:
        z = np.load(a.frame)
        rgb, depth, K = z['rgb'], z['depth'].astype(np.float32), z['K'].astype(float)
    else:
        rgb, depth, K, _ = PI.frame()
    Tbc = np.array(cc.load_json(cc.latest_t_base_cam())['T_base_cam'])
    Tbb = np.array(json.loads(PI.BASKET.read_text())['T'])
    Tkb = np.linalg.inv(Tbb)
    mq = pp.robot_quadrants(Tbb)[a.quadrant]
    gaps = np.array(pp.basket_gaps()[mq])
    gi = pp.pin_gap(Tbb, mq, a.pin)
    g = gaps[gi]
    floor = g[2]
    guess = (Tbb @ np.r_[g[0], g[1], floor + 0.13, 1.0])[:3]
    X, UV = PI.cloud(depth, K, Tbc, step=1)
    Xb = X @ Tkb[:3, :3].T + Tkb[:3, 3]
    keep = Xb[:, 2] > floor + 0.03
    r = PI.plate_plane(X, guess, np.random.default_rng(0), keep=keep)
    if r is None:
        sys.exit('no depth points at the slot')
    Q, c, n, idx = r
    centre, mad = PI.circle_centre(Q, c, n)
    nb = Tkb[:3, :3] @ n
    nb *= np.sign(nb[0]) if nb[0] else 1.0
    cb = (Tkb @ np.r_[centre, 1.0])[:3]
    e1 = np.cross(nb, [0, 0, 1.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(nb, e1)
    th = np.linspace(0, 2 * np.pi, 1440, endpoint=False)
    rim = cb + PI.R_PLATE * (np.cos(th)[:, None] * e1 + np.sin(th)[:, None] * e2)
    low = rim[np.argmin(rim[:, 2])]
    res = dict(time=time.strftime('%Y%m%d_%H%M%S'), target=f'{a.quadrant}:{a.pin}', points=int(len(Q)),
               rim_fit_mad_mm=round(mad * 1000, 1), normal_base=n.round(4).tolist(), centre_base=centre.round(4).tolist(),
               basket=dict(centre_mm=(cb * 1000).round(1).tolist(), lowest_rim_vs_floor_mm=round(float(low[2] - floor) * 1000, 1),
                           lowest_rim_x_vs_gap_mm=round(float(low[0] - g[0]) * 1000, 1),
                           tilt_deg=round(float(np.degrees(np.arctan2(nb[2], nb[0]))), 1),
                           yaw_deg=round(float(np.degrees(np.arctan2(nb[1], nb[0]))), 1)))
    out = PI.ROOT / 'outputs/inhand_20261030'
    out.mkdir(parents=True, exist_ok=True)
    stem = f'seated_{a.quadrant}_{a.pin}_{res["time"]}'
    np.savez_compressed(out / f'{stem}_frame.npz', rgb=rgb, depth=depth, K=K)
    (out / f'{stem}.json').write_text(json.dumps(res, indent=1))
    import cv2
    vis = cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR)
    for u, v in UV[idx]:
        cv2.circle(vis, (int(u), int(v)), 1, (0, 255, 0), -1)
    cv2.imwrite(str(out / f'{stem}.png'), vis)
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
