"""Real tine tip positions in the basket frame from one side frame: slot_reader's numbered tips back-projected onto the
plane of the model's tip height (z of that row's model tips, basket frame) -> x along the row per tine, vs the model.
Q3 is the check (the camera pose was fitted there), Q4 is the finding.  Read-only.
    ~/miniconda3/envs/foundationpose/bin/python experiments/side_cam/tine_positions.py FRAME.png"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'experiments/side_cam'))
import slot_reader as S  # noqa: E402

D, SR = S.D, S.SR


def main():
    bgr = cv2.imread(sys.argv[1])
    net = S.SE.load(S.CKPT)
    parts, B = D.basket_parts(False)
    tines, rail = D.tine_table(parts, B), B['pins']['rail_z']
    Tbc = SR.side_pose()                                        # T_basket_cam
    K = D.K_WRIST
    out = {}
    for q in ('Q3', 'Q4'):
        r = S.read(bgr, q, net)
        if any(x is None for x in r['rows']):
            print(q, 'ambiguous:', r['why'])
            continue
        model = {row: sorted([t for t in tines if t['q'] == q and t['row'] == row], key=lambda t: t['num'])
                 for row in (0, 1)}
        res = []
        for img_row in r['rows']:
            # which model row is this image row: the one whose projected tips are nearest in v
            cand = []
            for row, ts in model.items():
                tips = np.array([D.root_tip(t, rail)[1] for t in ts])
                uv, _ = D.project(tips, np.linalg.inv(Tbc))
                cand.append((abs(uv[:, 1].mean() - img_row[:, 1].mean()), row, tips))
            _, row, tips = min(cand)
            z = tips[:, 2].mean()
            for i, (u, v) in enumerate(img_row):                 # tine i+1 (from the far end) = model num?
                ray = np.linalg.inv(K) @ [u, v, 1.0]
                d = Tbc[:3, :3] @ ray
                o = Tbc[:3, 3]
                s = (z - o[2]) / d[2]
                p = o + s * d
                m = min(tips, key=lambda t: abs(t[0] - p[0]))
                res.append(dict(row=row, tine=i + 1, x_real_mm=round(p[0] * 1000, 1), y_real_mm=round(p[1] * 1000, 1),
                                dx_to_nearest_model_mm=round((p[0] - m[0]) * 1000, 1)))
        out[q] = res
        dx = np.array([r_['dx_to_nearest_model_mm'] for r_ in res])
        for row in (0, 1):
            d_ = np.array([r_['dx_to_nearest_model_mm'] for r_ in res if r_['row'] == row])
            xs = np.array([r_['x_real_mm'] for r_ in res if r_['row'] == row])
            print(f'{q} row {row}: real pitch {np.median(np.abs(np.diff(xs))):.1f} mm, offset to the nearest model tine '
                  f'median {np.median(d_):+.1f} mm (min {d_.min():+.1f}, max {d_.max():+.1f})')
    p = ROOT / 'outputs/side_cam_q3_20261029/tine_positions.json'
    p.write_text(json.dumps(out, indent=1))
    print('saved', p)


if __name__ == '__main__':
    main()
