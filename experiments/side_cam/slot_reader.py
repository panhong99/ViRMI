"""(보스 10/29 night) docking observation from the side camera: one frame -> the target slot band of a pin and, when a plate
is seen, where its lowest rim point sits relative to that band, in tine pitches (the units the docking policy sees).
    ~/miniconda3/envs/foundationpose/bin/python experiments/side_cam/slot_reader.py FRAME.png --quadrant Q3 --pin 6 [--out x.png]
Steps: quadrant crop from the measured side pose -> run_side_v1 tips -> row_filter (two rows, even spacing) -> number the
tines from the robot-far end (tine 1 next to the middle clip; pin k = gap between tines k and k+1 = plate_plans.pin_gap)
-> band = the 4 tips of tines k, k+1 on both rows.  A row that does not show 13 tines is 'ambiguous' (no numbers): the
numbering would slide by a whole tine.  Read-only."""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'experiments/slot_cnn'))
import data as D  # noqa: E402
import eval as E  # noqa: E402
import side_eval as SE  # noqa: E402
import side_render as SR  # noqa: E402

CKPT = 'calibration/data/slot_cnn/run_side_v2/ep07.pt'   # 10/29 night: sim 0.994 / 0.987 (v1 0.988 / 0.973)
N_TINES = 13


def read(bgr, quadrant, net=None):
    """-> dict(rows=[(13, 2) full-frame px or None] top row first, crop, why)"""
    net = net or SE.load(CKPT)
    parts, B = D.basket_parts(False)
    tines = D.tine_table(parts, B)
    Tcw = np.linalg.inv(SR.side_pose())
    x0, y0, w, h = SR.crop_box(tines, quadrant, Tcw)
    z = cv2.resize(bgr[y0:y0 + h, x0:x0 + w], (D.W, D.H), interpolation=cv2.INTER_CUBIC)
    p = E.predict(net, z, 0.3)['tip']
    P = np.c_[p[:, 0] * w / D.W + x0, p[:, 1] * h / D.H + y0] if len(p) else np.zeros((0, 2))
    P = P[SE.row_filter(P)] if len(P) else P
    rows, why = [], []
    if len(P):
        v = P[:, 1]                                            # the filter keeps two lines: split them at the widest v gap
        o = np.argsort(v)
        k = int(np.argmax(np.diff(v[o]))) if len(P) > 1 else 0
        groups = [P[o[:k + 1]], P[o[k + 1:]]]
    else:
        groups = []
    for g in sorted(groups, key=lambda g: g[:, 1].mean()):
        g = g[np.argsort(-g[:, 0])]                            # far end (middle clip) is image right in this view
        if len(g) == N_TINES:
            rows.append(g)
        else:
            rows.append(None)
            why.append(f'row with {len(g)} tines (need {N_TINES})')
    while len(rows) < 2:
        rows.append(None)
        why.append('row missing')
    return dict(rows=rows, crop=(x0, y0, w, h), why='; '.join(why))


def band(r, pin):
    """The 4 tips around pin (1-based) as [top k, top k+1, bottom k+1, bottom k] or None."""
    if any(x is None for x in r['rows']) or not 1 <= pin <= N_TINES - 1:
        return None
    t, b = r['rows']
    return np.array([t[pin - 1], t[pin], b[pin], b[pin - 1]])


def offset_pitch(r, pin, pt):
    """Image point (e.g. the plate's lowest rim point) -> offset from the band centre line along the rows, in pitches
    (+ = toward higher pin numbers), interpolated between the two rows at the point's height."""
    t, b = r['rows']
    out = []
    for row in (t, b):
        c = (row[pin - 1] + row[pin]) / 2
        pitch = np.median(np.linalg.norm(np.diff(row, axis=0), axis=1))
        along = (row[pin] - row[pin - 1]) / np.linalg.norm(row[pin] - row[pin - 1])
        out.append(((pt - c) @ along) / pitch)
    s = np.clip((pt[1] - t[:, 1].mean()) / max(1.0, b[:, 1].mean() - t[:, 1].mean()), 0, 1)
    return float((1 - s) * out[0] + s * out[1])


def main():
    a = argparse.ArgumentParser()
    a.add_argument('frame')
    a.add_argument('--quadrant', default='Q3')
    a.add_argument('--pin', type=int, default=6)
    a.add_argument('--out', default='')
    a = a.parse_args()
    bgr = cv2.imread(a.frame)
    r = read(bgr, a.quadrant)
    bd = band(r, a.pin)
    print(f'{a.quadrant} pin {a.pin}: ' + ('band ' + json.dumps(np.round(bd, 1).tolist()) if bd is not None else f'no band ({r["why"]})'))
    if a.out:
        img = bgr.copy()
        for row in r['rows']:
            if row is not None:
                for i, p in enumerate(row):
                    cv2.circle(img, (int(p[0]), int(p[1])), 3, (0, 0, 255), -1)
                    cv2.putText(img, str(i + 1), (int(p[0]) - 4, int(p[1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
        if bd is not None:
            over = img.copy()
            cv2.fillPoly(over, [bd.astype(np.int32)], (60, 230, 230))
            img = cv2.addWeighted(over, 0.45, img, 0.55, 0)
        x0, y0, w, h = r['crop']
        cv2.imwrite(a.out, cv2.resize(img[y0:y0 + h, x0:x0 + w], (D.W, D.H)))
        print('saved', a.out)


if __name__ == '__main__':
    main()
