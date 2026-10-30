"""(보스 10/29 night) slot CNN on real side-camera frames: the quadrant crop from the measured camera pose (side_render.crop_box),
'ground truth' = the model's tine tips projected with that pose (calibration/data/side_cam/T_base_side.json, PnP 0.7 px on
Q3; Q4 is extrapolated, so a few px worse).  Tip recall = model tips with a prediction within --tol px (full-frame px),
precision = predictions within --tol of a model tip of THIS quadrant (other quadrants' tines, clips and the frame count as
false).  Read-only.
    ~/miniconda3/envs/foundationpose/bin/python experiments/slot_cnn/side_eval.py --ckpt A.pt B.pt --frames F.png ..."""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from scipy.optimize import linear_sum_assignment

sys.path.insert(0, str(Path(__file__).resolve().parent))
import data as D  # noqa: E402
import eval as E  # noqa: E402
import side_render as SR  # noqa: E402
from train import TineNet  # noqa: E402


def row_filter(P, n_max=13, tol=4.0):
    """Keep tips that sit on one of two image lines AND on that row's even tine spacing (clip / frame / wheel false tips
    at the row ends break the spacing).  P (n, 2) full-frame px -> kept indices."""
    left, keep = np.arange(len(P)), []
    for _ in range(2):
        if len(left) < 4:
            break
        Q = P[left]                                           # (10/30) every point pair at once, not 1500 random draws
        i, j = np.triu_indices(len(Q), 1)                     # (26 ms -> ~1 ms a quadrant; same best line, ties -> least residual)
        m = np.abs(Q[j, 0] - Q[i, 0]) >= 20
        if not m.any():
            break
        i, j = i[m], j[m]
        a = (Q[j, 1] - Q[i, 1]) / (Q[j, 0] - Q[i, 0])
        b = Q[i, 1] - a * Q[i, 0]
        r = np.abs(Q[None, :, 1] - (a[:, None] * Q[None, :, 0] + b[:, None]))
        inl = r < tol
        k = np.lexsort((np.where(inl, r, 0).sum(1), -inl.sum(1)))[0]
        best = left[inl[k]]
        row = best[np.argsort(P[best, 0])]
        if len(row) >= 4:
            gaps = np.diff(P[row, 0])
            pitch = float(np.median(gaps))
            ok = np.ones(len(row), bool)                      # peel ends whose gap is off the pitch by > 25 %
            while ok.sum() > 3:
                idx = np.nonzero(ok)[0]
                g0, g1 = P[row[idx[1]], 0] - P[row[idx[0]], 0], P[row[idx[-1]], 0] - P[row[idx[-2]], 0]
                if abs(g0 - pitch) > 0.25 * pitch:
                    ok[idx[0]] = False
                elif abs(g1 - pitch) > 0.25 * pitch:
                    ok[idx[-1]] = False
                else:
                    break
            row = row[ok][:n_max]
        keep += list(row)
        left = np.setdiff1d(left, best)
    return np.array(sorted(keep), int)


def load(ck):
    net = TineNet(pretrained=False).cuda().eval()
    sd = torch.load(D.ROOT / ck, map_location='cuda')
    net.load_state_dict(sd.get('model', sd))
    return net


def main():
    a = argparse.ArgumentParser()
    a.add_argument('--ckpt', nargs='+', required=True)
    a.add_argument('--frames', nargs='+', required=True)
    a.add_argument('--quadrants', nargs='+', default=['Q3', 'Q4'])
    a.add_argument('--tol', type=float, default=5.0, help='full-frame px')
    a.add_argument('--th', type=float, default=0.3)
    a.add_argument('--filter', action='store_true', help='row_filter() on the predictions')
    a.add_argument('--register', action='store_true', help='shift each row of model tips (<= 1 tine pitch along the row, '
                   '+-6 px across) to the predictions first: the basket model is off in Q4 (1-2 cm), not the camera '
                   '(Q3 matches it to 0.7 px).  The shift is reported; recall/precision then measure the CNN only')
    a.add_argument('--out', default='outputs/side_cam_q3_20261029/eval')
    a = a.parse_args()
    out = D.ROOT / a.out
    out.mkdir(parents=True, exist_ok=True)
    parts, B = D.basket_parts(False)
    tines, rail = D.tine_table(parts, B), B['pins']['rail_z']
    Tcw = np.linalg.inv(SR.side_pose())
    res = {}
    for ck in a.ckpt:
        net = load(ck)
        tot = dict(gt=0, hit=0, pred=0, good=0)
        per = {}
        for fr in a.frames:
            bgr = cv2.imread(str(D.ROOT / fr))
            for q in a.quadrants:
                x0, y0, w, h = SR.crop_box(tines, q, Tcw)
                z = cv2.resize(bgr[y0:y0 + h, x0:x0 + w], (D.W, D.H), interpolation=cv2.INTER_CUBIC)
                p = E.predict(net, z, a.th)['tip']
                P = np.c_[p[:, 0] * w / D.W + x0, p[:, 1] * h / D.H + y0] if len(p) else np.zeros((0, 2))
                if a.filter and len(P):
                    k_ = row_filter(P)
                    P, p = P[k_], p[k_]
                rows = [np.array([D.root_tip(t, rail)[1] for t in tines if t['q'] == q and t['row'] == r]) for r in (0, 1)]
                gs = []
                for G in rows:
                    g, _ = D.project(G, Tcw)
                    if a.register and len(P):
                        pitch = float(np.median(np.linalg.norm(np.diff(g, axis=0), axis=1)))
                        along = (g[-1] - g[0]) / np.linalg.norm(g[-1] - g[0])
                        best = (-1, 0, None)
                        for sa in np.arange(-pitch, pitch + 0.01, 0.5):
                            for sc in np.arange(-6, 6.01, 1.0):
                                gg = g + sa * along + sc * np.array([-along[1], along[0]])
                                d = np.linalg.norm(gg[:, None] - P[None], axis=2).min(1)
                                k = (d < a.tol).sum()
                                if k > best[0] or (k == best[0] and d[d < a.tol].sum() < best[1]):
                                    best = (k, d[d < a.tol].sum(), gg, sa)
                        g = best[2]
                        print(f'  {Path(fr).stem} {q} row: model shifted {best[3]:+.1f} px along the row ({best[3] / pitch:+.2f} pitch)')
                    gs.append(g)
                g = np.vstack(gs)
                hit = good = 0
                if len(P) and len(g):
                    d = np.linalg.norm(g[:, None] - P[None], axis=2)
                    r, c = linear_sum_assignment(d)
                    hit = int((d[r, c] < a.tol).sum())
                    good = hit
                per[f'{Path(fr).stem}:{q}'] = f'recall {hit}/{len(g)}, precision {good}/{len(P)}'
                tot['gt'] += len(g); tot['hit'] += hit; tot['pred'] += len(P); tot['good'] += good
                img = bgr[y0:y0 + h, x0:x0 + w].copy()
                img = cv2.resize(img, (D.W, D.H), interpolation=cv2.INTER_CUBIC)
                for u, v in g:
                    cv2.circle(img, (int((u - x0) * D.W / w), int((v - y0) * D.H / h)), 9, (0, 255, 0), 1)
                for u, v, _ in p:
                    cv2.circle(img, (int(u), int(v)), 4, (0, 0, 255), -1)
                cv2.putText(img, f'{Path(ck).parent.name}/{Path(ck).stem} {q}: {per[f"{Path(fr).stem}:{q}"]}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.imwrite(str(out / f'{Path(ck).parent.name}_{Path(ck).stem}_{Path(fr).stem}_{q}.png'), img)
        res[ck] = dict(recall=f'{tot["hit"]}/{tot["gt"]} = {tot["hit"] / max(1, tot["gt"]):.3f}',
                       precision=f'{tot["good"]}/{tot["pred"]} = {tot["good"] / max(1, tot["pred"]):.3f}', per=per)
        print(ck, res[ck]['recall'], res[ck]['precision'], json.dumps(per, ensure_ascii=False), flush=True)
    (out / 'summary.json').write_text(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
