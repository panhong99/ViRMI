"""Held-out sim side views (exact labels): tip recall / precision of a checkpoint, with and without side_eval.row_filter.
    ~/miniconda3/envs/foundationpose/bin/python experiments/slot_cnn/side_sim_eval.py DATA CKPT [CKPT ...]"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

sys.path.insert(0, str(Path(__file__).resolve().parent))
import data as D  # noqa: E402
import eval as E  # noqa: E402
import side_eval as SE  # noqa: E402

TOL = 12.0                                                    # px in the 1280 x 720 crop (~3-4 px in the full frame)


def main():
    d = Path(sys.argv[1])
    L = json.loads((d / 'labels.json').read_text())
    for ck in sys.argv[2:]:
        net = SE.load(ck)
        for filt in (False, True):
            gt = hit = pred = 0
            per_img = []
            for s in L['samples']:
                img = cv2.imread(str(d / f'{s["i"]:06d}.jpg'))
                img = cv2.resize(img[:D.OH - 4], (D.W, D.H))      # stored at network size -> crop size
                p = E.predict(net, img, 0.3)['tip'][:, :2]
                if filt and len(p):
                    p = p[SE.row_filter(p * 0.3125)]               # filter tolerances are full-frame px (crop ~3.2x)
                g = np.array([[t['tip'][0] / D.SX, t['tip'][1] / D.SY] for t in s['tines'] if t['tip'] and t['tip_vis']])
                k = 0
                if len(p) and len(g):
                    dd = np.linalg.norm(g[:, None] - p[None], axis=2)
                    r, c = linear_sum_assignment(dd)
                    k = int((dd[r, c] < TOL).sum())
                gt += len(g); hit += k; pred += len(p)
                per_img.append(k == len(g) and k == len(p))
            print(f'{ck} filter={filt}: recall {hit}/{gt} = {hit / gt:.3f}, precision {hit}/{pred} = {hit / max(1, pred):.3f}, '
                  f'images all-correct {sum(per_img)}/{len(per_img)}', flush=True)


if __name__ == '__main__':
    main()
