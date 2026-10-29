"""Side camera pose from the slot CNN's Q3 tips (zoomed crop, mapped back to the frame) matched to the model's Q3 tine tips
at the basket_fixed pose: tips grouped into two image rows, each row sorted along u, every (row assignment, index offset)
tried with solvePnP; best = lowest median reprojection error.  Writes calibration/data/side_cam/T_base_side.json.
    ~/miniconda3/envs/foundationpose/bin/python experiments/side_cam/pnp.py outputs/side_cam_q3_20261029/side_full.png"""
import itertools
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'experiments/slot_cnn'))
sys.path.append(str(ROOT / 'rl'))
import data as D  # noqa: E402
import eval as E  # noqa: E402
from train import TineNet  # noqa: E402

CROP = (470, 220, 400, 225)                           # Q3 in the 10/29 side view (보스 box)


def detect(bgr, crop=CROP, th=0.3):
    net = TineNet(pretrained=False).cuda().eval()
    sd = torch.load(ROOT / 'calibration/data/slot_cnn/run_v10_cross/ep02.pt', map_location='cuda')
    net.load_state_dict(sd.get('model', sd))
    x0, y0, w, h = crop
    z = cv2.resize(bgr[y0:y0 + h, x0:x0 + w], (1280, 720), interpolation=cv2.INTER_CUBIC)
    p = E.predict(net, z, th)['tip']
    return np.c_[p[:, 0] * w / 1280 + x0, p[:, 1] * h / 720 + y0, p[:, 2]]


def rows_of(tips, tol=4.0):
    """The two tip rows: the two best RANSAC lines v = a u + b (>= 6 tips within tol px each), top row first."""
    rng = np.random.default_rng(0)
    left, rows = tips.copy(), []
    for _ in range(2):
        best = None
        for _ in range(2000):
            i, j = rng.choice(len(left), 2, replace=False)
            if abs(left[i, 0] - left[j, 0]) < 30:
                continue
            a = (left[j, 1] - left[i, 1]) / (left[j, 0] - left[i, 0])
            if abs(a) > 0.3:
                continue
            b = left[i, 1] - a * left[i, 0]
            inl = np.abs(left[:, 1] - (a * left[:, 0] + b)) < tol
            if best is None or inl.sum() > best.sum():
                best = inl
        rows.append(left[best])
        left = left[~best]
    rows.sort(key=lambda r: r[:, 1].mean())
    return [r[np.argsort(r[:, 0])] for r in rows]


def model_tips(Tb, q='Q3'):
    parts, B = D.basket_parts(False)
    rows = {}
    for t in D.tine_table(parts, B):
        if t['q'] == q:
            rows.setdefault(t['row'], []).append(D.root_tip(t, B['pins']['rail_z'])[1])
    return {r: np.array(sorted(v, key=lambda p: p[0])) @ Tb[:3, :3].T + Tb[:3, 3] for r, v in rows.items()}


def main():
    bgr = cv2.imread(sys.argv[1])
    K = np.array(json.loads((ROOT / 'calibration/data/side_cam/T_base_side.json').read_text())['K']) \
        if (ROOT / 'calibration/data/side_cam/T_base_side.json').exists() else D.K_WRIST
    Tb = np.array(json.loads((ROOT / 'tesollo/demo_safe/basket_fixed.json').read_text())['T'])
    tips = detect(bgr)
    npz = Path(sys.argv[1]).with_suffix('.npz')
    depth = np.load(npz)['depth'].astype(float)
    sys.path.insert(0, str(ROOT / 'calibration'))
    import basket_vision as bv
    MB = bv.model_points(20000, z_min=-0.15) @ Tb[:3, :3].T + Tb[:3, 3]

    def depth_agree(Rc, t):
        Pc = MB @ Rc.T + t
        u = (K[0, 0] * Pc[:, 0] / Pc[:, 2] + K[0, 2]).astype(int)
        v = (K[1, 1] * Pc[:, 1] / Pc[:, 2] + K[1, 2]).astype(int)
        ok = (Pc[:, 2] > 0.2) & (u >= 0) & (u < depth.shape[1]) & (v >= 0) & (v < depth.shape[0])
        d = depth[v[ok], u[ok]]
        good = d > 0
        return float((np.abs(d[good] - Pc[ok, 2][good]) < 0.02).mean()) if good.any() else 0.0
    img_rows = rows_of(tips)
    print('image rows (top, bottom):', [len(r) for r in img_rows], ' model rows:', {r: len(m) for r, m in model_tips(Tb).items()})
    M = model_tips(Tb)
    best = None
    for (ra, rb) in ((0, 1), (1, 0)):
        for flip in (False, True):                     # image u along +x or -x of the basket
            ma, mb = (M[ra][::-1], M[rb][::-1]) if flip else (M[ra], M[rb])
            for oa in range(-6, 7):
                for ob in range(-6, 7):
                    obj, im = [], []
                    for rows_img, mod, off in ((img_rows[0], ma, oa), (img_rows[1], mb, ob)):
                        for i, p in enumerate(rows_img):
                            j = i + off
                            if 0 <= j < len(mod):
                                obj.append(mod[j]); im.append(p[:2])
                    if len(obj) < 12:
                        continue
                    obj, im = np.array(obj), np.array(im)
                    ok, rv, tv, inl = cv2.solvePnPRansac(obj, im, K, None, reprojectionError=6.0, iterationsCount=300)
                    if not ok or inl is None or len(inl) < 10:
                        continue
                    inl = inl[:, 0]                    # the tips are nearly planar: both mirror solutions, keep the one
                    n_s, rvs, tvs, _ = cv2.solvePnPGeneric(obj[inl], im[inl], K, None, flags=cv2.SOLVEPNP_SQPNP)   # above the
                    for rv, tv in zip(rvs, tvs):                                                          # table, looking down
                        Rc, _ = cv2.Rodrigues(rv)
                        cam = -Rc.T @ tv[:, 0]
                        # (보스 10/29: top rows = Q3, bottom = Q4, handle at the bottom) the camera is on the handle
                        # side (basket -y) above the table; the two equal tine rows also fit the mirror view
                        if cam[2] < 0.3 or (Rc.T @ [0, 0, 1.0])[2] > -0.2 or cam[1] > Tb[1, 3]:
                            continue
                        pr, _ = cv2.projectPoints(obj, rv, tv, K, None)
                        err = np.linalg.norm(pr[:, 0] - im, axis=1)
                        if np.median(err) > 3:
                            continue
                        # the tines repeat: a fit shifted by whole tines reprojects as well -> the whole basket model
                        # against the measured depth decides (fraction of model points within 2 cm of the depth)
                        score = (-depth_agree(Rc, tv[:, 0]), np.median(err))
                        if best is None or score < best[0]:
                            best = (score, rv, tv, (ra, rb, flip, oa, ob), int((err < 6).sum()), len(obj))
    (agree, med), rv, tv, hyp, ninl, nobj = best
    print(f'depth agreement {-agree:.2f}')
    Rc, _ = cv2.Rodrigues(rv)
    T_cam_base = np.eye(4); T_cam_base[:3, :3], T_cam_base[:3, 3] = Rc, tv[:, 0]
    T = np.linalg.inv(T_cam_base)
    print(f'hypothesis {hyp}: {ninl}/{nobj} inliers, median reprojection {med:.1f} px')
    print('camera position in base [mm]', (T[:3, 3] * 1000).round(0), ' optical axis', T[:3, 2].round(3))
    out = ROOT / 'calibration/data/side_cam/T_base_side.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(T_base_cam=T.tolist(), median_px=float(med), inliers=f'{ninl}/{nobj}', serial='338122303684',
                                   K=K.tolist(), source=f'{sys.argv[1]}: CNN Q3 tips -> model tips (basket_fixed) PnP')))
    vis = bgr.copy()
    Tcb = np.linalg.inv(T)
    for r in M.values():
        for P in r:
            pc = Tcb[:3, :3] @ P + Tcb[:3, 3]
            u, w = K[0, 0] * pc[0] / pc[2] + K[0, 2], K[1, 1] * pc[1] / pc[2] + K[1, 2]
            cv2.circle(vis, (int(u), int(w)), 4, (0, 255, 0), 1)
    for p in tips:
        cv2.circle(vis, (int(p[0]), int(p[1])), 2, (0, 0, 255), -1)
    cv2.imwrite(str(ROOT / 'outputs/side_cam_q3_20261029/pnp_check.png'), vis)
    print('saved', out)


if __name__ == '__main__':
    main()
