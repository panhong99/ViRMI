"""(보스 10/29 night) side-wall D455 338122303684 pose in the robot base: its depth cloud fitted to the basket model at the
basket pose the fixed camera measures (tesollo/demo_safe/basket_fixed.json).  Read-only (a saved frame).
    ~/miniconda3/envs/foundationpose/bin/python experiments/side_cam/extrinsic.py outputs/side_cam_q3_20261029/side_full.npz
Table plane (RANSAC) -> points 1-20 cm above it -> trimmed ICP from a grid of camera guesses looking at the basket from the
handle side (-y), best = lowest trimmed residual.  Writes calibration/data/side_cam/T_base_side.json."""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'calibration'))


def cloud(depth, K, stride=3, zmax=2.0):
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[v, u]
    ok = (z > 0.2) & (z < zmax)
    x = (u[ok] - K[0, 2]) * z[ok] / K[0, 0]
    y = (v[ok] - K[1, 2]) * z[ok] / K[1, 1]
    return np.c_[x, y, z[ok]]


def plane_ransac(P, it=400, tol=0.008, rng=np.random.default_rng(0)):
    best = (0, None)
    for _ in range(it):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        d = -n @ a
        k = int((np.abs(P @ n + d) < tol).sum())
        if k > best[0]:
            best = (k, (n, d))
    n, d = best[1]
    return (n, d) if d > 0 else (-n, -d)          # normal toward the camera (origin): height > 0 on its side


def icp(S, M_tree, M, T0, iters=40, trim=0.6):
    T = T0.copy()
    for _ in range(iters):
        Q = S @ T[:3, :3].T + T[:3, 3]
        d, i = M_tree.query(Q)
        keep = d <= np.quantile(d, trim)
        A, B = Q[keep], M[i[keep]]
        ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
        Rm = Vt.T @ U.T
        if np.linalg.det(Rm) < 0:
            Vt[-1] *= -1
            Rm = Vt.T @ U.T
        dT = np.eye(4)
        dT[:3, :3], dT[:3, 3] = Rm, cb - Rm @ ca
        T = dT @ T
    Q = S @ T[:3, :3].T + T[:3, 3]
    d, _ = M_tree.query(Q)
    return T, float(np.sqrt(np.mean(np.sort(d)[:int(trim * len(d))] ** 2)))


def look_at(eye, target, up=(0, 0, 1.0)):
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.stack([x, y, z], 1)              # camera x right, y down, z forward
    T[:3, 3] = eye
    return T


def main():
    import basket_vision as bv
    f = np.load(sys.argv[1])
    depth, K = f['depth'].astype(float), f['K'].astype(float)
    Tb = np.array(json.loads((ROOT / 'tesollo/demo_safe/basket_fixed.json').read_text())['T'])
    P = cloud(depth, K)
    n, d = plane_ransac(P)
    h = P @ n + d                                     # height above the table plane
    S = P[(h > 0.01) & (h < 0.20)]
    print(f'{len(P)} points, table plane normal {n.round(3)}, {len(S)} points 1-20 cm above it', flush=True)
    M = bv.model_points(30000, z_min=-0.15) @ Tb[:3, :3].T + Tb[:3, 3]
    tree = cKDTree(M)
    c = Tb[:3, 3] + [0, 0, -0.07]
    best = None
    for dist in (0.6, 0.8, 1.0, 1.2):
        for elev in (25, 40, 55, 70):
            for az in range(-150, -20, 10):           # around the handle side (basket -y), robot at -x
                a, e = np.radians(az), np.radians(elev)
                eye = c + dist * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
                T0 = look_at(eye, c)                  # T_base_cam guess
                T, r = icp(S, tree, M, T0, iters=25)
                if best is None or r < best[1]:
                    best = (T, r, (dist, elev, az))
    T, r = icp(S, tree, M, best[0], iters=80, trim=0.5)
    print(f'best guess dist/elev/az {best[2]}, trimmed rms {r * 1000:.1f} mm', flush=True)
    print('camera position in base [mm]', (T[:3, 3] * 1000).round(0), ' optical axis', T[:3, 2].round(3))
    out = ROOT / 'calibration/data/side_cam/T_base_side.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(T_base_cam=T.tolist(), rms_mm=r * 1000, serial='338122303684', K=K.tolist(),
                                   source=f'{sys.argv[1]} depth ICP to the basket model at basket_fixed.json')))
    print('saved', out)


if __name__ == '__main__':
    main()
