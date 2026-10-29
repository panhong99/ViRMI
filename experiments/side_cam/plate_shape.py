"""Real plate shape from the fixed D455 (read-only): YOLOE plate masks -> depth points in base -> for each plate lying
on a stand: diameter (rim circle) and dish depth (rim plane to the lowest/highest centre point), vs the sim mesh
rl/assets/plate029_simplified.obj (26.7 cm, 2.5 cm).  Run from the repo root (YOLOE text model cache)."""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'calibration'))
sys.path.insert(0, str(ROOT / 'real_time_project'))
sys.path.insert(0, str(ROOT / 'tesollo'))


def main():
    import calib_core as cc
    import demo_watch as dw
    import plate_plans as pp
    from fixed_frames import FixedFrames
    Tbc = np.array(cc.load_json(cc.latest_t_base_cam())['T_base_cam'])
    det = dw.Detector(['ceramic plate', 'red ceramic plate'])
    cam = FixedFrames(pp.C['cameras']['fixed_serial'])
    try:
        for _ in range(20 if cam.cam is not None else 0):
            cam.read()
        fr = cam.read()
    finally:
        cam.stop()
    K = np.asarray(fr.K, float)
    for name, conf, box, m in det(fr.rgb, conf=0.3):
        v, u = np.nonzero(m)
        z = fr.depth[v, u]
        ok = (z > 0.2) & (z < 2.0)
        u, v, z = u[ok], v[ok], z[ok]
        Pc = np.c_[(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z]
        P = Pc @ Tbc[:3, :3].T + Tbc[:3, 3]
        if len(P) < 500:
            continue
        c = np.median(P, 0)
        r = np.linalg.norm(P[:, :2] - c[:2], axis=1)
        R = np.percentile(r, 98)
        rim = P[r > 0.85 * R]
        A = np.c_[rim[:, 0], rim[:, 1], np.ones(len(rim))]          # rim plane z = a x + b y + c
        coef, *_ = np.linalg.lstsq(A, rim[:, 2], rcond=None)
        h = P[:, 2] - (np.c_[P[:, 0], P[:, 1], np.ones(len(P))] @ coef)
        inner = h[r < 0.4 * R]
        tilt = np.degrees(np.arctan(np.hypot(coef[0], coef[1])))
        print(f'{name} conf {conf:.2f} at ({c[0]:.3f}, {c[1]:+.3f}, {c[2]:.3f}) m: diameter ~{2 * R * 100:.1f} cm, rim plane tilt '
              f'{tilt:.0f} deg, centre vs rim plane median {np.median(inner) * 1000:+.1f} mm (p10 {np.percentile(inner, 10) * 1000:+.1f}, '
              f'p90 {np.percentile(inner, 90) * 1000:+.1f}), {len(P)} points')
    v = np.array([list(map(float, l.split()[1:4])) for l in open(ROOT / 'rl/assets/plate029_simplified.obj') if l.startswith('v ')])
    print(f'sim mesh: diameter {(v[:, 0].max() - v[:, 0].min()) * 100:.1f} cm, height {(v[:, 2].max() - v[:, 2].min()) * 100:.1f} cm')


if __name__ == '__main__':
    main()
