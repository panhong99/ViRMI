"""Tesollo hand grasp: which object (plate / bowl) and where, from the fixed D455 + YOLOE (read-only, no robot command).

The object centre = middle of its mask's depth points in the robot base frame (xy: 5-95 % range midpoint, round objects),
median over a few frames, refused if the frames disagree or the class flips.  The arm plan moves the taught grasp pose by
(centre now - centre saved when the pose was taught, tesollo/arm_poses.yaml obj_centre).
    ~/miniconda3/envs/foundationpose/bin/python tesollo/perceive.py              # what and where, dx/dy to the taught spot
    ~/miniconda3/envs/foundationpose/bin/python tesollo/perceive.py --save_ref   # object AT the taught spot: save its centre
The camera must be free (no perception process / pipeline running).  A picture with the mask goes to --png.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
ARM_POSES = HERE / 'arm_poses.yaml'
# YOLOE prompt -> object; 'white ceramic bowl' found bowls that plain 'bowl' missed on the 10/26 fixed view (demo_watch)
PROMPTS = {'plate': 'plate', 'white ceramic bowl': 'bowl'}


def detect(model, rgb, conf):
    """-> [(object, conf, mask)] for the plate / bowl prompts."""
    names = list(PROMPTS)
    r = model.predict(rgb[:, :, ::-1].copy(), conf=conf, verbose=False, retina_masks=True)[0]
    if r.boxes is None or not len(r.boxes) or r.masks is None:
        return []
    masks = r.masks.data.cpu().numpy() > 0.5
    return [(PROMPTS[names[int(k)]], float(c), m) for k, c, m in
            zip(r.boxes.cls.cpu().numpy(), r.boxes.conf.cpu().numpy(), masks)]


def centre(mask, depth, K, Tbc, min_points=150):
    """Mask -> (centre xyz in base: xy = 5-95 % midpoint, z = 95 % height, points) or None."""
    sys.path.insert(0, str(ROOT / 'calibration'))
    from basket_vision import cloud
    P, _ = cloud(cv2.erode(mask.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool), depth, K, Tbc)
    P = P[(P[:, 2] > 0.005) & (P[:, 2] < 0.40)]                 # above the table, below the camera clutter
    if len(P) < min_points:
        return None
    lo, hi = np.percentile(P[:, :2], 5, axis=0), np.percentile(P[:, :2], 95, axis=0)
    return np.r_[(lo + hi) / 2, np.percentile(P[:, 2], 95)], len(P)


def near_taught(obj, xy, A, radius):
    """Distance of a detection from where that object is grasped (saved centre, else the taught flange xy)."""
    ref = A[obj].get('obj_centre') or [v / 1000 for v in A[obj]['flange_posx'][:2]]
    return float(np.hypot(xy[0] - ref[0], xy[1] - ref[1]))


def measure(n=5, conf=0.25, only=None, png=None, max_spread_mm=8.0, radius=0.30):
    """n frames of the fixed D455 -> dict(ok, obj, centre, conf, spread_mm, why).  The table holds other plates and
    bowls (10/27: a bowl 1.5 m away won on confidence), so only detections within `radius` of the grasp spot count,
    the nearest one wins."""
    sys.path.insert(0, str(ROOT / 'calibration'))
    sys.path.insert(0, str(ROOT / 'real_time_project'))
    import calib_core as cc
    import plate_plans as pp
    from fixed_frames import FixedFrames
    from ultralytics import YOLOE
    Tbc = np.array(cc.load_json(cc.latest_t_base_cam())['T_base_cam'])
    model = YOLOE(str(ROOT / 'yoloe-v8l-seg.pt'))
    cc.yoloe_set_classes(model, list(PROMPTS))
    A = yaml.safe_load(ARM_POSES.read_text())
    cam = FixedFrames(pp.C['cameras']['fixed_serial'])
    found = []
    try:
        for _ in range(15 if cam.cam is not None else 0):     # auto exposure settles (a shared camera has)
            cam.read()
        for i in range(n):
            fr = cam.read()
            dets = [d for d in detect(model, fr.rgb, conf) if only is None or d[0] == only]
            cands = []
            for obj, c, m in dets:
                got = centre(m, fr.depth, np.asarray(fr.K, float), Tbc)
                if got is not None:
                    cands.append((obj, c, got[0], got[1], m, near_taught(obj, got[0], A, radius)))
            near = [d for d in cands if d[5] <= radius]
            best = min(near, key=lambda d: d[5]) if near else None
            found.append(best)
            if i == 0 and png:                                   # every detection: green = picked, yellow = others
                img = fr.rgb.copy()
                for d in cands:
                    col = (60, 255, 60) if d is best else (255, 220, 0)
                    img[d[4]] = (0.5 * img[d[4]] + 0.5 * np.array(col)).astype(np.uint8)
                    v, u = np.nonzero(d[4])
                    cv2.putText(img, f'{d[0]} {d[1]:.2f} {d[5] * 100:.0f}cm', (int(u.mean()) - 40, int(v.min()) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
                Path(png).parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(png), img[:, :, ::-1])
            time.sleep(0.3)
    finally:
        cam.stop()
    if any(f is None for f in found):
        return dict(ok=False, why=f'no plate / bowl within {radius * 100:.0f} cm of the grasp spot in '
                                  f'{sum(f is None for f in found)}/{n} frames')
    objs = {f[0] for f in found}
    if len(objs) > 1:
        return dict(ok=False, why=f'class changes between frames: {sorted(objs)}')
    C = np.array([f[2] for f in found])
    med = np.median(C, axis=0)
    spread = float(np.abs(C[:, :2] - med[:2]).max() * 1000)
    ok = spread <= max_spread_mm
    return dict(ok=ok, obj=found[0][0], centre=med.round(4).tolist(), conf=round(float(np.median([f[1] for f in found])), 3),
                points=int(np.median([f[3] for f in found])), spread_mm=round(spread, 1),
                why='' if ok else f'frames disagree by {spread:.1f} mm (> {max_spread_mm})')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--object', choices=('plate', 'bowl'), help='look only for this one (default: whichever is seen)')
    p.add_argument('--save_ref', action='store_true', help='the object sits at the taught grasp spot: save its centre')
    p.add_argument('--frames', type=int, default=5)
    p.add_argument('--radius', type=float, default=0.30, help='(보스 11/02, a plate 31 cm out) only detections this close [m] '
                   'to the taught grasp spot count, the nearest wins')
    p.add_argument('--conf', type=float, default=0.12, help='(10/27 fixed view: red plate 0.17-0.26)')
    p.add_argument('--png', default=str(ROOT / 'outputs/hand_grasp_20261027/perceive_last.png'))
    a = p.parse_args()
    r = measure(a.frames, a.conf, a.object, a.png, radius=a.radius)
    if not r['ok']:
        sys.exit(f'REFUSED: {r["why"]}')
    print(f'object {r["obj"]} (conf {r["conf"]}), centre {[round(v * 1000, 1) for v in r["centre"]]} mm in base, '
          f'{r["points"]} points, frames within {r["spread_mm"]} mm;  picture {a.png}', flush=True)
    A = yaml.safe_load(ARM_POSES.read_text())
    if a.save_ref:
        A[r['obj']]['obj_centre'] = r['centre']
        A[r['obj']]['obj_centre_saved'] = time.strftime('%Y-%m-%d %H:%M:%S')
        ARM_POSES.write_text(yaml.safe_dump(A, sort_keys=False, allow_unicode=True))
        print(f'saved as the {r["obj"]} reference (taught spot) in {ARM_POSES}')
        return
    ref = A[r['obj']].get('obj_centre')
    if ref is None:
        sys.exit(f'no saved {r["obj"]} reference: put it at the taught spot and run --save_ref once')
    dx, dy = r['centre'][0] - ref[0], r['centre'][1] - ref[1]
    print(f'shift from the taught spot: dx {dx * 1000:+.1f} mm, dy {dy * 1000:+.1f} mm')
    print(f'RESULT {r["obj"]} {dx:.4f} {dy:.4f}')


if __name__ == '__main__':
    main()
