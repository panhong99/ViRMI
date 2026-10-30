"""(보스 10/30) the held plate's pose in the hand from the fixed D455 depth, and where it would land in the slot.
Read-only (no robot / hand / file of the demo touched): the frame from the spectator view's shared frames
(/dev/shm/fixed_latest.npz, else the fixed camera is opened), the joints from the read-only ROS service.
    ~/miniconda3/envs/foundationpose/bin/python experiments/inhand/plate_inhand.py --quadrant Q3 --pin 6
Run it while the arm stands still with the plate held (demo_safe.sh --leg_enter: at the Enter before the post-grasp leg).
Plate = the largest plane (RANSAC, 4 mm) of the depth points within 0.30 m of the flange and 2 cm over the table, its
centre = a 13.5 cm circle fitted to the plane points' outline (the hand hides part of the rim).  Then the plate is put
in the hand at the target's taught slot pose (taught_plan.py, the fixed basket): its normal vs the tines (yaw about the
vertical, pitch vs the 12.5 deg lean) and the gap its lowest rim crosses on each tine row (L = basket +y, R = -y),
numbered as the pins.  Saves frame + result to outputs/inhand_20261030/."""
import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'calibration'))
BASKET = ROOT / 'tesollo/demo_safe/basket_fixed.json'
PLAN_PY = Path.home() / 'miniconda3/envs/foundationpose/bin/python'
R_PLATE = 0.135                                    # real plates 27.3 / 26.8 cm across (10/29 fixed-camera depth)
LEAN = np.radians(12.5)


def frame(new=False):
    """tesollo/fixed_frames.py: the spectator view's shared frame (new: one taken after this call), else the camera
    opened once."""
    import plate_plans as pp
    sys.path.insert(0, str(ROOT / 'tesollo'))
    from fixed_frames import FixedFrames
    cam = FixedFrames(pp.C['cameras']['fixed_serial'])
    if new and cam.cam is None:
        cam.last = time.time()                                   # the arm has just stopped: a frame after that
    try:
        for _ in range(1 if cam.cam is None else 15):          # an opened camera: auto exposure settles
            f = cam.read()
    finally:
        cam.stop()
    return np.asarray(f.rgb), np.asarray(f.depth, np.float32), np.asarray(f.K, float), \
        'shared' if cam.cam is None else 'opened'


def joints():
    cmd = ['env', '-u', 'PYTHONPATH', '-u', 'PYTHONHOME', '-u', 'LD_LIBRARY_PATH', '-u', 'CONDA_PREFIX', '-u',
           'CONDA_DEFAULT_ENV', 'PATH=/usr/bin:/bin', 'bash', '-c',
           f'source /opt/ros/jazzy/setup.bash; source {ROOT.parent / "doosan_ws/install/setup.bash"}; '
           f'exec /usr/bin/python3 {ROOT / "tesollo/read_posj.py"}']
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=40)
    if out.returncode:
        sys.exit(f'joints: {out.stderr.strip()[-300:]}')
    return np.array(json.loads(out.stdout.strip().splitlines()[-1]))


def cloud(depth, K, T_base_cam, step=2):
    v, u = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
    z = depth[v, u]
    ok = (z > 0.2) & (z < 2.5)
    u, v, z = u[ok], v[ok], z[ok]
    Xc = np.c_[(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z]
    return Xc @ T_base_cam[:3, :3].T + T_base_cam[:3, 3], np.c_[u, v]


def plate_plane(X, flange, rng, tol=0.004, iters=600):
    sel = (np.linalg.norm(X - flange, axis=1) < 0.30) & (X[:, 2] > 0.02)
    P = X[sel]
    if len(P) < 200:
        return None
    best = None
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((P - a) @ n) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    Q = P[best]
    c = Q.mean(0)
    n = np.linalg.svd(Q - c)[2][2]
    return Q, c, n, np.nonzero(sel)[0][best]


def circle_centre(Q, c, n):
    """In-plane 13.5 cm circle through the outline (hull) of the plane points -> centre (base)."""
    a = np.cross(n, [0, 0, 1.0]) if abs(n[2]) < 0.9 else np.cross(n, [1.0, 0, 0])
    a /= np.linalg.norm(a)
    b = np.cross(n, a)
    p2 = np.c_[(Q - c) @ a, (Q - c) @ b].astype(np.float32)
    hull = cv2.convexHull(p2).reshape(-1, 2)
    from scipy.optimize import least_squares
    r = least_squares(lambda m: np.linalg.norm(hull - m, axis=1) - R_PLATE, p2.mean(0), loss='soft_l1', f_scale=0.005)
    m = r.x
    return c + m[0] * a + m[1] * b, float(np.median(np.abs(np.linalg.norm(hull - m, axis=1) - R_PLATE)))


def slot_pose(quadrant, pin):
    out = subprocess.run([str(PLAN_PY), str(ROOT / 'tesollo/taught_plan.py'), '--index', '0', '--quadrant', quadrant,
                          '--pin', str(pin), '--rise', '0.30', '--wrist_less', '25', '--smooth', '--basket', str(BASKET)],
                         capture_output=True, text=True, cwd=str(ROOT))
    m = re.search(r'^plan: (\S+)', out.stdout, re.M)
    if not m:
        sys.exit(f'taught_plan: {(out.stdout + out.stderr).strip()[-300:]}')
    return np.array(json.loads(Path(m.group(1)).read_text())['legs']['insert']['waypoints_deg'][-1])


def main():
    a = argparse.ArgumentParser()
    a.add_argument('--quadrant', default='Q3')
    a.add_argument('--pin', type=int, default=6)
    a.add_argument('--joints', type=float, nargs=6, help='instead of reading the robot (offline check)')
    a.add_argument('--frame', default='', help='a saved .npz (rgb, depth, K) instead of the camera')
    a.add_argument('--new_frame', action='store_true', help='a shared frame taken after the call (dg5fs_sdk --inhand_check)')
    a = a.parse_args()
    import calib_core as cc
    import plate_plans as pp
    t0 = time.time()
    k = pp.Kinematics()
    if a.frame:
        z = np.load(a.frame)
        rgb, depth, K, src = z['rgb'], z['depth'].astype(np.float32), z['K'].astype(float), a.frame
    else:
        rgb, depth, K, src = frame(a.new_frame)
    q = np.array(a.joints) if a.joints else joints()
    t_in = time.time() - t0
    Tbc = np.array(cc.load_json(cc.latest_t_base_cam())['T_base_cam'])
    F = k.fk(np.radians(q))
    X, UV = cloud(depth, K, Tbc)
    r = plate_plane(X, F[:3, 3], np.random.default_rng(0))
    if r is None:
        sys.exit('no depth points near the hand')
    Q, c, n, idx = r
    centre, rim_mad = circle_centre(Q, c, n)
    t_meas = time.time() - t0 - t_in
    # plate in the flange, then at the taught slot pose of the target
    n_f, c_f = F[:3, :3].T @ n, np.linalg.inv(F) @ np.r_[centre, 1.0]
    Tbb = np.array(json.loads(BASKET.read_text())['T'])
    Fs = k.fk(np.radians(slot_pose(a.quadrant, a.pin)))
    Tkb = np.linalg.inv(Tbb)
    nb = Tkb[:3, :3] @ Fs[:3, :3] @ n_f
    nb *= np.sign(nb[0]) if nb[0] else 1.0
    cb = (Tkb @ Fs @ c_f)[:3]
    yaw = np.degrees(np.arctan2(nb[1], nb[0]))
    pitch = np.degrees(np.arctan2(nb[2], nb[0]) - LEAN)
    mq = pp.robot_quadrants(Tbb)[a.quadrant]
    gaps = np.array(pp.basket_gaps()[mq])
    ng = len(gaps)
    away = (Tbb[:3, 0] @ np.r_[Tbb[:2, 3], 0.0]) >= 0
    e1 = np.cross(nb, [0, 0, 1.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(nb, e1)
    th = np.linspace(0, 2 * np.pi, 1440, endpoint=False)
    rim = cb + R_PLATE * (np.cos(th)[:, None] * e1 + np.sin(th)[:, None] * e2)
    rows = {}
    for name, y in (('L', gaps[0, 1] + 0.0475), ('R', gaps[0, 1] - 0.0475)):
        s = np.abs(rim[:, 1] - y) < 0.003
        if not s.any():
            rows[name] = None
            continue
        j = np.argmin(np.where(s, rim[:, 2], 9))
        gi = int(np.argmin(np.abs(gaps[:, 0] - rim[j, 0])))
        rows[name] = dict(pin=(ng - gi) if away else gi + 1, off_mm=round(float(rim[j, 0] - gaps[gi, 0]) * 1000, 1),
                          low_mm=round(float(rim[j, 2] - gaps[gi, 2]) * 1000, 1))
    tgt = pp.pin_gap(Tbb, mq, a.pin)
    res = dict(time=time.strftime('%Y%m%d_%H%M%S'), source=src, joints_deg=q.round(2).tolist(), points=int(len(Q)),
               rim_fit_mad_mm=round(rim_mad * 1000, 1), normal_base=n.round(4).tolist(), centre_base=centre.round(4).tolist(),
               normal_flange=n_f.round(4).tolist(), centre_flange=c_f[:3].round(4).tolist(),
               target=f"{a.quadrant}:{a.pin}", centre_basket_mm=(cb * 1000).round(1).tolist(), at_slot=dict(yaw_deg=round(float(yaw), 1), pitch_vs_tines_deg=round(float(pitch), 1),
               centre_x_vs_gap_mm=round(float(cb[0] - gaps[tgt, 0]) * 1000, 1), rows=rows),
               seconds=dict(frame_and_joints=round(t_in, 2), measure=round(t_meas, 3)))
    out = ROOT / 'outputs/inhand_20261030'
    out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out / f'{res["time"]}_frame.npz', rgb=rgb, depth=depth, K=K, joints=q)
    (out / f'{res["time"]}.json').write_text(json.dumps(res, indent=1))
    vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if rgb.shape[2] == 3 else rgb
    vis = vis.copy()
    for u, v in UV[idx]:
        cv2.circle(vis, (int(u), int(v)), 1, (0, 255, 0), -1)
    cv2.imwrite(str(out / f'{res["time"]}_plate.png'), vis)
    print(json.dumps(res, indent=1))
    print(f'-> at {a.quadrant} pin {a.pin}: yaw {yaw:+.1f} deg, pitch vs tines {pitch:+.1f} deg; '
          f'rows L {rows["L"]["pin"] if rows["L"] else "-"} / R {rows["R"]["pin"] if rows["R"] else "-"}  (saved {out})')


if __name__ == '__main__':
    main()
