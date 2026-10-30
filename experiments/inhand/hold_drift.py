"""(보스 10/30) does the held plate turn in the hand while it is held still ("팅, 팅": the hand's force loop)?  Read-only:
the fixed camera's shared frames every --every s for --seconds, the joints at the start and the end (the arm must stand
still), the plate plane + centre as plate_inhand.py.  Prints each sample's face-normal turn and yaw-at-slot change vs the
first, and saves the series to outputs/inhand_20261030/drift_<tag>_<time>.json.
    ~/miniconda3/envs/foundationpose/bin/python experiments/inhand/hold_drift.py --tag lock --seconds 30
Run it while the arm stands at the lift top (demo_safe.sh ... --inhand_check --leg_enter: it waits there for Enter)."""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plate_inhand as PI  # noqa: E402


def measure(k, q, rgb, depth, K, Tbc):
    """-> (normal, centre) of the held plate in the flange frame, rim fit [mm], points; None if no plate."""
    F = k.fk(np.radians(q))
    X, _ = PI.cloud(depth, K, Tbc)
    r = PI.plate_plane(X, F[:3, 3], np.random.default_rng(0), keep=PI.off_arm(k, q, X))
    if r is None:
        return None
    Q, c, n, _ = r
    centre, mad = PI.circle_centre(Q, c, n)
    for _ in range(2):
        d = (Q - centre) @ n
        inside = np.linalg.norm((Q - centre) - d[:, None] * n, axis=1) < PI.R_PLATE + 0.01
        if inside.sum() < 100:
            break
        Q = Q[inside]
        c = Q.mean(0)
        n = np.linalg.svd(Q - c)[2][2]
        centre, mad = PI.circle_centre(Q, c, n)
    n_f = F[:3, :3].T @ n
    n_f *= np.sign(n_f[2]) if n_f[2] else 1.0
    return n_f, (np.linalg.inv(F) @ np.r_[centre, 1.0])[:3], mad, len(Q)


def main():
    a = argparse.ArgumentParser()
    a.add_argument('--tag', default='hold', help='a name for this run (e.g. lock / nolock)')
    a.add_argument('--seconds', type=float, default=30.0)
    a.add_argument('--every', type=float, default=2.0)
    a.add_argument('--quadrant', default='Q3')
    a.add_argument('--pin', type=int, default=6)
    a.add_argument('--joints', type=float, nargs=6, help='instead of reading the robot (offline check)')
    a = a.parse_args()
    import calib_core as cc
    import plate_plans as pp
    k = pp.Kinematics()
    Tbc = np.array(cc.load_json(cc.latest_t_base_cam())['T_base_cam'])
    q0 = np.array(a.joints) if a.joints else PI.joints()
    Tbb = np.array(json.loads(PI.BASKET.read_text())['T'])
    Rs = np.linalg.inv(Tbb)[:3, :3] @ k.fk(np.radians(PI.slot_pose(a.quadrant, a.pin, 'slot')))[:3, :3]
    sys.path.insert(0, str(PI.ROOT / 'tesollo'))
    from fixed_frames import FixedFrames
    cam = FixedFrames(pp.C['cameras']['fixed_serial'])
    rows, t0 = [], time.time()
    try:
        while time.time() - t0 < a.seconds:
            t1 = time.time()
            f = cam.read()
            m = measure(k, q0, np.asarray(f.rgb), np.asarray(f.depth, np.float32), np.asarray(f.K, float), Tbc)
            if m is not None:
                n_f, c_f, mad, npts = m
                nb = Rs @ n_f
                nb *= np.sign(nb[0]) if nb[0] else 1.0
                rows.append(dict(t=round(t1 - t0, 2), normal_flange=n_f.round(4).tolist(), centre_flange_mm=(c_f * 1000).round(1).tolist(),
                                 yaw_at_slot_deg=round(float(np.degrees(np.arctan2(nb[1], nb[0]))), 2), rim_fit_mad_mm=round(mad * 1000, 1),
                                 points=npts))
                r0 = rows[0]
                turn = np.degrees(np.arccos(np.clip(np.dot(n_f, r0['normal_flange']), -1, 1)))
                print(f'{t1 - t0:5.1f} s: face turned {turn:4.1f} deg, yaw at slot {rows[-1]["yaw_at_slot_deg"] - r0["yaw_at_slot_deg"]:+5.1f} '
                      f'deg vs the first, rim fit {mad * 1000:.1f} mm', flush=True)
            time.sleep(max(0.0, a.every - (time.time() - t1)))
    finally:
        cam.stop()
    q1 = np.array(a.joints) if a.joints else PI.joints()
    moved = float(np.abs(q1 - q0).max())
    if not rows:
        sys.exit('no plate measured')
    N = np.array([r['normal_flange'] for r in rows])
    turns = np.degrees(np.arccos(np.clip(N @ N[0], -1, 1)))
    yaws = np.array([r['yaw_at_slot_deg'] for r in rows])
    res = dict(tag=a.tag, time=time.strftime('%Y%m%d_%H%M%S'), joints_deg=q0.round(2).tolist(), arm_moved_deg=round(moved, 2),
               samples=len(rows), face_turn_max_deg=round(float(turns.max()), 1), yaw_range_deg=round(float(np.ptp(yaws)), 1),
               yaw_std_deg=round(float(yaws.std()), 2), series=rows)
    out = PI.ROOT / 'outputs/inhand_20261030' / f'drift_{a.tag}_{res["time"]}.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, indent=1))
    print(f'-> {a.tag}: {len(rows)} samples over {a.seconds:.0f} s, face turned up to {res["face_turn_max_deg"]} deg, yaw range '
          f'{res["yaw_range_deg"]} deg (std {res["yaw_std_deg"]}){"  !! the arm moved " + str(round(moved, 1)) + " deg" if moved > 0.5 else ""}; '
          f'saved {out}')


if __name__ == '__main__':
    main()
