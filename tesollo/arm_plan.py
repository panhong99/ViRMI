"""Arm plan for the Tesollo hand grasp (tesollo/arm_poses.yaml): home -> above (grasp pose + RISE m, same orientation) ->
straight down -> [hand: dg5fs_sdk.py --object] -> straight up -> home.  Checks joint limits, IK continuity, and the
lowest point of the hand (12 cm disc around 5-22 cm along the tool z) on every leg.  Writes a plan.json (dry run) for
the mover; plans expire after 30 min, so make it right before the run:
    ~/miniconda3/envs/foundationpose/bin/python tesollo/arm_plan.py plate|bowl [rise_m=0.12] [--dx M --dy M]
--dx/--dy (base frame, <= 0.15 m) move the taught grasp pose with the object (tesollo/perceive.py: measured object centre
minus the centre saved when the pose was taught); the hand orientation stays the taught one (round objects).
"""
import os, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'calibration'))
os.chdir(HERE.parent / 'calibration')   # calib_core / plate_plans read their files relative to calibration/
import numpy as np, yaml
import plate_plans as pp, calib_core as cc, ros2_nodes
import argparse
ap = argparse.ArgumentParser()
ap.add_argument('obj', choices=('plate', 'bowl'))
ap.add_argument('rise', type=float, nargs='?', default=0.12)
ap.add_argument('--dx', type=float, default=0.0)
ap.add_argument('--dy', type=float, default=0.0)
ap.add_argument('--max_shift', type=float, default=0.15, help='(보스 11/02, opt-in larger) the object may move this far [m]')
args = ap.parse_args()
obj, rise = args.obj, args.rise
assert np.hypot(args.dx, args.dy) <= args.max_shift, \
    f'object moved {np.hypot(args.dx, args.dy) * 100:.1f} cm from the taught spot (> {args.max_shift * 100:.0f})'
A = yaml.safe_load((HERE / 'arm_poses.yaml').read_text())[obj]
home = np.array(pp.C['robot']['home_joints_deg'])
k = pp.Kinematics()
lo, hi = np.array(ros2_nodes.joint_limits_deg()).T
grasp = np.array(A['posj_deg'])
if args.dx or args.dy:                 # the taught grasp, translated with the object
    T = k.fk(np.radians(grasp)); T[0, 3] += args.dx; T[1, 3] += args.dy
    q = k.ik(T, np.radians(grasp)); assert q is not None, 'IK failed for the shifted grasp pose'
    assert np.abs(k.fk(q)[:3, 3] - T[:3, 3]).max() < 1e-3
    grasp = np.degrees(q)
    assert np.abs(grasp - np.array(A['posj_deg'])).max() < 45, 'shifted grasp pose is far from the taught one in joints'
    print(f'shifted by dx {args.dx * 1000:+.0f} mm, dy {args.dy * 1000:+.0f} mm: grasp joints {grasp.round(2).tolist()}')
Tg = k.fk(np.radians(grasp))
def line(q0, dz, n=11):   # straight vertical line from the FK of q0 by dz, IK seeded step by step
    T0 = k.fk(np.radians(q0)); out, q = [q0], np.radians(q0)
    for s in np.linspace(0, 1, n)[1:]:
        T = T0.copy(); T[2, 3] += s * dz
        q = k.ik(T, q); assert q is not None, 'IK failed'
        assert np.abs(k.fk(q)[:3, 3] - T[:3, 3]).max() < 1e-3
        out.append(np.degrees(q))
    out = np.array(out)
    assert np.abs(np.diff(out, axis=0)).max() < 10, f'IK jump {np.abs(np.diff(out, axis=0)).max():.1f} deg'
    return out
up = line(grasp, rise)            # grasp -> above
above = up[-1]
for q in np.vstack([up, [home]]):
    assert ((q > lo + 3) & (q < hi - 3)).all(), f'joint limit {q.round(1)}'
def lowest(path):
    m = 9.0
    for a, b in zip(path[:-1], path[1:]):
        for s in np.linspace(0, 1, 11):
            T = k.fk(np.radians(a + s * (b - a)))
            for d in (0.05, 0.13, 0.22):
                c = T[:3, 3] + d * T[:3, 2]
                for t in np.linspace(0, 2 * np.pi, 24, endpoint=False):
                    m = min(m, (c + 0.12 * (np.cos(t) * T[:3, 0] + np.sin(t) * T[:3, 1]))[2])
    return m
# (보스 10/29) home -> above -> down as ONE spline: 2 joint-line knots on the way over (the spline otherwise swings 5 cm
# sideways into the descent) and a knot every 3 cm down; in the controller's spline model the hand stays within 1 mm
# of the vertical below 5 cm (4 mm at 8 cm) and passes 'above' without stopping
approach = [home + (above - home) * s for s in (1 / 3, 2 / 3)] + [above] + list(line(grasp, rise, n=5)[::-1][1:])
legs = {f'{obj}_above': [home, above], f'{obj}_down': list(up[::-1]), f'{obj}_lift': list(up), f'{obj}_home': [above, home],
        f'{obj}_approach': [home] + approach}
for nm, path in legs.items():
    print(f'{nm}: lowest hand point z {lowest(np.array(path)):+.3f} m, max joint change {np.abs(np.array(path[-1]) - path[0]).max():.1f} deg')
print('grasp flange', (Tg[:3, 3] * 1000).round(1), ' above flange', (k.fk(np.radians(above))[:3, 3] * 1000).round(1))
print('above joints', above.round(2).tolist())
Tbc = np.array(cc.load_json(cc.latest_t_base_cam())['T_base_cam'])
print('plan:', pp.taught_plan({k_: [list(map(float, q)) for q in v] for k_, v in legs.items()}, Tbc, pp.C['plate_ik_bc'],
                              f'tesollo hand grasp {obj} dx {args.dx:+.3f} dy {args.dy:+.3f}'))
