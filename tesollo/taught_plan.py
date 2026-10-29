"""Tesollo plate: the basket legs from a pendant-taught insert (tesollo/taught_inserts.json, dg5fs_sdk.py --teach).

보스 10/29: teach 1 = over the basket, teach 2 = down to 1 cm over the slot, teach 3 = in the slot.  After the grasp:
    lift (tesollo arm_plan lift)  ->  to_above: lift end -> teach 1 (joint line)          [merged: lift_carry]
    descend: teach 1 -> teach 2 -> teach 3      ascend: teach 3 -> teach 2 -> teach 1      to_home: teach 1 -> home
Joint lines in <= 2 deg steps.  If the basket measured now (--basket) is away from the one at teaching, every taught pose
is moved with it (flange pose in the basket frame, IK from the taught joints); within 3 mm / 0.3 deg the joints as taught.
    ~/miniconda3/envs/foundationpose/bin/python tesollo/taught_plan.py [--index -1] [--basket /tmp/tesollo_basket_X.json]
"""
import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'calibration'))
sys.path.insert(0, str(HERE))
import numpy as np
import yaml
import plate_plans as pp
from basket_plan import lift_line, merge_lift_carry, shifted_grasp

TAUGHT = HERE / 'taught_inserts.json'


def line(qs, step=2.0):
    """Joint lines through qs, <= step deg per move."""
    out = [list(map(float, qs[0]))]
    for a, b in zip(qs[:-1], qs[1:]):
        a, b = np.array(a, float), np.array(b, float)
        n = max(1, int(np.ceil(np.abs(b - a).max() / step)))
        out += [(a + (b - a) * i / n).tolist() for i in range(1, n + 1)]
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--index', type=int, default=-1, help='taught_inserts.json entry (default the last)')
    p.add_argument('--basket', help='basket_vision.json measured now (T); default: as at the teaching')
    p.add_argument('--dx', type=float, default=0.0)
    p.add_argument('--dy', type=float, default=0.0)
    p.add_argument('--rise', type=float, default=0.12)
    a = p.parse_args()
    e = json.loads(TAUGHT.read_text())[a.index]
    pts = e['waypoints_deg']
    if len(pts) < 2:
        sys.exit(f'taught entry {a.index} has {len(pts)} points (need over the basket ... in the slot)')
    k = pp.Kinematics()
    A = yaml.safe_load((HERE / 'arm_poses.yaml').read_text())['plate']
    grasp = shifted_grasp(k, A['posj_deg'], a.dx, a.dy)
    lift = [list(map(float, w)) for w in lift_line(k, grasp, a.rise)]
    home = list(map(float, pp.C['robot']['home_joints_deg']))
    Tt = np.array(e['basket_T'])
    note = 'basket as taught'
    if a.basket:
        Tn = np.array(json.loads(Path(a.basket).read_text())['T'])
        D = Tn @ np.linalg.inv(Tt)
        dmm = np.linalg.norm(D[:3, 3]) * 1000
        ddeg = np.degrees(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))
        if dmm > 3 or ddeg > 0.3:
            moved, prev = [], np.radians(pts[0])
            for q in pts:
                T = D @ k.fk(np.radians(q))
                sol = k.ik(T, prev)
                if sol is None or np.abs(k.fk(sol)[:3, 3] - T[:3, 3]).max() > 1e-3:
                    sys.exit(f'REFUSED: the taught pose {q} has no IK with the basket moved {dmm:.0f} mm / {ddeg:.1f} deg')
                moved.append(np.degrees(sol).tolist())
                prev = sol
            if np.abs(np.diff(np.array(moved), axis=0)).max() > 30:
                sys.exit('REFUSED: the moved taught poses jump (wrist flip)')
            pts = moved
            note = f'taught poses moved with the basket ({dmm:.0f} mm, {ddeg:.1f} deg)'
        else:
            note = f'basket within {dmm:.1f} mm / {ddeg:.2f} deg of the teaching: joints as taught'
    above, ins = pts[0], pts[-1]
    legs = dict(to_grasp=[home, list(map(float, grasp))], lift=lift, to_above=line([lift[-1], above]),
                descend=line(pts), ascend=line(pts[::-1]), to_home=line([above, home]))
    for name, w in legs.items():
        W = np.array(w)
        if np.abs(W[:, 4]).max() > 132 or np.abs(W[:, 2]).max() > 157:
            sys.exit(f'REFUSED: {name} near a joint limit')
    # the carry over the basket footprint: the flange >= 30 cm over the rim (the hand + hanging plate are ~25 cm long)
    Tb = np.array(json.loads(Path(a.basket).read_text())['T']) if a.basket else Tt
    over = [(np.linalg.inv(Tb) @ k.fk(np.radians(q))[:, 3])[:3] for q in legs['to_above']]
    inside = [b[2] for b in over if abs(b[0]) < 0.30 and abs(b[1]) < 0.30]
    if inside and min(inside) < 0.30:
        sys.exit(f'REFUSED: the carry passes {min(inside) * 100:.0f} cm over the basket rim (flange) - teach the first pose higher')
    print(f'carry over the basket: flange >= {min(inside) * 100 if inside else float("nan"):.0f} cm over the rim', flush=True)
    plan = dict(dry_run=True, accepted=True, reasons=[], margin_m=0.0, created=time.strftime('%Y-%m-%d %H:%M:%S'),
                source=f'taught insert #{a.index} {e["quadrant"]} pin {e["pin"]} ({e["time"]}), {note}',
                basket_T=e['basket_T'], plate_in_tcp=e.get('plate_in_tcp'),
                legs={n: dict(waypoints_deg=w, min_clearance_straight_m=0.0, planner='taught') for n, w in legs.items()})
    out = pp.DATA / 'rl_plans' / (time.strftime('%Y%m%d_%H%M%S') + '_hand_taught')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'plan.json').write_text(json.dumps(plan, indent=1))
    print(f'basket: {note}')
    print(f'OK: taught {e["quadrant"]} pin {e["pin"]}: over the basket {np.round(above, 1).tolist()}, '
          f'in the slot {np.round(ins, 1).tolist()} ({len(pts)} taught poses)')
    for n, w in legs.items():
        print(f'  {n}: {len(w)} waypoints, end {[round(x, 1) for x in w[-1]]}')
    merge_lift_carry(out / 'plan.json')
    print('plan:', out / 'plan.json')


if __name__ == '__main__':
    main()
