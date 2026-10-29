"""Tesollo plate: the post-grasp path from a pendant-taught insert (tesollo/taught_inserts.json, dg5fs_sdk.py --teach).

보스 10/29: teach 1 = over the basket, teach 2 = 1 cm over the slot, teach 3 = in the slot.  Grasp is the last stop;
after it few knots, no collision sweep (보스: nothing to hit, 1 cm waypoints + per-point FK checks banned):
    post_grasp:  grasp -> up 5 cm -> up --rise -> teach 1 (over the slot) -> teach 2 (1 cm over it)   ONE spline
    insert:      teach 2 -> teach 3, the only slow part (보스: 40 deg/s just before the insert, dg5fs_sdk --insert_vel)
    return_home: teach 3 -> teach 2 -> teach 1 -> home   (after the release, one spline)
Knots are not split into steps (the controller's spline smooths them; the mover gets --max_step_deg 90).  The taught poses follow the basket measured now (flange pose in the basket frame, IK) and another
pin of the taught quadrant (shifted by the gap spacing along the pin row); within 3 mm / 0.3 deg the joints as taught.
    ~/miniconda3/envs/foundationpose/bin/python tesollo/taught_plan.py [--index -1] [--basket X.json] [--pin 12]
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
from basket_plan import shifted_grasp

TAUGHT = HERE / 'taught_inserts.json'
MAX_KNOTS, KNOT_DEG = 20, 30.0


def knots(qs, step=KNOT_DEG):
    """Joint-line knots through qs, no joint more than `step` deg between two knots."""
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
    p.add_argument('--quadrant', help='must be the taught one (default it)')
    p.add_argument('--pin', type=int, help='another pin of the taught quadrant (default the taught pin)')
    p.add_argument('--dx', type=float, default=0.0)
    p.add_argument('--dy', type=float, default=0.0)
    p.add_argument('--rise', type=float, default=0.20, help='straight lift after the grasp before the carry [m] (보스 20 cm)')
    a = p.parse_args()
    e = json.loads(TAUGHT.read_text())[a.index]
    pts = e['waypoints_deg']
    if len(pts) < 2:
        sys.exit(f'taught entry {a.index} has {len(pts)} points (need over the basket ... in the slot)')
    if a.quadrant and a.quadrant != e['quadrant']:
        sys.exit(f'REFUSED: only {e["quadrant"]} is taught (asked {a.quadrant})')
    k = pp.Kinematics()
    A = yaml.safe_load((HERE / 'arm_poses.yaml').read_text())['plate']
    grasp = np.array(shifted_grasp(k, A['posj_deg'], a.dx, a.dy), float)
    home = list(map(float, pp.C['robot']['home_joints_deg']))
    Tt = np.array(e['basket_T'])
    Tn = np.array(json.loads(Path(a.basket).read_text())['T']) if a.basket else Tt
    D = Tn @ np.linalg.inv(Tt)                                   # the basket since the teaching
    pin = a.pin or e['pin']
    if pin != e['pin']:                                          # the same quadrant, another gap: along the pin row
        mq = pp.robot_quadrants(Tt)[e['quadrant']]
        g = pp.basket_gaps()[mq]
        d_b = g[pp.pin_gap(Tt, mq, pin)] - g[pp.pin_gap(Tt, mq, e['pin'])]
        S = np.eye(4)
        S[:3, 3] = Tt[:3, :3] @ d_b
        D = D @ S
    dmm = np.linalg.norm(D[:3, 3]) * 1000
    ddeg = np.degrees(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))
    if dmm > 3 or ddeg > 0.3:
        moved, prev = [], np.radians(pts[0])
        for q in pts:
            T = D @ k.fk(np.radians(q))
            sol = k.ik(T, prev)
            if sol is None or np.abs(k.fk(sol)[:3, 3] - T[:3, 3]).max() > 1e-3:
                sys.exit(f'REFUSED: the taught pose {q} has no IK moved {dmm:.0f} mm / {ddeg:.1f} deg')
            moved.append(np.degrees(sol).tolist())
            prev = sol
        if np.abs(np.diff(np.array(moved), axis=0)).max() > 30:
            sys.exit('REFUSED: the moved taught poses jump (wrist flip)')
        pts = moved
        note = f'taught poses moved {dmm:.0f} mm / {ddeg:.1f} deg (basket now' + (f', pin {e["pin"]} -> {pin})' if pin != e['pin'] else ')')
    else:
        note = f'joints as taught (basket within {dmm:.1f} mm / {ddeg:.2f} deg)'
    T = k.fk(np.radians(grasp))                                  # the lift: straight up --rise; a knot at 5 cm keeps the
    T5 = T.copy()                                                # spline's first part vertical off the stand (10/29: with
    T5[2, 3] += 0.05                                             # only the top knot it swung 14 mm sideways at 2 cm, 26 mm
    T[2, 3] += a.rise                                            # at 5 cm; with it 9 / 2 mm)
    low = np.degrees(k.ik(T5, np.radians(grasp)))
    up = np.degrees(k.ik(T, np.radians(low)))
    post = [grasp.tolist(), low.tolist(), up.tolist()] + [list(map(float, q)) for q in pts[:-1]]
    ins = [list(map(float, q)) for q in pts[-2:]]
    back = [list(map(float, q)) for q in [pts[-1]] + pts[-2::-1] + [home]]
    for name, w in (('post_grasp', post), ('insert', ins), ('return_home', back)):
        if len(w) > MAX_KNOTS:
            sys.exit(f'REFUSED: {name} needs {len(w)} knots (> {MAX_KNOTS})')
        W = np.array(w)
        if np.abs(W[:, 4]).max() > 132 or np.abs(W[:, 2]).max() > 157:
            sys.exit(f'REFUSED: {name} near a joint limit')
    # report only (no gate): the flange over the basket on the carry, basket frame (rim = 0)
    over = [(np.linalg.inv(Tn) @ k.fk(np.radians(q))[:, 3])[:3] for q in pp.equal_time_spline(np.array(post[:4]), per=40)[0]]
    inside = [b[2] for b in over if abs(b[0]) < 0.30 and abs(b[1]) < 0.30]
    plan = dict(dry_run=True, accepted=True, reasons=[], margin_m=0.0, created=time.strftime('%Y-%m-%d %H:%M:%S'),
                source=f'taught insert #{a.index} {e["quadrant"]} pin {pin} (taught pin {e["pin"]}, {e["time"]}), {note}; '
                       'sparse, no collision sweep', basket_T=Tn.tolist(), plate_in_tcp=e.get('plate_in_tcp'), tool_length_m=0.12,
                legs={n: dict(waypoints_deg=np.round(w, 3).tolist(), min_clearance_straight_m=0.0, planner='taught sparse')
                      for n, w in (('to_grasp', [home, grasp.tolist()]), ('post_grasp', post), ('insert', ins),
                                   ('return_home', back))})
    out = pp.DATA / 'rl_plans' / (time.strftime('%Y%m%d_%H%M%S') + f'_{int(time.time() * 1000) % 1000:03d}_hand_taught')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'plan.json').write_text(json.dumps(plan, indent=1))
    print(f'basket: {note}')
    print(f'OK: taught {e["quadrant"]} pin {pin}: post_grasp {len(post)} knots (grasp, up 5 / {a.rise * 100:.0f} cm, teach 1, '
          f'teach 2), insert 2 knots, return_home {len(back)} knots; carry flange over the basket >= '
          f'{min(inside) * 100 if inside else float("nan"):.0f} cm over the rim (TCP not checked)')
    for n in ('post_grasp', 'insert', 'return_home'):
        print(f'  {n}: {len(plan["legs"][n]["waypoints_deg"])} waypoints, end {plan["legs"][n]["waypoints_deg"][-1]}')
    print('plan:', out / 'plan.json')


if __name__ == '__main__':
    main()
