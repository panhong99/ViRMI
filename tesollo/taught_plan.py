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
OBSTACLE_MARGIN_M = 0.015


def knots(qs, step=KNOT_DEG):
    """Joint-line knots through qs, no joint more than `step` deg between two knots."""
    out = [list(map(float, qs[0]))]
    for a, b in zip(qs[:-1], qs[1:]):
        a, b = np.array(a, float), np.array(b, float)
        n = max(1, int(np.ceil(np.abs(b - a).max() / step)))
        out += [(a + (b - a) * i / n).tolist() for i in range(1, n + 1)]
    return out


def live_obstacle_clearance(k, waypoints, plate_in_tcp, obstacles):
    """Dense controller-spline samples of the held plate against live camera OBBs."""
    if not obstacles:
        return []
    qs = pp.equal_time_spline(np.asarray(waypoints, float), per=30)[0]
    local = pp.plate_cloud(np.asarray(plate_in_tcp, float), np.eye(4))
    points = np.vstack([local @ F[:3, :3].T + F[:3, 3] for F in (k.fk(np.radians(q)) for q in qs)])
    clear = []
    for _, c, size, R in (o[:4] for o in obstacles):
        L = np.abs((points - np.asarray(c, float)) @ np.asarray(R, float)) - np.asarray(size, float) / 2
        d = np.linalg.norm(np.maximum(L, 0.0), axis=1) + np.minimum(L.max(1), 0.0)
        clear.append(float(d.min()))
    return clear


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--index', type=int, default=-1, help='taught_inserts.json entry (default the last)')
    p.add_argument('--basket', help='basket_vision.json measured now (T); default: as at the teaching')
    p.add_argument('--quadrant', help='must be the taught one (default it)')
    p.add_argument('--pin', type=int, help='another pin of the taught quadrant (default the taught pin)')
    p.add_argument('--dx', type=float, default=0.0)
    p.add_argument('--dy', type=float, default=0.0)
    p.add_argument('--rise', type=float, default=0.30, help='straight lift after the grasp before the carry [m] (보스 10/29: 30 cm)')
    p.add_argument('--smooth', action='store_true', help='(보스 10/29: the moves looked stepwise) also write carry_insert = grasp -> '
                   'in the slot as ONE spline with even knots (~--knot_deg apart; the lift a Cartesian knot every 5 cm), slowing '
                   'into the slot by closer knots, and an even return_home')
    p.add_argument('--knot_deg', type=float, default=5.0, help='--smooth: joint step between knots [deg]')
    p.add_argument('--wrist_less', type=float, default=25.0, help='turn the hand this much less (grasp -> insert) than taught, '
                   'about the held plate\'s lowest point [deg] (보스 10/29: 25 less is enough for the slot; 0 = as taught)')
    p.add_argument('--pivot', choices=('model', 'slot'), default='model', help='(보스 10/30, opt-in slot) --wrist_less pivot: '
                   'model = the planner model plate\'s lowest point (179 mm off the real plate bottom: the 25 deg turn moved '
                   'the plate 69 mm along the row, 4 gaps - the L1 / L2 landings); slot = where the plate bottom was at the '
                   'teaching (the taught gap on the floor), carried with the flange')
    p.add_argument('--slot_dx', type=float, default=0.0, help='(보스 10/30, opt-in) move the taught insert this much [mm] along the '
                   'basket x (+ = toward higher-x gaps, ~17.4 mm a gap)')
    p.add_argument('--slot_dy', type=float, default=0.0, help='(보스 10/30, opt-in) move the taught insert this much [mm] along the '
                   'basket y (+ = toward the L row, - = R)')
    p.add_argument('--normal_flange', type=float, nargs=3, help='--yaw_corr: the held plate\'s face normal in the flange frame '
                   '(plate_inhand.py normal_flange)')
    p.add_argument('--yaw_corr', type=float, default=0.0, help='(보스 10/30, opt-in, needs --pivot slot) turn the held plate '
                   'this much [deg] about the basket\'s vertical through the plate bottom at every taught pose (the in-hand '
                   'yaw measured after the lift, negated: plate_inhand.py)')
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
    basket_data = json.loads(Path(a.basket).read_text()) if a.basket else {}
    Tn = np.array(basket_data['T']) if basket_data else Tt
    live_obstacles = [o[:4] for o in basket_data.get('basket_obstacles', [])]
    D = Tn @ np.linalg.inv(Tt)                                   # the basket since the teaching
    pin = a.pin or e['pin']
    if pin != e['pin']:                                          # the same quadrant, another gap: along the pin row
        mq = pp.robot_quadrants(Tt)[e['quadrant']]
        g = pp.basket_gaps()[mq]
        d_b = g[pp.pin_gap(Tt, mq, pin)] - g[pp.pin_gap(Tt, mq, e['pin'])]
        S = np.eye(4)
        S[:3, 3] = Tt[:3, :3] @ d_b
        D = D @ S
    if a.slot_dx or a.slot_dy:                                   # (보스 10/30: "a little to the left") the whole insert moved
        S = np.eye(4)
        S[:3, 3] = Tt[:3, :3] @ np.array([a.slot_dx, a.slot_dy, 0.0]) / 1000
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
    if a.wrist_less:
        # (보스 10/29) the taught insert leans the hand back more than the slot needs: each taught pose turned back by
        # --wrist_less about the wrist pitch axis (J5 at the slot, toward the grasp), pivot = the model plate's lowest point
        # at that pose (the plate bottom stays where it was taught).  Solved with J4 / J6 kept as taught (J1-J3, J5 free):
        # a full IK lands near J5 = 0 and flipped J4 / J6 by 40-80 deg
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
        cloud = pp.plate_cloud(np.asarray(e['plate_in_tcp'], float), np.eye(4))
        if a.pivot == 'slot':                                    # the plate bottom at the teaching, in the flange frame
            mq_t = pp.robot_quadrants(Tt)[e['quadrant']]
            g_t = pp.basket_gaps()[mq_t][pp.pin_gap(Tt, mq_t, e['pin'])]
            F_t = k.fk(np.radians(e['waypoints_deg'][-1]))
            p_slot = (np.linalg.inv(F_t) @ Tt @ np.r_[g_t[0], g_t[1], g_t[2], 1.0])[:3]
        q3 = np.radians(pts[-1])
        d = q3.copy()
        d[4] -= np.radians(1.0)
        axis = Rotation.from_matrix(k.fk(d)[:3, :3] @ k.fk(q3)[:3, :3].T).as_rotvec()
        Rb = Rotation.from_rotvec(axis / np.linalg.norm(axis) * np.radians(a.wrist_less)).as_matrix()
        turned = []
        for i_q, q in enumerate(pts):
            q0 = np.radians(q)
            F = k.fk(q0)
            if a.pivot == 'slot':
                p_f = p_slot
                piv = F[:3, :3] @ p_f + F[:3, 3]
            else:
                low = cloud @ F[:3, :3].T + F[:3, 3]
                piv = low[np.argmin(low[:, 2])]
                p_f = F[:3, :3].T @ (piv - F[:3, 3])             # the pivot in the flange frame
            Rt = Rb @ F[:3, :3]

            def res(x, q0=q0, p_f=p_f, piv=piv, Rt=Rt):
                qq = q0.copy()
                qq[[0, 1, 2, 4]] = x
                G = k.fk(qq)
                return np.r_[(G[:3, :3] @ p_f + G[:3, 3] - piv) * 100, Rotation.from_matrix(Rt.T @ G[:3, :3]).as_rotvec()]
            x = least_squares(res, q0[[0, 1, 2, 4]]).x
            r = res(x)
            # (10/29 real: pin 12 at a basket measured 0.9 deg turned: over the basket 6.1 deg off) only the slot poses
            # need the exact turn; over the basket (nothing to touch) 15 deg
            tol = 6 if i_q >= len(pts) - 2 else 15
            if np.linalg.norm(r[:3]) > 0.1 or np.degrees(np.linalg.norm(r[3:])) > tol:
                sys.exit(f'REFUSED: the wrist turned back {a.wrist_less} deg at {q}: plate bottom off {np.linalg.norm(r[:3]) * 10:.1f} mm, '
                         f'orientation off {np.degrees(np.linalg.norm(r[3:])):.1f} deg')
            qq = q0.copy()
            qq[[0, 1, 2, 4]] = x
            turned.append(np.degrees(qq).tolist())
        print(f'wrist: {a.wrist_less:.0f} deg less than taught (J5 at the slot {pts[-1][4]:+.1f} -> {turned[-1][4]:+.1f}, '
              'J4 / J6 as taught, plate bottom on the taught spot)')
        note += f', wrist {-a.wrist_less:+.0f} deg'
        pts = turned
    if a.yaw_corr:
        # (보스 10/30) the held plate turned about the basket's vertical through its bottom.  The wrist sits near J5 = 0
        # here: a full IK swings J4 / J6 by 60-85 deg.  A round plate only needs its bottom point and its face normal
        # (5 numbers): J1-J3, J5, J6 solve them with J4 as taught (J6 ~ 1.1 deg per deg of yaw)
        if not (a.wrist_less and a.pivot == 'slot' and a.normal_flange):
            sys.exit('--yaw_corr needs --pivot slot, --wrist_less and --normal_flange (plate_inhand.py)')
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
        nf = np.asarray(a.normal_flange, float) / np.linalg.norm(a.normal_flange)
        Rz = Rotation.from_rotvec(Tn[:3, 2] * np.radians(a.yaw_corr)).as_matrix()
        yawed = []
        for q in pts:
            q0 = np.radians(q)
            F = k.fk(q0)
            piv = F[:3, :3] @ p_slot + F[:3, 3]
            n_t = Rz @ (F[:3, :3] @ nf)

            def res(x, q0=q0, piv=piv, n_t=n_t):
                qq = q0.copy()
                qq[[0, 1, 2, 4, 5]] = x
                G = k.fk(qq)
                return np.r_[(G[:3, :3] @ p_slot + G[:3, 3] - piv) * 100, (G[:3, :3] @ nf - n_t) * 10]
            x = least_squares(res, q0[[0, 1, 2, 4, 5]]).x
            r = res(x)
            if np.linalg.norm(r[:3]) > 0.1 or np.degrees(np.linalg.norm(r[3:]) / 10) > 1.0:
                sys.exit(f'REFUSED: yaw {a.yaw_corr:+.1f} deg at {np.round(q, 1).tolist()}: plate bottom off '
                         f'{np.linalg.norm(r[:3]) * 10:.1f} mm, face off {np.degrees(np.linalg.norm(r[3:]) / 10):.1f} deg')
            qq = q0.copy()
            qq[[0, 1, 2, 4, 5]] = x
            yawed.append(np.degrees(qq).tolist())
        print(f'yaw: the plate turned {a.yaw_corr:+.1f} deg about the vertical through its bottom (J4 as taught; J6 at the '
              f'slot {pts[-1][5]:+.1f} -> {yawed[-1][5]:+.1f}, J5 {pts[-1][4]:+.1f} -> {yawed[-1][4]:+.1f})')
        note += f', yaw {a.yaw_corr:+.1f} deg'
        pts = yawed
    T = k.fk(np.radians(grasp))                                  # the lift: straight up --rise; a knot at 5 cm keeps the
    T5 = T.copy()                                                # spline's first part vertical off the stand (10/29: with
    T5[2, 3] += 0.05                                             # only the top knot it swung 14 mm sideways at 2 cm, 26 mm
    T[2, 3] += a.rise                                            # at 5 cm; with it 9 / 2 mm)
    low = np.degrees(k.ik(T5, np.radians(grasp)))
    up = np.degrees(k.ik(T, np.radians(low)))
    post = [grasp.tolist(), low.tolist(), up.tolist()] + [list(map(float, q)) for q in pts[:-1]]
    a2, a3 = np.array(pts[-2], float), np.array(pts[-1], float)
    ins = [a2.tolist(), ((a2 + a3) / 2).tolist(), a3.tolist()]   # the mover needs >= 2 targets after the start (10/29 real:
                                                                 # 2 knots were refused and the plate stopped 1 cm over the slot)
    back = [list(map(float, q)) for q in [pts[-1]] + pts[-2::-1] + [home]]
    legs = [('to_grasp', [home, grasp.tolist()]), ('post_grasp', post), ('insert', ins), ('return_home', back)]
    if a.smooth:
        # (보스 10/29) one spline from the grasp into the slot.  The controller spline gives every knot the same time, so
        # knots ~knot_deg apart keep the speed even (the old 4 / 24 / 27 / 20 deg gaps sped up and slowed 6x inside one
        # leg); the lift keeps a Cartesian knot every 5 cm (vertical); into the slot the knots close up -> it slows down
        # there by itself (no stop at 1 cm over the slot, no second mover start)
        lift, q = [grasp.tolist()], np.radians(grasp)
        T0 = k.fk(np.radians(grasp))
        for dz in np.arange(0.05, a.rise + 1e-6, 0.05):
            T = T0.copy()
            T[2, 3] += dz
            q = k.ik(T, q)
            lift.append(np.degrees(q).tolist())
        step = a.knot_deg                                        # (10/29 sweep: pin 12 + plate 5 cm left -> 21 knots)
        while True:                                              # wider steps until the leg fits MAX_KNOTS
            carry = lift + knots([lift[-1]] + [list(map(float, q_)) for q_ in pts[:-1]], step)[1:]
            if len(carry) + 2 <= MAX_KNOTS:
                break
            step *= 1.15
        n_in = 2                                                 # the last ~1 cm: 2 short steps (~half the carry speed)
        carry += [(a2 + (a3 - a2) * i / n_in).tolist() for i in range(1, n_in + 1)]
        back_s = knots(back, a.knot_deg * 1.6)
        legs += [('carry_insert', carry), ('return_home_smooth', back_s)]
    checked_leg = dict(legs)['carry_insert' if a.smooth else 'post_grasp']
    clear = live_obstacle_clearance(k, checked_leg, e['plate_in_tcp'], live_obstacles)
    if clear and min(clear) < OBSTACLE_MARGIN_M:
        hit = int(np.argmin(clear))
        sys.exit(f'REFUSED: live basket obstacle {hit + 1} is only {clear[hit] * 1000:.1f} mm from the held plate '
                 f'(< {OBSTACLE_MARGIN_M * 1000:.0f} mm)')
    for name, w in legs[1:]:
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
                       'sparse, live placed-plate check only', basket_T=Tn.tolist(), plate_in_tcp=e.get('plate_in_tcp'),
                tool_length_m=0.12, obstacles=live_obstacles,
                obstacle_check=dict(count=len(live_obstacles), margin_m=OBSTACLE_MARGIN_M,
                                    min_clearance_m=min(clear) if clear else None),
                legs={n: dict(waypoints_deg=np.round(w, 3).tolist(), min_clearance_straight_m=0.0, planner='taught sparse')
                      for n, w in legs})
    out = pp.DATA / 'rl_plans' / (time.strftime('%Y%m%d_%H%M%S') + f'_{int(time.time() * 1000) % 1000:03d}_hand_taught')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'plan.json').write_text(json.dumps(plan, indent=1))
    print(f'basket: {note}')
    if live_obstacles:
        print(f'live basket obstacles: {len(live_obstacles)}, held-plate clearance {min(clear) * 1000:.1f} mm')
    print(f'OK: taught {e["quadrant"]} pin {pin}: post_grasp {len(post)} knots (grasp, up 5 / {a.rise * 100:.0f} cm, teach 1, '
          f'teach 2), insert 3 knots, return_home {len(back)} knots; carry flange over the basket >= '
          f'{min(inside) * 100 if inside else float("nan"):.0f} cm over the rim (TCP not checked)')
    for n in [n_ for n_, _ in legs[1:]]:
        print(f'  {n}: {len(plan["legs"][n]["waypoints_deg"])} waypoints, end {plan["legs"][n]["waypoints_deg"][-1]}')
    print('plan:', out / 'plan.json')


if __name__ == '__main__':
    main()
