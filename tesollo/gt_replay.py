"""(보스 11/02) the first real success replayed at another pin: the GT insert (tesollo/gt_inserts/*.json, 11/02 10:08 Q3 pin 6
-> L6/R6) moved along the pin row by the gap spacing (+ the basket's own move since the GT), with the same flange path and
plate angles, plus small hand-tuned residuals for this plate (--dx_mm / --dy_mm along / across the row, --dyaw_deg about the
vertical, --dpitch_deg about the row's horizontal; the turns about the GT plate's lowest rim at the slot, ramped in over the
carry, full in the insert).  Joints by the same J4-held continuation IK as tesollo/policy_insert.py.  Refused (exit 2, the
caller keeps its plan) when there is no IK, a joint leaves the URDF limits or jumps > --max_step_deg between 2 mm knots.
No robot I/O.  Writes a copy of the plan with the legs policy_carry / policy_insert / policy_return (as policy_insert.py) and
prints 'plan: <path>'.
    ~/miniconda3/envs/foundationpose/bin/python tesollo/gt_replay.py --plan PLAN.json --q_top J1..J6 --pin 12 \
        [--dyaw_deg 0 --dpitch_deg 0 --dx_mm 0 --dy_mm 0]"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / 'calibration'))
sys.path.insert(0, str(HERE))
GT = HERE / 'gt_inserts/Q3_pin6_20261102_1008.json'


def rows(F, H, Tbb, gaps, away):
    """where the plate's rim crosses the L / R tine rows at flange pose F -> [L, R] in pins (fractional; plate_inhand.py)"""
    X = np.linalg.inv(Tbb) @ F @ H
    th = np.linspace(0, 2 * np.pi, 2880, endpoint=False)
    rim = (X @ np.c_[0.134 * np.cos(th), 0.134 * np.sin(th), 0 * th, 1 + 0 * th].T).T[:, :3]
    ng, out = len(gaps), []
    order = np.argsort(gaps[:, 0])
    for y in (gaps[0, 1] + 0.0475, gaps[0, 1] - 0.0475):
        s = np.abs(rim[:, 1] - y) < 0.003
        j = np.argmin(np.where(s, rim[:, 2], 9))
        g = float(np.interp(rim[j, 0], gaps[order, 0], order.astype(float)))
        out.append(round(ng - g if away else g + 1, 2))
    return out


def main():
    from scipy.spatial.transform import Rotation as R
    import plate_plans as pp
    import policy_insert as pi
    from taught_plan import knots, KNOT_DEG, MAX_KNOTS
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--plan', required=True, help='this run\'s taught_plan.py plan json (its return_home is kept)')
    a.add_argument('--q_top', type=float, nargs=6, required=True, help='the lift-top joints [deg] (where the arm waits)')
    a.add_argument('--quadrant', default='Q3')
    a.add_argument('--pin', type=int, required=True)
    a.add_argument('--gt', default=str(GT))
    a.add_argument('--basket', default=str(HERE / 'demo_safe/basket_fixed.json'))
    a.add_argument('--dx_mm', type=float, default=0.0, help='+ = along the basket x (the pin row)')
    a.add_argument('--dy_mm', type=float, default=0.0, help='+ = basket y (toward the L row)')
    a.add_argument('--dyaw_deg', type=float, default=0.0, help='turn about the vertical (+ = counter-clockwise from above)')
    a.add_argument('--dpitch_deg', type=float, default=0.0, help='turn about the basket y (the face leans more / less)')
    a.add_argument('--max_step_deg', type=float, default=8.0, help='largest joint change between 2 mm knots')
    a = a.parse_args()
    t0 = time.time()
    k = pp.Kinematics()
    lo, hi = np.array(k.limits_deg()).T
    gt = json.loads(Path(a.gt).read_text())
    if gt['quadrant'] != a.quadrant:
        print(f'REFUSED: the GT is {gt["quadrant"]} (asked {a.quadrant})')
        sys.exit(2)
    plan = json.loads(Path(a.plan).read_text())
    carry_gt, ins_gt = gt['legs']['policy_carry'][1:], gt['legs']['policy_insert']
    W = np.array(carry_gt + ins_gt[1:], float)                # GT knots after its lift top
    qs, F, idx = pi.dense_path(k, W)
    i_t2 = idx[len(carry_gt) - 1]                              # the GT insert start (1 cm over the slot)
    Tg, Tbb = np.array(gt['basket_T']), np.array(json.loads(Path(a.basket).read_text())['T'])
    model_qd = pp.robot_quadrants(Tbb)[a.quadrant]
    gaps = np.array(pp.basket_gaps()[model_qd])
    away = (Tbb[:3, 0] @ np.r_[Tbb[:2, 3], 0.0]) >= 0
    gap, gap_gt = pp.pin_gap(Tbb, model_qd, a.pin), pp.pin_gap(Tg, pp.robot_quadrants(Tg)[a.quadrant], gt['pin'])
    H, _ = pi.measured_h(np.load(pi.JOB)['H_nom'], gt['inhand']['normal_flange'], F[-1], Tg)
    move = Tbb @ np.linalg.inv(Tg)                             # the basket since the GT
    shift = Tbb[:3, :3] @ (gaps[gap] - gaps[gap_gt] + np.r_[a.dx_mm, a.dy_mm, 0.0] / 1000)
    end = move @ F[-1]
    end[:3, 3] += shift
    _, piv_b = pi.lowest_rim(end, H, Tbb)                      # the turns' pivot: the plate bottom at the slot
    piv = (Tbb @ np.r_[piv_b, 1.0])[:3]
    targets = []
    for i in range(len(F)):
        T_ = move @ F[i]
        T_[:3, 3] += shift
        s = min(1.0, i / max(1, i_t2))
        Rt = R.from_rotvec(Tbb[:3, 2] * math.radians(a.dyaw_deg * s)).as_matrix() \
            @ R.from_rotvec(Tbb[:3, 1] * math.radians(a.dpitch_deg * s)).as_matrix()
        C = T_ @ H
        C[:3, :3] = Rt @ C[:3, :3]
        C[:3, 3] = piv + Rt @ (C[:3, 3] - piv)
        targets.append(C @ np.linalg.inv(H))
    Q, why = pi.follow_ik(k, targets, qs, H)
    if Q is None:
        print(f'REFUSED: {why}')
        sys.exit(2)
    if (Q < lo).any() or (Q > hi).any():
        print(f'REFUSED: outside the URDF joint limits (margin {min((Q - lo).min(), (hi - Q).min()):.1f} deg)')
        sys.exit(2)
    steps = np.abs(np.diff(Q, axis=0)).max(1)
    if steps.max() > a.max_step_deg:
        print(f'REFUSED: a joint jumps {steps.max():.1f} deg between 2 mm knots (knot {int(steps.argmax())})')
        sys.exit(2)
    carry = [list(map(float, a.q_top))] + pi.sparse(Q[:i_t2 + 1], 6.0, MAX_KNOTS - 2)
    step_in = 1.5
    while True:                                                # in the tines: the knot path within 2 mm of the plan
        ins = pi.sparse(Q[i_t2:], step_in, MAX_KNOTS - 2)
        dev_in = pi.path_dev_mm(k, H, ins, Q[i_t2:])
        if dev_in <= 2.0 or step_in < 0.3:
            break
        step_in /= 1.5
    if len(ins) < 3:
        ins = [ins[0], ((np.array(ins[0]) + np.array(ins[-1])) / 2).tolist(), ins[-1]]
    legs = plan['legs']
    back = knots([ins[-1]] + pi.sparse(Q[::-1], 8.0, MAX_KNOTS - 3)[1:] + [legs['return_home']['waypoints_deg'][-1]], KNOT_DEG * 1.6)
    src = f'gt_replay ({Path(a.gt).name} pin {gt["pin"]} -> {a.pin})'
    legs['policy_carry'] = dict(legs['post_grasp'], waypoints_deg=carry, planner=src)
    legs['policy_insert'] = dict(legs['insert'], waypoints_deg=ins, planner=src)
    legs['policy_return'] = dict(legs['return_home'], waypoints_deg=back, planner=src + ' reversed')
    Fe, Fe_gt = k.fk(np.radians(Q[-1])), k.fk(np.radians(W[-1]))
    r_now, r_gt = rows(Fe, H, Tbb, gaps, away), rows(Fe_gt, H, Tg, gaps, away)
    report = dict(time=time.strftime('%Y%m%d_%H%M%S'), gt=str(a.gt), quadrant=a.quadrant, pin=a.pin, gt_pin=gt['pin'], gap=int(gap),
                  shift_mm=(np.linalg.inv(Tbb)[:3, :3] @ shift * 1000).round(1).tolist(),
                  basket_moved_since_gt_mm=round(float(np.linalg.norm(move[:3, 3])) * 1000, 1),
                  residual=dict(dx_mm=a.dx_mm, dy_mm=a.dy_mm, dyaw_deg=a.dyaw_deg, dpitch_deg=a.dpitch_deg),
                  rows_pred_LR=r_now, rows_pred_LR_minus_gt_pin_shift=[round(r_now[0] - r_gt[0] - (a.pin - gt['pin']), 2),
                                                                         round(r_now[1] - r_gt[1] - (a.pin - gt['pin']), 2)],
                  joint_margin_deg=round(float(min((Q - lo).min(), (hi - Q).min())), 1), max_joint_step_deg=round(float(steps.max()), 2),
                  lift_top_to_first_knot_deg=round(float(np.abs(Q[0] - np.array(a.q_top)).max()), 1),
                  knots=dict(carry=len(carry), insert=len(ins), back=len(back)), knot_path_dev_insert_mm=round(dev_in, 1),
                  seconds=round(time.time() - t0, 2))
    plan['gt_replay'] = report
    out = Path(a.plan).with_name('plan_gt.json')
    out.write_text(json.dumps(plan, indent=1))
    log = ROOT / 'outputs/gt_replay' / f'{report["time"]}.json'
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text(json.dumps(report, indent=1))
    print(json.dumps(report))
    print(f'plan: {out}')


if __name__ == '__main__':
    main()
