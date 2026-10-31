"""(보스 10/31) the point-cloud docking policy (rl/dock_pc_train.py, v11) on the real insert, planned at the lift top.

The taught insert (taught_plan.py plan: post_grasp ... teach 1, teach 2 + insert teach 2 -> teach 3) is rolled out in the
kinematic dock env with THIS basket, THIS gap and the in-hand plate as measured (experiments/inhand/plate_inhand.py: the face
normal in the flange frame; the centre is kept from the job's calibrated H unless --use_centre, the measured centre was
biased on 10/30).  The policy's corrections (basket x / y, pitch about basket y, yaw about the vertical, the extra descent)
go onto the dense flange path, then a continuation IK gives the joints.  Refused (exit 2, the caller keeps its plan) when there is
no IK with J4 as taught, a joint leaves the URDF limits (보스 10/31: whatever the URDF allows is allowed), a joint
jumps > --max_step_deg between 2 mm knots, or the policy does not seat the plate in the sim.  No robot I/O.  Writes a copy of the plan with
  policy_carry  : lift top -> the corrected path up to teach 2 (1 cm over the slot)
  policy_insert : the corrected teach 2 -> into the slot (+ the extra descent)
  policy_return : the corrected path reversed -> home (after the release)
and prints 'plan: <path>'.
    ~/miniconda3/envs/foundationpose/bin/python tesollo/policy_insert.py --plan PLAN.json --q_top J1..J6 \
        --normal_flange nx ny nz --quadrant Q3 --pin 6 [--basket tesollo/demo_safe/basket_fixed.json]"""
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
sys.path.insert(0, str(ROOT / 'rl'))
sys.path.insert(0, str(HERE))
CKPT = ROOT / 'outputs/dock_rl_20261030/pc_rand_v11/final.pt'
JOB = ROOT / 'rl/logs/hand_basket/job_taught_1029_cal.npz'
# the env the v11 policy was trained in (dock_pc_train.py v7 -> v11), without the training randomisation
ENV_KW = dict(pc_goal=True, pc_tol_mm=5.0, pc_yaw=True, yaw_deg=0.0, pc_disc_dist=True, target_x_mm=10.0, angle_tol_deg=6.0,
              residual_pitch_deg=40.0, descend_extra_mm=40.0, hover_until=0.15,
              basket_xy_mm=0.0, basket_yaw_deg=0.0, pitch_frac=0.0, lean_deg=0.0, hang_deg=0.0, grip_mm=0.0, tilt_deg=0.0)


def dense_path(k, wp, step_mm=2.0):
    """joint knots -> (joint knots every ~step_mm of flange travel, their flange poses, index of each input knot)"""
    qs, idx = [], []
    for qa, qb in zip(wp[:-1], wp[1:]):
        idx.append(len(qs))
        Fa, Fb = k.fk(np.radians(qa)), k.fk(np.radians(qb))
        n = max(2, int(np.ceil(np.linalg.norm(Fb[:3, 3] - Fa[:3, 3]) * 1000 / step_mm)) + 1)
        qs += [qa + s * (qb - qa) for s in np.linspace(0, 1, n)[:-1]]
    idx.append(len(qs))
    qs.append(wp[-1])
    return np.array(qs), np.array([k.fk(np.radians(q)) for q in qs]), idx


def sparse(Q, step_deg, max_knots):
    """every-2-mm joint knots -> a few: keep a knot once some joint moved step_deg since the last kept one (wider steps
    until <= max_knots); first and last always kept"""
    Q = np.asarray(Q, float)
    while True:
        out, last = [Q[0]], Q[0]
        for q in Q[1:-1]:
            if np.abs(q - last).max() >= step_deg:
                out.append(q)
                last = q
        out.append(Q[-1])
        if len(out) <= max_knots:
            return [list(map(float, q)) for q in out]
        step_deg *= 1.2


def lowest_rim(F, H, Tbb, r=0.134):
    """the plate's lowest rim point at flange pose F: (in the flange frame, in the basket frame)"""
    th = np.linspace(0, 2 * np.pi, 360, endpoint=False)
    rim = np.c_[r * np.cos(th), r * np.sin(th), np.zeros_like(th), np.ones_like(th)]
    Xb = (np.linalg.inv(Tbb) @ F @ H @ rim.T).T
    i = int(np.argmin(Xb[:, 2]))
    return (H @ rim[i])[:3], Xb[i, :3]


def path_dev_mm(k, H, knots_deg, dense_deg):
    """largest distance [mm] of the plate centre on the dense corrected path from the joint-line path through the knots"""
    W = np.asarray(knots_deg, float)
    S = np.array([(k.fk(np.radians(a + s * (b - a))) @ H)[:3, 3] for a, b in zip(W[:-1], W[1:]) for s in np.linspace(0, 1, 25)])
    D = np.array([(k.fk(np.radians(q)) @ H)[:3, 3] for q in dense_deg])
    return float(np.sqrt(((D[:, None] - S[None]) ** 2).sum(-1)).min(1).max() * 1000)


def measured_h(H_nom, normal_f, F_end, Tbb, centre_f=None):
    """the in-hand plate frame from the measured face normal: H_nom turned onto it about the plate's lowest rim point at
    the taught slot pose (the point the --taught_pivot slot / --inhand_yaw plans keep on the gap, 10/30 real L5/R5); the
    plate hinges about the pinch, not about its centre.  --use_centre instead puts the centre where it was measured."""
    from scipy.spatial.transform import Rotation as R
    n0 = H_nom[:3, 2]
    n = np.asarray(normal_f, float)
    n /= np.linalg.norm(n)
    n *= np.sign(n @ n0) or 1.0
    ax = np.cross(n0, n)
    ang = np.arctan2(np.linalg.norm(ax), n0 @ n)
    Rm = R.from_rotvec(ax / (np.linalg.norm(ax) or 1.0) * ang).as_matrix()
    p_b, _ = lowest_rim(F_end, H_nom, Tbb)
    H = np.eye(4)
    H[:3, :3] = Rm @ H_nom[:3, :3]
    H[:3, 3] = p_b + Rm @ (H_nom[:3, 3] - p_b) if centre_f is None else np.asarray(centre_f, float)
    return H, float(np.degrees(ang))


def rollout(F, H, Tbb, gaps_b, gap, ckpt):
    """the policy in the dock env on this path -> (residual per env knot (S,4), zdown [m], end metrics)"""
    import torch
    from dock_env import DockEnv, _resample_pose
    import dock_pc_train as T
    env = DockEnv(1, seed=0, **ENV_KW)
    env.reset()
    d = env.device
    f32 = lambda x: torch.tensor(np.asarray(x), dtype=torch.float32, device=d)  # noqa: E731
    env.paths = f32(_resample_pose(F, env.path_steps))[None]
    env.job = torch.zeros(1, dtype=torch.long, device=d)
    env.job_gap = torch.tensor([gap], dtype=torch.long, device=d)
    env.gaps = f32(gaps_b)[None]
    env.H, env.Tbb = f32(H)[None], f32(Tbb)[None]
    env.pitch.fill_(env.pitch0); env.lean.fill_(env.lean0); env.yaw0.zero_()
    net = T.PointNetAC(env.pc_m, env.act_dim).to(d)
    net.load_state_dict(torch.load(ckpt, map_location=d, weights_only=True))
    net.w.fill_(0.0)
    obs = env._observe()
    res = np.zeros((env.path_steps, 4))
    info = {}
    with torch.no_grad():
        for _ in range(env.steps - 1):
            obs, _, done, info = env.step(net.deterministic(obs))
            t = int(env.t[0])
            res[t, :3] = env.residual[0, :3].cpu().numpy()
            res[t, 3] = float(env.yaw0[0] + env.residual[0, 3])
            if bool(done[0]):
                break
    t_end = int(env.t[0])
    res[t_end + 1:] = res[t_end]                             # contact before the last knots: hold the last correction
    x_ok, y_ok, h_ok, a_ok, ang = env.criteria()
    _, _, _, bb, rx, yc, _ = env._geometry()
    end = dict(kin_success=bool(info['success'][0]), fit=round(float(info['fit'][0]), 3),
               disc_dist_mm=round(float(info['chamfer'][0]) * 1000, 1), angle_deg=round(math.degrees(float(ang[0])), 1),
               bottom_x_mm=round(float(bb[0, 0] - rx[0]) * 1000, 1), bottom_y_mm=round(float(bb[0, 1] - yc[0]) * 1000, 1))
    return res, float(env.zdown[0]), end, env.floor_z, env.tine_h


def main():
    from scipy.spatial.transform import Rotation as R
    import plate_plans as pp
    from taught_plan import knots, KNOT_DEG, MAX_KNOTS
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--plan', required=True, help='taught_plan.py plan json (post_grasp / insert legs)')
    a.add_argument('--q_top', type=float, nargs=6, required=True, help='the lift-top joints [deg] (where the arm waits)')
    a.add_argument('--normal_flange', type=float, nargs=3, required=True)
    a.add_argument('--centre_flange', type=float, nargs=3, default=None)
    a.add_argument('--use_centre', action='store_true', help='also take the measured centre (10/30: biased, off by default)')
    a.add_argument('--quadrant', default='Q3')
    a.add_argument('--pin', type=int, default=6)
    a.add_argument('--basket', default=str(HERE / 'demo_safe/basket_fixed.json'))
    a.add_argument('--ckpt', default=str(CKPT))
    a.add_argument('--max_step_deg', type=float, default=8.0, help='largest joint change between 2 mm knots')
    a = a.parse_args()
    t0 = time.time()
    k = pp.Kinematics()
    lo, hi = np.array(k.limits_deg()).T
    plan = json.loads(Path(a.plan).read_text())
    legs = plan['legs']
    wp = np.array(legs['post_grasp']['waypoints_deg'][-2:] + legs['insert']['waypoints_deg'][1:], float)   # t1, t2, mid, t3
    qs, F, idx = dense_path(k, wp)
    i_t2 = idx[1]
    Tbb = np.array(json.loads(Path(a.basket).read_text())['T'])
    model_qd = pp.robot_quadrants(Tbb)[a.quadrant]
    gaps_b = np.array(pp.basket_gaps()[model_qd])
    gap = pp.pin_gap(Tbb, model_qd, a.pin)
    H_nom = np.load(JOB)['H_nom']
    H, turned = measured_h(H_nom, a.normal_flange, F[-1], Tbb, a.centre_flange if a.use_centre else None)
    _, b0 = lowest_rim(F[-1], H, Tbb)                            # no correction: where the plate bottom would end
    uncorrected = dict(x_mm=round(float(b0[0] - gaps_b[gap][0]) * 1000, 1), y_mm=round(float(b0[1] - gaps_b[gap][1]) * 1000, 1),
                       face_turn_vs_job_deg=round(turned, 1))
    res, zdown, end, floor_z, tine_h = rollout(F, H, Tbb, gaps_b, gap, a.ckpt)
    if not end['kin_success']:                                   # the policy itself does not get it in: keep the plan
        print(f'REFUSED: the policy does not seat it in the sim ({end})')
        sys.exit(2)
    # the corrections on every dense knot (as rl/hand_basket.py --dock_traj replays them)
    L = len(F)
    rr = np.stack([np.interp(np.linspace(0, 1, L), np.linspace(0, 1, len(res)), res[:, c]) for c in range(4)], 1)
    targets, h_centre = [], []
    for i in range(L):
        C = F[i] @ H
        C2 = C.copy()
        C2[:3, 3] += Tbb[:3, :3] @ np.r_[rr[i, 0], rr[i, 1], 0.0]
        C2[:3, :3] = R.from_rotvec(Tbb[:3, 2] * rr[i, 3]).as_matrix() @ R.from_rotvec(Tbb[:3, 1] * rr[i, 2]).as_matrix() @ C2[:3, :3]
        targets.append(C2 @ np.linalg.inv(H))
        h_centre.append((np.linalg.inv(Tbb) @ C2)[2, 3] - floor_z)
    n_down = int(math.ceil(zdown / 0.002 - 1e-6))
    for j in range(1, n_down + 1):                              # the extra descent after the path end (DockEnv zdown)
        T_ = targets[-1].copy()
        T_[:3, 3] -= Tbb[:3, 2] * min(0.002, zdown - 0.002 * (j - 1))
        targets.append(T_)
        h_centre.append(h_centre[-1] - min(0.002, zdown - 0.002 * (j - 1)))
    # continuation IK, J4 as taught (10/30 --yaw_corr: a full IK flips J4 / J6 near J5 = 0); the plate is a disc, so the
    # centre (3) + the face normal (2) are the constraints and J1-J3, J5, J6 the unknowns
    from scipy.optimize import least_squares
    qs_d = np.radians(np.vstack([qs, np.repeat(qs[-1:], len(targets) - len(qs), 0)]))
    nz, c0 = H[:3, 2], H[:3, 3]
    q = qs_d[0].copy()
    Q = []
    for i, T_ in enumerate(targets):
        C = T_ @ H
        q[3] = qs_d[i][3]

        def res(x, j4=q[3]):
            qq = np.r_[x[:3], j4, x[3:]]
            G = k.fk(qq)
            return np.r_[(G[:3, :3] @ c0 + G[:3, 3] - C[:3, 3]) * 100, (G[:3, :3] @ nz - C[:3, 2]) * 10]
        sol = least_squares(res, np.r_[q[:3], q[4:]])
        q = np.r_[sol.x[:3], q[3], sol.x[3:]]
        r = res(sol.x)
        if np.linalg.norm(r[:3]) > 0.1 or np.degrees(np.linalg.norm(r[3:]) / 10) > 0.5:
            print(f'REFUSED: no IK with J4 as taught at knot {i} (centre off {np.linalg.norm(r[:3]) * 10:.1f} mm, '
                  f'face off {np.degrees(np.linalg.norm(r[3:]) / 10):.1f} deg)')
            sys.exit(2)
        Q.append(np.degrees(q))
    Q = np.array(Q)
    if (Q < lo).any() or (Q > hi).any():
        print(f'REFUSED: outside the URDF joint limits (margin {min((Q - lo).min(), (hi - Q).min()):.1f} deg)')
        sys.exit(2)
    # (10/31) the taught insert itself runs at J5 ~ -8.6 (--wrist_less 25).  With J4 held as taught, J5 passing 0 is a plain
    # joint-space move (no J4 / J6 flip); a flip would show as a joint jump below
    j5_cross = int((np.sign(Q[1:, 4]) != np.sign(Q[:-1, 4])).sum())
    steps = np.abs(np.diff(Q, axis=0)).max(1)
    if steps.max() > a.max_step_deg:
        print(f'REFUSED: a joint jumps {steps.max():.1f} deg between 2 mm knots (knot {int(steps.argmax())})')
        sys.exit(2)
    first_move = np.abs(Q[0] - np.array(a.q_top)).max()
    # few knots (the controller spline smooths them, as the taught legs): the carry from the lift top in the air, the
    # insert closer (the plate is in the tines), the way back as the insert reversed then home
    carry = [list(map(float, a.q_top))] + sparse(Q[:i_t2 + 1], 6.0, MAX_KNOTS - 2)
    step_in = 1.5
    while True:                                                  # in the tines: the knot path within 2 mm of the plan
        ins = sparse(Q[i_t2:], step_in, MAX_KNOTS - 2)
        dev_in = path_dev_mm(k, H, ins, Q[i_t2:])
        if dev_in <= 2.0 or step_in < 0.3:
            break
        step_in /= 1.5
    dev_carry = path_dev_mm(k, H, carry[1:], Q[:i_t2 + 1])
    if len(ins) < 3:                                             # the mover needs >= 2 targets after the start
        ins = [ins[0], ((np.array(ins[0]) + np.array(ins[-1])) / 2).tolist(), ins[-1]]
    back = knots([ins[-1]] + sparse(Q[::-1], 8.0, MAX_KNOTS - 3)[1:] + [legs['return_home']['waypoints_deg'][-1]], KNOT_DEG * 1.6)
    plan['legs']['policy_carry'] = dict(legs['post_grasp'], waypoints_deg=carry, planner='policy_insert (dock RL v11)')
    plan['legs']['policy_insert'] = dict(legs['insert'], waypoints_deg=ins, planner='policy_insert (dock RL v11)')
    plan['legs']['policy_return'] = dict(legs['return_home'], waypoints_deg=back, planner='policy_insert reversed')
    report = dict(time=time.strftime('%Y%m%d_%H%M%S'), ckpt=str(a.ckpt), quadrant=a.quadrant, pin=a.pin, model_qd=model_qd, gap=int(gap),
                  normal_flange=list(a.normal_flange), used_centre=bool(a.use_centre), uncorrected_bottom_vs_gap=uncorrected, sim_end=end,
                  max_correction=dict(x_mm=round(float(np.abs(rr[:, 0]).max()) * 1000, 1), y_mm=round(float(np.abs(rr[:, 1]).max()) * 1000, 1),
                                      pitch_deg=round(math.degrees(float(np.abs(rr[:, 2]).max())), 1),
                                      yaw_deg=round(math.degrees(float(np.abs(rr[:, 3]).max())), 1), extra_descent_mm=round(zdown * 1000, 1)),
                  joint_margin_deg=round(float(min((Q - lo).min(), (hi - Q).min())), 1), j5_min_abs_deg=round(float(np.abs(Q[:, 4]).min()), 1), j5_zero_crossings=j5_cross,
                  max_joint_step_deg=round(float(steps.max()), 2), lift_top_to_first_knot_deg=round(float(first_move), 1),
                  knots=dict(carry=len(carry), insert=len(ins), back=len(back)),
                  knot_path_dev_mm=dict(carry=round(dev_carry, 1), insert=round(dev_in, 1)), seconds=round(time.time() - t0, 2))
    plan['policy_insert'] = report
    out = Path(a.plan).with_name('plan_policy.json')
    out.write_text(json.dumps(plan, indent=1))
    log = ROOT / 'outputs/policy_insert' / f'{report["time"]}.json'
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text(json.dumps(report, indent=1))
    print(json.dumps(report))
    print(f'plan: {out}')


if __name__ == '__main__':
    main()
