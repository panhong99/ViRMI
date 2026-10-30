"""Tesollo hand, plate: from the lifted grasp to the basket, down to the pins - the 2F-85 basket planner
(calibration/plate_plans.quadrant_plan) with the Tesollo grasp instead of the gripper's:
  - grasp joints = the taught plate grasp (tesollo/arm_poses.yaml), moved by --dx/--dy like arm_plan.py
  - H (plate in the hand) from those joints and the plate centre (flat plate, x toward the hand), the same rule as
    plate_plans.hand_plate, with the hand's own length / radius for the collision model
  - basket pose = a basket_vision.json (default: the 11/3 fixed basket; a live run's measurement with --basket)
Plate legs (one plan.json, dry run): 3-point lift, horizontal carry, vertical descend, return_home (vertical rise + home).
Bowl (--object bowl): kept upright as grasped and set down in the middle of a quadrant (default Q1), its bottom
BOWL_DROP_M above the basket rim; carry = joint line from the lift to 12 cm above that, checked for the bowl bottom
>= 5 cm over the rim and <= 15 deg tilt; legs lift, to_above, descend, ascend, to_home.
Read-only (no robot command).
    ~/miniconda3/envs/foundationpose/bin/python tesollo/basket_plan.py --quadrant Q3 --pin 6 [--dx M --dy M]
    ~/miniconda3/envs/foundationpose/bin/python tesollo/basket_plan.py --object bowl --quadrant Q1 [--dx M --dy M]
"""
import argparse
import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'calibration'))
START_CWD = Path.cwd()                  # (relative paths on the command line are resolved against this)
os.chdir(HERE.parent / 'calibration')   # calib_core / plate_plans read their files relative to calibration/
import numpy as np
import yaml
import calib_core as cc
import plate_plans as pp

# DG-5F-S palm as a cylinder on the flange for the collision model (2F-85: 0.165 / 0.06).  At the taught plate grasp the
# hand axis is horizontal and the plate centre sits 16 cm along it, 8 cm off it: a 20 cm cylinder runs through the held
# plate itself (every insert refused, 10/27), so only the palm (12 cm) is modelled; the fingers hold the top rim.
HAND_LENGTH_M, HAND_RADIUS_M = 0.12, 0.06
PLATE_SHIFT_M = tuple(float(v) for v in __import__('os').environ.get('PLATE_SHIFT', '0 0.05').split()) # (보스 10/27 real Q3) the hand must end 5 cm further toward base +y: the real plate sits that
                            # much -y of the model in the hand at the insert, so H is corrected by it (a 2nd pass in the insert
                            # pose's tool frame) and the same gap is planned again; moving the target instead put the model
                            # plate into the basket wall (38/38 refused)
# (10/27 night) the plate does not stay flat in the 5-finger grasp: 보스 sees it held almost slot-ready, and the planner
# needs a 96 deg hand turn with a flat plate but 54 / 34 deg if it hangs 40 / 60 deg (Q3 pin 6).  --hang_deg turns H by
# that about this grasp tool-frame axis (the axis of the 96 deg turn); 0 = flat (the old rule) until it is measured.
HANG_AXIS = (-0.94, 0.35, 0.01)
BOWL_HEIGHT_M = 0.07        # perceive gives the bowl rim top (10/27 ref 0.072 m on the table): bottom = top - this
BOWL_DROP_M = -0.09         # bowl bottom vs the basket rim when it is let go (보스 10/27 real Q3: +3 cm stopped 10 cm too high; 10/30: 2 cm lower)
BOWL_SHIFT_M = (0.0, 0.10)  # + this (base xy) on the quadrant middle (보스 10/27 real Q3: 10 cm more toward robot +y)
# (10/29) PLATE_SHIFT_M / BOWL_SHIFT_M were the stale fixed basket file, not the hand: measured live 10/29 the basket sat
# (-37, +109) mm, 0.9 deg from demo_basket_fixed.json - the +y the real runs needed.  They apply only with that file;
# a live basket_vision measurement (--basket <json>) uses none.
FIXED_BASKET = HERE.parent / 'calibration/data/basket/demo_basket_fixed.json'
BOWL_RISE_M = 0.22          # lift (and above-basket height): the bowl bottom must clear the rim (0.144 m) + 5 cm on the carry
PLATE_RISE_M = 0.14         # (10/29 보스) grasp -> 7 cm -> 14 cm: three lift knots including the grasp pose
PLATE_LIFT_WAYPOINTS = 3
CARRY_MAX_WAYPOINTS = 20
PLACE_CYCLE_MAX_WAYPOINTS = 20   # (10/29 Claude: was 10; the descent now has a knot every 5 cm)
# basket frame quadrant extents (calibration/demo_watch.py HALF / SPLIT): x near (-0.265, 0.035) / far (0.035, 0.265),
# y left (0.0095, 0.26) / right (-0.26, 0.0095)
QUAD_XY = {'Q1': (0.15, -0.125), 'Q2': (0.15, 0.135), 'Q3': (-0.115, 0.135), 'Q4': (-0.115, -0.125)}


def shifted_grasp(k, grasp, dx, dy):
    """The taught grasp joints moved by (dx, dy) in base, same hand orientation."""
    if not (dx or dy):
        return np.asarray(grasp, float)
    T = k.fk(np.radians(grasp))
    T[0, 3] += dx
    T[1, 3] += dy
    q = k.ik(T, np.radians(grasp))
    assert q is not None and np.abs(k.fk(q)[:3, 3] - T[:3, 3]).max() < 1e-3, 'IK failed for the shifted grasp'
    return np.degrees(q)


def wrist_near(k, q, prev):
    """(보스 10/30: J4 spun ~170 deg on the bowl lift) the same flange pose by the other wrist branch (J4 + 180, -J5,
    J6 - 180, +-360 on J4 / J6) when it is closer to prev: the lift crosses J5 = 0 and the IK took the flipped one. rad."""
    lo, hi = np.array([j[3] for j in k.joints]), np.array([j[4] for j in k.joints])
    best = np.asarray(q, float)
    for s4 in (np.pi, -np.pi):
        for s6 in (np.pi, -np.pi, 3 * np.pi, -3 * np.pi):
            c = np.asarray(q, float).copy()
            c[3], c[4], c[5] = q[3] + s4, -q[4], q[5] + s6
            if ((c >= lo) & (c <= hi)).all() and np.abs(c - prev).max() < np.abs(best - prev).max():
                best = c
    return best


LIFT_NEAR = [True]          # (10/30) lift_line on the nearest wrist branch; False = the IK's own knots (before 10/30)


def lift_line(k, q0, dz, n=11, j5_min=8.0):
    """Straight up from the FK of q0 (as arm_plan.py).  (보스 10/30: J4 spun on the bowl lift) the bowl lift passes the
    wrist singularity (J5 ~ 0): there exact IK knots turn J4 / J6 against each other by up to 170 deg.  Knots solved
    every cm on the wrist branch nearest the last knot with |J5| >= j5_min; knots with |J5| < j5_min are replaced by the
    joint line between their good neighbours (the bowl leaves the vertical by a few mm there, the hand keeps its turn)."""
    T0, q = k.fk(np.radians(q0)), np.radians(q0)
    if not LIFT_NEAR[0]:
        out = [np.asarray(q0, float)]
        for s in np.linspace(0, 1, n)[1:]:
            T = T0.copy()
            T[2, 3] += s * dz
            q = k.ik(T, q)
            out.append(np.degrees(q))
        return out
    m = max(int(np.ceil(abs(dz) / 0.01)), n - 1)
    ss = np.linspace(0, 1, m + 1)
    dense, good, ref = [q], [True], q
    for s in ss[1:]:
        T = T0.copy()
        T[2, 3] += s * dz
        q = k.ik(T, q)
        assert q is not None, 'IK failed on the lift'
        c = wrist_near(k, q, ref)
        if c is not q:                                   # refine on that branch (the flip is exact only to ~0.1 mm)
            q = k.ik(T, c)
        ok = abs(np.degrees(q[4])) >= j5_min
        if ok:
            ref = q
        dense.append(q)
        good.append(ok)
    good[-1] = True
    gi = [i for i, g in enumerate(good) if g]
    for i in range(len(dense)):
        if not good[i]:
            a_ = max(g for g in gi if g < i)
            b_ = min(g for g in gi if g > i)
            dense[i] = dense[a_] + (i - a_) / (b_ - a_) * (dense[b_] - dense[a_])
    D = np.degrees(np.array(dense))
    return [np.array([np.interp(s, ss, D[:, j]) for j in range(6)]) for s in np.linspace(0, 1, n)]


def hand_plate(k, grasp_deg, centre, tool):
    """(H) plate frame (centre, z up, x toward the hand) in the tool frame at the grasp - plate_plans.hand_plate's rule."""
    T_tcp = k.fk(np.radians(grasp_deg)) @ tool
    n = np.array([0.0, 0.0, 1.0])
    c = np.asarray(centre, float)
    a = T_tcp[:3, 3] - c
    a -= (a @ n) * n
    x = a / np.linalg.norm(a)
    T_pl = np.eye(4)
    T_pl[:3, :3] = np.stack([x, np.cross(n, x), n], 1)
    T_pl[:3, 3] = c
    return np.linalg.inv(T_tcp) @ T_pl


def rz(t):
    c, s = np.cos(t), np.sin(t)
    R = np.eye(4)
    R[:2, :2] = [[c, -s], [s, c]]
    return R


def cart_line(k, q0, p1, yaw=0.0, n=40):
    """Flange from FK(q0) turned `yaw` about the base z axis (with joint 1: the bowl is round, so its yaw is free and
    the wrist stays put; a fixed hand yaw swung J4/J6 by > 100 deg, 10/27) and moved straight to position p1, IK seeded
    step by step."""
    T0 = k.fk(np.radians(q0))
    out, q = [np.asarray(q0, float)], np.radians(q0)
    for s in np.linspace(0, 1, n)[1:]:
        T = rz(s * yaw) @ T0
        T[:3, 3] = (1 - s) * T0[:3, 3] + s * np.asarray(p1)
        q = k.ik(T, q)
        assert q is not None and np.abs(k.fk(q)[:3, 3] - T[:3, 3]).max() < 1e-3, 'IK failed on the carry line'
        out.append(np.degrees(q))
    step = np.abs(np.diff(np.array(out), axis=0)).max()
    assert step < 10, f'IK jump {step:.1f} deg on the carry line'
    return out


PLATE_OVER_RIM_M = 0.05     # (보스 10/29, the sparse carry hit the basket) held plate's lowest point over the basket rim


def sparse_demo_plan(path, k, lift, home, tool_m=HAND_LENGTH_M):
    """10/29 demo path: 3-point lift, <=20-point horizontal carry, <=10-point down/up/home cycle.

    No environment collision checker is called here.  IK and joint-limit checks remain, and the controller executes
    lift+carry as one spline, then descent, release, and up+home as one spline.
    """
    from scipy.spatial.transform import Rotation, Slerp
    import ros2_nodes

    plan = json.loads(Path(path).read_text())
    L = plan['legs']
    q_lift = np.asarray(lift[-1], float)
    q_insert = np.asarray(L['descend']['waypoints_deg'][-1], float)
    T0, Ti = k.fk(np.radians(q_lift)), k.fk(np.radians(q_insert))
    # (Claude 10/29, after the real run hit the basket) the carry ran at the lift height while the hand turned the plate
    # upright: its lowest point went 12-19 cm under the rim.  The carry height is now where the held plate (plan
    # plate_in_tcp, flange frame) stays PLATE_OVER_RIM_M over the rim inside the basket (+5 cm around its walls) along
    # the whole turn, checked on the controller spline itself; up there straight, across, then the straight descent.
    tool = np.eye(4)
    tool[2, 3] = tool_m
    cloud = pp.plate_cloud(np.asarray(plan['plate_in_tcp'], float), tool)
    Tbi = np.linalg.inv(np.asarray(plan['basket_T'], float))

    def over_rim(F):                                         # plate's lowest point over/near the basket, basket frame
        B = (cloud @ F[:3, :3].T + F[:3, 3]) @ Tbi[:3, :3].T + Tbi[:3, 3]
        near = (np.abs(B[:, 0]) < 0.35) & (np.abs(B[:, 1]) < 0.34)
        return float(B[near, 2].min()) if near.any() else np.inf

    rots = Slerp([0.0, 1.0], Rotation.from_matrix([T0[:3, :3], Ti[:3, :3]]))

    def pose(f, z):
        T = np.eye(4)
        T[:3, :3] = rots(f).as_matrix()
        T[:3, 3] = T0[:3, 3] + f * (Ti[:3, 3] - T0[:3, 3])
        T[2, 3] = z
        return T

    z_c = max(T0[2, 3], Ti[2, 3])
    for _ in range(8):
        worst = min(over_rim(pose(f, z_c)) for f in np.linspace(0, 1, 41))
        if worst >= PLATE_OVER_RIM_M:
            break
        z_c += PLATE_OVER_RIM_M - worst + 0.005
    carry = None
    for n in (6, 8, 12, 16, CARRY_MAX_WAYPOINTS):
        Tup = T0.copy()
        Tup[2, 3] = z_c
        out, prev, ok = [q_lift], np.radians(q_lift), True
        for T in ([Tup] if z_c > T0[2, 3] + 0.01 else []) + [pose(f, z_c) for f in np.linspace(0.0, 1.0, n)[1:]]:
            try:
                prev = k.ik(T, prev)
            except RuntimeError:
                ok = False
                break
            out.append(np.degrees(prev))
        if not ok or np.abs(np.diff(np.asarray(out), axis=0)).max() > 40.0:
            continue
        dense, _ = pp.equal_time_spline(np.asarray(list(lift) + out[1:], float), per=24)
        spl = min(over_rim(k.fk(np.radians(q))) for q in dense)
        if spl >= PLATE_OVER_RIM_M - 0.01:                   # the spline between the knots too (1 cm tolerance)
            carry = out
            break
    if carry is None:
        raise RuntimeError(f'sparse carry: no <= {CARRY_MAX_WAYPOINTS}-knot carry keeps the plate '
                           f'{PLATE_OVER_RIM_M * 100:.0f} cm over the rim (carry height {z_c:.3f} m)')
    Ta = pose(1.0, z_c)                                       # over the slot at the carry height

    # Slot insertion is the only vertical descent.  The spline needs only its two endpoints.
    down, prev = [], np.radians(carry[-1])
    n_down = max(2, int(np.ceil((Ta[2, 3] - Ti[2, 3]) / 0.05)) + 1)   # a knot every 5 cm: the joint line between two
    for f in np.linspace(0.0, 1.0, n_down):                            # knots 17 cm apart swung 6.4 mm off the vertical
        T = Ta.copy()
        T[2, 3] = (1.0 - f) * Ta[2, 3] + f * Ti[2, 3]
        prev = k.ik(T, prev)
        down.append(np.degrees(prev))

    # One stored cycle, <=10 unique knots: descend, reverse vertically, then joint-line to base/home.
    q_above, q_home = np.asarray(down[0]), np.asarray(home, float)
    home_segments = max(1, int(np.ceil(np.abs(q_home - q_above).max() / 40.0)))
    cycle_prefix = list(down) + list(down[-2::-1])
    cycle = list(cycle_prefix)
    cycle += [q_above + f * (q_home - q_above) for f in np.linspace(0.0, 1.0, home_segments + 1)[1:]]
    if len(cycle) > PLACE_CYCLE_MAX_WAYPOINTS:
        raise RuntimeError(f'down/up/home needs {len(cycle)} waypoints (> {PLACE_CYCLE_MAX_WAYPOINTS})')
    return_home = [down[-1]] + list(down[-2::-1]) + cycle[len(cycle_prefix):]
    if len(down) + len(return_home) > PLACE_CYCLE_MAX_WAYPOINTS:
        raise RuntimeError(f'descend + return needs {len(down) + len(return_home)} stored waypoints '
                           f'(> {PLACE_CYCLE_MAX_WAYPOINTS})')

    lo, hi = np.asarray(ros2_nodes.joint_limits_deg(), float).T
    for name, knots in (('lift_carry', list(lift) + list(carry[1:])), ('descend', down),
                        ('return_home', return_home), ('place_cycle', cycle)):
        W = np.asarray(knots, float)
        if len(W) > 1 and np.abs(np.diff(W, axis=0)).max() > 45.0:
            raise RuntimeError(f'{name}: joint step over 45 deg')
        dense, _ = pp.equal_time_spline(W, per=40)
        if np.minimum(dense - lo, hi - dense).min() < 2.0:
            raise RuntimeError(f'{name}: spline enters the 2 deg joint-limit margin')

    def leg(points, label):
        return dict(waypoints_deg=np.round(np.asarray(points), 3).tolist(), planner=label,
                    min_clearance_straight_m=0.0)

    L['lift'] = leg(lift, '3-point 7 cm vertical lift')
    L['to_above'] = leg(carry, 'horizontal sparse carry, unchecked')
    L['descend'] = leg(down, 'vertical slot insertion, unchecked')
    L['return_home'] = leg(return_home, 'vertical rise + base/home, unchecked')
    for old in ('ascend', 'to_home', 'abort_home'):
        L.pop(old, None)
    total = 3 + len(lift) + len(carry) + len(cycle)          # the four stage counts shown to the operator
    if total > 40:
        raise RuntimeError(f'demo path has {total} waypoints (> 40)')
    plan.update(margin_m=0.0, accepted=True, reasons=[], collision_check='skipped after grasp',
                demo_sparse=dict(approach=3, lift=len(lift), carry=len(carry), place_cycle=len(cycle), total=total,
                                 carry_height_m=round(float(z_c), 3), plate_over_rim_m=round(float(spl), 3),
                                 release_after='descend', speed_deg_s=60.0))
    plan.setdefault('quadrant_plan', {})['drop_start'] = len(carry) - 1
    plan['source'] = plan.get('source', '') + ' | 10/29 sparse demo <=40, post-grasp collision unchecked'
    Path(path).write_text(json.dumps(plan, indent=1))
    return path


def bowl_plan(a, k, Tbb, Tbc):
    """Bowl upright from the grasp into a quadrant (see the module doc) -> plan path."""
    import ros2_nodes
    A = yaml.safe_load((HERE / 'arm_poses.yaml').read_text())['bowl']
    if 'obj_centre' not in A:
        sys.exit('no bowl reference: tesollo/perceive.py --object bowl --save_ref first')
    grasp = shifted_grasp(k, A['posj_deg'], a.dx, a.dy)
    lift = lift_line(k, grasp, a.rise)
    home = np.array(pp.C['robot']['home_joints_deg'], float)
    Tg = k.fk(np.radians(grasp))
    bottom = np.array([A['obj_centre'][0] + a.dx, A['obj_centre'][1] + a.dy, A['obj_centre'][2] - BOWL_HEIGHT_M])
    p_b = np.linalg.inv(Tg) @ np.r_[bottom, 1.0]                 # bowl bottom in the flange frame (held)
    model_qd = pp.robot_quadrants(Tbb)[a.quadrant]
    target = (Tbb @ np.r_[QUAD_XY[model_qd], 0.0, 1.0])[:3] + [BOWL_SHIFT_M[0], BOWL_SHIFT_M[1], BOWL_DROP_M]
    yaw = np.arctan2(target[1], target[0]) - np.arctan2(bottom[1], bottom[0])      # turn with joint 1
    Tp = rz(yaw) @ Tg                                             # the grasp turned about base z: where the bowl bottom goes
    drop = min(a.rise, BOWL_RISE_M)    # (보스 10/29: lift 30 cm) the descent stays 22 cm (30: 17 mm off the vertical in Q4)
    above_xyz = Tp[:3, 3] + (target - (Tp @ p_b)[:3]) + [0.0, 0.0, drop]
    # (10/27) the lift pose is near the wrist singularity (J5 ~ -9 deg): a Cartesian line swaps J4/J6 in > 10 deg steps.
    # IK from the lift pose with only joint 1 turned, then a joint-space line (tilt / height checked below)
    T_above = Tp.copy()
    T_above[:3, 3] = above_xyz
    seed = np.radians(lift[-1]) + np.radians([np.degrees(yaw), 0, 0, 0, 0, 0])
    q_above = k.ik(T_above, seed)
    assert q_above is not None and np.abs(k.fk(q_above)[:3, 3] - above_xyz).max() < 1e-3, 'IK failed above the basket'
    above = np.degrees(q_above)
    carry = [lift[-1] + s * (above - lift[-1]) for s in np.linspace(0, 1, 25)]
    # descend: the place IK seeded from above, a joint-space line (a Cartesian line flipped J4/J6 by > 100 deg near the
    # wrist singularity, 10/27), checked below to stay within 1.5 cm of the vertical and level (Q2-Q4: 10-11 mm)
    T_place = k.fk(np.radians(above))
    T_place[2, 3] -= drop
    q_place = k.ik(T_place, np.radians(above))
    assert q_place is not None and np.abs(k.fk(q_place)[:3, 3] - T_place[:3, 3]).max() < 1e-3, 'IK failed at the basket'
    down = [above + s * (np.degrees(q_place) - above) for s in np.linspace(0, 1, 13)]
    Ta = k.fk(np.radians(above))
    for q in down:
        T = k.fk(np.radians(q))
        off = np.hypot(*(T[:2, 3] - Ta[:2, 3]))
        tilt_d = np.degrees(np.arccos(np.clip((T[:3, :3] @ Ta[:3, :3].T @ [0, 0, 1.0])[2], -1, 1)))
        assert off <= 0.015 and tilt_d <= 5, f'descend leaves the vertical: {off * 1000:.0f} mm, {tilt_d:.1f} deg'
    step = max(np.abs(np.diff(np.array(down), axis=0)).max(), np.abs(np.diff(np.array(carry), axis=0)).max())
    assert np.abs(np.array(down[-1]) - above).max() < 60, 'descend turns a joint > 60 deg'

    lo, hi = np.array(ros2_nodes.joint_limits_deg()).T
    rim = Tbb[2, 3]
    for q in carry:
        T = k.fk(np.radians(q))
        assert (T @ p_b)[2] >= rim + 0.05, f'bowl bottom {(T @ p_b)[2]:.3f} m < rim + 5 cm on the carry'
        tilt = np.degrees(np.arccos(np.clip((T[:3, :3] @ Tg[:3, :3].T @ [0, 0, 1.0])[2], -1, 1)))
        assert tilt <= 15, f'bowl tilts {tilt:.0f} deg on the carry'
    for q in list(lift) + list(down) + carry + [home]:
        assert ((q > lo + 3) & (q < hi - 3)).all(), f'joint limit {np.round(q, 1)}'
    legs = {'lift': lift, 'to_above': carry, 'descend': down, 'ascend': down[::-1], 'to_home': [above, home]}
    if getattr(a, 'smooth', False):
        # (보스 10/29: the moves looked stepwise) grasp -> down in the quadrant as ONE spline, <= 20 knots: the lift a
        # Cartesian knot every 10 cm, the carry / descent joint lines cut into even steps (the controller spline gives
        # every knot the same time: even steps = even speed; the descent's shorter steps slow it into the basket)
        lift_s = lift_line(k, grasp, a.rise, n=int(round(a.rise / 0.10)) + 1)
        while np.abs(np.diff(np.array(lift_s), axis=0)).max() > 9.5 and len(lift_s) < 9:   # (10/30) even steps, see n_c
            lift_s = lift_line(k, grasp, a.rise, n=len(lift_s) + 1)
        n_d = 3                                            # (보스 10/29: bowl lift 50 cm) the carry steps grow to stay <= 20 knots
        # (보스 10/30: bowl and plate at one speed) carry steps ~ the lift's (9.5 deg; was 14): the spline time follows the
        # largest step, so one big carry step slowed the whole bowl move (plate carry_insert 43 cm/s, bowl was 22-38)
        n_c = min(int(np.ceil(np.abs(above - lift_s[-1]).max() / 9.5)), 20 - len(lift_s) - n_d)
        carry_s = [lift_s[-1] + s_ * (above - lift_s[-1]) for s_ in np.linspace(0, 1, n_c + 1)[1:]]
        down_s = [above + s_ * (np.array(down[-1]) - above) for s_ in np.linspace(0, 1, n_d + 1)[1:]]
        legs['carry_down'] = list(lift_s) + carry_s + down_s
        n_h = int(np.ceil(np.abs(home - above).max() / max(1.0, np.abs(np.array(down[-1]) - above).max() / n_d)))   # (10/30) steps as the rise's
        legs['up_home'] = [down[-1]] + [down[-1] + s_ * (above - np.array(down[-1])) for s_ in np.linspace(0, 1, n_d + 1)[1:]] \
            + [above + s_ * (home - above) for s_ in np.linspace(0, 1, n_h + 1)[1:]]
        assert len(legs['carry_down']) <= 20, f'carry_down {len(legs["carry_down"])} knots'
    print(f'basket {Tbb[:3, 3].round(3).tolist()}; bowl into {a.quadrant} (model {model_qd}) at {target.round(3).tolist()} '
          f'(bottom {BOWL_DROP_M * 100:.0f} cm over the rim)', flush=True)
    return pp.taught_plan({n: [list(map(float, w)) for w in v] for n, v in legs.items()}, Tbc, pp.C['plate_ik_bc'],
                          f'tesollo bowl to basket {a.quadrant} dx {a.dx:+.3f} dy {a.dy:+.3f}')


# (보스 10/29) after the grasp the hand follows the 2F-85 gripper's insert path: the gripper pipeline's planner run for the
# basket now, with the gripper holding a plate as in a real gripper run (its plate-in-tool and lift), its 11/3 demo
# settings (calibration/config.yaml basket_quadrant_sim, keep face) and the plates already in the quadrant.  The hand
# takes those joints with a constant wrist offset (J4-J6, --wrist; default = hand - gripper at the lift end), J1-J3 /
# the offset ramped in over the first half of the carry from the hand's own lift.  No hand collision model.
GRIPPER_REF = HERE.parent / 'calibration/data/real_pipeline/20261026_112417_plate_b1245h1g/preview_plan.json'   # 10/26 Q3 demo


def gripper_path(a, k, Tbb, Tbc, mq, lift, grasp, home):
    sys.path.insert(0, str(HERE.parent / 'rl'))
    import basket_env as be
    ref = json.loads(Path(a.gripper_ref).read_text())
    Hg = np.array(ref['plate_in_tcp'])
    tg = np.eye(4)
    tg[2, 3] = pp.C['plate_ik_bc'].get('tool_length_m', 0.165)
    g_lift = ref['legs']['lift']['waypoints_deg']
    q = dict(pp.C['plate_ik_bc'], basket_carry_mode='rule', basket_carry_rrt_fallback=False,
             basket_carry_rise_m=None, _fast_unchecked_carry=True)   # sparse demo path; no post-grasp collision sweep
    sq = pp.C['plate_ik_bc']['basket_quadrant_sim'][mq] if 'basket_quadrant_sim' in pp.C['plate_ik_bc'] else pp.C['basket_quadrant_sim'][mq]
    kw = dict(tilt_deg=sq['lean'] * be.B['full']['tine_lean_deg'][mq],
              centre_h=sq['centre_h'] + q.get('basket_quadrant_sim_real_dh_m', 0.0),
              x_shift_m=sq['aim_m'] + q.get('basket_quadrant_model_dx_m', {}).get(mq, 0.0),
              faces=(-1.0, 1.0) if sq['flip'] else (1.0, -1.0),        # the demo's face first (keep_face alone: none at 10/29)
              y_extra_m=q.get('basket_quadrant_sim_real_dy_m', {}).get(mq, 0.0))
    rq = pp.robot_quadrants(Tbb)
    plates = [(rq[v.split(':')[0]], pp.pin_gap(Tbb, rq[v.split(':')[0]], int(v.split(':')[1]))) for v in a.plates]
    gap = pp.pin_gap(Tbb, mq, a.pin)
    print(f'gripper path: plate-in-tool / lift from {a.gripper_ref}; {mq} gap {gap}, plates there {plates}, {kw}', flush=True)
    path, rep_ = pp.quadrant_plan(Tbb, mq, gap, Hg, tg, g_lift[-1], Tbc, q, yaw_trim_deg=q.get('basket_quadrant_yaw_trim_deg', 0.0),
                                  grasp_legs={'to_grasp': ref['legs']['to_grasp']['waypoints_deg'], 'lift': g_lift},
                                  grasp_deg=ref['legs']['to_grasp']['waypoints_deg'][-1], plates=plates, **kw)
    if path is None:
        sys.exit('NO PLAN (gripper path): ' + str(__import__('collections').Counter(t[1].split(' ')[0] for t in rep_.get('tried', []))))
    plan = json.loads(Path(path).read_text())
    L = plan['legs']
    h_end, g0 = np.array(lift[-1]), np.array(L['to_above']['waypoints_deg'][0])
    off = np.zeros(6)
    off[3:] = (h_end - g0)[3:] if a.wrist is None else np.array(a.wrist, float)
    w = np.array(L['to_above']['waypoints_deg']) + off
    ramp = np.clip(1 - np.linspace(0, 2, len(w)), 0, 1)[:, None]      # the hand's lift end -> the gripper path by mid-carry
    w = w + ramp * (h_end - w[0])
    L['to_above']['waypoints_deg'] = w.tolist()
    for leg in ('descend', 'ascend'):
        L[leg]['waypoints_deg'] = (np.array(L[leg]['waypoints_deg']) + off).tolist()
    th = np.array(L['to_home']['waypoints_deg'])
    L['to_home']['waypoints_deg'] = (th + np.linspace(1, 0, len(th))[:, None] * off).tolist()   # the offset fades out at home
    L.pop('abort_home', None)
    L['to_grasp']['waypoints_deg'] = [home, list(map(float, grasp))]
    L['lift']['waypoints_deg'] = [list(map(float, x)) for x in lift]
    lim = np.array([135.0 if j == 4 else (160.0 if j == 2 else 360.0) for j in range(6)]) - 3
    for leg in ('to_above', 'descend', 'ascend', 'to_home'):
        W = np.array(L[leg]['waypoints_deg'])
        if (np.abs(W) > lim).any():
            sys.exit(f'REFUSED (gripper path): {leg} beyond a joint limit with wrist offset {off[3:].round(1).tolist()}')
    plan['source'] += f' | gripper path for the hand, wrist offset J4-J6 {off[3:].round(1).tolist()} deg (no hand collision check)'
    Path(path).write_text(json.dumps(plan, indent=1))
    sparse_demo_plan(path, k, lift, home, tool_m=float(tg[2, 3]))   # the gripper plate model
    plan = json.loads(Path(path).read_text())
    L = plan['legs']
    print(f'wrist offset J4/J5/J6 {off[3:].round(1).tolist()} deg (--wrist d4 d5 d6 to change)', flush=True)
    print(f'OK: gripper path, carry {rep_.get("carry")}, gripper clearance/margin {rep_.get("clearance_ratio")}')
    for leg, v in L.items():
        print(f'  {leg}: {len(v["waypoints_deg"])} waypoints, end {[round(x, 1) for x in v["waypoints_deg"][-1]]}')
    print(f'  total: {plan["demo_sparse"]["total"]} waypoints '
          f'(approach 3 + lift {plan["demo_sparse"]["lift"]} + carry {plan["demo_sparse"]["carry"]} + '
          f'place cycle {plan["demo_sparse"]["place_cycle"]})')
    merge_lift_carry(path)
    print('plan:', path)


def merge_lift_carry(path):
    """(as the gripper pipeline: 'lift + carry를 한 spline으로') lift + to_above as the single leg lift_carry."""
    mp = pp.merged_plan([(path, 'lift'), (path, 'to_above')], 'lift_carry')
    print('merged:', mp if mp is not None else 'none (too many waypoints: lift, to_above one after another)')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--object', default='plate', choices=('plate', 'bowl'))
    p.add_argument('--quadrant', choices=('Q1', 'Q2', 'Q3', 'Q4'), help='default Q3 (plate), Q1 (bowl)')
    p.add_argument('--pin', type=int, default=6, help='pin pair (1 = farthest from the robot), as the demo')
    p.add_argument('--dx', type=float, default=0.0)
    p.add_argument('--dy', type=float, default=0.0)
    p.add_argument('--plate', type=float, nargs=3, help='plate centre (base, m) at the grasp; default: saved reference + dx/dy')
    p.add_argument('--centre_h', type=float, default=0.185,
                   help='plate centre above the basket floor at the end of the descent [m] (smaller = deeper); 0.185 = the 2F-85 '
                        'demo (--sim_q: centre_h 0.165 + real dh 0.020)')
    p.add_argument('--model', action='store_true', help='ignore a measured plate.in_hand (measure_held.py): flat-plate model')
    p.add_argument('--hang_deg', type=float, default=0.0, help='plate turned in the hand after the grasp [deg] (see HANG_AXIS)')
    p.add_argument('--sim_corr', default='', help='(opt-in) rl/hand_basket.py cem json: this pin\'s learned (dx, dy, dz) in the '
                                                  'basket frame -> x_shift (on the config trim), y_extra, centre_h + dz.  Learned '
                                                  'for one nominal hang with a measured in-hand pose: use with measure_held.py')
    p.add_argument('--wrist_less', type=float, default=0.0,
                   help='(opt-in, 보스 10/29) turn the hand this much less from the grasp to the insert [deg], about the plate '
                        'bottom on the slot (the flat-plate model turns ~96; the real plate needs ~45 less)')
    p.add_argument('--gripper_path', action='store_true', help='(보스 10/29) after the grasp follow the 2F-85 gripper insert '
                   'path (its planner at this basket) with a wrist offset; no hand collision check')
    p.add_argument('--gripper_ref', default=str(GRIPPER_REF), help='real gripper run preview_plan.json: plate-in-tool + lift')
    p.add_argument('--wrist', type=float, nargs=3, help='--gripper_path: J4 J5 J6 offset [deg] on the gripper joints '
                   '(default: hand - gripper at the lift end)')
    p.add_argument('--plates', nargs='*', default=[], help='--gripper_path: plates already in the basket, Q3:6 ...')
    p.add_argument('--smooth', action='store_true', help='(bowl) also write carry_down (grasp -> in the quadrant, one '
                   'spline, even knots) and up_home (보스 10/29)')
    p.add_argument('--rise', type=float, default=PLATE_RISE_M,
                   help='plate lift before the carry [m] (default 0.14 = two 7 cm moves, three knots)')
    p.add_argument('--basket', default=str(FIXED_BASKET), help='basket_vision.json (T); default the fixed 11/3 file (stale: '
                                                                'measure it, see dg5fs_sdk.py / calibration/basket_vision.py)')
    a = p.parse_args()
    a.basket = str(START_CWD / a.basket) if not Path(a.basket).is_absolute() else a.basket
    stale = Path(a.basket).resolve() == FIXED_BASKET.resolve()
    if not stale:
        global PLATE_SHIFT_M, BOWL_SHIFT_M
        PLATE_SHIFT_M, BOWL_SHIFT_M = (0.0, 0.0), (0.0, 0.0)
    print(f'basket from {a.basket}' + (' (FIXED 11/3 file + the 10/27 by-eye shifts)' if stale else ' (measured: no shifts)'),
          flush=True)
    a.quadrant = a.quadrant or ('Q3' if a.object == 'plate' else 'Q1')
    if a.object == 'bowl' and a.rise == PLATE_RISE_M:
        a.rise = BOWL_RISE_M
    if a.object == 'bowl':
        k = pp.Kinematics()
        args_ = (a, k, np.array(json.loads(Path(a.basket).read_text())['T']),
                 np.array(cc.load_json(cc.latest_t_base_cam())['T_base_cam']))
        try:
            path = bowl_plan(*args_)
        except AssertionError as e:                   # (10/30) the other wrist branch can fail a check the old one met
            print(f'bowl plan on the nearest wrist branch failed ({e}); the IK\'s own lift knots (J4 may turn)', flush=True)
            LIFT_NEAR[0] = False
            path = bowl_plan(*args_)
        plan = json.loads(Path(path).read_text())
        print('OK: bowl')
        for leg, v in plan['legs'].items():
            print(f'  {leg}: {len(v["waypoints_deg"])} waypoints, end {[round(x, 1) for x in v["waypoints_deg"][-1]]}')
        merge_lift_carry(path)
        print('plan:', path)
        return
    A = yaml.safe_load((HERE / 'arm_poses.yaml').read_text())['plate']
    if a.plate is None:
        if 'obj_centre' not in A:
            sys.exit('no plate reference: tesollo/perceive.py --object plate --save_ref first, or give --plate x y z')
        a.plate = [A['obj_centre'][0] + a.dx, A['obj_centre'][1] + a.dy, A['obj_centre'][2]]
    k = pp.Kinematics()
    grasp = shifted_grasp(k, A['posj_deg'], a.dx, a.dy)
    lift = lift_line(k, grasp, a.rise, n=PLATE_LIFT_WAYPOINTS)
    home = list(map(float, pp.C['robot']['home_joints_deg']))
    tool = np.eye(4)
    tool[2, 3] = HAND_LENGTH_M
    centre = np.array(a.plate) - [0.0, 0.0, 0.015]            # perceive gives the rim top: the plate middle ~1.5 cm lower
    H = hand_plate(k, grasp, centre, tool)
    measured = 'in_hand' in A and not a.model
    if measured:                                                  # tesollo/measure_held.py: FP of the held plate
        H = np.array(A['in_hand']['H'])
        print(f'plate in the hand: measured {A["in_hand"]["measured"]} (normal {A["in_hand"]["turn_vs_flat_deg"]} deg from '
              'the flat model); no hang / shift corrections', flush=True)
    elif a.hang_deg:
        from scipy.spatial.transform import Rotation
        ax = np.array(HANG_AXIS) / np.linalg.norm(HANG_AXIS)
        Rh = np.eye(4)
        Rh[:3, :3] = Rotation.from_rotvec(ax * np.radians(a.hang_deg)).as_matrix()
        H = Rh @ H
    # the gripper pipeline's default carry (pipeline.configure_carry_planner 'direct'): rise / across / drop by rule, no
    # saved prior, no RRT (10/27 보스: the RRT carry from the config's basket_carry_mode looked wrong on the real arm)
    q = dict(pp.C['plate_ik_bc'], tool_length_m=HAND_LENGTH_M, tool_radius_m=HAND_RADIUS_M,
             basket_carry_mode='rule', basket_carry_rrt_fallback=False,
             basket_carry_rise_m=None, _fast_unchecked_carry=True)
    Tbb = np.array(json.loads(Path(a.basket).read_text())['T'])
    model_qd = pp.robot_quadrants(Tbb)[a.quadrant] if hasattr(pp, 'robot_quadrants') else a.quadrant
    gap = pp.pin_gap(Tbb, model_qd, a.pin)
    Tbc = np.array(cc.load_json(cc.latest_t_base_cam())['T_base_cam'])
    print(f'basket {Tbb[:3, 3].round(3).tolist()} from {a.basket}; {a.quadrant} (model {model_qd}) pin {a.pin} -> gap {gap}; '
          f'plate centre {np.round(centre, 3).tolist()}', flush=True)
    if a.gripper_path:
        return gripper_path(a, k, Tbb, Tbc, model_qd, lift, grasp, home)
    grasp_legs = {'to_grasp': [home, list(map(float, grasp))], 'lift': [list(map(float, w)) for w in lift]}
    kw = {}
    if a.sim_corr:
        cj = json.loads((START_CWD / a.sim_corr).read_text())
        d = cj['corr'].get(f'{a.quadrant}:{a.pin}')
        if d is None:
            sys.exit(f'--sim_corr: no correction for {a.quadrant}:{a.pin} in {a.sim_corr}: {sorted(cj["corr"])}')
        print(f'sim correction {a.quadrant}:{a.pin} (job {cj.get("job")}): dx {d[0] * 1000:+.1f}, dy {d[1] * 1000:+.1f}, '
              f'dz {d[2] * 1000:+.1f} mm', flush=True)
    path, rep = pp.quadrant_plan(Tbb, model_qd, gap, H, tool, list(map(float, lift[-1])), Tbc, q, grasp_legs=grasp_legs,
                                 grasp_deg=list(map(float, grasp)), centre_h=a.centre_h, **kw)
    if path is not None and any(PLATE_SHIFT_M) and not measured and not a.sim_corr:
        end = json.loads(Path(path).read_text())['legs']['descend']['waypoints_deg'][-1]
        R = (k.fk(np.radians(end)) @ tool)[:3, :3]
        C = np.eye(4)
        C[:3, 3] = R.T @ -np.array([PLATE_SHIFT_M[0], PLATE_SHIFT_M[1], 0.0])
        H = C @ H                                                   # plate in the tool frame, moved -shift (base) at the insert
        path, rep = pp.quadrant_plan(Tbb, model_qd, gap, H, tool, list(map(float, lift[-1])), Tbc, q, grasp_legs=grasp_legs,
                                     grasp_deg=list(map(float, grasp)), centre_h=a.centre_h)
    if path is not None and a.sim_corr:
        # (10/28) as rl/hand_basket.py learned it: the planned descent moved by (dx, dy, dz) in the basket frame, the carry
        # ramped into it.  Through the planner's offsets instead, its wall model refused every candidate (Q3:6: +9.5 mm y)
        # while the sim plate went in (pin penetration <= 1.1 mm along these paths).
        plan = json.loads(Path(path).read_text())
        sh = Tbb[:3, :3] @ np.array(d, float)

        def moved(w, frac):
            out, prev = [], np.radians(w[0])
            for qq, f in zip(w, frac):
                T = k.fk(np.radians(qq))
                T[:3, 3] += f * sh
                qn = k.ik(T, prev)
                assert qn is not None and np.abs(k.fk(qn)[:3, 3] - T[:3, 3]).max() < 1e-3, 'IK failed moving the descent'
                out.append(np.degrees(qn).tolist())
                prev = qn
            assert np.abs(np.diff(np.array(out), axis=0)).max() < 15, 'IK jump moving the descent'
            return out
        L = plan['legs']
        L['to_above']['waypoints_deg'] = moved(L['to_above']['waypoints_deg'], np.linspace(0, 1, len(L['to_above']['waypoints_deg'])))
        for leg in ('descend', 'ascend'):
            L[leg]['waypoints_deg'] = moved(L[leg]['waypoints_deg'], np.ones(len(L[leg]['waypoints_deg'])))
        for leg in ('to_home', 'abort_home'):                    # they start where the moved ascend ends
            if leg in L:
                L[leg]['waypoints_deg'][0] = L['ascend']['waypoints_deg'][-1]
        plan['source'] += f' + sim correction {a.quadrant}:{a.pin} {np.round(np.array(d) * 1000, 1).tolist()} mm'
        Path(path).write_text(json.dumps(plan, indent=1))
    if path is not None and a.wrist_less:
        # (보스 10/29 real Q3 photo) the slot position is right but the plate leans ~45 deg: the flat-plate model turns the
        # hand ~96 deg from the grasp, the real held plate needs about 45 less.  The hand turn (grasp -> insert, about its
        # own axis) is cut by --wrist_less about the model plate's lowest point (stays on the slot), full on the descent /
        # ascent and ramped in over to_above.  H then follows the real plate: upright where the model plate was.
        from scipy.spatial.transform import Rotation
        plan = json.loads(Path(path).read_text())
        L = plan['legs']
        cloud = pp.plate_cloud(H, tool)
        tcp = lambda qq: k.fk(np.radians(qq)) @ tool                                     # noqa: E731
        Tg, Ti = tcp(grasp), tcp(L['descend']['waypoints_deg'][-1])
        rv = Rotation.from_matrix(Tg[:3, :3].T @ Ti[:3, :3]).as_rotvec()
        turn = np.degrees(np.linalg.norm(rv))
        axis = Ti[:3, :3] @ (rv / np.linalg.norm(rv))                                    # base frame

        def turned(w, frac):
            out, prev = [], np.radians(w[0])
            for qq, f in zip(w, frac):
                F = k.fk(np.radians(qq))
                piv = (cloud @ F[:3, :3].T + F[:3, 3])
                piv = piv[np.argmin(piv[:, 2])]
                Rb = Rotation.from_rotvec(-axis * np.radians(f * a.wrist_less)).as_matrix()
                T = F.copy()
                T[:3, :3] = Rb @ F[:3, :3]
                T[:3, 3] = Rb @ (F[:3, 3] - piv) + piv
                qn = k.ik(T, prev)
                assert qn is not None and np.abs(k.fk(qn)[:3, 3] - T[:3, 3]).max() < 1e-3, 'IK failed turning the wrist'
                out.append(np.degrees(qn).tolist())
                prev = qn
            assert np.abs(np.diff(np.array(out), axis=0)).max() < 15, 'IK jump turning the wrist'
            return out
        for leg in ('descend', 'ascend'):
            L[leg]['waypoints_deg'] = turned(L[leg]['waypoints_deg'], np.ones(len(L[leg]['waypoints_deg'])))
        # the carry ramps into the turned above pose in joint space (a Cartesian ramp crossed J5 ~ 0: J4/J6 flipped 29 deg)
        w = np.array(L['to_above']['waypoints_deg'])
        dq = np.array(L['descend']['waypoints_deg'][0]) - w[-1]
        L['to_above']['waypoints_deg'] = (w + np.linspace(0, 1, len(w))[:, None] * dq).tolist()
        for leg in ('to_home', 'abort_home'):
            if leg in L:
                L[leg]['waypoints_deg'][0] = L['ascend']['waypoints_deg'][-1]
        F_new = tcp(L['descend']['waypoints_deg'][-1])
        rv2 = Rotation.from_matrix(Tg[:3, :3].T @ F_new[:3, :3]).as_rotvec()
        print(f'wrist: hand turn {turn:.1f} -> {np.degrees(np.linalg.norm(rv2)):.1f} deg '
              f'(--wrist_less {a.wrist_less}); post-grasp collision check skipped', flush=True)
        plan['source'] += f' + wrist {-a.wrist_less:+.0f} deg'
        Path(path).write_text(json.dumps(plan, indent=1))
    if path is None:
        sys.exit(f'NO PLAN: ' + str(__import__('collections').Counter(t[1].split(' ')[0] for t in rep.get('tried', []))) + f' over {len(rep.get("tried", []))} tries')
    sparse_demo_plan(path, k, lift, home)
    plan = json.loads(Path(path).read_text())
    print(f'OK: phi {rep.get("phi_deg")}, carry {rep.get("carry")}, accepted {rep.get("accepted")}, '
          f'clearance/margin {rep.get("clearance_ratio")}')
    for leg, v in plan['legs'].items():
        w = v['waypoints_deg']
        print(f'  {leg}: {len(w)} waypoints, end {[round(x, 1) for x in w[-1]]}')
    print(f'  total: {plan["demo_sparse"]["total"]} waypoints '
          f'(approach 3 + lift {plan["demo_sparse"]["lift"]} + carry {plan["demo_sparse"]["carry"]} + '
          f'place cycle {plan["demo_sparse"]["place_cycle"]})')
    merge_lift_carry(path)
    print('plan:', path)


if __name__ == '__main__':
    main()
