"""Plate task plans that need no Isaac at run time (pipeline.py --task plate): the per-radius sim plan library
(any direction by turning joint 1) and the taught basket-slot paths (teach / replay).  Conda foundationpose Python."""
import json
import os
import socket
import subprocess
from pathlib import Path
import time

import numpy as np

from calib_core import DATA, cpath, load_config
from robot_model import HOME_JOINTS_DEG, Kinematics, polyline_path

C = load_config()
ROOT = __import__("pathlib").Path(__file__).resolve().parent.parent


def split_long_moves(waypoints, max_deg):
    # Keep offline planning independent of pipeline's ROS/UDP initialization.
    out=[np.asarray(waypoints[0],dtype=float)]
    for b in waypoints[1:]:
        a,b=out[-1],np.asarray(b,dtype=float)
        n=max(1,int(np.ceil(np.degrees(np.abs(b-a).max())/max_deg-1e-9)))
        out += [a+(b-a)*k/n for k in range(1,n+1)]
    return out


def fresh_joints(msg, now=None):
    from pipeline import fresh_joints as fresh
    return fresh(msg, now)


# ================================================================ plan library (rl/eval.py library)

def library_target(p, q):
    """Plate root (FP) -> (radius m, azimuth rad) of the rollout point: FP + offset, the offset turned with the plate
    azimuth (it was measured with the plate straight ahead; the whole motion turns with joint 1)."""
    az = np.arctan2(p[1], p[0])
    c, s = np.cos(az), np.sin(az)
    ox, oy = q['offset_m']
    t = np.array([p[0] + c * ox - s * oy, p[1] + s * ox + c * oy])
    return float(np.hypot(*t)), float(np.arctan2(t[1], t[0]))


_CHECKS = {}


def library_checkers(Tbc, q):
    """FK clearance like the sim shield (rl/config.yaml shield): frame + D455 body >= frame margin, table >= table
    margin, with the 2F-85 as a tool cylinder.  Built once per process."""
    from robot_model import CollisionChecker, D455_BOX, frame_layout, measured_cell
    import yaml
    sh = yaml.safe_load((ROOT / 'rl' / 'config.yaml').read_text())['shield']
    cell=measured_cell()
    key=json.dumps(dict(camera=np.asarray(Tbc).tolist(),cell=cell,
                        tool_length=q.get('tool_length_m',.165),tool_radius=q.get('tool_radius_m',.06),
                        tool_model=q.get('tool_model','cylinder'),
                        frame_margin=sh['frame_margin'],table_margin=sh['table_margin']),sort_keys=True)
    if key not in _CHECKS:
        kin, boxes = Kinematics(), frame_layout(cell)
        frame = [b for b in boxes if b[0] != 'table'] + [('d455_body', np.asarray(Tbc)[:3, 3].tolist(), list(D455_BOX))]
        out = {}
        for name, bx, m in (('frame', frame, sh['frame_margin']), ('table', [b for b in boxes if b[0] == 'table'], sh['table_margin'])):
            c = CollisionChecker(kin, bx, margin=m, disk_cache=True)   # (10/19) fixed cell: field read from disk
            c.set_tool(q.get('tool_length_m', 0.165), q.get('tool_radius_m', 0.06))
            if q.get('tool_model')=='sim_open_gripper':
                import sys
                sys.path.insert(0,str(ROOT/'rl'))
                from rl_utils import gripper_surface_points, load_config as rl_config
                c.tool=gripper_surface_points(rl_config())
            elif q.get('tool_model','cylinder')!='cylinder':
                raise ValueError('Unknown library tool model')
            out[name] = (c, m)
        _CHECKS[key] = out
    return _CHECKS[key]


def library_ok_radii(q):
    lib = np.load(cpath(q['library']))
    r = lib['radius_m'][lib['ok']]
    return float(r.min()), float(r.max())


def height_library_config(q, plate_z):
    """Opt-in height index. Never extrapolate an untested height or silently fall back to dz=0."""
    if not q.get('library_height_index'):
        return q
    if plate_z is None or not np.isfinite(plate_z):
        raise RuntimeError('Height-axis library needs a finite pre-approach plate root z')
    index=json.loads(cpath(q['library_height_index']).read_text())
    entries=[e for e in index['entries'] if e['n_ok']>0]
    if not entries:
        raise RuntimeError('Height-axis index contains no successful candidate')
    entry=min(entries,key=lambda e:abs(e['plate_root_z_m']-plate_z))
    error=abs(entry['plate_root_z_m']-plate_z)
    if error>index.get('height_tolerance_m',.0025):
        raise RuntimeError(f'Plate z {plate_z:.4f} m outside tested library heights (nearest error {error*1000:.1f} mm)')
    return dict(q,library=entry['library'],selected_plate_z_m=entry['plate_root_z_m'],height_error_m=error)


def nearest_direction_candidates(lib,r,tolerance=.004):
    """Match the nearest successful radius separately for each phi/pitch, then rank directions."""
    good=np.flatnonzero(lib['ok'])
    good=good[np.abs(lib['radius_m'][good]-r)<=tolerance]
    keys=lib.files if hasattr(lib,'files') else lib
    phi=lib['phi_deg'] if 'phi_deg' in keys else np.zeros(len(lib['ok']))
    pitch=lib['pitch_deg'] if 'pitch_deg' in keys else np.zeros(len(lib['ok']))
    selected=[]
    for p,t in sorted(set(zip(phi[good],pitch[good]))):
        ids=good[(phi[good]==p)&(pitch[good]==t)]
        selected.append(ids[np.argmin(abs(lib['radius_m'][ids]-r))])
    return np.asarray(selected,dtype=int)


def _library_plan_candidate(lib,i,r,az,q,checks,plate_z):
    c, end = int(lib['close_step'][i]), int(lib['success_step'][i])
    Q = lib['q_deg'][:end + 1, i].astype(float).copy()
    Q[:, 0] += np.degrees(az)
    home = np.array(HOME_JOINTS_DEG, dtype=float)

    def thin(path, step):
        out = [path[0]]
        for w in path[1:]:
            if np.abs(w - out[-1]).max() >= step:
                out.append(w)
        if np.abs(path[-1] - out[-1]).max() > 1e-3:
            out.append(path[-1])
        return np.array(out)

    direct = np.degrees(split_long_moves([np.radians(home), np.radians(Q[0])], 20.0))   # ONE joint move, split for the mover
    grasp = thin(Q[:c + 1], 1.5)
    lift = thin(Q[c:], 1.5)
    legs = dict(to_grasp=np.concatenate([direct, grasp[1:]]), lift=lift, lower=lift[::-1],
                from_grasp=np.concatenate([grasp[::-1], direct[::-1][1:]]))
    plan = dict(dry_run=True, source='plan_library', library=str(cpath(q['library'])), library_radius_m=float(lib['radius_m'][i]),
                radius_m=r, azimuth_deg=float(np.degrees(az)), checkpoint=q['checkpoint'],
                created=time.strftime('%Y-%m-%d %H:%M:%S'), margin_m=0.0, gripper_force=64,
                cell_margins_m={k: m for k, (_, m) in checks.items()},
                tool_model=q.get('tool_model','cylinder'),
                measured_plate_z_m=plate_z, selected_plate_z_m=q.get('selected_plate_z_m'),
                height_error_m=q.get('height_error_m'),
                approach_phi_deg=float(lib['phi_deg'][i]) if 'phi_deg' in lib.files else 0.0,
                approach_pitch_deg=float(lib['pitch_deg'][i]) if 'pitch_deg' in lib.files else 0.0,
                gripper_close_after_leg='to_grasp', gripper_open_after_leg='lower', legs={}, reasons=[])
    for name, wp in legs.items():
        dense = polyline_path(np.radians(wp), 0.25)
        clear = {k: float(c.path_clearance(dense)) for k, (c, _) in checks.items()}
        # the mover compares min_clearance_straight_m with margin_m: store the clearance BEYOND each margin
        plan['legs'][name] = dict(waypoints_deg=np.round(wp, 3).tolist(), clearance_m=clear, planner='plan library',
                                  min_clearance_straight_m=min(clear[k] - m for k, (_, m) in checks.items()))
        for k, (_, m) in checks.items():
            if clear[k] < m:
                plan['reasons'].append(f'{name}: {clear[k] * 1000:.0f} mm from the {k} (< {m * 1000:.0f} mm)')
        print(f'   [library] {name:10s}: {len(wp):3d} waypoints, frame {clear["frame"] * 1000:4.0f} mm, '
              f'table {clear["table"] * 1000:4.0f} mm', flush=True)
    plan['accepted'] = not plan['reasons']
    return plan


def library_plans(r, az, q, Tbc, plate_z=None):
    """(10/19 night, opt-in) every accepted library grasp at (r, az), best clearance first (plan_from_library keeps only
    the first).  For choosing the grasp together with the insert: a far plate's best-clearance grasp can leave no insert."""
    import contextlib
    import io
    q = height_library_config(q, plate_z)
    lib = np.load(cpath(q['library']), allow_pickle=False)
    cands = nearest_direction_candidates(lib, r, q.get('library_tol_m', .004))
    if not len(cands):
        raise RuntimeError(f'radius {r:.3f} m has no successful library path within tolerance')
    checks = library_checkers(Tbc, q)
    with contextlib.redirect_stdout(io.StringIO()):
        plans = [_library_plan_candidate(lib, int(i), r, az, q, checks, plate_z) for i in cands]
    lo, hi = np.array(__import__('ros2_nodes').joint_limits_deg()).T
    ok = [p for p in plans if p['accepted'] and all(((np.array(leg['waypoints_deg']) > lo + 3) &
                                                     (np.array(leg['waypoints_deg']) < hi - 3)).all() for leg in p['legs'].values())]
    ok.sort(key=lambda p: (min(leg['min_clearance_straight_m'] for leg in p['legs'].values()),
                           -abs(p['approach_phi_deg']), -abs(p['approach_pitch_deg'])), reverse=True)
    if not ok:
        raise RuntimeError('no library grasp clear of the cell')
    return ok


def plan_from_library(r,az,q,Tbc,plate_z=None):
    """Nearest validated radius/height; choose the surviving phi/pitch with most cell clearance.
    One legacy radius path remains supported. No hardware commands."""
    import contextlib
    import io
    q=height_library_config(q,plate_z)
    lib=np.load(cpath(q['library']),allow_pickle=False)
    candidates=nearest_direction_candidates(lib,r,q.get('library_tol_m',.004))
    if not len(candidates):
        raise RuntimeError(f'radius {r:.3f} m has no successful library path within tolerance')
    checks=library_checkers(Tbc,q)
    plans=[]
    for i in candidates:
        with contextlib.redirect_stdout(io.StringIO()):
            plans.append(_library_plan_candidate(lib,int(i),r,az,q,checks,plate_z))
    lo,hi=np.array(__import__('ros2_nodes').joint_limits_deg()).T   # (10/15 실기) controller J3 +-135: skip, don't alarm
    for p in plans:
        bad=[n for n,leg in p['legs'].items() if not ((np.array(leg['waypoints_deg'])>lo+3)&(np.array(leg['waypoints_deg'])<hi-3)).all()]
        if bad:
            p['accepted']=False
            p['reasons']=list(p.get('reasons',[]))+[f'controller joint limit (3 deg margin) in {",".join(bad)}']
    valid=[p for p in plans if p['accepted']]
    def score(p):
        clearance=min(leg['min_clearance_straight_m'] for leg in p['legs'].values())
        return (clearance,-round(abs(p['library_radius_m']-r),9),
                -abs(p['approach_phi_deg']),-abs(p['approach_pitch_deg']))
    plan=max(valid or plans,key=score)
    plan['evaluated_candidates']=len(plans)
    for name,leg in plan['legs'].items():
        clear=leg['clearance_m']
        print(f'   [library] {name:10s}: {len(leg["waypoints_deg"]):3d} waypoints, '
              f'frame {clear["frame"]*1000:4.0f} mm, table {clear["table"]*1000:4.0f} mm',flush=True)
    out = DATA / 'rl_plans' / (time.strftime('%Y%m%d_%H%M%S') + f'_{int(time.time() * 1000) % 1000:03d}')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'plan.json').write_text(json.dumps(plan, indent=1))
    if not plan['accepted']:
        raise RuntimeError('library plan too close to the cell: ' + '; '.join(plan['reasons']))
    return out / 'plan.json'



# ================================================================ basket slots: teaching + replay (2026-10-08)

TEACH_DIR = DATA / 'teach'
TEACH_POSES = (('carry', '운반 자세: 접시를 든 채 바구니 쪽으로 옮기기 좋은 높은 자세'),
               ('above', '슬롯 바로 위: 접시를 슬롯 방향으로 맞추고 핀보다 3~5 cm 위'),
               ('insert', '삽입 완료: 접시 바닥이 슬롯 바닥에 닿기 직전(핀 사이)'),
               ('retreat', '후퇴: 그리퍼를 연 뒤 접시를 건드리지 않고 빠져나온 자세'))


def next_slot():
    f = TEACH_DIR / 'next_slot.json'
    return json.loads(f.read_text())['slot'] if f.is_file() else 1


def set_next_slot(k):
    TEACH_DIR.mkdir(parents=True, exist_ok=True)
    (TEACH_DIR / 'next_slot.json').write_text(json.dumps(dict(slot=k, updated=time.strftime('%Y-%m-%d %H:%M:%S'))))


def teach_slot(k, rec, choose_gripper, lift_end_deg, meta=None):
    """Human moves the robot with the pendant (jog / direct teaching); Enter stores the live joints + flange posx.
    The pipeline only READS joints here.  The gripper opens (Human choice) between `insert` and `retreat`."""
    print(f'\n=== 슬롯 {k} 티칭 ===\n펜던트로 로봇을 움직이세요 (이 동안 PC는 관절값만 읽습니다).\n'
          f'각 자세에 도착하면 Enter. 다시 찍으려면 r Enter, 그만두려면 q Enter.', flush=True)
    poses = {}
    for name, what in TEACH_POSES:
        if name == 'retreat':
            choose_gripper(f'slot{k}_release')
        while True:
            ans = input(f'  [{name}] {what} > ').strip().lower()
            if ans == 'q':
                raise RuntimeError('teaching cancelled')
            if ans == 'r':
                continue
            msg = rec.last
            if not fresh_joints(msg):
                print('  관절값이 오래됐습니다 (bringup 연결 확인). 다시 Enter.', flush=True)
                continue
            poses[name] = dict(posj=[float(v) for v in msg['posj']], posx=[float(v) for v in msg['posx']])
            rec.event('teach_pose', slot=k, pose=name, **poses[name])
            print(f'     저장: posj {np.round(msg["posj"], 1).tolist()}', flush=True)
            break
    meta = dict(meta or {})
    if meta.get('quadrant'):                                 # (10/14 보스) BC label: where the plate actually went
        ans = input(f'  [{meta["quadrant"]}] 접시가 들어간 틈 번호 (N = N번과 N+1번 빗살 사이, 1 = 로봇에서 가장 먼 빗살; 모르면 Enter) > ').strip()
        meta['landed_pin'] = int(ans) if ans.isdigit() else None
    TEACH_DIR.mkdir(parents=True, exist_ok=True)
    path = TEACH_DIR / f'slot_{k:02d}.json'
    if path.exists():
        path.replace(path.with_name(f'slot_{k:02d}_{time.strftime("%Y%m%d_%H%M%S")}.json'))   # keep older teachings
    path.write_text(json.dumps(dict(slot=k, created=time.strftime('%Y-%m-%d %H:%M:%S'), poses=poses,
                                    lift_end_deg=[float(v) for v in lift_end_deg], gripper_force=64, **meta), indent=1))
    print(f'슬롯 {k} 저장: {path}\n로봇은 펜던트로 홈에 돌려놓고, 자동 모드로 바꾼 뒤 run.sh connect 를 다시 실행하세요.', flush=True)
    return path


def taught_plan(points, Tbc, q, source):
    """Plan.json from taught joint keyposes: points = {leg: [q_deg, ...]}.  Each leg is densified on the straight
    joint lines between its keyposes (>= 2 deg steps, <= ~90 waypoints for the mover spline) so the smooth spline does
    not swing away from the taught path; FK frame clearance is checked (the basket is NOT in the cell model)."""
    def dense(pts, name):
        pts = [np.asarray(p, dtype=float) for p in pts]
        travel = sum(np.abs(b - a).max() for a, b in zip(pts[:-1], pts[1:]))
        # (10/13) 2-deg steps made the spline ~2x slower than the grasp leg (20-deg steps) at the same set speed: only the
        # plate's way down keeps them; every other leg is split like the grasp leg
        step = 2.0 if ('descend' in name or name in ('shift', 'to_policy_start')) else q.get('transport_step_deg', 20.0)
        # >= 3 moves per leg: a 1-waypoint movesj is accepted by the controller but does not move (10/12 real)
        return np.degrees(split_long_moves([np.radians(p) for p in pts], min(max(step, travel / 85), max(travel / 3, 1e-3))))

    checks = library_checkers(Tbc, q)
    plan = dict(dry_run=True, source=source, created=time.strftime('%Y-%m-%d %H:%M:%S'), margin_m=0.0, legs={}, reasons=[])
    for name, pts in points.items():
        wp = dense(pts, name)
        clear = float(checks['frame'][0].path_clearance(polyline_path(np.radians(wp), 0.25)))
        m = checks['frame'][1]
        plan['legs'][name] = dict(waypoints_deg=np.round(wp, 3).tolist(), clearance_m=dict(frame=clear),
                                  min_clearance_straight_m=clear - m, planner='taught')
        if clear < m:
            plan['reasons'].append(f'{name}: {clear * 1000:.0f} mm from the frame (< {m * 1000:.0f} mm)')
        print(f'   [{source}] {name}: {len(wp)} waypoints, frame {clear * 1000:.0f} mm', flush=True)
    plan['accepted'] = not plan['reasons']
    out = DATA / 'rl_plans' / (time.strftime('%Y%m%d_%H%M%S') + f'_{int(time.time() * 1000) % 1000:03d}_taught')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'plan.json').write_text(json.dumps(plan, indent=1))
    if not plan['accepted']:
        raise RuntimeError('taught plan refused: ' + '; '.join(plan['reasons']))
    return out / 'plan.json'


def unchecked_sparse_plan(points, source):
    """Write already solved joint knots without FK/collision densification.

    Used only for the post-grasp fast path requested on 10/29.  The robot-side
    mover still enforces plan age, joint limits and <=45 degree knot jumps.
    """
    legs = {}
    for name, pts in points.items():
        wp = thin_waypoints(pts)
        legs[name] = dict(waypoints_deg=np.round(np.asarray(wp, float), 3).tolist(),
                          min_clearance_straight_m=0.0, planner='sparse unchecked post-grasp')
    plan = dict(dry_run=True, accepted=True, source=source, created=time.strftime('%Y-%m-%d %H:%M:%S'),
                margin_m=0.0, reasons=[], collision_check='skipped after grasp', legs=legs)
    out = DATA / 'rl_plans' / (time.strftime('%Y%m%d_%H%M%S') + f'_{int(time.time() * 1000) % 1000:03d}_sparse')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'plan.json').write_text(json.dumps(plan, indent=1))
    return out / 'plan.json'


def slot_plan(k, lift_end_deg, Tbc, q):
    """Slot k after an automatic grasp: lift end -> carry -> above -> insert | insert -> retreat -> home."""
    t = json.loads((TEACH_DIR / f'slot_{k:02d}.json').read_text())['poses']
    P = {n: t[n]['posj'] for n in t}
    return taught_plan(dict(to_slot=[lift_end_deg, P['carry'], P['above'], P['insert']],
                            slot_out=[P['insert'], P['retreat'], list(HOME_JOINTS_DEG)]), Tbc, q, f'teach slot {k}')


# ---------------------------------------------------------------- whole task taught (pick at a fixed spot + slot)

FULL_POSES = (('pre_grasp', '잡기 전: 그리퍼 열린 채 접시 테두리 앞 5~10 cm'),
              ('grasp', '잡는 자세: 손가락 사이에 테두리가 들어온 위치'),
              ('lift', '들어 올린 자세: 접시가 받침에서 약 10 cm 위'),
              ('carry', '운반 자세: 바구니 쪽으로 옮기기 좋은 높은 자세'),
              ('above', '슬롯 바로 위: 접시를 슬롯 방향으로 세우고 핀보다 3~5 cm 위'),
              ('insert', '삽입: 바닥에서 약 1 cm 띄운 위치(핀 사이)'),
              ('retreat', '후퇴: 그리퍼를 연 뒤 접시를 건드리지 않고 빠져나온 자세'))
FULL_GRIPPER = {'lift': 'close', 'retreat': 'open'}      # gripper action BEFORE recording this pose


def next_full():
    f = TEACH_DIR / 'next_full.json'
    return json.loads(f.read_text())['slot'] if f.is_file() else 1


def set_next_full(k):
    TEACH_DIR.mkdir(parents=True, exist_ok=True)
    (TEACH_DIR / 'next_full.json').write_text(json.dumps(dict(slot=k, updated=time.strftime('%Y-%m-%d %H:%M:%S'))))


def teach_full(k, last_joints, choose_gripper, event, meta=None):
    """Human jogs through FULL_POSES with the pendant; Enter stores the live joints (read only).  The plate must lie
    at the same marked spot whenever this teaching is replayed."""
    print(f'\n=== 전체 티칭 {k} (접시는 표시한 고정 위치) ===\n펜던트로 로봇을 움직이세요. PC는 관절값만 읽습니다.\n'
          f'각 자세에서 Enter, 다시 찍기 r, 그만두기 q.', flush=True)
    poses = {}
    for name, what in FULL_POSES:
        if name in FULL_GRIPPER:
            print(f'  그리퍼를 {"닫으세요 (c)" if FULL_GRIPPER[name] == "close" else "여세요 (o)"}', flush=True)
            choose_gripper(f'teach{k}_{FULL_GRIPPER[name]}')
        while True:
            ans = input(f'  [{name}] {what} > ').strip().lower()
            if ans == 'q':
                raise RuntimeError('teaching cancelled')
            if ans == 'r':
                continue
            msg = last_joints()
            if not fresh_joints(msg):
                print('  관절값이 오래됐습니다 (bringup 연결 확인). 다시 Enter.', flush=True)
                continue
            poses[name] = dict(posj=[float(v) for v in msg['posj']], posx=[float(v) for v in msg['posx']])
            event('teach_pose', slot=k, pose=name, **poses[name])
            print(f'     저장: posj {np.round(msg["posj"], 1).tolist()}', flush=True)
            break
    meta = dict(meta or {})
    if meta.get('quadrant'):                                 # (10/14 보스) BC label: where the plate actually went
        ans = input(f'  [{meta["quadrant"]}] 접시가 들어간 틈 번호 (N = N번과 N+1번 빗살 사이, 1 = 로봇에서 가장 먼 빗살; 모르면 Enter) > ').strip()
        meta['landed_pin'] = int(ans) if ans.isdigit() else None
    TEACH_DIR.mkdir(parents=True, exist_ok=True)
    path = TEACH_DIR / f'full_{k:02d}.json'
    if path.exists():
        path.replace(path.with_name(f'full_{k:02d}_{time.strftime("%Y%m%d_%H%M%S")}.json'))
    path.write_text(json.dumps(dict(slot=k, created=time.strftime('%Y-%m-%d %H:%M:%S'), poses=poses, gripper_force=64, **meta),
                               indent=1))
    print(f'저장: {path}\n펜던트로 홈 복귀 → 자동 모드 → run.sh connect 재실행 후 replay 하세요.', flush=True)
    return path


def full_plan(k, Tbc, q):
    t = json.loads((TEACH_DIR / f'full_{k:02d}.json').read_text())['poses']
    P = {n: t[n]['posj'] for n in t}
    home = list(HOME_JOINTS_DEG)
    return taught_plan(dict(to_grasp=[home, P['pre_grasp'], P['grasp']],
                            to_slot=[P['grasp'], P['lift'], P['carry'], P['above'], P['insert']],
                            slot_out=[P['insert'], P['retreat'], home]), Tbc, q, f'full teach {k}')


# ---------------------------------------------------------------- FP grasp + golden (taught) basket path

TCP_M = 0.165                                               # flange -> 2F-85 pinch point (rl/env_setup.py)


def _tcp(kin, q_deg):
    T = kin.fk(np.radians(q_deg)).copy()
    return T @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, TCP_M], [0, 0, 0, 1.0]])


def _plate_in_hand(T_tcp, rim_to_centre):
    """Gripper -> plate transform at the grasp: the plate lies flat (z up), its centre rim_to_centre ahead of the
    pinch point along the horizontal tool direction.  Same rule for the taught and the new grasp, so the residual
    model error mostly cancels in the correction."""
    d = T_tcp[:3, 2].copy()
    d[2] = 0.0
    d /= np.linalg.norm(d)
    P = np.eye(4)
    P[:3, 0], P[:3, 1], P[:3, 2] = d, np.cross([0, 0, 1.0], d), [0, 0, 1.0]
    P[:3, 3] = T_tcp[:3, 3] + d * rim_to_centre
    return np.linalg.inv(T_tcp) @ P


def golden_slot_plan(golden_path, grasp_deg, lift_end_deg, Tbc, q, basket_delta=None, plate_xy=None):
    """Plate grasped wherever FoundationPose saw it (plan to_grasp end = grasp_deg), then the taught basket path of
    the golden teaching: carry / above / insert / retreat re-solved so that the PLATE gets the taught poses although
    the new grasp (approach phi / pitch) differs:  T_tcp_new = D . T_tcp_taught . G_taught . G_new^-1  (IK seeded with
    the taught joints, refused if IK fails or a joint jumps).  D = basket_delta = T_basket_now . T_basket_at_teach^-1
    (markers, --place basket): the taught slot path moves with the basket; None = basket where it was taught."""
    lifts = q.get('basket_transit_lift_m', 0.0)
    if isinstance(lifts, (list, tuple)):    # (10/12 Human: carry higher) highest above-lift that the arm still reaches
        for i, lift in enumerate(lifts):
            try:
                out = golden_slot_plan(golden_path, grasp_deg, lift_end_deg, Tbc, q | {'basket_transit_lift_m': lift},
                                       basket_delta, plate_xy)
                print(f'   [golden] above lifted +{lift * 100:.0f} cm', flush=True)
                return out
            except RuntimeError as e:
                if i == len(lifts) - 1 or 'unreachable' not in str(e):
                    raise
    import yaml
    g = yaml.safe_load((ROOT / 'rl' / 'config.yaml').read_text())['geometry']
    rim_to_centre = g['rim_r'] - g['grasp_inset']
    t = json.loads(cpath(golden_path).read_text())['poses']
    kin = Kinematics()
    G_t = _plate_in_hand(_tcp(kin, t['grasp']['posj']), rim_to_centre)
    T_gn = _tcp(kin, grasp_deg)
    if plate_xy is None:
        G_n = _plate_in_hand(T_gn, rim_to_centre)
    else:   # (10/12 real) FoundationPose plate centre at the grasp: the library grasp is not radial (35 deg off the
        #     tool direction, 78 mm instead of 113 mm) -> the radial model put the plate 5 cm +y and 4.5 cm high
        d = np.r_[np.asarray(plate_xy, float) - T_gn[:2, 3], 0.0]
        d /= np.linalg.norm(d)
        F = np.eye(4)
        F[:3, 0], F[:3, 1], F[:3, 2] = d, np.cross([0, 0, 1.0], d), [0, 0, 1.0]
        F[:3, 3] = [plate_xy[0], plate_xy[1], T_gn[2, 3]]
        G_n = np.linalg.inv(T_gn) @ F
    # round plate: its turn about its own normal is free -> pick the one that keeps the tool closest to the taught tool
    # (else a non-radial grasp asks the wrist to turn ~35 deg for nothing)
    def turn(psi):
        R = np.eye(4)
        R[:2, :2] = [[np.cos(psi), -np.sin(psi)], [np.sin(psi), np.cos(psi)]]
        return R
    cands = [G_t @ turn(psi) @ np.linalg.inv(G_n) for psi in np.radians(np.arange(-180, 180, 1.0))]
    corr = min(cands, key=lambda C_: np.arccos(np.clip((np.trace(C_[:3, :3]) - 1) / 2, -1, 1)))
    flange_from_tcp = np.linalg.inv(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, TCP_M], [0, 0, 0, 1.0]]))
    D = np.eye(4) if basket_delta is None else np.asarray(basket_delta, float)
    if basket_delta is None:    # (10/12) golden: the basket is where it was found today (basket_shift_m), not measured
        D[:3, 3] = q.get('basket_shift_m', [0.0, 0.0, 0.0])
    dyaw = float(np.degrees(np.arctan2(D[1, 0], D[0, 0])))
    # (10/12 real run) Human: carry / above 5-10 cm higher (the plate ticked on the basket going over), whole slot path
    # nudged in base (Human: a bit toward -y); both from config plate_ik_bc so they can be tuned without code
    lift_m = q.get('basket_transit_lift_m', 0.0)
    nudge = np.asarray(q.get('basket_nudge_m', [0.0, 0.0, 0.0]), float)
    if q.get('basket_tilt_deg', 0.0):     # (10/12 Human) tilted 15 deg the plate went in 2 pins (4 cm) toward the robot
        nudge = nudge + np.asarray(D[:3, 0]) * q.get('basket_tilt_x_comp_m', 0.0)   # along the basket x (pin rows)
    # extra turn of the whole slot path about the vertical through the (moved) taught plate centre at the insert
    a = np.radians(q.get('basket_yaw_deg', 0.0))
    c = (D @ _tcp(kin, t['insert']['posj']) @ G_t)[:3, 3] + nudge
    Rz = np.eye(4)
    Rz[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    Rz[:2, 3] = c[:2] - Rz[:2, :2] @ c[:2]
    # (10/12 Human) tilt the plate about the horizontal axis across the pin rows (basket y) through its lowest rim point
    # at the insert, so it slides in: + = the plate top toward +x (away from the robot)
    b = np.radians(q.get('basket_tilt_deg', 0.0))
    Pi = Rz @ (D @ _tcp(kin, t['insert']['posj']) @ G_t)
    Pi[:3, 3] += nudge + [0.0, 0.0, q.get('basket_insert_up_m', 0.0)]
    n_ = Pi[:3, 2]
    down = -(np.array([0, 0, 1.0]) - n_[2] * n_)
    low = Pi[:3, 3] + g['rim_r'] * down / np.linalg.norm(down)
    ax = D[:3, 1] / np.linalg.norm(D[:3, 1])
    K_ = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    Ry = np.eye(4)
    Ry[:3, :3] = np.eye(3) + np.sin(b) * K_ + (1 - np.cos(b)) * K_ @ K_
    Ry[:3, 3] = low - Ry[:3, :3] @ low
    new = {}
    for name in ('carry', 'above', 'insert', 'retreat'):
        if name == 'retreat' and q.get('basket_tilt_deg', 0.0) and q.get('basket_retreat_up_m'):
            # (10/12) tilted: the taught retreat (12 cm up) is out of reach -> straight up from the insert, same tool turn
            sol = None              # highest straight-up retreat the arm reaches (10/12: 8 cm failed with a closer grasp)
            for up in (q['basket_retreat_up_m'], 0.06, 0.04, 0.03):
                target = kin.fk(np.radians(new['insert'])).copy()
                target[2, 3] += up
                try:
                    sol = np.degrees(kin.ik(target, np.radians(new['insert'])))
                    break
                except RuntimeError:
                    continue
            if sol is None:
                raise RuntimeError('golden retreat: cannot even go 3 cm straight up from the insert (out of reach)')
            new[name] = sol
            print(f'   [golden] retreat  straight up {up * 100:.0f} cm from the insert', flush=True)
            continue
        seed = np.radians(t[name]['posj'])
        seed[0] += np.radians(dyaw + q.get('basket_yaw_deg', 0.0))   # a turned basket turns the taught pose about the base too
        target = D @ _tcp(kin, t[name]['posj']) @ corr
        target[:3, 3] += nudge + [0.0, 0.0, {'carry': q.get('basket_carry_lift_m', lift_m), 'above': lift_m}.get(name, 0.0)]
        if name == 'insert':    # (10/12 real, e-stop) Human: the taught depth forces the plate in -> stop higher
            target[2, 3] += q.get('basket_insert_up_m', 0.0)
        target = Rz @ target
        if name == 'insert':    # tilt only at the insert: the plate turns while it descends (above stays high and upright)
            target = Ry @ target
        if name == 'carry':     # flat plate high over the wall: keep the taught tool turn (corrected position only);
            target[:3, :3] = (D @ _tcp(kin, t[name]['posj']))[:3, :3]   # the wrist sits near j5 = 0 here and would swing
        target = target @ flange_from_tcp
        # seeds: the taught pose (j1 turned toward the moved target), then the previous solved pose (10/12: a basket
        # moved 5 cm left the single taught seed in a local minimum -> 'Unreachable IK target')
        tp = _tcp(kin, t[name]['posj'])[:2, 3]
        s1 = seed.copy()
        s1[0] += np.arctan2(target[1, 3], target[0, 3]) - np.arctan2(tp[1], tp[0]) - np.radians(dyaw + q.get('basket_yaw_deg', 0.0))
        sol = None
        for sd in [seed, s1] + ([np.radians(new[prev])] if (prev := {'above': 'carry', 'insert': 'above', 'retreat': 'insert'}.get(name)) in new else []):
            try:
                sol = np.degrees(kin.ik(target, sd))
                break
            except RuntimeError:
                continue
        if sol is None:         # (10/12) many random seeds before calling it unreachable; keep the one nearest the seed
            rng, lim = np.random.default_rng(0), np.radians(np.array(kin.limits_deg()))
            sols = []
            for _ in range(60):
                try:
                    sols.append(np.degrees(kin.ik(target, rng.uniform(np.maximum(lim[:, 0], -np.pi), np.minimum(lim[:, 1], np.pi)))))
                except RuntimeError:
                    pass
            if sols:
                sol = min(sols, key=lambda v: np.abs(v - np.degrees(seed)).max())
                print(f'   [golden] {name}: {len(sols)}/60 random seeds solve it (nearest kept)', flush=True)
        if sol is None:
            raise RuntimeError(f'golden {name}: unreachable (target TCP {np.round((target @ np.linalg.inv(flange_from_tcp))[:3, 3], 3).tolist()}, tool z {np.round(target[:3, 2], 2).tolist()}, flange {np.round(target[:3, 3], 3).tolist()}) - 바구니가 로봇에서 너무 멂: 로봇 쪽(-x)으로 옮기세요')
        err = np.linalg.norm(kin.fk(np.radians(sol))[:3, 3] - target[:3, 3])
        if err > 0.002:
            raise RuntimeError(f'golden {name}: IK misses the target by {err * 1000:.0f} mm (out of reach?)')
        if np.abs(sol - np.degrees(seed)).max() > 60:
            raise RuntimeError(f'golden {name}: corrected IK jumps {np.abs(sol - np.degrees(seed)).max():.0f} deg')
        new[name] = sol
        print(f'   [golden] {name:8s} corrected by {np.abs(sol - np.degrees(seed)).max():5.1f} deg (max joint)', flush=True)
    # (10/12 Human) four stages, each its own leg so the run can stop between them: 1 to_above, 2 descend_pre + descend
    # (+ open), 3 ascend, 4 to_home; abort_home = from above straight home (plate still held), pre_up = back to above
    # descend is split at 'pre' (TCP pre_up_m above the insert, plate bottom ~2.5 cm over the pin tops) for a look;
    # pre lies on the joint line above -> insert, which is straight in TCP within ~2 mm (10/12 check)
    pre_up = q.get('basket_pre_up_m', 0.075) - q.get('basket_insert_up_m', 0.0)   # pre stays where it was
    z_ins = _tcp(kin, new['insert'])[2, 3]
    lo, hi = 0.0, 1.0                                       # fraction from insert toward above
    for _ in range(40):
        mid = (lo + hi) / 2
        z = _tcp(kin, np.asarray(new['insert']) + mid * (np.asarray(new['above']) - new['insert']))[2, 3]
        lo, hi = (mid, hi) if z - z_ins < pre_up else (lo, mid)
    pre = list(np.asarray(new['insert']) + lo * (np.asarray(new['above']) - new['insert']))
    home = list(HOME_JOINTS_DEG)
    out = taught_plan(dict(to_above=[lift_end_deg, new['carry'], new['above']], descend_pre=[new['above'], pre],
                           descend=[pre, new['insert']], ascend=[new['insert'], new['retreat']],
                           to_home=[new['retreat'], home], abort_home=[new['above'], home],
                           pre_up=[pre, new['above']]), Tbc, q,
                      f'golden {cpath(golden_path).name}' + ('' if basket_delta is None else ' (basket moved)'))
    # for the pre-stop wrist measurement: plate centre y at the slot and the plate's lowest rim height at pre
    Ti = D @ _tcp(kin, t['insert']['posj']) @ G_t
    z_low = Ti[2, 3] - (g['rim_r'] - 0.0) * np.sqrt(max(0.0, 1 - Ti[2, 2] ** 2))
    plan = json.loads(out.read_text())
    plan['basket_slot'] = dict(plate_centre_y=float(c[1]),
                               # the plate is placed like the taught one, so its rim sits pre_up_m (+ z nudge) higher
                               plate_bottom_at_pre_z=float(z_low + nudge[2] + q.get('basket_pre_up_m', 0.075)))
    out.write_text(json.dumps(plan, indent=1))
    return out


def shifted_plan(plan_path, dx, Tbc, q):
    """(10/12 real) pre-stop correction: the same slot legs from 'pre' on, with the TCP moved dx (m) along base x
    (wrist photo: plate edge vs pin gap).  Legs: shift (pre -> pre'), descend, ascend, to_home, pre_up, abort_home."""
    kin = Kinematics()
    L = json.loads(Path(plan_path).read_text())['legs']
    pre, ins, ret = L['descend']['waypoints_deg'][0], L['descend']['waypoints_deg'][-1], L['ascend']['waypoints_deg'][-1]
    above = L['pre_up']['waypoints_deg'][-1]

    def moved(qd):
        T = kin.fk(np.radians(qd)).copy()
        T[0, 3] += dx
        sol = np.degrees(kin.ik(T, np.radians(qd)))
        if np.linalg.norm(kin.fk(np.radians(sol))[:3, 3] - T[:3, 3]) > 0.001 or np.abs(sol - qd).max() > 10:
            raise RuntimeError(f'shift {dx * 1000:+.1f} mm: IK failed near {np.round(qd, 1).tolist()}')
        return list(sol)
    p2, i2, r2 = moved(pre), moved(ins), moved(ret)
    home = list(HOME_JOINTS_DEG)
    return taught_plan(dict(shift=[pre, p2], descend=[p2, i2], ascend=[i2, r2], to_home=[r2, home],
                            pre_up=[p2, above], abort_home=[above, home]), Tbc, q,
                       f'{Path(plan_path).parent.name} shifted x {dx * 1000:+.1f} mm')


# ---------------------------------------------------------------- Isaac rollout export (moved from pipeline.py 10/12)
def isaac_cmd(q, mode):
    """rl/eval.py <mode> command line for the plate policy in q (export: one-shot, serve: warm server)."""
    cmd = [q['isaaclab'], '-p', str(ROOT / 'rl/eval.py'), mode, '--headless', '--checkpoint', q['checkpoint'],
           '--num_envs', str(q.get('export_envs', 16))]
    if q.get('residual_base'):                              # pin the BC base of this checkpoint (rl/config.yaml may change)
        cmd += ['--residual_base', q['residual_base']]
    else:
        cmd += ['--residual', 'off']
    if q.get('approach') == 'ik':                           # straight TCP line to the rim, BC from handoff_m on
        cmd += ['--approach', 'ik', '--handoff_m', str(q.get('handoff_m', 0.04)),
                '--approach_fast', str(q.get('approach_fast_sim', 1.0))]
    return cmd


def isaac_env():
    return {k: v for k, v in os.environ.items() if k not in ('CONDA_PREFIX', 'CONDA_DEFAULT_ENV', 'PYTHONPATH', 'PYTHONHOME')}


def export_from_server(x, y, yaw, q):
    """Ask a warm `pipeline.py --task plate --export_server` (rl/eval.py serve) first: seconds instead of ~1 min of Isaac
    start-up.  None when no server runs or it serves another policy."""
    req = dict(plate_x=x, plate_y=y, plate_yaw_deg=yaw, checkpoint=str(q['checkpoint']),
               approach=q.get('approach', 'policy'), residual_base=q.get('residual_base') or None)
    try:
        with socket.create_connection(('127.0.0.1', C['udp'].get('export_server', 5012)), timeout=2.0) as conn:
            conn.settimeout(300)
            conn.sendall((json.dumps(req) + '\n').encode())
            reply = json.loads(conn.makefile().readline())
    except (OSError, ValueError):
        return None
    if reply.get('mismatch'):
        print(f'   export server runs another policy ({reply["mismatch"]["server"]}); fresh Isaac instead', flush=True)
        return None
    for line in reply.get('lines', []):
        print('   ' + line, flush=True)
    if not reply.get('ok') or not reply.get('accepted'):
        raise RuntimeError(f'RL plan not created / not accepted (export server): {reply.get("error", reply.get("lines"))}')
    return Path(reply['plan'])


def export_plan(x, y, yaw, log, q):
    """Isaac rollout of the plate policy at the measured plate pose -> calibration/data/rl_plans/<time>/plan.json."""
    plan = export_from_server(x, y, yaw, q)
    if plan is not None:
        log.write_text(f'export server: {plan}\n')
        return plan
    cmd = isaac_cmd(q, 'export') + ['--plate_x', f'{x:.4f}', '--plate_y', f'{y:.4f}', '--plate_yaw_deg', f'{yaw:.1f}']
    for attempt in (1, 2):                                  # Isaac sometimes segfaults while starting (X11 / kit)
        with log.open('wb') as stream:
            subprocess.run(cmd, cwd=ROOT, env=isaac_env(), stdout=stream, stderr=subprocess.STDOUT, timeout=900)
        text = log.read_text(errors='replace')
        if 'EXPORT_PLAN ' in text or not ('Segmentation fault' in text or 'Crash detected' in text) or attempt == 2:
            break
        log.replace(log.with_name('export_crash.log'))
        print('   Isaac 시작 중 비정상 종료 -> 한 번 다시 실행합니다', flush=True)
    for line in text.splitlines():
        if line.startswith('[export]'):
            print('   ' + line, flush=True)
    found = [l for l in text.splitlines() if l.startswith('EXPORT_PLAN ')]
    if not found or not found[-1].endswith('accepted=True'):
        raise RuntimeError(f'RL plan not created / not accepted; see {log}')
    return Path(found[-1].split()[1])


def basket_range_note(Tbb, q):
    """(10/13) where the measured basket is vs the RL training range (config basket_policy_range)."""
    r, x, y = q.get('basket_policy_range', {}), float(Tbb[0][3]), float(Tbb[1][3])
    yaw = float(np.degrees(np.arctan2(Tbb[1][0], Tbb[0][0])))
    ok = lambda k, v: r.get(k) is None or r[k][0] <= v <= r[k][1]               # noqa: E731
    where, base = f'바구니 x {x:.3f} m, y {y:+.3f} m, 각도 {yaw:+.1f} deg', ok('x', x) and ok('yaw_deg', yaw)
    print(f'   [범위] {where}: 학습 범위 안' if base and ok('y', y) else
          f'   [범위 주의] {where}: 학습 범위 밖, sim 확인만 된 범위 -> sim 판정을 보고 진행' if base and ok('y_sim_checked', y) else
          f'   [범위 밖] {where}: 학습 범위 x {r.get("x")} y {r.get("y")} 각도 {r.get("yaw_deg")} 밖 -> sim 판정에서 떨어지면 '
          f'티칭 경로 (이 위치는 티칭 경로도 미검증)', flush=True)


def thin_waypoints(w, tol_deg=0.05):
    """(10/13) drop waypoints within tol_deg of the previous one: a spline with repeated points (the sim pause after the
    release) is accepted by the controller and not executed."""
    out = [list(w[0])]
    for p in w[1:]:
        if np.abs(np.subtract(p, out[-1])).max() > tol_deg:
            out.append(list(p))
    return out


def policy_plan(job, out, Tbc, q, wait_s=240):
    """(10/13) wait for the sim policy rollout (rl/basket/expert.py realplan), accept it only if the exact case inserted
    and >= half of the in-hand-noise variants did; -> plan.json (FK frame clearance checked) or None (golden fallback)."""
    t0 = time.time()
    while job.poll() is None and time.time() - t0 < wait_s:
        time.sleep(1)
    res_path = out / 'realplan.json'
    if job.poll() is None or not res_path.exists():
        print(f'   정책 경로 계산이 끝나지 않음 -> 티칭 경로로 넣기 ({out / "realplan.log"})', flush=True)
        return None
    r = json.loads(res_path.read_text())
    if r.get('reason'):
        print(f"   정책 경로: {r['reason']}", flush=True)
    print(f"   정책 sim 결과: 정확 {r['sim_success_exact']}, 변형 {r['sim_success_variants']:.2f}, 틈 mm {r['final_gap_mm'][:3]}", flush=True)
    if not r.get('legs') or not r['sim_success_exact'] or r['sim_success_variants'] < 0.5:
        print('   정책 경로 기준 미달 -> 티칭 경로로 넣기', flush=True)
        return None
    L = r['legs']
    start, down, up = L['to_policy_start']['waypoints_deg'], L['policy_descend']['waypoints_deg'], L['policy_ascend']['waypoints_deg']
    extra = q.get('basket_policy_extra_down_m', 0.0)      # (10/13 보스) seat a little deeper before the release: straight down
    if extra > 0:
        T = Kinematics().fk(np.radians(down[-1]))
        T[2, 3] -= extra
        down = down + [np.degrees(Kinematics().ik(T, np.radians(down[-1]))).round(3).tolist()]
    thin = thin_waypoints                                  # repeated waypoints: the controller does not move
    start, down, up = thin(start), thin(down), thin([down[-1]] + up)
    lo, hi = np.array(__import__('ros2_nodes').joint_limits_deg()).T          # (10/13) controller limits (J5 +-135, the sim rise went past):
    up = up[:next((i for i, w in enumerate(up) if not ((np.array(w) > lo + 3) & (np.array(w) < hi - 3)).all()), len(up))]   # rise while inside
    if len(up) < 3:
        print('   정책 상승 경로가 관절 한계에 걸림 -> 티칭 경로로 넣기', flush=True)
        return None
    last = up[-1]
    try:
        return taught_plan(dict(to_policy_start=start, policy_descend=down, policy_ascend=up,
                                to_home=[last, list(HOME_JOINTS_DEG)], policy_abort=[start[-1], start[0], list(HOME_JOINTS_DEG)]),
                           Tbc, q, 'learned policy (BC + residual RL)')
    except RuntimeError as e:
        print(f'   정책 경로 거부({e}) -> 티칭 경로로 넣기', flush=True)
        return None


def merged_plan(parts, name, max_wp=100):
    """(10/13 보스) one spline over consecutive legs of (possibly different) plan files with no stop between them:
    parts = [(plan_path, leg), ...] -> plan.json with the single leg `name`, or None (> max_wp waypoints for movesj)."""
    wp, slack = [], []
    for path, leg in parts:
        pl = json.loads(Path(path).read_text())
        if not pl.get('accepted') or leg not in pl['legs']:
            return None
        wp += pl['legs'][leg]['waypoints_deg']
        slack.append(pl['legs'][leg]['min_clearance_straight_m'] - pl.get('margin_m', 0.0))
    wp = thin_waypoints(wp)
    if len(wp) > max_wp or len(wp) < 3:
        return None
    plan = dict(dry_run=True, accepted=True, source='merged ' + ' + '.join(f'{lg}' for _, lg in parts), reasons=[], margin_m=0.0,
                created=time.strftime('%Y-%m-%d %H:%M:%S'), legs={name: dict(waypoints_deg=wp, min_clearance_straight_m=min(slack))})
    out = DATA / 'rl_plans' / (time.strftime('%Y%m%d_%H%M%S') + f'_{int(time.time() * 1000) % 1000:03d}_merged')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'plan.json').write_text(json.dumps(plan, indent=1))
    print(f'   [merged] {name}: {len(wp)} waypoints ({plan["source"]})', flush=True)
    return out / 'plan.json'


# (10/22 보스 "파지 이후는 stop&go 말고 go") real legs 10/18-10/22 (393): duration ~ 0.36 s per movesj waypoint + 2.8 x
# travel/vel, so the 60-point carry took ~30 s at 60 deg/s.  movesj passes its points at about equal time (even_carry).
MOVESJ_S_PER_POINT, MOVESJ_TRAVEL_FACTOR = 0.36, 2.8


def movesj_seconds(wp_deg, vel):
    """Fitted real duration (s) of one movesj leg (incl. the mover start)."""
    w = np.asarray(wp_deg, float)
    return MOVESJ_S_PER_POINT * len(w) + MOVESJ_TRAVEL_FACTOR * float(np.abs(np.diff(w, axis=0)).max(1).sum()) / vel


def equal_time_spline(knots, per=24):
    """The movesj path model: a cubic through the knots at equal time, at rest at both ends -> dense joints."""
    from scipy.interpolate import CubicSpline
    K = np.asarray(knots, float)
    s = CubicSpline(np.arange(len(K)), K, axis=0, bc_type='clamped')
    t = np.linspace(0, len(K) - 1, (len(K) - 1) * per + 1)
    return s(t), t


def rdp_joint(w, tol):
    """Indices of a Douglas-Peucker thinning of a joint path (max joint deviation from the chord <= tol)."""
    w = np.asarray(w, float)
    keep = {0, len(w) - 1}
    stack = [(0, len(w) - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        f = (np.arange(a + 1, b) - a) / (b - a)
        dev = np.abs(w[a + 1:b] - (w[a] + f[:, None] * (w[b] - w[a]))).max(1)
        i = int(np.argmax(dev))
        if dev[i] > tol:
            keep.add(a + 1 + i)
            stack += [(a, a + 1 + i), (a + 1 + i, b)]
    return sorted(keep)


def even_knots(w, idx, max_seg_deg=30.0, ratio=2.5, tail_seg_deg=None):
    """Add path points to the knot indices `idx` until no step is over max_seg_deg and neighbouring steps differ at
    most `ratio`x (the last one also vs the first tail step): equal time per knot, so steps that jump make the spline
    swing; the inserted knots are points of the checked path."""
    w, idx = np.asarray(w, float), list(idx)
    seg = lambda i: float(np.abs(w[idx[i + 1]] - w[idx[i]]).max())          # noqa: E731
    for _ in range(200):
        bad = None
        for i in range(len(idx) - 1):
            nxt = seg(i + 1) if i + 2 < len(idx) else tail_seg_deg
            prv = seg(i - 1) if i > 0 else None
            if seg(i) > max_seg_deg or (nxt and seg(i) > ratio * nxt) or (prv and seg(i) > ratio * prv):
                bad = i
                break
        if bad is None or idx[bad + 1] - idx[bad] < 2:
            break
        idx.insert(bad + 1, (idx[bad] + idx[bad + 1]) // 2)
    return idx


def one_go_knots(head, tail, held, inserting, floor=None, tols=(10.0, 7.0, 5.0, 3.5, 2.5, 1.8, 1.2), tail_every=1,
                 n_down=None, max_down_xy_mm=2.0):
    """(10/22 보스) lift + carry (`head`, joint rad, thinned) then the drop + straight-down insert (`tail`, kept point
    for point: equal time per point keeps it slow) as ONE movesj.  The fewest head knots whose equal-time spline stays
    clear: held-plate checker up to the drop, insert checker after, `floor` (a FloorCheck or None) over the head.
    -> (knots rad, report) or (None, report)."""
    head, tail = np.asarray(head, float), np.asarray(tail, float)
    tail = tail[sorted(set(range(0, len(tail), tail_every)) | {len(tail) - 1})]   # 1 cm steps -> 2 cm: fewer points
    hd = np.degrees(head)
    tail_seg = float(np.degrees(np.abs(tail[1] - tail[0]).max())) if len(tail) > 1 else None
    tried = []
    for tol in tols:
        idx = even_knots(hd, rdp_joint(hd, tol), tail_seg_deg=tail_seg)
        K = np.vstack([head[idx], tail[1:]])
        nh = len(idx) - 1                                   # knot index where the drop starts
        S, t = equal_time_spline(K)
        i0 = np.minimum(np.floor(t).astype(int), len(K) - 2)
        chord = K[i0] + (t - i0)[:, None] * (K[i0 + 1] - K[i0])
        a, b = S[t <= nh], S[t >= nh]
        c_held = held.path_clearance(a)
        c_ins = inserting.path_clearance(b)
        c_fl = floor.path_clearance(a) if floor is not None else np.inf
        rep = dict(tol_deg=tol, knots=len(K), head_knots=len(idx), clear_held=round(float(c_held), 2),
                   clear_insert=round(float(c_ins), 2), clear_floor=None if floor is None else round(float(c_fl), 2),
                   spline_vs_chords_deg=round(float(np.degrees(np.abs(S - chord).max())), 2),
                   insert_vs_chords_deg=round(float(np.degrees(np.abs(S - chord)[t >= nh].max())), 2))
        if n_down:                                          # the straight-down insert: TCP off the vertical (mm)
            kin = held.kin
            Sd = S[t >= len(K) - n_down]
            P_ = np.array([kin.fk(x)[:3, 3] for x in Sd[::3]])
            rep['down_off_vertical_mm'] = round(float(1000 * np.linalg.norm(P_[:, :2] - P_[-1, :2], axis=1).max()), 2)
        tried.append(rep)
        if min(c_held, c_ins, c_fl) >= 1.0 and rep.get('down_off_vertical_mm', 0.0) <= max_down_xy_mm:
            return K, dict(rep, tried=tried)
    return None, dict(tried=tried)


def _unchecked_one_go_knots(head, tail, max_waypoints=20, max_step_deg=40.0):
    """Reduce a pre-grasped lift/carry/insert path to controller knots only.

    This intentionally performs no FK or collision work.  RDP keeps bends in
    joint space, ``even_knots`` restores any knot needed to keep a single
    joint jump below ``max_step_deg``, and the straight insert tail is kept.
    """
    head, tail = np.asarray(head, float), np.asarray(tail, float)
    hd = np.degrees(head)
    lo, hi = np.asarray(__import__('ros2_nodes').joint_limits_deg(), float).T
    tail_seg = float(np.degrees(np.abs(tail[1] - tail[0]).max())) if len(tail) > 1 else None
    tried = []
    for tol in (3.5, 5.0, 7.0, 10.0, 15.0, 25.0, 40.0):
        idx = even_knots(hd, rdp_joint(hd, tol), max_seg_deg=max_step_deg, tail_seg_deg=tail_seg)
        K = np.vstack([head[idx], tail[1:]])
        jump = float(np.degrees(np.abs(np.diff(K, axis=0)).max())) if len(K) > 1 else 0.0
        dense, _ = equal_time_spline(np.degrees(K))
        joint_margin = float(np.minimum(dense - lo, hi - dense).min())
        tried.append(dict(tol_deg=tol, knots=len(K), max_step_deg=round(jump, 2),
                          min_joint_margin_deg=round(joint_margin, 2)))
        if len(K) <= max_waypoints and jump <= max_step_deg + 1e-6 and joint_margin >= 2.0:
            return K, dict(tol_deg=tol, knots=len(K), head_knots=len(idx),
                           max_step_deg=round(jump, 2), min_joint_margin_deg=round(joint_margin, 2), tried=tried)
    return None, dict(tried=tried)


def one_go_plan(grasp_plan, insert_plan, Tbc, q, Tbb, H, tool, plates=(), vel=60.0,
                check_collision=True, max_waypoints=20):
    """(10/22 보스, opt-in --one_go) after the grasp: lift + carry + straight-down insert as ONE spline, then (after the
    release) ascend + home as one more.  -> (plan.json path, report) or (None, report)."""
    G, P = json.loads(Path(grasp_plan).read_text()), json.loads(Path(insert_plan).read_text())
    k = (P.get('quadrant_plan') or {}).get('drop_start')
    if k is None:
        return None, dict(why='insert plan has no drop_start (not a high carry)')
    lift = np.radians(G['legs']['lift']['waypoints_deg'])
    carry = np.radians(P['legs']['to_above']['waypoints_deg'])
    down = np.radians(P['legs']['descend']['waypoints_deg'])
    if np.degrees(np.abs(lift[-1] - carry[0]).max()) > 0.5 or np.degrees(np.abs(carry[-1] - down[0]).max()) > 0.5:
        return None, dict(why='legs do not join')
    head = np.vstack([lift, carry[1:k + 1]])
    tail = np.vstack([carry[k:], down[1:]])
    K, rep = None, {}
    if check_collision:
        held, inserting, _ = quadrant_checkers(Tbc, q, Tbb, H, tool, plates=plates)
        for every in (2, 1):                                 # 2 cm steps down if it stays straight, else 1 cm
            K, rep = one_go_knots(head, tail, held, inserting, tail_every=every,
                                  n_down=(len(down) + every - 1) // every)
            if K is not None and len(K) <= max_waypoints:
                break
        if K is not None and len(K) > max_waypoints:
            K = None
    else:
        K, rep = _unchecked_one_go_knots(head, tail, max_waypoints=max_waypoints)
        rep.update(clear_held=None, clear_insert=None, down_off_vertical_mm=None,
                   collision_check='skipped after grasp')
    old = len(lift) + len(carry) - 1
    if K is None:
        why = 'no thinned spline clears' if check_collision else f'cannot reduce to <= {max_waypoints} safe joint steps'
        return None, dict(rep, why=why)
    up = P['legs']['ascend']['waypoints_deg'] + P['legs']['to_home']['waypoints_deg'][1:]
    up = np.degrees(split_long_moves(np.radians(up), 40.0)).tolist()
    Kd = np.round(np.degrees(K), 3).tolist()
    rep.update(before_points=old + len(down) - 1, after_points=len(Kd),
               est_s=round(movesj_seconds(Kd, vel), 1),
               est_s_before=round(movesj_seconds(np.vstack([np.degrees(lift), np.degrees(carry[1:])]), vel)
                                  + movesj_seconds(np.degrees(down), q.get('basket_quadrant_vel_deg_s', 10.0)), 1))
    plan = dict(dry_run=True, accepted=True, source=f'one_go {Path(grasp_plan).parent.name} + {Path(insert_plan).parent.name}',
                reasons=[], margin_m=0.0, created=time.strftime('%Y-%m-%d %H:%M:%S'), one_go=rep,
                legs=dict(one_go=dict(waypoints_deg=Kd, min_clearance_straight_m=0.0, drop_knot=rep['head_knots'] - 1),
                          retreat=dict(waypoints_deg=thin_waypoints(up), min_clearance_straight_m=0.0)))
    out = DATA / 'rl_plans' / (time.strftime('%Y%m%d_%H%M%S') + f'_{int(time.time() * 1000) % 1000:03d}_one_go')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'plan.json').write_text(json.dumps(plan, indent=1))
    return out / 'plan.json', rep


# ---------------------------------------------------------------- basket reach map (10/13 보스: basket anywhere in reach, 4 quadrants)
QUADRANTS = {'Q1': ('far', 'right'), 'Q2': ('far', 'left'), 'Q3': ('near', 'left'), 'Q4': ('near', 'right')}   # top view, robot at the bottom


def basket_gaps():
    """{quadrant: [gap centre on the basket floor (basket frame), ...]} ordered robot side -> far side.  Basket frame: x robot
    side -> far side, y = left seen from the robot (the blue handle is at -y, 10/14 Codex I).  Pair A (y +0.129) left, pair B (y -0.110) right."""
    import sys
    sys.path.insert(0, str(ROOT / 'rl'))
    import basket_env as be
    r, floor, cx = be.B['full']['pin_r'], be.B['full']['floor_z'], be.B['full'].get('clip_x', 0.0)   # quadrants split at the clip
    pins = [p0 for kind, p0, p1, *rest in be.full_basket_parts() if kind == 'cyl' and rest and abs(rest[0] - r) < 1e-9]
    out = {}
    for qd, (fn, lr) in QUADRANTS.items():
        yc = be.B['full']['pair_centres_y'][0 if lr == 'left' else 1]
        xs = sorted({round(float(p[0]), 4) for p in pins if abs(p[1] - yc) < 0.06 and ((p[0] > cx) == (fn == 'far'))})
        out[qd] = [np.array([(a + b) / 2, yc, floor]) for a, b in zip(xs[:-1], xs[1:])]
    return out


def plate_in_hand(run_dir=None):
    """H = plate (centre, z = normal, x = toward the grasp point) in the TCP frame, from a real run's grasp (FP + joints)."""
    d = Path(run_dir) if run_dir else sorted(p.parent for p in (DATA / 'real_pipeline').glob('*/realplan_req.json'))[-1]
    req = json.loads((d / 'realplan_req.json').read_text())
    return hand_plate(req['grasp_deg'], req['plate_root_fp']) + (d.name,)


def hand_plate(grasp_deg, plate_root, normal=(0.0, 0.0, 1.0), tilt_deg=None):
    """(H, tool): the plate at `plate_root` (FP root = bottom centre, base) with this normal, held by the TCP at these
    joints.  Grasp: plate flat; after the lift (10/13): the FP normal there.  `tilt_deg` = (toward the grasp point, toward
    the rim tangent): the plate turns about its centre in the fingers after the lift (Isaac 10/13: (-13, +5) deg, centre
    within 5 mm; config plate_hand_tilt_deg, default none)."""
    kin, tool = Kinematics(), np.eye(4)
    tool[2, 3] = C['plate_ik_bc'].get('tool_length_m', 0.165)
    T_tcp = kin.fk(np.radians(grasp_deg)) @ tool
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    n = n if n[2] >= 0 else -n
    c = np.asarray(plate_root[:3]) + 0.01 * n
    a = T_tcp[:3, 3] - c
    a -= (a @ n) * n
    x = a / np.linalg.norm(a)
    T_pl = np.eye(4)
    T_pl[:3, :3] = np.stack([x, np.cross(n, x), n], 1)
    T_pl[:3, 3] = c
    tx, ty = np.radians(C['plate_ik_bc'].get('plate_hand_tilt_deg', [0.0, 0.0]) if tilt_deg is None else tilt_deg)
    if tx or ty:                                             # new normal in the plate frame, shortest turn from z
        m = np.array([np.tan(tx), np.tan(ty), 1.0])
        m /= np.linalg.norm(m)
        ax = np.cross([0, 0, 1.0], m)
        R = np.eye(4)
        R[:3, :3] = __import__('scipy.spatial.transform', fromlist=['Rotation']).Rotation.from_rotvec(
            ax / np.linalg.norm(ax) * np.arccos(m[2])).as_matrix()
        T_pl = T_pl @ R
    return np.linalg.inv(T_tcp) @ T_pl, tool


def _reach_one(job):
    """One basket pose x quadrant -> (ok, best phi, why).  Runs in a forked worker (globals set by basket_reach)."""
    bx, by, yaw, qd = job
    G = _REACH
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    Tb = np.eye(4)
    Tb[:2, :2] = [[c, -s], [s, c]]
    Tb[:3, 3] = [bx, by, G['basket_z']]
    gap = (Tb @ np.r_[G['gaps'][qd][G['gap_index']], 1.0])[:3]
    n = Tb[:3, 0]                                             # slot axis = basket x; concave side to +n
    t = np.radians(G['tilt_deg'])
    zt = n * np.cos(t) + np.array([0, 0, 1.0]) * np.sin(t)
    v = -n * np.sin(t) + np.array([0, 0, 1.0]) * np.cos(t)
    why = 'ik'
    for phi in G['phis']:                                     # grasp point turned about the plate normal (round plate)
        ph = np.radians(phi)
        x = v * np.cos(ph) + np.cross(zt, v) * np.sin(ph)
        Tp = np.eye(4)
        Tp[:3, :3] = np.stack([x, np.cross(zt, x), zt], 1)
        Tp[:3, 3] = gap + v * G['centre_h'] / np.cos(t)
        qs = []
        for lift in (0.0, G['above_m']):
            T6 = (Tp @ np.linalg.inv(G['H'])) @ np.linalg.inv(G['tool'])
            T6[2, 3] += lift
            q = None
            for seed in ([qs[-1]] if qs else []) + G['seeds']:
                try:
                    q = G['kin'].ik(T6, seed)
                    if ((np.degrees(q) > G['lo'] + 3) & (np.degrees(q) < G['hi'] - 3)).all():
                        break
                    q, why = None, 'limit'
                except RuntimeError:
                    q = None
            if q is None:
                break
            qs.append(q)
        if len(qs) < 2:
            continue
        if np.abs(np.degrees(qs[1] - qs[0])).max() > 60:
            why = 'branch'
            continue
        path = polyline_path(np.array([qs[1], qs[0]]), 0.25)
        if any(chk.path_clearance(path) < m for chk, m in G['checks']):
            why = 'frame'
            continue
        return True, phi, 'ok'
    return False, None, why


def basket_reach(a):
    """Map of basket centre positions (table grid) where the plate can be put upright into each quadrant's gap: IK for
    the insert and the start 11 cm above, controller limits (J5 +-135), frame/table clearance, phi = grasp point about
    the plate normal.  Not checked: the arm vs the basket walls, carrying the plate there, grasping elsewhere."""
    import multiprocessing as mp
    import ros2_nodes
    global _REACH
    q = C['plate_ik_bc']
    H, tool, run = plate_in_hand(a.run)
    gold = json.loads((DATA / 'basket' / 'slot_01_in_basket.json').read_text())['posj_at_teach']
    rng = np.random.default_rng(0)
    kin = Kinematics()
    lo, hi = np.array(ros2_nodes.joint_limits_deg()).T
    seeds = [np.radians(gold['insert']), np.radians(gold['above']), np.radians(HOME_JOINTS_DEG)]
    seeds += [np.radians(rng.uniform(np.maximum(lo, -170), np.minimum(hi, 170))) for _ in range(a.random_seeds)]
    from calib_core import latest_t_base_cam, load_json
    ch = library_checkers(np.array(load_json(latest_t_base_cam())['T_base_cam']), q)
    _REACH = dict(H=H, tool=tool, kin=kin, lo=lo, hi=hi, seeds=seeds, gaps=basket_gaps(), gap_index=a.gap, tilt_deg=a.tilt,
                  centre_h=a.centre_h, above_m=a.above, basket_z=a.basket_z, phis=sorted(range(-90, 91, a.phi_step), key=abs),
                  checks=[ch['frame'], ch['table']])
    xs = np.round(np.arange(a.x[0], a.x[1] + 1e-9, a.step), 3)
    ys = np.round(np.arange(a.y[0], a.y[1] + 1e-9, a.step), 3)
    jobs = [(x, y, a.yaw, qd) for qd in QUADRANTS for y in ys for x in xs]
    t0 = time.time()
    with mp.get_context('fork').Pool(a.workers) as pool:
        res = pool.map(_reach_one, jobs, chunksize=4)
    grid = {qd: np.zeros((len(ys), len(xs)), int) for qd in QUADRANTS}
    why = {qd: np.empty((len(ys), len(xs)), object) for qd in QUADRANTS}
    for (x, y, _, qd), (ok, phi, w) in zip(jobs, res):
        i, j = list(ys).index(y), list(xs).index(x)
        grid[qd][i, j], why[qd][i, j] = int(ok), w
    out = DATA / 'basket' / f'reach_{time.strftime("%Y%m%d_%H%M%S")}'
    out.mkdir(parents=True, exist_ok=True)
    meta = dict(run=run, xs=xs.tolist(), ys=ys.tolist(), yaw_deg=a.yaw, gap_index=a.gap, tilt_deg=a.tilt, centre_h_m=a.centre_h,
                above_m=a.above, basket_z=a.basket_z, phi_step=a.phi_step, seconds=round(time.time() - t0))
    (out / 'reach.json').write_text(json.dumps(dict(meta, grid={k: v.tolist() for k, v in grid.items()},
                                                    why={k: v.tolist() for k, v in why.items()}), indent=0))
    sym = dict(ok='#', ik='.', limit='j', branch='b', frame='f')
    print(f'basket reach ({time.time() - t0:.0f} s, H from {run}, yaw {a.yaw} deg, gap {a.gap}, tilt {a.tilt} deg)\n'
          f'# = reachable, . = no IK, j = joint limit, b = insert/above on different branches, f = frame/table')
    print('rows: basket centre y (top = +y, left of the robot); columns: x (right = far)')
    for qd in QUADRANTS:
        print(f'\n{qd} {QUADRANTS[qd]}: {grid[qd].sum()}/{grid[qd].size}')
        print('       ' + ''.join(f'{x:5.2f}' for x in xs))
        for i in range(len(ys) - 1, -1, -1):
            print(f'  {ys[i]:+.2f}' + ''.join(f'    {sym[w]}' for w in why[qd][i]))
    all4 = np.prod([grid[qd] for qd in QUADRANTS], 0)
    print(f'\nall four quadrants: {all4.sum()} positions')
    _reach_figure(xs, ys, grid, all4, out / 'reach.png')
    print(f'-> {out}')


def _reach_figure(xs, ys, grid, all4, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 5, figsize=(19, 4.2), sharey=True)
    ext = [xs[0] - (xs[1] - xs[0]) / 2, xs[-1] + (xs[1] - xs[0]) / 2, ys[0] - (ys[1] - ys[0]) / 2, ys[-1] + (ys[1] - ys[0]) / 2]
    for ax, (name, g) in zip(axs, list(grid.items()) + [('all four', all4)]):
        ax.imshow(g, origin='lower', extent=ext, cmap='Greens', vmin=0, vmax=1.4, aspect='equal')
        ax.set_xlabel('basket centre x [m]  (far ->)')
        ax.text(0.03, 0.95, f'{name} {QUADRANTS.get(name, "")}  {int(g.sum())}', transform=ax.transAxes, va='top')
        ax.plot([0], [0], 'k^') if ext[0] <= 0 else None
        for sp in ('top', 'right'):
            ax.spines[sp].set_visible(False)
    axs[0].set_ylabel('basket centre y [m]  (left of the robot ->)')
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------- quadrant plan (10/13: no taught path, any basket pose in reach)
PLATE_R = 0.134                                            # plate mesh 26.7 cm across, 2.5 cm thick


CARRY_MAX_WAYPOINTS = 60                                   # high carry leg, thinned (mover movesj limit 100, merged with the lift)
CARRY_STEP_DEG = 3.0                                       # (10/19 보스: smoother after the grasp) even joint step of the carry


def even_carry(path, k, step_deg=CARRY_STEP_DEG, n_max=CARRY_MAX_WAYPOINTS):
    """(10/19 보스) the carry resampled at one joint step (largest joint change per point), the drop start `k` kept as a point.
    movesj passes its points at about equal time, so uneven steps (1-6 deg from the rise/across/drop chains) made the speed
    jump; even steps give one smooth motion.  Returns (path, new k); at most n_max points."""
    w = np.asarray(path, float)
    seg = np.abs(np.diff(w, axis=0)).max(axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    if s[-1] <= 0:
        return [w[0], w[-1]], 1
    n = int(min(n_max - 1, max(2, np.ceil(np.degrees(s[-1]) / step_deg))))
    n_a = int(np.clip(round(n * s[k] / s[-1]), 1 if k > 0 else 0, n - 1 if k < len(w) - 1 else n))
    t = np.r_[np.linspace(0, s[k], n_a + 1), np.linspace(s[k], s[-1], n - n_a + 1)[1:]]
    out = np.stack([np.interp(t, s, w[:, j]) for j in range(w.shape[1])], axis=1)
    return list(out), n_a
STANDING_PLATE = dict(low_m=0.027, thick_m=0.02)           # a plate already in a gap: lowest rim ~27 mm above the floor


def basket_boxes(Tbb, wall=0.012, top=0.03, plates=()):
    """Basket body as 4 oriented wall boxes (base frame): 53 x 52 cm, floor -0.144 .. rim + rails/handle (`top`) in the
    basket frame.  The pins are not boxes: the plate goes between them by design.  `plates`: (model quadrant, gap index)
    of plates already standing in the basket (10/18 보스: insert the larger pin first, the next plate passes over it)."""
    import sys
    sys.path.insert(0, str(ROOT / 'rl'))
    import basket_env as be
    sx, sy = be.B['full']['size_xy']
    z0, z1 = be.B['full']['floor_z'], top
    zc, h = (z0 + z1) / 2, z1 - z0
    R = np.asarray(Tbb)[:3, :3]
    loc = [('wall_robot', [-sx / 2 + wall / 2, 0, zc], [wall, sy, h]), ('wall_far', [sx / 2 - wall / 2, 0, zc], [wall, sy, h]),
           ('wall_right', [0, -sy / 2 + wall / 2, zc], [sx, wall, h]), ('wall_left', [0, sy / 2 - wall / 2, zc], [sx, wall, h])]
    mk = C['basket'].get('marker_blocks')                    # (10/16 보스) marker plates/blocks on the rim: the plate hit one
    if mk and not C['basket'].get('markers_off'):           # (10/22 보스 마커 뗌, opt-in --markers_off): no blocks on the rim
        layout = json.loads(cpath(C['basket']['layout']).read_text())
        for i in C['basket']['marker_ids'].values():
            if str(i) in layout:
                m = np.asarray(layout[str(i)], float)
                loc.append((f'marker_{i}', [m[0], m[1], m[2] + mk['above_m'] - mk['height_m'] / 2],
                            [mk['size_m'], mk['size_m'], mk['height_m']]))
    for qd, g in plates:                                     # upright plate, 26.8 cm across: 7 thin slices along y, each up to the
        c = np.array(basket_gaps()[qd][g], float)            # disc's top over that slice (a full box would block the next row)
        zc, n = z0 + STANDING_PLATE['low_m'] + PLATE_R, 7
        for k, y0 in enumerate(np.linspace(-PLATE_R, PLATE_R, n + 1)[:-1]):
            y1 = y0 + 2 * PLATE_R / n
            top = zc + np.sqrt(max(0.0, PLATE_R ** 2 - min(y0 ** 2, y1 ** 2) * (y0 * y1 > 0)))
            loc.append((f'plate_{qd}_{g + 1}_{k}', [c[0], c[1] + (y0 + y1) / 2, (z0 + top) / 2],
                        [STANDING_PLATE['thick_m'], y1 - y0, top - z0]))
    return [(n, (np.asarray(Tbb) @ np.r_[c, 1.0])[:3].tolist(), sz, R) for n, c, sz in loc]


def plate_cloud(H, tool):
    """Plate disk samples in the link_6 frame (attached to the hand)."""
    a = np.linspace(0, 2 * np.pi, 32, endpoint=False)
    pts = [np.c_[r * np.cos(a), r * np.sin(a), np.full(len(a), z)] for r in (PLATE_R, 0.09, 0.045) for z in (-0.01, 0.015)]
    pts = np.vstack(pts + [[[0, 0, 0]]])
    return (np.c_[pts, np.ones(len(pts))] @ (tool @ H).T)[:, :3]


_BASKET_CHECKERS = {}


def quadrant_checkers(Tbc, q, Tbb, H, tool, plates=()):
    """Collision checkers for the quadrant plan: (with the plate in the hand, after the release)."""
    import copy
    from robot_model import CollisionChecker, MultiChecker
    lib = library_checkers(Tbc, q)
    c = np.asarray(Tbb)[:3, 3]

    def basket_checker(pl, margin=q.get('basket_wall_margin_m', 0.01)):
        boxes = basket_boxes(Tbb, plates=pl)
        key = (repr(boxes), margin, q.get('tool_length_m', 0.165), q.get('tool_radius_m', 0.06))
        if key not in _BASKET_CHECKERS:                      # (10/19 보스: shorter stop after the lift) the distance field
            ch = CollisionChecker(Kinematics(), boxes, resolution=0.005, margin=margin,   # (~1 s each) is built once per basket
                                  bounds=((c[0] - 0.5, c[0] + 0.5), (c[1] - 0.5, c[1] + 0.5), (-0.05, 0.6)))
            ch.set_tool(q.get('tool_length_m', 0.165), q.get('tool_radius_m', 0.06))
            _BASKET_CHECKERS.clear() if len(_BASKET_CHECKERS) > 8 else None
            _BASKET_CHECKERS[key] = ch
        return _BASKET_CHECKERS[key]
    basket = basket_checker(plates)
    plate = plate_cloud(H, tool)

    def plates_checker(pl, margin):                          # standing plates only, robot + gripper cylinder (no held plate)
        boxes = [b for b in basket_boxes(Tbb, plates=pl) if b[0].startswith('plate_')]
        key = ('plates', repr(boxes), margin, q.get('tool_length_m', 0.165), q.get('tool_radius_m', 0.06))
        if key not in _BASKET_CHECKERS:
            ch = CollisionChecker(Kinematics(), boxes, resolution=0.005, margin=margin,
                                  bounds=((c[0] - 0.5, c[0] + 0.5), (c[1] - 0.5, c[1] + 0.5), (-0.05, 0.6)))
            ch.set_tool(q.get('tool_length_m', 0.165), q.get('tool_radius_m', 0.06))
            _BASKET_CHECKERS[key] = ch
        return _BASKET_CHECKERS[key]

    def with_plate(ch):
        ch2 = copy.copy(ch)
        ch2.tool = np.vstack([ch.tool, plate]) if ch.tool is not None else plate
        return ch2
    held = [with_plate(lib['frame'][0]), with_plate(lib['table'][0]), with_plate(basket)]
    insert = held                                            # (10/14 실기) the plate rim vs the walls too (pins are not boxes)
    m_ins = q.get('basket_insert_wall_margin_m')             # (10/19 보스: Q3 plate a little more to the left = toward the +y wall)
    m_ins = q.get('basket_wall_margin_m', 0.01) if m_ins is None else m_ins
    if plates:                                               # (10/18) the plate in the hand goes down past a standing plate by
        insert = held[:2] + [with_plate(basket_checker((), m_ins))]   # design (dish 2.5 cm deep): walls/markers only for it, but
        if q.get('basket_insert_gripper_vs_plates', True):   # (10/19 보스: the gripper touched the red plate at Q4:11 while the
            insert.append(plates_checker(plates, m_ins))     # white went into pin 7) the arm + gripper keep off standing plates
    else:
        insert = held[:2] + [with_plate(basket_checker((), m_ins))]
    free = [lib['frame'][0], lib['table'][0], basket]
    obs = q.get('_obstacles') or []                          # (10/22 보스 장애물 데모, opt-in) seen obstacles: (name, centre, size, R)
    if obs:
        ob = obstacle_checker(obs, q)
        held, insert, free = held + [with_plate(ob)], insert + [with_plate(ob)], free + [ob]
    return MultiChecker(held), MultiChecker(insert), MultiChecker(free)


def obstacle_checker(boxes, q, pad=0.35):
    """(10/22) a distance field around the seen obstacle boxes only (5 mm, their extent + pad): ~0.2 s to build."""
    from robot_model import CollisionChecker
    P = np.vstack([np.asarray(c) + np.asarray(R) @ (np.array(sg) * np.asarray(sz) / 2)
                   for _, c, sz, R in boxes for sg in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3)])
    lo, hi = P.min(0) - pad, P.max(0) + pad
    lo[2] = max(lo[2], -0.05)
    ch = CollisionChecker(Kinematics(), boxes, resolution=0.005, margin=q.get('obstacle_margin_m', 0.02),
                          bounds=tuple(zip(lo, hi)))
    ch.set_tool(q.get('tool_length_m', 0.165), q.get('tool_radius_m', 0.06))
    return ch


def obstacle_clearance(path, H, tool, boxes, legs=('to_above', 'descend', 'one_go'), part=None):
    """(10/23 보스 장애물 안내) the held plate along a plan's carry vs each box -> [min distance m (< 0 = inside), ...].
    part 'carry' / 'drop': to_above before / from its drop start (the plan's quadrant_plan.drop_start)."""
    P_ = json.loads(Path(path).read_text())
    plan = P_['legs']
    W = [w for leg in legs if leg in plan for w in plan[leg]['waypoints_deg']]
    if part is not None:
        k = (P_.get('quadrant_plan') or {}).get('drop_start')
        W = plan['to_above']['waypoints_deg']
        W = W if k is None else (W[:k + 1] if part == 'carry' else W[k:])
    if not W or not boxes:
        return []
    kin, cl = Kinematics(), plate_cloud(H, tool)
    P = np.vstack([cl @ T[:3, :3].T + T[:3, 3] for T in (kin.fk(np.radians(w)) for w in W)])
    out = []
    for _, c, size, R in (b[:4] for b in boxes):
        L = np.abs((P - np.asarray(c)) @ np.asarray(R)) - np.asarray(size) / 2
        d = np.linalg.norm(np.maximum(L, 0.0), axis=1) + np.minimum(L.max(1), 0.0)
        out.append(float(d.min()))
    return out


def upright_box(c, size, R):
    """An oriented box -> (centre, half sizes, R) of the smallest box around it with a vertical z (turned about z only)."""
    R = np.asarray(R, float)
    P = np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3) * np.asarray(size) / 2 @ R.T
    a = R[:2, int(np.argmax(np.linalg.norm(R[:2], axis=0)))]          # the most horizontal box axis sets the yaw
    yaw = np.arctan2(a[1], a[0]) if np.linalg.norm(a) > 1e-9 else 0.0
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1.0]])
    L = P @ Rz
    return np.asarray(c, float) + Rz @ ((L.max(0) + L.min(0)) / 2), (L.max(0) - L.min(0)) / 2, Rz


def plate_obstacle(T_plate, thick=0.03):
    """(10/22) a plate seen at pose T (FP: z = plate normal) -> oriented box (name, centre, size, R)."""
    T = np.asarray(T_plate, float)
    return ('obstacle_plate', T[:3, 3].tolist(), [2 * PLATE_R, 2 * PLATE_R, thick], T[:3, :3].tolist())


class FloorCheck:
    """(10/19 보스: RRT when the rule carry fails) a 'checker' for MultiChecker: over the basket (its footprint + `pad`)
    the plate's lowest point stays >= z_floor (marker blocks + clearance).  clearance/margin = 1 at the floor, +1 per
    `scale` m above, so the RRT's planning margin keeps it a little higher.  Away from the basket: no limit."""

    def __init__(self, kin, cloud, Tbb, z_floor, pad=0.08, scale=0.02):
        import sys
        sys.path.insert(0, str(ROOT / 'rl'))
        import basket_env as be
        self.kin, self.cloud, self.Tbi = kin, np.asarray(cloud, float), np.linalg.inv(np.asarray(Tbb, float))
        self.half = np.asarray(be.B['full']['size_xy'], float) / 2 + pad
        self.z_floor, self.scale, self.margin = float(z_floor), float(scale), 1.0

    def clearance(self, q):
        T = self.kin.fk(q)
        P = self.cloud @ T[:3, :3].T + T[:3, 3]
        loc = P @ self.Tbi[:3, :3].T + self.Tbi[:3, 3]
        over = np.all(np.abs(loc[:, :2]) <= self.half, axis=1)
        if not over.any():
            return 10.0
        return 1.0 + (float(P[over, 2].min()) - self.z_floor) / self.scale

    def path_clearance(self, path):
        return min(self.clearance(q) for q in path)


def rrt_carry(held, floor, start, goal, seeds=(0, 1), max_wrist_deg=100.0, max_nodes=1500, step_deg=2.0, span_deg=90.0):
    """(10/19 보스) bidirectional RRT for the carry with the floor over the basket; a path that turns J4 or J6 more than
    max_wrist_deg away from the start (wrist spin, 보스 10/19) is refused and the next seed tried."""
    from robot_model import MultiChecker
    mc = MultiChecker(held.checkers + [floor])
    ends = {n: (round(floor.clearance(x), 2), round(held.clearance(x), 2)) for n, x in (('start', start), ('goal', goal))}
    if min(min(v) for v in ends.values()) < 1.0:
        raise RuntimeError(f'rrt carry: an end is not free (floor, held) {ends}')
    errs = []
    for sd in seeds:
        try:
            path, how = mc.plan(start, goal, seed=sd, max_nodes=max_nodes, step_deg=step_deg, span_deg=span_deg)   # (10/19) seconds, not minutes
        except RuntimeError as e:
            errs.append(str(e))
            continue
        w = np.degrees(np.abs(np.array(path)[:, [3, 5]] - np.asarray(start)[[3, 5]])).max()
        if w > max_wrist_deg:
            errs.append(f'wrist {w:.0f} deg')
            continue
        return path, f'rrt-floor {how}'
    raise RuntimeError(f'rrt carry {ends}: ' + '; '.join(errs[-2:]))


def high_carry(kin, start, goal, cloud, z_low, ok_lim, step_m=0.02, sparse=False):
    """(10/18 보스: the plate hit the marker plates on a low carry) carry with the plate's lowest point never under `z_low`
    (base z, or a function of the plate points in base -> the floor there): straight up from `start`, across at that height, then straight down onto `goal` (the above pose, plate
    upright).  Across: the joint-space line between the two high poses (like the old straight carry), each point lifted
    straight up wherever the plate would dip under `z_low`; else position straight + orientation slerp, lifted the same way.
    -> (joint waypoints (rad), index where the straight-down part starts)."""
    from scipy.spatial.transform import Rotation, Slerp
    low = lambda T: float((cloud @ T[:3, :3].T + T[:3, 3])[:, 2].min())    # noqa: E731
    need = (lambda T: float(z_low(cloud @ T[:3, :3].T + T[:3, 3]))) if callable(z_low) else (lambda T: float(z_low))  # noqa: E731
    T0, T1 = kin.fk(start), kin.fk(goal)
    up0, up1 = max(0.0, need(T0) - low(T0)), max(0.0, need(T1) - low(T1))

    cart_step = 0.15 if sparse else 0.01
    joint_step_deg = 30.0 if sparse else 2.0
    max_jump_deg = 40.0 if sparse else 8.0

    def vertical(T, d_from, d_to):
        out = []
        for d in np.linspace(d_from, d_to, max(2, int(np.ceil(abs(d_to - d_from) / cart_step)) + 1))[1:]:
            T_ = T.copy()
            T_[2, 3] += d
            out.append(T_)
        return out

    def chain(poses, prev, what):
        out = []
        for T in poses:
            try:
                sol = kin.ik(T, prev)
            except RuntimeError:
                raise RuntimeError(f'high carry: {what} ik') from None
            if not ok_lim(sol) or np.degrees(np.abs(sol - prev)).max() > max_jump_deg:
                raise RuntimeError(f'high carry: {what} limit/flip')
            out.append(sol)
            prev = sol
        return out
    def arm_vertical(q0, up, what):                          # (10/19 실기) straight up by J2/J3/J5 only: J1, J4, J6 stay (no wrist
        from scipy.optimize import least_squares             # flip near J5 = 0); J5 keeps the tool axis (the plate stays level)
        T0_ = kin.fk(q0)
        p0, z0 = T0_[:3, 3], T0_[:3, 2]
        idx = [1, 2, 4]
        out, prev = [], q0
        for d in np.linspace(0.0, up, max(2, int(np.ceil(up / cart_step)) + 1))[1:]:
            def res(x):
                q_ = q0.copy()
                q_[idx] = x
                T_ = kin.fk(q_)
                return np.r_[(T_[:3, 3] - p0 - [0, 0, d]) * 10, (T_[:3, 2] - z0) * 0.5]
            fit = least_squares(res, prev[idx], max_nfev=300)
            if np.abs(res(fit.x)[:3]).max() > 1e-3:
                raise RuntimeError(f'high carry: {what} arm reach')
            sol = q0.copy()
            sol[idx] = fit.x
            if not ok_lim(sol) or np.degrees(np.abs(sol - prev)).max() > max_jump_deg:
                raise RuntimeError(f'high carry: {what} arm limit')
            out.append(sol)
            prev = sol
        return out

    def straight_up(T, q0, up, what):
        if up <= 0:
            return []
        try:
            return chain(vertical(T, 0.0, up), q0, what)
        except RuntimeError as e:
            try:
                return arm_vertical(q0, up, what)
            except RuntimeError as e2:
                raise RuntimeError(f'{e}; {e2}') from None
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    rise = straight_up(T0, start, up0, 'rise')
    q_up0 = rise[-1] if rise else start
    drop_up = straight_up(T1, goal, up1, 'drop')             # built upward from the goal, used reversed
    q_up1 = drop_up[-1] if drop_up else goal
    def lift_by_arm(qi, seed):
        """qi with J2/J3 changed so the plate's lowest point is at z_low and the flange keeps its distance from the base
        axis (the wrist joints stay on the joint line: no IK flip near the J5 = 0 wrist singularity)."""
        from scipy.optimize import least_squares
        p0 = kin.fk(qi)[:3, 3]
        r0 = np.hypot(p0[0], p0[1])

        def res(x):
            q_ = qi.copy()
            q_[1:3] = x
            T = kin.fk(q_)
            return [(low(T) - need(T)) * 10, (np.hypot(T[0, 3], T[1, 3]) - r0) * 10]
        fit = least_squares(res, seed[1:3], max_nfev=200)
        if np.abs(res(fit.x)).max() > 1e-3:
            raise RuntimeError('high carry: arm lift unreachable')
        q_ = qi.copy()
        q_[1:3] = fit.x
        return q_

    def raised_joint_line():                                 # the old carry's joint line, lifted where the plate would dip
        n = max(2, int(np.ceil(np.degrees(np.abs(q_up1 - q_up0)).max() / joint_step_deg)) + 1)
        out, prev, off = [], q_up0, np.zeros(6)
        for f in np.linspace(0, 1, n)[1:-1]:
            qi = q_up0 + (q_up1 - q_up0) * f
            T = kin.fk(qi)
            dz = need(T) - low(T)
            sol = lift_by_arm(qi, qi + off) if dz > 0 else qi
            if not ok_lim(sol) or np.degrees(np.abs(sol - prev)).max() > max_jump_deg:
                raise RuntimeError(f'high carry: joint line limit/flip at {f:.2f} (+{dz:.3f} m, '
                                   f'J{int(np.argmax(np.abs(sol - prev))) + 1} {np.degrees(np.abs(sol - prev)).max():.0f} deg)')
            out.append(sol)
            prev, off = sol, sol - qi
        if np.degrees(np.abs(q_up1 - prev)).max() > max_jump_deg:
            raise RuntimeError('high carry: across limit/flip')
        return out

    def slerp_line():                                        # straight in space (the wrist may not follow it, e.g. J5 through 0)
        rots = Slerp([0, 1], Rotation.from_matrix([T0[:3, :3], T1[:3, :3]]))
        p0, p1 = T0[:3, 3] + [0, 0, up0], T1[:3, 3] + [0, 0, up1]
        poses = []
        step = 0.15 if sparse else step_m
        for f in np.linspace(0, 1, max(2, int(np.ceil(np.linalg.norm(p1 - p0) / step)) + 1))[1:-1]:
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = rots(f).as_matrix(), p0 + (p1 - p0) * f
            T[2, 3] += max(0.0, need(T) - low(T))
            poses.append(T)
        out = chain(poses, q_up0, 'across')
        if np.degrees(np.abs(q_up1 - (out[-1] if out else q_up0))).max() > max_jump_deg:
            raise RuntimeError('high carry: across limit/flip')
        return out
    try:
        across = raised_joint_line()
    except RuntimeError as e:
        try:
            across = slerp_line()
        except RuntimeError as e2:
            raise RuntimeError(f'{e}; {e2}') from None
    path = [start] + rise + across + [q_up1]
    return path + drop_up[::-1][1:] + [goal], len(path) - 1

def quadrant_plan(Tbb, quadrant, gap, H, tool, start_deg, Tbc, q, grasp_legs=None, tilt_deg=0.0, centre_h=0.18, above_m=0.11,
                  phi_step=10, seeds=None, yaw_trim_deg=0.0, y_shift_m=None, x_shift_m=None, faces=(1.0, -1.0), y_extra_m=0.0,
                  grasp_deg=None, plates=(), grasp_low_z=None, phi_first=None):
    """Plan.json for one quadrant gap from the plate in the hand: to_above (carry, straight or RRT; with config
    basket_carry_rise_m: high carry, see high_carry), descend (straight down, plate upright, +basket_policy_extra_down_m),
    ascend (straight up), to_home, abort_home.  `plates`: gaps already holding a plate (index in this quadrant, or
    (model quadrant, index)).  `grasp_low_z`: the plate's lowest point on the stand (base z) for the rise rule (else
    the in-hand plate model at the grasp joints, which sits ~3 cm low once the plate has tilted in the hand).
    -> (path, report)."""
    if q.get('basket_carry_rrt_fallback') and not q.get('_rrt_pass') and q.get('basket_carry_mode', 'rule') != 'rrt':   # (10/19) every face/phi with the rule first; the
        args_ = dict(Tbb=Tbb, quadrant=quadrant, gap=gap, H=H, tool=tool, start_deg=start_deg, Tbc=Tbc, grasp_legs=grasp_legs,
                     tilt_deg=tilt_deg, centre_h=centre_h, above_m=above_m, phi_step=phi_step, seeds=seeds,
                     yaw_trim_deg=yaw_trim_deg, y_shift_m=y_shift_m, x_shift_m=x_shift_m, faces=faces, y_extra_m=y_extra_m,
                     grasp_deg=grasp_deg, plates=plates, grasp_low_z=grasp_low_z, phi_first=phi_first)
        path, rep = quadrant_plan(q={**q, 'basket_carry_rrt_fallback': False}, **args_)   # RRT carry only when none has
        if path is not None:                                                            # a rule carry
            return path, rep
        path2, rep2 = quadrant_plan(q={**q, '_rrt_pass': True}, **args_)
        if path2 is None:
            rep2['tried'] = rep['tried'] + rep2['tried']
        return path2, rep2
    import ros2_nodes
    from robot_model import polyline_path as pp
    kin = Kinematics()
    lo, hi = np.array(ros2_nodes.joint_limits_deg()).T
    plates = [tuple(g) if isinstance(g, (tuple, list)) else (quadrant, int(g)) for g in plates]   # gap here or (model qd, gap)
    fast_unchecked = bool(q.get('_fast_unchecked_carry'))
    held = inserting = free = None
    if not fast_unchecked:
        held, inserting, free = quadrant_checkers(Tbc, q, Tbb, H, tool, plates=plates)
    rise = q.get('basket_carry_rise_m')
    if rise is not None:                                     # (10/18 보스) carry floor: >= rise above the plate at the grasp,
        cloud = plate_cloud(H, tool)                         # clear of the marker blocks and of standing plates
        low_at = lambda dg: float((cloud @ kin.fk(np.radians(dg))[:3, :3].T + kin.fk(np.radians(dg))[:3, 3])[:, 2].min())  # noqa: E731
        if grasp_deg is None and grasp_legs:
            grasp_deg = grasp_legs['to_grasp'][-1]
        clear = dict(marker=q.get('basket_carry_clear_m', 0.05), plate=q.get('basket_carry_plate_clear_m', 0.05))
        tops = [(n_.split('_')[0], c[2] + sz[2] / 2) for n_, c, sz, _ in basket_boxes(Tbb, plates=plates) if n_.startswith(('marker', 'plate'))]
        tops += [('marker', c[2] + sz[2] / 2) for n_, c, sz, _ in basket_boxes(Tbb) if n_.startswith('wall')]   # (10/23: with
        # --markers_off nothing else kept the carry floor over the rim: a 10 cm rise grazed the walls; the marker tops are higher)
        g_low = float(grasp_low_z) if grasp_low_z is not None else low_at(grasp_deg if grasp_deg is not None else start_deg)
        z_base = max([g_low + float(rise)]                   # (10/19 보스) grasp_low_z: the plate's bottom on the stand (FP)
                     + [t + clear[k] for k, t in tops if k == 'marker'])
        stand = [(np.asarray(c), np.asarray(sz) / 2, R_) for n_, c, sz, R_ in basket_boxes(Tbb, plates=plates) if n_.startswith('plate')]
        stand_all = stand + [upright_box(c, sz, R_) for _, c, sz, R_ in (q.get('_obstacles') or [])]   # (10/22 장애물 데모) seen

        def z_low(P):                                        # (10/18) a standing plate only raises the floor near it: its top +
            z = z_base                                       # clear right over it, 1 m per m lower with the horizontal distance
            for c, h, R_ in stand_all:
                loc = (P - c) @ R_
                d = np.linalg.norm(np.maximum(np.abs(loc[:, :2]) - h[:2], 0.0), axis=1).min()
                z = max(z, c[2] + h[2] + clear['plate'] - d)
            return z
        z_top = max([z_base] + [c[2] + h[2] + clear['plate'] for c, h, _ in stand])
    g_ = np.array(basket_gaps()[quadrant][gap], float)
    if y_shift_m is None:                                    # (10/14 실기) keep the rim off the side wall: 26.8 cm plate,
        import sys                                           # pair A at y +0.129 put the rim on the +y wall
        sys.path.insert(0, str(ROOT / 'rl'))
        import basket_env as be
        room = be.B['full']['size_xy'][1] / 2 - 0.012 - q.get('basket_wall_margin_m', 0.01) - PLATE_R - 0.005
        y_shift_m = float(np.clip(0.0, -room - g_[1], room - g_[1])) if abs(g_[1]) > room else 0.0
    g_[1] += y_shift_m + y_extra_m                          # (10/15 보스) y_extra: per-quadrant real trim on top of the wall shift
    if x_shift_m is None:                                    # (10/14 실기) the plate lands 2-3 cm farther (+x) than planned
        x_shift_m = q.get('basket_quadrant_x_trim_m', 0.0)
    g_[0] += x_shift_m
    gap_b = (np.asarray(Tbb) @ np.r_[g_, 1.0])[:3]
    n, t = np.asarray(Tbb)[:3, 0], np.radians(tilt_deg)
    if yaw_trim_deg:                                         # (10/14 실기) plate face turned about the vertical, + = CCW seen from above
        c_, s_ = np.cos(np.radians(yaw_trim_deg)), np.sin(np.radians(yaw_trim_deg))
        n = np.array([c_ * n[0] - s_ * n[1], s_ * n[0] + c_ * n[1], n[2]])
    v = -n * np.sin(t) + np.array([0, 0, 1.0]) * np.cos(t)
    extra = q.get('basket_policy_extra_down_m', 0.0)
    seeds = seeds or [np.radians(start_deg), np.radians(HOME_JOINTS_DEG)]
    ok_lim = lambda qq: ((np.degrees(qq) > lo + 3) & (np.degrees(qq) < hi - 3)).all()   # noqa: E731
    tried = []
    # (10/14 18:xx) the plate is the same both ways: facing -x lets the far quadrants (Q1/Q2) solve with the wrist flipped
    # (J4 ~ -155, as 보스 taught them); +x alone gave 'ik/limit' there at every basket x 0.90-0.99
    p0 = 0 if phi_first is None else phi_first              # (10/19 보스: shorter stop after the lift) the replan tries the
    for side, phi in [(sd_, p_) for sd_ in faces             # phi of the plan before the lift first, then the nearest ones
                      for p_ in sorted(range(-90, 91, phi_step), key=lambda a: (abs(a - p0), abs(a)))]:
        ph = np.radians(phi)
        zt = side * (n * np.cos(t) + np.array([0, 0, 1.0]) * np.sin(t))
        x = v * np.cos(ph) + np.cross(zt, v) * np.sin(ph)
        Tp = np.eye(4)
        Tp[:3, :3] = np.stack([x, np.cross(zt, x), zt], 1)
        Tp[:3, 3] = gap_b + v * centre_h / np.cos(t)
        T6 = Tp @ np.linalg.inv(H) @ np.linalg.inv(tool)
        down, prev = [], None
        n_down = 4 if fast_unchecked else 13
        for lift in np.linspace(above_m, -extra, n_down):    # sparse fast path, or legacy ~1 cm steps
            Tl = T6.copy()
            Tl[2, 3] += lift
            sol = None
            for sd in ([prev] if prev is not None else seeds):
                try:
                    cand = kin.ik(Tl, sd)
                except RuntimeError:
                    continue
                if ok_lim(cand) and (prev is None or np.degrees(np.abs(cand - prev)).max() < 8):
                    sol = cand
                    break
            if sol is None:
                break
            down.append(sol)
            prev = sol
        if len(down) < n_down:
            tried.append(((int(side), phi), 'ik/limit'))
            continue
        c_ins = None
        if not fast_unchecked:
            c_ins = inserting.path_clearance(pp(np.array(down), 0.5))
            if c_ins < 1.0:
                tried.append(((int(side), phi), f'insert clearance {c_ins:.2f}'))
                continue
        try:
            if rise is None:
                if fast_unchecked:
                    carry, k, how = [np.radians(start_deg), down[0]], 0, 'direct sparse unchecked'
                else:
                    carry, how = held.plan(np.radians(start_deg), down[0])
            else:
                try:
                    if q.get('basket_carry_mode', 'rule') == 'rrt':   # (10/19 18:0x 보스 'RRT써서 경로 찾게') RRT first
                        raise RuntimeError('carry mode rrt')
                    carry, k = high_carry(kin, np.radians(start_deg), down[0], cloud, z_low, ok_lim,
                                          sparse=fast_unchecked)
                    how = ('sparse unchecked ' if fast_unchecked else '') + f'high {z_base:.3f}' + (f'/{z_top:.3f}' if stand else '')
                except RuntimeError as e_rule:              # (10/19 보스) the rule failed: RRT with the floor over the basket
                    if not (q.get('basket_carry_rrt_fallback', False) or q.get('basket_carry_mode') == 'rrt'):
                        raise
                    z_mk = max([t + clear['marker'] for k_, t in tops if k_ == 'marker'] or [0.0])
                    if quadrant in q.get('basket_carry_rrt_over_plates', ()):   # (10/19 보스: Q4 2nd plate hit the standing
                        z_mk = max([z_mk] + [t + clear['plate'] for k_, t in tops if k_ == 'plate'])   # one before the descent:
                    z_mk += (q.get('basket_carry_rrt_extra_m', {}) or {}).get(quadrant, 0.0) if stand else 0.0   # over its top
                    try:
                        fl = FloorCheck(kin, cloud, Tbb, z_mk)
                        Tg = kin.fk(down[0])                 # straight up over the slot until the floor (+1 cm), like the
                        up = max(0.0, z_mk + 0.01 - float((cloud @ Tg[:3, :3].T + Tg[:3, 3])[:, 2].min()))   # rule's drop
                        drop, prev = [down[0]], down[0]
                        for d in np.linspace(0.0, up, max(2, int(np.ceil(up / 0.01)) + 1))[1:]:
                            T_ = Tg.copy()
                            T_[2, 3] += d
                            sol = kin.ik(T_, prev)
                            if not ok_lim(sol) or np.degrees(np.abs(sol - prev)).max() > 8:
                                raise RuntimeError('rrt carry: drop limit/flip')
                            drop.append(sol)
                            prev = sol
                        from robot_model import MultiChecker
                        rr = q.get('basket_rrt', {}) or {}  # (10/19 보스) RRT knobs in config, like the grasp library's
                        mc_ = MultiChecker(held.checkers + [fl])
                        top = None
                        import curobo_carry                # (10/20 보스, opt-in CARRY_PLANNER=curobo) cuRobo first, RRT second
                        if curobo_carry.enabled(q):
                            try:
                                s_ = np.radians(start_deg)
                                cp, cs = curobo_carry.carry(q, held, H, tool, Tbb, z_mk, fl.half, s_, drop[-1])
                                wr = np.degrees(np.abs(np.array(cp)[:, [3, 5]] - s_[[3, 5]])).max()
                                c_cu = mc_.path_clearance(pp(np.array(cp), 0.5))
                                if c_cu >= 1.0 and wr <= rr.get('max_wrist_deg', 100.0):
                                    top, how = cp, f'curobo {cs:.2f}s'
                                else:
                                    print(f'   curobo path refused: clearance {c_cu:.2f}, wrist {wr:.0f} -> RRT', flush=True)
                            except RuntimeError as e_cu:
                                print(f'   {e_cu} -> RRT', flush=True)
                        prior = q.get('_carry_prior')      # (10/19 보스) the carry that worked here before, reused first
                        if prior and top is None:
                            pw = np.radians(np.asarray(prior['waypoints_deg'], float))
                            cand = [np.radians(start_deg)] + list(pw[1:int(prior['drop_start'])]) + [drop[-1]]
                            wr = np.degrees(np.abs(np.array(cand)[:, [3, 5]] - np.radians(start_deg)[[3, 5]])).max()
                            c_pr = mc_.path_clearance(pp(np.array(cand), 0.5))
                            if c_pr >= 1.0 and wr <= rr.get('max_wrist_deg', 100.0):
                                top, how = cand, f'prior {prior.get("source", "")[-40:]}'
                            elif os.environ.get('PRIOR_DEBUG'):
                                print(f'   prior refused: clearance {c_pr:.2f}, wrist {wr:.0f}', flush=True)
                        s0, pre = np.radians(start_deg), []
                        lift_max = q.get('basket_carry_start_lift_m', 0.0)   # (10/19 night, opt-in) the lifted plate a bit
                        if top is None and lift_max > 0 and mc_.clearance(s0) < 1.0:   # inside the margin (frame post):
                            T0_ = kin.fk(s0)                                            # a short straight move away
                            inward = -np.r_[T0_[:2, 3], 0.0] / max(np.linalg.norm(T0_[:2, 3]), 1e-9)   # first (up, back
                            for dvec in (np.r_[0, 0, 1.0], inward, (inward + np.r_[0, 0, 1.0]) / np.sqrt(2)):   # toward the
                                pre, prev_ = [], s0                                     # robot, or both), then the RRT
                                for d in np.arange(0.01, lift_max + 1e-9, 0.01):
                                    T_ = T0_.copy()
                                    T_[:3, 3] += d * dvec
                                    try:
                                        sol = kin.ik(T_, prev_)
                                    except RuntimeError:
                                        break
                                    if not ok_lim(sol) or np.degrees(np.abs(sol - prev_)).max() > 8:
                                        break
                                    pre.append(sol)
                                    prev_ = sol
                                    if mc_.clearance(sol) >= 1.0:
                                        break
                                if pre and mc_.clearance(pre[-1]) >= 1.0:
                                    s0 = pre[-1]
                                    break
                                pre = []
                        if top is None:
                            top, how = rrt_carry(held, fl, s0, drop[-1], seeds=tuple(rr.get('seeds', (0, 1))),
                                                 max_wrist_deg=rr.get('max_wrist_deg', 100.0), max_nodes=rr.get('max_nodes', 1500),
                                                 step_deg=rr.get('step_deg', 2.0), span_deg=rr.get('span_deg', 90.0))
                    except RuntimeError as e_rrt:
                        raise RuntimeError(f'{e_rule}; {e_rrt}') from None
                    if pre and not how.startswith('prior'):
                        top = [np.radians(start_deg)] + pre[:-1] + list(top)
                        how += f' +lift {len(pre)} cm'
                    carry = [np.asarray(c_) for c_ in top] + [np.asarray(c_) for c_ in drop[::-1][1:]]
                    k = len(top) - 1
                if fast_unchecked:
                    c_car = None
                else:
                    thr = 1.0                                     # (10/19 night, opt-in start lift) never closer than the lifted
                    if q.get('basket_carry_start_lift_m', 0.0) > 0:   # start itself (it is already there), else the margin
                        thr = min(1.0, max(0.5, held.clearance(np.asarray(carry[0]))) - 1e-6)   # never below half the margin
                    c_car = min(held.path_clearance(pp(np.array(carry[:k + 1]), 0.5)) / thr,
                                inserting.path_clearance(pp(np.array(carry[k:]), 0.5)))
                    if c_car < 1.0:
                        raise RuntimeError(f'high carry clearance {c_car:.2f}')
                    fl_ok = (lambda c, k_: fl.path_clearance(pp(np.array(c[:k_ + 1]), 0.5)) >= 1.0) if how.startswith(('rrt', 'prior', 'curobo')) \
                        else (lambda c, k_: True)                 # (10/19 night) the even resample may cut a corner under the floor
                    clear_of = lambda c, k_: min(held.path_clearance(pp(np.array(c[:k_ + 1]), 0.5)) / thr,   # noqa: E731
                                                 inserting.path_clearance(pp(np.array(c[k_:]), 0.5))) if fl_ok(c, k_) else 0.0
                    ev, k_ev = even_carry(carry, k)               # (10/19 보스) evenly spaced, <= 60 points (movesj limit 100)
                    if clear_of(ev, k_ev) >= 1.0:
                        carry, k = ev, k_ev
                    elif len(carry) > CARRY_MAX_WAYPOINTS:         # the even points cut a corner too close: the planned points,
                        n_ = len(carry) - 1                        # thinned by index (before 10/19 14:2x)
                        keep = np.unique(np.r_[np.linspace(0, k, max(2, CARRY_MAX_WAYPOINTS * k // n_)).round(),
                                               np.linspace(k, n_, max(2, CARRY_MAX_WAYPOINTS * (n_ - k) // n_)).round()]).astype(int)
                        carry, k = [carry[i] for i in keep], int(np.searchsorted(keep, k))
                        c_car = clear_of(carry, k)
                        if c_car < 1.0:
                            raise RuntimeError(f'high carry (thinned) clearance {c_car:.2f}')
                    if not fl_ok(carry, k):                       # (10/20 night) the RRT checks the floor every 2 deg; entering
                        raise RuntimeError('rrt carry: under the floor between samples')   # the basket between two samples
            if fast_unchecked:
                home, how_h = [down[0], np.radians(HOME_JOINTS_DEG)], 'direct unchecked'
            else:
                home, how_h = free.plan(down[0], np.radians(HOME_JOINTS_DEG))      # could dip 7 mm: 0.5 deg check
        except RuntimeError as e:
            tried.append(((int(side), phi), str(e)))
            continue
        legs = dict(to_above=[np.degrees(w).tolist() for w in carry], descend=[np.degrees(w).tolist() for w in down],
                    ascend=[np.degrees(w).tolist() for w in down[::-1]], to_home=[np.degrees(w).tolist() for w in home],
                    abort_home=([np.degrees(w).tolist() for w in home] if fast_unchecked else
                                [np.degrees(w).tolist() for w in free.plan(down[0], np.radians(HOME_JOINTS_DEG))[0]]))
        if grasp_legs:
            legs = dict(grasp_legs, **legs)
        path = (unchecked_sparse_plan(legs, f'quadrant {quadrant} gap {gap + 1} sparse unchecked post-grasp')
                if fast_unchecked else taught_plan(legs, Tbc, q, f'quadrant {quadrant} gap {gap + 1}'))
        plan = json.loads(Path(path).read_text())
        rep = dict(quadrant=quadrant, gap=gap + 1, phi_deg=phi, face=int(side), drop_start=None if rise is None else int(k), y_shift_m=round(y_shift_m, 4), x_shift_m=round(x_shift_m, 4), carry=how, home=how_h, tried=tried, accepted=plan['accepted'],
                   clearance_ratio=(dict(carry=None, insert=None, after=None) if fast_unchecked else
                                    dict(carry=round(held.path_clearance(pp(np.array(carry), 0.5)) if rise is None else c_car, 2),
                                         insert=round(c_ins, 2), after=round(free.path_clearance(pp(np.array(home), 0.5)), 2))))
        plan.update(basket_T=np.asarray(Tbb).tolist(), plate_in_tcp=np.asarray(H).tolist(), quadrant_plan=rep)
        Path(path).write_text(json.dumps(plan, indent=1))
        return path, rep
    return None, dict(quadrant=quadrant, gap=gap + 1, tried=tried)


def quadrant_plans(a):
    """Offline: one plan per quadrant for a basket pose and a plate position (the newest real grasp turned about joint 1
    to that plate), printed as a table; the chosen quadrant goes to the RViz preview (run.sh viz)."""
    import socket
    H, tool, run = plate_in_hand(a.run)
    d = DATA / 'real_pipeline' / run
    ev = [json.loads(x) for x in (d / 'robot_state.jsonl').read_text().splitlines() if '"event"' in x]
    gplan = json.loads(Path(next(e for e in ev if e['event'] == 'plan')['plan']).read_text())['legs']
    req = json.loads((d / 'realplan_req.json').read_text())
    p0 = np.asarray(req['plate_root_fp'][:2])
    dj = 0.0 if a.plate is None else np.degrees(np.arctan2(a.plate[1], a.plate[0]) - np.arctan2(p0[1], p0[0]))
    if a.plate is not None and abs(np.hypot(*a.plate) - np.hypot(*p0)) > 0.02:
        print(f'note: plate radius {np.hypot(*a.plate):.3f} m vs grasped {np.hypot(*p0):.3f} m: only the direction is changed')
    turn = lambda w: [w[0] + dj] + list(w[1:])              # noqa: E731
    grasp = {k: [turn(w) for w in gplan[k]['waypoints_deg']] for k in ('to_grasp', 'lift')}
    Tbc = np.array(__import__('calib_core').load_json(__import__('calib_core').latest_t_base_cam())['T_base_cam'])
    q = C['plate_ik_bc']
    c, s_ = np.cos(np.radians(a.basket[2])), np.sin(np.radians(a.basket[2]))
    Tbb = np.eye(4)
    Tbb[:2, :2] = [[c, -s_], [s_, c]]
    Tbb[:3, 3] = [a.basket[0], a.basket[1], a.basket_z]
    print(f'basket ({a.basket[0]:.2f}, {a.basket[1]:+.2f}) yaw {a.basket[2]:+.0f}; plate direction turned {dj:+.1f} deg (grasp of {run})')
    out = {}
    for qd in a.quadrants:
        t0 = time.time()
        kw = dict(tilt_deg=a.tilt)
        if a.sim_q:                                          # (10/15 night) the Isaac per-quadrant settings (no in-hand sensing),
            import sys                                       # for an RViz preview only - candidate values, not approved real ones
            sys.path.insert(0, str(ROOT / 'rl'))
            import basket_env as be
            sq = C['basket_quadrant_sim'][qd]
            kw = dict(tilt_deg=sq['lean'] * be.B['full']['tine_lean_deg'][qd], x_shift_m=sq['aim_m'], centre_h=sq['centre_h'],
                      faces=(-1.0, 1.0) if sq['flip'] else (1.0, -1.0))
            print(f'{qd}: sim settings {sq} -> tilt {kw["tilt_deg"]:+.2f} deg', flush=True)
        path, rep = quadrant_plan(Tbb, qd, a.gap, H, tool, grasp['lift'][-1], Tbc, q, grasp_legs=grasp, **kw)
        out[qd] = path
        print(f'{qd} gap {a.gap + 1}: ' + (f'OK phi {rep["phi_deg"]:+d}, carry {rep["carry"]}, home {rep["home"]}, '
                                          f'clearance/margin {rep["clearance_ratio"]}, accepted {rep["accepted"]} ({time.time() - t0:.0f} s) -> {path}'
                                          if path else f'NO PLAN ({time.time() - t0:.0f} s): {rep["tried"][:4]}'), flush=True)
    show = out.get(a.show) or next((p_ for p_ in out.values() if p_), None)
    if show:
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(json.dumps(dict(plan=str(show))).encode(),
                                                                ('127.0.0.1', C['udp'].get('viz', 5011)))
        print(f'RViz preview (if run.sh viz is running): {show}')


def measured_basket(cfg=None):
    """Basket pose (base) from the fixed camera's markers, read-only.  Each marker ID has its own stored position in the
    basket frame (config basket.layout), so the fit (2-D Kabsch) finds the basket at any yaw, also turned around."""
    from calib_core import cpath
    from camera_tools import basket_fit_robust, basket_measure
    cfg = cfg or C
    layout = json.loads(cpath(cfg['basket']['layout']).read_text())
    T, res, seen = basket_fit_robust(basket_measure(cfg), layout, cfg['basket'].get('max_fit_residual_m', 0.008))
    return T, res, seen


def robot_quadrants(Tbb):
    """{robot quadrant: basket-model quadrant}.  Q1..Q4 are named as seen from the robot (top view, robot at the bottom,
    far = +x, left = +y); the model's quadrants are fixed to the basket (markers), so a turned basket maps onto them:
    the labels move one quadrant per 90 deg of basket yaw (nearest; always one-to-one, also at 45 deg)."""
    ccw = ['Q1', 'Q2', 'Q3', 'Q4']                           # far-right, far-left, near-left, near-right = counter-clockwise
    k = int(np.round(np.degrees(np.arctan2(Tbb[1][0], Tbb[0][0])) / 90.0)) % 4
    return {ccw[(ccw.index(m) + k) % 4]: m for m in ccw}


def pin_gap(Tbb, model_qd, pin):
    """Gap index (basket_gaps order) for pin number `pin` (1 = the pair farthest from the robot in the quadrant, counted
    toward the robot = wrist image top -> bottom) : the plate goes into the gap just robot-side of pair `pin`."""
    n = len(basket_gaps()[model_qd])
    if not 1 <= pin <= n:
        raise ValueError(f'--pin {pin}: this quadrant has gaps 1..{n}')
    T = np.asarray(Tbb)
    away = (T[:3, 0] @ np.r_[T[:2, 3], 0.0]) >= 0           # basket x points away from the robot
    return n - pin if away else pin - 1


def look_pose(Tbb, model_qd, Tbc, q, cam_h=(0.42, 0.36, 0.48, 0.30, 0.26), seed=None):
    """Joints for a wrist photo straight down on one quadrant: camera centred over its gaps `cam_h` above the basket
    frame, image top = the pin rows' end away from the robot (pin pairs stacked top -> bottom); turned 180 / +-90 deg if out of reach.
    (10/26) 0.30 / 0.26 m only when nothing higher reaches (far basket: Q1 at x 1.15 m).
    -> (joints deg, image turn deg, camera height) or None."""
    import ros2_nodes
    from calib_core import load_json, newest_wrist_result
    kin = Kinematics()
    lo, hi = np.array(ros2_nodes.joint_limits_deg()).T
    Tfc = np.array(load_json(newest_wrist_result())['T_flange_cam'])
    free = quadrant_checkers(Tbc, q, Tbb, np.eye(4), np.eye(4))[2]
    T = np.asarray(Tbb)
    target = (T @ np.r_[np.mean(basket_gaps()[model_qd], 0)[:2], 0.0, 1.0])[:3]
    away = np.sign(T[:3, 0] @ np.r_[T[:2, 3], 0.0]) or 1.0    # basket x (pin rows) pointing away from the robot
    for h in cam_h:
        for turn in (0, 180, 90, -90):
            t = np.radians(turn)
            xb = away * (T[:3, 0] * np.cos(t) + T[:3, 1] * np.sin(t))
            Tc = np.eye(4)
            Tc[:3, 1] = -xb                                   # image down = toward the robot side
            Tc[:3, 2] = [0, 0, -1.0]
            Tc[:3, 0] = np.cross(Tc[:3, 1], Tc[:3, 2])
            Tc[:3, 3] = target + [0, 0, h]
            for sd in ([seed] if seed is not None else []) + [np.radians(HOME_JOINTS_DEG)]:
                try:
                    j = kin.ik(Tc @ np.linalg.inv(Tfc), sd)
                except RuntimeError:
                    continue
                jd = np.degrees(j)
                if ((jd > lo + 3) & (jd < hi - 3)).all() and free.clearance(j) >= 1.0:
                    return jd, turn, h
    return None


def look_plans(a):
    """Wrist photos of the basket quadrants (read-only planning): basket from the markers (or --basket), one look pose per
    quadrant, legs `<Q>_look` (home -> look) and `<Q>_home` (look -> home) in one plan.json, plus a photo request per
    quadrant for `camera_tools.py pre_photo --req` (photo + model pins) and `pins --req` (pair numbers)."""
    Tbc = np.array(__import__('calib_core').load_json(__import__('calib_core').latest_t_base_cam())['T_base_cam'])
    q = C['plate_ik_bc']
    if a.basket_json is not None:                              # (10/26) markers gone: a basket_vision.py result
        r_ = json.loads(Path(a.basket_json).read_text())
        if not r_.get('ok', True):
            raise SystemExit(f'basket json not ok: {r_.get("why")}')
        Tbb = np.array(r_['T'], float)
        print(f'basket from {a.basket_json}', flush=True)
    elif a.measure:
        Tbb, res, seen = measured_basket()
        print(f'basket from markers {seen}: fit {res * 1000:.1f} mm', flush=True)
    else:
        c, s_ = np.cos(np.radians(a.basket[2])), np.sin(np.radians(a.basket[2]))
        Tbb = np.eye(4)
        Tbb[:2, :2] = [[c, -s_], [s_, c]]
        Tbb[:3, 3] = [a.basket[0], a.basket[1], a.basket_z]
    yaw = np.degrees(np.arctan2(Tbb[1, 0], Tbb[0, 0]))
    print(f'basket centre ({Tbb[0, 3]:.3f}, {Tbb[1, 3]:+.3f}, {Tbb[2, 3]:.3f}) yaw {yaw:+.1f} deg', flush=True)
    free = quadrant_checkers(Tbc, q, Tbb, np.eye(4), np.eye(4))[2]
    rq = robot_quadrants(Tbb)
    home = np.radians(HOME_JOINTS_DEG)
    legs, looks = {}, {}
    for qd in a.quadrants:
        lp = look_pose(Tbb, rq[qd], Tbc, q)
        if lp is None:
            print(f'{qd}: no look pose (reach / joint limit / clearance)', flush=True)
            continue
        jd, turn, h = lp
        try:
            to, how = free.plan(home, np.radians(jd))
            back, _ = free.plan(np.radians(jd), home)
        except RuntimeError as e:
            print(f'{qd}: no path ({e})', flush=True)
            continue
        legs[f'{qd}_look'] = [np.degrees(w).tolist() for w in to]
        legs[f'{qd}_home'] = [np.degrees(w).tolist() for w in back]
        looks[qd] = dict(joints_deg=np.round(jd, 3).tolist(), model_quadrant=rq[qd], image_turn_deg=turn, camera_above_m=h, path=how)
        print(f'{qd} (model {rq[qd]}): camera {h:.2f} m above, image turned {turn:+d} deg, path {how}', flush=True)
    if not legs:
        raise SystemExit('no look pose for any quadrant')
    path = taught_plan(legs, Tbc, q, 'wrist look poses')
    plan = json.loads(Path(path).read_text())
    plan.update(basket_T=Tbb.tolist(), look=looks)
    Path(path).write_text(json.dumps(plan, indent=1))
    kin, stamp = Kinematics(), time.strftime('%Y%m%d_%H%M%S')
    shots = DATA / 'basket' / 'wrist_look'
    shots.mkdir(parents=True, exist_ok=True)
    print(f'\nplan: {path}\n(보스) per quadrant, robot at home:')
    for qd, lk in looks.items():
        req = shots / f'{stamp}_{qd}.req.json'
        req.write_text(json.dumps(dict(T_base_flange=kin.fk(np.radians(lk['joints_deg'])).tolist(), T_base_basket=Tbb.tolist(),
                                       png=str(shots / f'{stamp}_{qd}.png'), after=None, quadrant=qd, plan=str(path), **lk), indent=1))
        print(f'  bash calibration/run.sh mover --plan {path} --leg {qd}_look --smooth --go_per leg --vel_deg_s 20 --execute\n'
              f'  python calibration/camera_tools.py pre_photo --req {req}\n'
              f'  bash calibration/run.sh mover --plan {path} --leg {qd}_home --smooth --go_per leg --vel_deg_s 20 --execute')
    socket = __import__('socket')
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(json.dumps(dict(plan=str(path))).encode(), ('127.0.0.1', C['udp'].get('viz', 5011)))


def _random_one(job):
    """One random (basket pose, plate position, quadrant, gap) -> result row.  Forked worker; globals in _RANDOM."""
    import contextlib
    import io
    k, bx, by, yaw, px, py, qd, gap = job
    G = _RANDOM
    globals()['DATA'] = G['out']                              # the batch's plan.json files stay in its own folder
    np.random.seed(k)
    c, s_ = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    Tbb = np.eye(4)
    Tbb[:2, :2] = [[c, -s_], [s_, c]]
    Tbb[:3, 3] = [bx, by, G['basket_z']]
    row = dict(k=k, basket=[bx, by, yaw], plate=[px, py], quadrant=qd, gap=gap + 1)
    t0 = time.time()
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            r, az = library_target(np.array([px, py, G['plate_z']]), G['q'])     # FP root + offset, azimuth in RADIANS
            gpath = plan_from_library(r, az, G['q'], G['Tbc'], plate_z=G['plate_z'])
    except Exception as e:                                   # noqa: BLE001
        return dict(row, ok=False, stage='grasp', why=str(e)[:120], s=round(time.time() - t0, 1))
    gl = json.loads(Path(gpath).read_text())['legs']
    grasp = {n: gl[n]['waypoints_deg'] for n in ('to_grasp', 'lift')}
    H, tool = hand_plate(grasp['to_grasp'][-1], [px, py, G['plate_z']], tilt_deg=G.get('hand_tilt'))   # this grasp's plate in the hand
    model_qd = robot_quadrants(Tbb)[qd]
    if G.get('pins'):                                        # (10/16 보스) fixed target pins per robot quadrant
        pins_q = G['pins'][qd]
        gap = pin_gap(Tbb, model_qd, pins_q[min(int(gap * len(pins_q)), len(pins_q) - 1)])
    else:
        gap = int(gap * len(basket_gaps()[model_qd]))            # 6 / 8 / 12 / 12 gaps (10/14 measured basket)
    row['gap'] = gap + 1
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            path, rep = quadrant_plan(Tbb, model_qd, gap, H, tool, grasp['lift'][-1], G['Tbc'], G['q'], grasp_legs=grasp,
                                      phi_step=G['phi_step'], **G.get('plan_kw', {}),
                                      **(dict(faces=(-1.0, 1.0)) if G.get('far_flip') and model_qd in ('Q1', 'Q2') else {}))
    except Exception as e:                                   # noqa: BLE001
        return dict(row, ok=False, stage='place', why=str(e)[:120], s=round(time.time() - t0, 1))
    if path is None:
        why = sorted({w if w == 'ik/limit' else w.split(' ')[0] for _, w in rep['tried']})
        return dict(row, ok=False, stage='place', why=','.join(why), s=round(time.time() - t0, 1))
    n_gap = len(basket_gaps()[model_qd])
    pin = next(p_ for p_ in range(1, n_gap + 1) if pin_gap(Tbb, model_qd, p_) == gap)
    c_ = np.array(basket_gaps()[model_qd][gap], float)
    c_[1] += rep['y_shift_m']
    c_[0] += rep['x_shift_m']
    replay = dict(name=f'r{k}_{model_qd}_pin{pin}', quadrant=qd, plate=[px, py, G['plate_z']], Tb=Tbb.tolist(), plan=str(path),
                  model_qd=model_qd, gap=gap, pin=pin, target_centre=((Tbb @ np.r_[c_, 1.0])[:3] + [0, 0, 0.165]).tolist())
    return dict(row, ok=True, stage='done', why='', pin=pin, phi=rep['phi_deg'], face=rep.get('face', 1), clearance=rep['clearance_ratio'],
                plan=str(path), replay=replay, s=round(time.time() - t0, 1))


def random_plans(a):
    """Offline batch: random basket pose (x, y, yaw) + random plate position + random quadrant/gap -> full plan
    (library grasp + lift, quadrant carry/insert/home).  Success rate and why it fails, by basket position and yaw."""
    import multiprocessing as mp
    rng = np.random.default_rng(a.seed)
    out = DATA / 'basket' / f'random_{time.strftime("%Y%m%d_%H%M%S")}'
    out.mkdir(parents=True)
    run = sorted(p_.parent for p_ in (DATA / 'real_pipeline').glob('*/realplan_req.json'))[-1]
    if a.sim:                                                # (10/14) plans for the Isaac replay: no real-robot x trim,
        _RANDOM.update(plan_kw=dict(x_shift_m=0.0, centre_h=0.165), hand_tilt=(-12.9, 4.9))   # plate centre 16.5 cm, sim in-hand sag
    _RANDOM.update(far_flip=a.far_flip, quadrants=a.only_quadrants,
                   pins={qd: [int(v) for v in ps.split(',')] for qd, ps in (t.split(':') for t in a.pins)} if a.pins else None)
    _RANDOM.update(out=out, plate_z=float(json.loads((run / 'realplan_req.json').read_text())['plate_root_fp'][2]), q={**C['plate_rl'], **C['plate_ik_bc']}, basket_z=a.basket_z, phi_step=a.phi_step,
                   Tbc=np.array(__import__('calib_core').load_json(__import__('calib_core').latest_t_base_cam())['T_base_cam']))
    jobs = []
    while len(jobs) < a.n:
        bx, by = rng.uniform(*a.x), rng.uniform(*a.y)
        yaw = rng.uniform(-a.yaw_range, a.yaw_range)
        if a.plate_x:                                        # (10/14 19시) box around the parallel layout's plate
            px, py = rng.uniform(*a.plate_x), rng.uniform(*a.plate_y)
        else:
            r, az = rng.uniform(*a.plate_r), rng.uniform(-40, 40)
            px, py = r * np.cos(np.radians(az)), r * np.sin(np.radians(az))
        if abs(py) > 0.42 or np.hypot(px - bx, py - by) < 0.37 + PLATE_R + 0.05:   # plate not on / against the basket (53 cm square)
            continue
        jobs.append((len(jobs), bx, by, yaw, px, py, rng.choice(a.only_quadrants or list(QUADRANTS)), float(rng.random())))   # gap = fraction of the quadrant's gaps
    print(f'{len(jobs)} random setups (basket x {a.x} y {a.y} yaw +-{a.yaw_range}, plate '
          f'{f"x {a.plate_x} y {a.plate_y}" if a.plate_x else f"r {a.plate_r} az +-40"} |y|<=0.42) '
          f'with {a.workers} workers -> {out}', flush=True)
    rows = []
    with mp.get_context('fork').Pool(a.workers) as pool, (out / 'rows.jsonl').open('w') as f:
        for row in pool.imap_unordered(_random_one, jobs):
            rows.append(row)
            f.write(json.dumps(row) + '\n')
            f.flush()
            if len(rows) % 20 == 0:
                print(f'  {len(rows)}/{len(jobs)}: success {np.mean([r_["ok"] for r_ in rows]):.2f}', flush=True)
    with (out / 'replay.jsonl').open('w') as f:             # rl/basket/expert.py replay --plans
        for r_ in sorted(rows, key=lambda r_: r_['k']):
            if r_['ok']:
                f.write(json.dumps(r_['replay']) + '\n')
    ok = np.array([r_['ok'] for r_ in rows])
    summ = dict(n=len(rows), success=float(ok.mean()), plate_z_from=run.name, args={k: v for k, v in vars(a).items() if k != 'mode'},
                by_quadrant={qd: float(np.mean([r_['ok'] for r_ in rows if r_['quadrant'] == qd] or [0])) for qd in QUADRANTS},
                fails={})
    for r_ in rows:
        if not r_['ok']:
            key = f'{r_["stage"]}: {r_["why"][:60]}'
            summ['fails'][key] = summ['fails'].get(key, 0) + 1
    (out / 'summary.json').write_text(json.dumps(summ, indent=1))
    _random_figure(rows, out / 'random.png')
    print(json.dumps(summ, indent=1), flush=True)


def _random_figure(rows, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(10, 4.5))
    for r_ in rows:
        ax[0].scatter(r_['basket'][1], r_['basket'][0], c='tab:green' if r_['ok'] else 'tab:red', s=14)
        ax[1].scatter(r_['basket'][2], r_['basket'][0], c='tab:green' if r_['ok'] else 'tab:red', s=14)
    ax[0].set_xlabel('basket centre y [m] (+ = left)')
    ax[0].set_ylabel('basket centre x [m]')
    ax[0].invert_xaxis()
    ax[1].set_xlabel('basket yaw [deg]')
    ax[1].set_ylabel('basket centre x [m]')
    for a_ in ax:
        a_.spines[['top', 'right']].set_visible(False)
    fig.tight_layout()
    fig.savefig(path, dpi=120)


_RANDOM = {}
_REACH = {}

def obstacle_demo(a):
    """(10/22 보스 장애물 데모, offline) a real continuous-carry run: the carry without obstacles, then a plate standing on
    the table under the middle of that carry (--frac along it) as a seen obstacle -> the carry again.  Prints the
    planning time, writes obstacle_demo.png (top + side view) next to the plans, sends the obstacle plan to RViz."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from calib_core import latest_t_base_cam
    d = DATA / 'real_pipeline' / a.run if a.run else max((x for x in (DATA / 'real_pipeline').iterdir()
                                                          if '"slot_plan"' in (x / 'robot_state.jsonl').read_text(errors='ignore')
                                                          if (x / 'robot_state.jsonl').exists()), key=lambda x: x.name)
    ev = [json.loads(x) for x in (d / 'robot_state.jsonl').read_text().splitlines() if '"event"' in x]
    g = next(e for e in ev if e['event'] == 'plan')
    P = json.loads(Path(next(e['plan'] for e in ev if e['event'] == 'slot_plan')).read_text())
    G = json.loads(Path(g['plan']).read_text())['legs']
    H, Tbb, qp = np.asarray(P['plate_in_tcp']), np.array(P['basket_T']), P['quadrant_plan']
    Tbb[:2, 3] += a.basket_dxy                               # the basket moved (no markers: anywhere in reach)
    tool = np.eye(4)
    tool[2, 3] = C['plate_ik_bc'].get('tool_length_m', 0.165)
    Tbc = np.asarray(json.loads(Path(latest_t_base_cam()).read_text())['T_base_cam'])
    kin, cl = Kinematics(), plate_cloud(H, tool)

    def plan(obs):
        q = dict(C['plate_ik_bc'], _obstacles=obs, basket_carry_mode='rule', basket_carry_rrt_fallback=True)
        if a.rise is not None:
            q['basket_carry_rise_m'] = a.rise
        t0 = time.time()
        path, rep = quadrant_plan(Tbb, qp['quadrant'], qp['gap'] - 1, H, tool, G['lift']['waypoints_deg'][-1], Tbc, q,
                                  grasp_deg=G['to_grasp']['waypoints_deg'][-1], faces=(float(qp['face']),),
                                  phi_first=qp['phi_deg'], grasp_low_z=g['plate_root_fp'][2])
        dt = time.time() - t0
        if path is None:
            print(f'  no plan ({dt:.1f} s): {rep["tried"][:3]}')
            return None, None, dt
        L = np.radians(json.loads(Path(path).read_text())['legs']['to_above']['waypoints_deg'])
        pts = [cl @ kin.fk(w)[:3, :3].T + kin.fk(w)[:3, 3] for w in L]
        print(f'  {path}: {dt:.1f} s, carry {rep["carry"]}, clearance {rep["clearance_ratio"]}, '
              f'plate lowest point max {max(p_[:, 2].min() for p_ in pts):.3f} m')
        return path, pts, dt
    print('without obstacles:')
    p0, pts0, t_0 = plan([])
    m = pts0[int(len(pts0) * a.frac)].mean(0)
    v = pts0[-1].mean(0) - pts0[0].mean(0)
    v[2] = 0
    v /= np.linalg.norm(v)
    T = np.eye(4)                                            # standing on the table (top -0.02), face across the carry
    T[:3, :3] = np.stack([[0, 0, 1.0], np.cross(np.r_[v[:2], 0], [0, 0, 1.0]), np.r_[v[:2], 0]], 1)
    T[:3, 3] = [m[0], m[1], -0.02 + PLATE_R]
    obs = [plate_obstacle(T)]
    print(f'with a standing plate at ({m[0]:.3f}, {m[1]:+.3f}) m:')
    p1, pts1, t_1 = plan(obs)
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
    for pts, lab, col in ((pts0, 'no obstacle', '0.6'), (pts1, 'plate seen', 'tab:blue')):
        if pts is None:
            continue
        c = np.array([p_.mean(0) for p_ in pts])
        low = np.array([p_[:, 2].min() for p_ in pts])
        ax[0].plot(c[:, 1], c[:, 0], color=col, label=lab)
        ax[1].plot(np.r_[0, np.cumsum(np.linalg.norm(np.diff(c[:, :2], axis=0), axis=1))], low, color=col, label=lab)
    corners = (np.array([[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]]) * [0.015, PLATE_R]) @ np.stack([v[:2], [-v[1], v[0]]])
    ax[0].fill(m[1] + corners[:, 1], m[0] + corners[:, 0], color='tab:red', alpha=0.5, label='obstacle')
    bb = np.array([[-0.265, -0.26], [0.265, -0.26], [0.265, 0.26], [-0.265, 0.26], [-0.265, -0.26]]) @ Tbb[:2, :2].T + Tbb[:2, 3]
    ax[0].plot(bb[:, 1], bb[:, 0], color='k', lw=1, label='basket')
    ax[0].set_xlabel('y (m)')
    ax[0].set_ylabel('x (m)')
    ax[0].invert_xaxis()
    ax[0].set_aspect('equal')
    ax[1].axhline(-0.02 + 2 * PLATE_R, color='tab:red', ls='--', lw=1, label='obstacle top')
    ax[1].set_xlabel('plate centre path length (m)')
    ax[1].set_ylabel('plate lowest point z (m)')
    for x_ in ax:
        x_.spines[['top', 'right']].set_visible(False)
        x_.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    out = Path(p1 or p0).parent / 'obstacle_demo.png'
    fig.savefig(out, dpi=120)
    print(f'figure: {out}  (planning {t_0:.1f} s without, {t_1:.1f} s with the obstacle)')
    if p1 is not None:                                       # RViz preview (run.sh viz): grasp + carry + the obstacle box
        both = dict(json.loads(Path(p1).read_text()), preview_only=True, obstacles=obs)
        both['legs'] = dict({k: G[k] for k in ('to_grasp', 'lift')}, **both['legs'])
        (Path(p1).parent / 'preview_plan.json').write_text(json.dumps(both))
        import socket
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(json.dumps(dict(plan=str(Path(p1).parent / 'preview_plan.json'))).encode(),
                                                               ('127.0.0.1', C['udp'].get('viz', 5011)))
        print(f'RViz: {Path(p1).parent / "preview_plan.json"} (bash calibration/run.sh viz)')



def two_plate_demo(a):
    """(10/23 보스 장애물 데모, offline, RViz) plate 1 into an empty quadrant, then plate 2 with plate 1 standing there:
    the plan that ignores plate 1 (grey) vs the plan that goes over it (green), plate 1 shown as a red box.  The in-hand
    plate, grasp and basket come from a real run (--run); pins --pins (1st, 2nd; pin 1 = farthest from the robot)."""
    import socket
    from calib_core import latest_t_base_cam
    d = DATA / 'real_pipeline' / a.run if a.run else max((x for x in (DATA / 'real_pipeline').iterdir()
                                                          if '"slot_plan"' in (x / 'robot_state.jsonl').read_text(errors='ignore')
                                                          if (x / 'robot_state.jsonl').exists()), key=lambda x: x.name)
    ev = [json.loads(x) for x in (d / 'robot_state.jsonl').read_text().splitlines() if '"event"' in x]
    g = next(e for e in ev if e['event'] == 'plan')
    P = json.loads(Path(next(e['plan'] for e in ev if e['event'] == 'slot_plan')).read_text())
    G = json.loads(Path(g['plan']).read_text())['legs']
    H, Tbb, qp = np.asarray(P['plate_in_tcp']), np.array(P['basket_T']), P['quadrant_plan']
    Tbb[:2, 3] += a.basket_dxy
    mq = qp['quadrant']
    tool = np.eye(4)
    tool[2, 3] = C['plate_ik_bc'].get('tool_length_m', 0.165)
    Tbc = np.asarray(json.loads(Path(latest_t_base_cam()).read_text())['T_base_cam'])
    kin = Kinematics()
    q = dict(C['plate_ik_bc'], basket_carry_mode='rule')
    if a.rise is not None:
        q['basket_carry_rise_m'] = a.rise
    viz = ('127.0.0.1', C['udp'].get('viz', 5011))

    def plan(pin, plates, faces=(float(qp['face']), -float(qp['face']))):
        t0 = time.time()
        path, rep = quadrant_plan(Tbb, mq, pin_gap(Tbb, mq, pin), H, tool, G['lift']['waypoints_deg'][-1], Tbc, q,
                                  grasp_deg=G['to_grasp']['waypoints_deg'][-1], faces=faces,
                                  phi_first=qp['phi_deg'], grasp_low_z=g['plate_root_fp'][2], plates=plates)
        return path, rep, time.time() - t0

    def show(path, obstacles=(), compare=None):
        both = dict(json.loads(Path(path).read_text()), preview_only=True, obstacles=list(obstacles),
                    compare_legs=json.loads(Path(compare).read_text())['legs'] if compare else {})
        both['legs'] = dict({k: G[k] for k in ('to_grasp', 'lift')}, **both['legs'])
        out = Path(path).parent / 'preview_plan.json'
        out.write_text(json.dumps(both))
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(json.dumps(dict(plan=str(out))).encode(), viz)
        return out

    pin1, pin2 = a.demo_pins
    print(f'[1/2] 첫 번째 접시 -> {mq} {pin1}번 칸: 바구니가 비어 있음, 장애물 없음', flush=True)
    p1, rep1, dt1 = plan(pin1, ())
    if p1 is None:
        raise SystemExit(f'plate 1: no plan ({rep1["tried"][:3]})')
    print(f'   경로 OK [{dt1:.1f} s], 운반 {rep1["carry"]}  -> RViz: {show(p1)}', flush=True)
    face = (float(rep1['face']),)                            # (10/23 보스) plate 2 goes in the same way as plate 1
    T1 = kin.fk(np.radians(json.loads(Path(p1).read_text())['legs']['descend']['waypoints_deg'][-1])) @ tool @ H
    box = plate_obstacle(T1)
    box = (f'plate 1 ({mq}:{pin1})',) + tuple(box[1:])
    if not a.no_wait:
        input('   RViz에서 첫 번째 접시 경로를 확인하고 Enter: 두 번째 접시 ')
    print(f'[2/2] 두 번째 접시 -> {mq} {pin2}번 칸', flush=True)
    print(f'[장애물] 관측: 바구니 {mq} {pin1}번 칸에 꽂힌 접시 1개, 꼭대기 약 {(box[1][2] + max(box[2]) / 2) * 100:.0f} cm', flush=True)
    p0, rep0, dt0 = plan(pin2, (), face)
    disc = plate_cloud(np.eye(4), np.eye(4)) @ T1[:3, :3].T + T1[:3, 3]   # plate 1 as the disc (the box corners stick out ~5 cm)

    def gap(path, part=None, legs=('descend',)):
        P_ = json.loads(Path(path).read_text())
        W = P_['legs']['to_above']['waypoints_deg'] if part else P_['legs'][legs[0]]['waypoints_deg']
        k = P_['quadrant_plan'].get('drop_start')
        W = W if part is None or k is None else (W[:k + 1] if part == 'carry' else W[k:])
        cl = plate_cloud(H, tool)
        from scipy.spatial import cKDTree
        tree = cKDTree(disc)
        return min(tree.query(cl @ T[:3, :3].T + T[:3, 3])[0].min() for T in (kin.fk(np.radians(w)) for w in W))
    c0 = gap(p0, 'carry') if p0 else None
    if c0 is not None:
        print(f'[장애물] 이 접시를 무시한 운반 [{dt0:.1f} s]: 들고 가는 접시가 꽂힌 접시와 '
              + ('부딪힘' if c0 < 0.01 else f'{c0 * 100:.1f} cm') + (' -> 피하는 경로를 만듭니다' if c0 < 0.03 else ''), flush=True)
    p2, rep2, dt2 = plan(pin2, [(mq, pin_gap(Tbb, mq, pin1))], face)
    if p2 is None:
        raise SystemExit(f'plate 2: no plan the same way as plate 1 with plate 1 standing ({rep2["tried"][:3]}); '
                         f'{mq}: the gripper side of the plate decides the order (Q3 large pins first, Q4 small first)')
    c2, c2d = gap(p2, 'carry'), min(gap(p2, 'drop'), gap(p2))
    print(f'[장애물] 실제 경로 [{dt2:.1f} s]: 운반 높이 {rep2["carry"]} m, 꽂힌 접시와 운반 중 최소 {c2 * 100:.1f} cm, '
          f'옆 칸으로 내릴 때 {c2d * 100:.1f} cm (접시끼리 포개지는 거리, 그리퍼·팔은 꽂힌 접시 검사 유지) '
          f'(RViz: 초록 = 실제, 회색 = 무시한 경로, 빨간 상자 = 첫 번째 접시)  -> {show(p2, [box], p0)}', flush=True)

if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=('basket_reach', 'quadrant_plans', 'look_plans', 'random_plans', 'obstacle_demo', 'two_plate_demo'))
    p.add_argument('--basket', type=float, nargs=3, default=(0.75, 0.15, 0.0), metavar=('X', 'Y', 'YAW'), help='quadrant_plans: basket pose')
    p.add_argument('--plate', type=float, nargs=2, default=None, help='quadrant_plans: plate x y (turns the newest grasp about joint 1)')
    p.add_argument('--quadrants', nargs='+', default=['Q3', 'Q4', 'Q1', 'Q2'])
    p.add_argument('--show', default='Q3', help='quadrant_plans: quadrant sent to RViz')
    p.add_argument('--x', type=float, nargs=2, default=(0.50, 1.25), help='basket centre x range [m]')
    p.add_argument('--y', type=float, nargs=2, default=(-0.35, 0.35), help='basket centre y range [m]')
    p.add_argument('--step', type=float, default=0.05)
    p.add_argument('--yaw', type=float, default=0.0, help='basket yaw [deg]')
    p.add_argument('--gap', type=int, default=4, help='gap index in the quadrant (0 = robot side)')
    p.add_argument('--tilt', type=float, default=0.0, help='plate tilt, top toward the far side [deg] (policy: 0)')
    p.add_argument('--centre_h', type=float, default=0.18, help='plate centre above the basket floor at the insert [m]')
    p.add_argument('--above', type=float, default=0.11, help='start pose above the insert [m]')
    p.add_argument('--basket_z', type=float, default=0.1442, help='basket frame (marker plane) height in base [m]')
    p.add_argument('--phi_step', type=int, default=15)
    p.add_argument('--random_seeds', type=int, default=4)
    p.add_argument('--workers', type=int, default=16)
    p.add_argument('--n', type=int, default=300, help='random_plans: setups')
    p.add_argument('--plate_r', type=float, nargs=2, default=(0.86, 1.15), help='random_plans: plate radius range [m] (library path clears the frame up to ~1.15)')
    p.add_argument('--plate_x', type=float, nargs=2, default=None, help='random_plans: plate x range [m] (with --plate_y; replaces --plate_r)')
    p.add_argument('--plate_y', type=float, nargs=2, default=None, help='random_plans: plate y range [m]')
    p.add_argument('--sim_q', action='store_true', help='quadrant_plans: apply config basket_quadrant_sim (Isaac per-quadrant lean/aim/wrist flip) - preview only')
    p.add_argument('--far_flip', action='store_true', help='random_plans: Q1/Q2 try the plate facing -x first (wrist flipped, like 보스 taught Q2)')
    p.add_argument('--only_quadrants', nargs='+', default=None, help='random_plans: sample only these (robot) quadrants')
    p.add_argument('--pins', nargs='+', default=None, help='random_plans: (10/16) fixed pins per robot quadrant, e.g. Q3:4,9 Q4:4,9 '
                   '(pin 1 = farthest from the robot); use with --only_quadrants')
    p.add_argument('--sim', action='store_true', help='random_plans: plans for the Isaac replay (no x trim, centre 0.165, sim sag)')
    p.add_argument('--yaw_range', type=float, default=15.0, help='random_plans: basket yaw uniform in +-this [deg]')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--measure', action='store_true', help='look_plans: basket from the fixed camera markers (read-only)')
    p.add_argument('--basket_json', default=None, help='look_plans: basket pose from a basket_vision.py json (no markers)')
    p.add_argument('--run', default=None, help='real run folder for the in-hand plate (default: newest with realplan_req)')
    p.add_argument('--markers_off', action='store_true', help='(10/22) the marker blocks are off the rim: not in the collision model')
    p.add_argument('--frac', type=float, default=0.65, help='obstacle_demo: obstacle under this fraction of the carry')
    p.add_argument('--rise', type=float, default=0.10, help='obstacle_demo: carry rise over the grasp (m); markers are off')
    p.add_argument('--basket_dxy', type=float, nargs=2, default=(0.0, 0.0), help='obstacle_demo: move the run\'s basket (m)')
    p.add_argument('--demo_pins', type=int, nargs=2, default=(2, 6), help='two_plate_demo: 1st and 2nd plate pins')
    p.add_argument('--no_wait', action='store_true', help='two_plate_demo: no Enter between the plates')
    a = p.parse_args()
    C['basket']['markers_off'] = a.markers_off
    dict(basket_reach=basket_reach, quadrant_plans=quadrant_plans, look_plans=look_plans, random_plans=random_plans,
         obstacle_demo=obstacle_demo, two_plate_demo=two_plate_demo)[a.mode](a)
