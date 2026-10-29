#!/usr/bin/env python3
"""Experimental plate grasp only: roll robot J6 by 180 deg, support with fingers 2-5, gently close the thumb.

This is deliberately separate from dg5fs_sdk.py and demo_safe.sh.  With no --execute it only creates and checks an
arm plan.  With --execute it stops after grasping and keeps the hand controller alive until the operator types
``release``; it never lifts, carries, inserts, or returns home.

    # plan/check only
    ~/miniconda3/envs/foundationpose/bin/python tesollo/support_grasp180.py
    # real experiment (operator confirmations before every motion)
    ~/miniconda3/envs/foundationpose/bin/python tesollo/support_grasp180.py --execute
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAL = ROOT / 'calibration'
sys.path[:0] = [str(CAL), str(HERE)]
os.chdir(CAL)                         # calibration modules load config relative to this directory
import calib_core as cc
import plate_plans as pp
import ros2_nodes

JOINTS = 20


def joint_line(a, b, step_deg=30.0):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(np.ceil(np.abs(b - a).max() / step_deg)))
    return [a + (b - a) * s / n for s in range(n + 1)]


def vertical_line(k, q0_deg, dz, n=5):
    """Cartesian vertical line, with continuous IK seeds."""
    T0, q = k.fk(np.radians(q0_deg)), np.radians(q0_deg)
    out = [np.asarray(q0_deg, float)]
    for s in np.linspace(0, 1, n)[1:]:
        T = T0.copy()
        T[2, 3] += s * dz
        q = k.ik(T, q)
        if np.abs(k.fk(q)[:3, 3] - T[:3, 3]).max() > 1e-3:
            raise RuntimeError('vertical approach IK position error > 1 mm')
        out.append(np.degrees(q))
    if np.abs(np.diff(out, axis=0)).max() > 15:
        raise RuntimeError('vertical approach has an IK jump > 15 deg')
    return out


def lowest_hand_z(k, path):
    """Same conservative palm envelope used by arm_plan.py, sampled along the controller spline."""
    dense = pp.equal_time_spline(np.asarray(path), per=30)[0]
    z = 9.0
    for q in dense:
        T = k.fk(np.radians(q))
        for d in (0.05, 0.13, 0.22):
            c = T[:3, 3] + d * T[:3, 2]
            for t in np.linspace(0, 2 * np.pi, 24, endpoint=False):
                p = c + 0.12 * (np.cos(t) * T[:3, 0] + np.sin(t) * T[:3, 1])
                z = min(z, float(p[2]))
    return z


def make_plan(a):
    A = yaml.safe_load((HERE / 'arm_poses.yaml').read_text())['plate']
    k = pp.Kinematics()
    home = np.asarray(pp.C['robot']['home_joints_deg'], float)
    original = np.asarray(A['posj_deg'], float)
    grasp = original.copy()
    grasp[5] += a.j6_turn

    lo, hi = np.asarray(ros2_nodes.joint_limits_deg(), float).T
    if not (lo[5] + 3 < grasp[5] < hi[5] - 3):
        raise RuntimeError(f'J6 {grasp[5]:.1f} deg is outside the controller range {lo[5]:.1f}..{hi[5]:.1f}')
    if a.dx or a.dy:
        T = k.fk(np.radians(grasp))
        T[0, 3] += a.dx
        T[1, 3] += a.dy
        q = k.ik(T, np.radians(grasp))
        grasp = np.degrees(q)
        if np.abs(grasp - np.r_[original[:5], original[5] + a.j6_turn]).max() > 45:
            raise RuntimeError('shifted grasp selected a distant IK branch')

    up = vertical_line(k, grasp, a.rise, n=5)
    above = up[-1]
    approach = joint_line(home, above, 30.0) + list(reversed(up))[1:]
    W = np.asarray(approach)
    if not ((W > lo + 3) & (W < hi - 3)).all():
        bad = np.argwhere(~((W > lo + 3) & (W < hi - 3)))[0]
        raise RuntimeError(f'joint limit margin failed at waypoint {bad[0] + 1}, J{bad[1] + 1}: {W[tuple(bad)]:.1f} deg')
    if np.abs(np.diff(W, axis=0)).max() > 30.01:
        raise RuntimeError('approach knot change exceeds 30 deg')

    T0, T1 = k.fk(np.radians(original)), k.fk(np.radians(grasp))
    pos_delta = np.linalg.norm(T1[:3, 3] - T0[:3, 3]) * 1000
    rot_delta = np.degrees(np.linalg.norm(Rotation.from_matrix(T0[:3, :3].T @ T1[:3, :3]).as_rotvec()))
    low = lowest_hand_z(k, W)
    if low < -0.005:
        raise RuntimeError(f'conservative hand envelope goes below the table reference: z {low:.3f} m')

    Tbc = np.asarray(cc.load_json(cc.latest_t_base_cam())['T_base_cam'])
    path = pp.taught_plan({'support180_approach': W.tolist()}, Tbc, pp.C['plate_ik_bc'],
                          f'experimental plate support grasp: J6 {a.j6_turn:+.1f} deg; grasp only')
    print(f'J6: {original[5]:+.2f} -> {grasp[5]:+.2f} deg ({a.j6_turn:+.1f})')
    print(f'grasp TCP change from original: position {pos_delta:.3f} mm, orientation {rot_delta:.2f} deg')
    print(f'approach: {len(W)} knots, max change {np.abs(np.diff(W, axis=0)).max():.2f} deg, '
          f'conservative hand minimum z {low:.3f} m')
    print('grasp joints:', np.round(grasp, 2).tolist())
    print('plan:', path)
    return Path(path)


def execute(a, plan):
    """Run the SDK half in its Python, because the planner and DGSDK have different binary dependencies."""
    worker = HERE / 'support_grasp180_hand.py'
    probe = subprocess.run([a.hand_python, '-c', 'import cffi, yaml'], capture_output=True, text=True)
    if probe.returncode:
        raise RuntimeError(f'hand Python cannot import cffi/yaml: {a.hand_python}\n{probe.stderr.strip()}')
    cmd = [a.hand_python, str(worker), '--plan', str(plan), '--vel', str(a.vel), '--force', str(a.force),
           '--support_fraction', str(a.support_fraction), '--motion_ms', str(a.motion_ms),
           '--sdk', a.sdk, '--ip', a.ip, '--port', str(a.port)]
    print('hand controller Python:', a.hand_python)
    if subprocess.run(cmd).returncode:
        raise RuntimeError('hand/arm experiment stopped')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--execute', action='store_true', help='connect to the hand and run the arm; default is plan/check only')
    p.add_argument('--j6_turn', type=float, default=180.0, help='signed robot J6 change from the taught plate grasp [deg]')
    p.add_argument('--rise', type=float, default=0.12, help='height of the approach pose above the grasp [m]')
    p.add_argument('--dx', type=float, default=0.0, help='base-frame shift from the taught object x [m]')
    p.add_argument('--dy', type=float, default=0.0, help='base-frame shift from the taught object y [m]')
    p.add_argument('--vel', type=float, default=10.0, help='arm speed [deg/s], experimental default 10')
    p.add_argument('--force', type=float, default=1.0, help='thumb grasp force [N], 0.5..3.0')
    p.add_argument('--support_fraction', type=float, default=0.70,
                   help='fingers 2-5: fraction from plate_pre9 toward the last measured contact')
    p.add_argument('--motion_ms', type=int, default=4000, help='hand support-pose motion time [ms]')
    p.add_argument('--sdk', default='/home/pan/pan/tesollo_ws/dg_python-main/src')
    p.add_argument('--hand_python', default='/home/pan/miniconda3/bin/python3',
                   help='Python containing cffi for DGSDK; launched after planning')
    p.add_argument('--ip', default='169.254.186.73')
    p.add_argument('--port', type=int, default=502)
    a = p.parse_args()
    if np.hypot(a.dx, a.dy) > 0.15:
        p.error('sqrt(dx^2+dy^2) must be <= 0.15 m')
    if not 0.5 <= a.force <= 3.0:
        p.error('--force must be 0.5..3.0 N for this thumb-only experiment')
    if not 0 <= a.support_fraction <= 1:
        p.error('--support_fraction must be 0..1')
    if a.motion_ms < 3000:
        p.error('--motion_ms must be >= 3000 ms')
    if not 1 <= a.vel <= 20:
        p.error('--vel must be 1..20 deg/s')
    plan = make_plan(a)
    print('hand plan: fingers 2-5 position-held support; thumb only, force', a.force, 'N')
    print('demo files are not used or modified; this run ends at the grasp pose.')
    if a.execute:
        execute(a, plan)


if __name__ == '__main__':
    main()
