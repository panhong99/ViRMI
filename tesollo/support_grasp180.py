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
import json
import os
import subprocess
import sys
import time
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


def load_hand_pose(a):
    poses = yaml.safe_load((HERE / 'poses.yaml').read_text())
    pre = np.degrees(np.asarray(poses['plate_pre9']['joints_rad'], float))
    support = pre.copy()
    contact_path = HERE / 'grasp_contact.json'
    if contact_path.exists() and 'plate' in (contact := json.loads(contact_path.read_text())):
        measured = np.asarray(contact['plate']['joints'], float)
        support[4:] = pre[4:] + a.support_fraction * (measured[4:] - pre[4:])
        source = f'plate_pre9 + {a.support_fraction:.0%} toward contact {contact["plate"]["time"]}'
    else:
        source = 'plate_pre9 (no measured contact file)'
    # Thumb stays in the open/grasp-ready position. Fingers 2-5 form the fixed support shelf.
    support[:4] = pre[:4]
    return support.tolist(), source


def execute(a, plan):
    sys.path.insert(0, a.sdk)
    from dgsdk import (CommunicationMode, ControlMode, DGGraspMode, DGModel, DGResult, DGSDK, GripperSetting,
                       GripperSystemSetting, ReceivedDataType)

    support, source = load_hand_pose(a)
    g = DGSDK()
    started = False

    def check(name, result):
        print(f'  {name}: {result.name}', flush=True)
        if result != DGResult.NONE:
            raise RuntimeError(f'{name} failed: {result.name}')

    def confirm(text):
        if input(f'\n{text}  Enter = go, q = stop: ').strip().lower() == 'q':
            raise KeyboardInterrupt

    def show():
        d = g.get_gripper_data()
        q = list(d.joint[:JOINTS])
        for f in range(5):
            print(f'  finger {f + 1}' + (' thumb' if f == 0 else '      ') + ': ' +
                  ' '.join(f'{v:+6.1f}' for v in q[4 * f:4 * f + 4]))

    check('system', g.set_gripper_system(GripperSystemSetting.create(
        ip=a.ip, port=a.port, control_mode=ControlMode.DEVELOPER, communication_mode=CommunicationMode.ETHERNET)))
    g.on_connected(lambda: print('  -> connected', flush=True))
    g.on_disconnected(lambda: print('  -> disconnected', flush=True))
    check('connect', g.connect_to_gripper())
    try:
        check('option', g.set_gripper_option(GripperSetting.create(
            model=DGModel.DG_5F_S_LEFT, joint_count=0, finger_count=0,
            received_data_type=[ReceivedDataType.JOINT, ReceivedDataType.CURRENT, ReceivedDataType.TEMPERATURE, 0, 0, 0, 0])))
        for _ in range(20):
            time.sleep(0.1)
            result = g.system_start()
            if result != DGResult.NOT_FOUND_MODEL:
                break
        check('start', result)
        started = True
        check('hold target', g.move_joint_all(list(g.get_gripper_data().joint[:JOINTS])))
        check('PD mode', g.set_control_pid_mode(0))
        check('gains', g.set_joint_gain_pid_all([3.0] * JOINTS, [5.0] * JOINTS, [0.0] * JOINTS, [0.0] * JOINTS))

        confirm(f'hand: fingers 2-5 to support pose ({source}) in {a.motion_ms} ms; thumb stays ready')
        check('motion time', g.set_motion_time_all_equal(a.motion_ms))
        check('support pose', g.move_joint_all(support))
        time.sleep(a.motion_ms / 1000 + 0.5)
        show()

        confirm('arm: move through the checked J6-turned approach to the plate')
        cmd = ['bash', str(CAL / 'run.sh'), 'mover', '--plan', str(plan), '--leg', 'support180_approach', '--smooth',
               '--go_per', 'leg', '--vel_deg_s', str(a.vel), '--acc_deg_s2', str(a.vel * 0.5), '--execute',
               '--go_enter', '--spline_time']
        if subprocess.run(cmd).returncode:
            raise RuntimeError('arm mover stopped')

        confirm(f'thumb: close gently at {a.force:.1f} N; fingers 2-5 remain position-held')
        hold = [0] * 4 + [1] * 16
        check('position mode', g.set_position_mode_all(hold))
        check('thumb grasp data', g.set_grasp_data(DGGraspMode._5F_2FINGER_1_AND_2, a.force, 0, 1))
        check('thumb grasp', g.grasp(1))
        time.sleep(max(2.0, a.motion_ms / 1000))
        show()
        print('\nGRASPED AND STOPPED. The arm will not lift or move. Enter shows joints; type "release" to let go.', flush=True)
        while True:
            try:
                cmd = input('  support180> ').strip().lower()
            except KeyboardInterrupt:
                print('\n  still holding; type "release" to let go', flush=True)
                continue
            if cmd == 'release':
                break
            if cmd:
                print('  only "release" is accepted; the arm remains stopped')
            show()
    finally:
        if started:
            check('release', g.grasp(0))
            time.sleep(1.0)
            if input('open the hand fully? Enter = yes, n = leave support pose: ').strip().lower() != 'n':
                g.move_joint_all([0.0] * JOINTS)
                time.sleep(a.motion_ms / 1000 + 0.5)
            g.system_stop()
        g.disconnect_to_gripper()


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
