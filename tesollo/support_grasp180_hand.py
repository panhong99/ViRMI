#!/usr/bin/env python3
"""DGSDK worker for support_grasp180.py; run with the Python that contains cffi."""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
CAL = HERE.parent / 'calibration'
JOINTS = 20


def load_hand_pose(fraction):
    poses = yaml.safe_load((HERE / 'poses.yaml').read_text())
    pre = [math.degrees(float(v)) for v in poses['plate_pre9']['joints_rad']]
    support = pre.copy()
    contact_path = HERE / 'grasp_contact.json'
    if contact_path.exists() and 'plate' in (contact := json.loads(contact_path.read_text())):
        measured = [float(v) for v in contact['plate']['joints']]
        support[4:] = [p + fraction * (m - p) for p, m in zip(pre[4:], measured[4:])]
        source = f'plate_pre9 + {fraction:.0%} toward contact {contact["plate"]["time"]}'
    else:
        source = 'plate_pre9 (no measured contact file)'
    support[:4] = pre[:4]
    return support, source


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan', required=True)
    p.add_argument('--vel', type=float, required=True)
    p.add_argument('--force', type=float, required=True)
    p.add_argument('--support_fraction', type=float, required=True)
    p.add_argument('--motion_ms', type=int, required=True)
    p.add_argument('--sdk', required=True)
    p.add_argument('--ip', required=True)
    p.add_argument('--port', type=int, required=True)
    a = p.parse_args()

    sys.path.insert(0, a.sdk)
    from dgsdk import (CommunicationMode, ControlMode, DGGraspMode, DGModel, DGResult, DGSDK, GripperSetting,
                       GripperSystemSetting, ReceivedDataType)

    support, source = load_hand_pose(a.support_fraction)
    g = DGSDK()
    started = False

    def check(name, result):
        print(f'  {name}: {result.name}', flush=True)
        if result != DGResult.NONE:
            raise RuntimeError(f'{name} failed: {result.name}')

    def confirm(message):
        if input(f'\n{message}  Enter = go, q = stop: ').strip().lower() == 'q':
            raise KeyboardInterrupt

    def show():
        q = list(g.get_gripper_data().joint[:JOINTS])
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
            received_data_type=[ReceivedDataType.JOINT, ReceivedDataType.CURRENT,
                                ReceivedDataType.TEMPERATURE, 0, 0, 0, 0])))
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
        mover = ['bash', str(CAL / 'run.sh'), 'mover', '--plan', a.plan, '--leg', 'support180_approach', '--smooth',
                 '--go_per', 'leg', '--vel_deg_s', str(a.vel), '--acc_deg_s2', str(a.vel * 0.5), '--execute',
                 '--go_enter', '--spline_time']
        if subprocess.run(mover).returncode:
            raise RuntimeError('arm mover stopped')

        confirm(f'thumb: close gently at {a.force:.1f} N; fingers 2-5 remain position-held')
        check('position mode', g.set_position_mode_all([0] * 4 + [1] * 16))
        check('thumb grasp data', g.set_grasp_data(DGGraspMode._5F_2FINGER_1_AND_2, a.force, 0, 1))
        check('thumb grasp', g.grasp(1))
        time.sleep(max(2.0, a.motion_ms / 1000))
        show()
        print('\nGRASPED AND STOPPED. The arm will not lift or move. Enter shows joints; type "release" to let go.',
              flush=True)
        while True:
            try:
                command = input('  support180> ').strip().lower()
            except KeyboardInterrupt:
                print('\n  still holding; type "release" to let go', flush=True)
                continue
            if command == 'release':
                break
            if command:
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


if __name__ == '__main__':
    main()
