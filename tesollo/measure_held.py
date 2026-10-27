"""Tesollo hand: where the plate really is in the hand after the grasp + lift (read-only, no robot command).

The basket plan assumes the plate stays flat where it lay (plate_plans.hand_plate rule); 보스 sees it held almost
slot-ready, and the Q3 insert needed +5 cm.  This measures it like the 2F-85 pipeline's post-lift replan: the fixed D455
+ YOLOE + FoundationPose (plate mesh) give the plate pose in base, the robot joints give the hand; their ratio is the
plate in the hand (H), saved to tesollo/arm_poses.yaml plate.in_hand and used by basket_plan.py from then on.
    ~/miniconda3/envs/foundationpose/bin/python tesollo/measure_held.py            # the plate held, arm still
The camera must be free (no pipeline / perception running); the Doosan bringup must be up.  Prints the hang angle and
the offset against the old flat-plate model.
"""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / 'calibration'))
sys.path.insert(0, str(HERE))
os.chdir(ROOT / 'calibration')
import numpy as np
import yaml

DOOSAN = ROOT.parent / 'doosan_ws/install/setup.bash'


def robot_joints():
    """Joints [deg] through the same clean ROS environment as run.sh's ros()."""
    cmd = ['env', '-u', 'PYTHONPATH', '-u', 'PYTHONHOME', '-u', 'LD_LIBRARY_PATH', '-u', 'CONDA_PREFIX', '-u',
           'CONDA_DEFAULT_ENV', 'PATH=/usr/bin:/bin', 'bash', '-c',
           f'source /opt/ros/jazzy/setup.bash; source {DOOSAN}; exec /usr/bin/python3 {HERE / "read_posj.py"}']
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=40)
    if out.returncode:
        sys.exit(f'robot joints: {out.stderr.strip()[-300:]}')
    return np.array(json.loads(out.stdout.strip().splitlines()[-1]))


def held_H(k, q_deg, root, n, tool):
    """Plate (root = mesh bottom centre, normal n, base) held by the hand at these joints -> H in the tool frame, by the
    plate_plans.hand_plate rule (centre 1 cm above the root, x toward the hand)."""
    n = np.asarray(n, float) / np.linalg.norm(n)
    T_tcp = k.fk(np.radians(q_deg)) @ tool
    c = np.asarray(root, float) + 0.01 * n
    a = T_tcp[:3, 3] - c
    a -= (a @ n) * n
    T_pl = np.eye(4)
    T_pl[:3, :3] = np.stack([a / np.linalg.norm(a), np.cross(n, a / np.linalg.norm(a)), n], 1)
    T_pl[:3, 3] = c
    return np.linalg.inv(T_tcp) @ T_pl


def main():
    import basket_plan as bp
    import calib_core as cc
    import pipeline as pl
    from sensors import FpStream
    C = bp.pp.C
    Tbc = np.array(cc.load_json(cc.latest_t_base_cam())['T_base_cam'])
    q0 = robot_joints()
    print(f'robot joints {q0.round(2).tolist()}', flush=True)
    st = C['stability']
    fp = FpStream(C['udp']['fp_pose'], st['n'], st['mm'], st['deg'], symmetric_z=True,
                  normalize=lambda T_: cc.inv_T(Tbc) @ pl.plate_upright(Tbc @ T_))
    env = {k: v for k, v in os.environ.items() if k not in ('PYTHONPATH', 'PYTHONHOME')}
    perc = subprocess.Popen([sys.executable, '-u', str(ROOT / 'real_time_project/main.py'), '--detector', 'yoloe',
                             '--width', '640', '--height', '480', '--serial', str(C['cameras']['fixed_serial']),
                             '--prompt', C['plate_ik_bc']['prompt'], '--conf', '0.15',
                             '--mesh_file', str(cc.cpath(C['paths']['plate_obj']))], env=env,
                            stdout=open('/tmp/measure_held_perception.log', 'w'), stderr=subprocess.STDOUT)
    T = None
    try:
        t_end = time.time() + 120
        while time.time() < t_end and T is None:
            if perc.poll() is not None:
                sys.exit('perception ended (see /tmp/measure_held_perception.log)')
            fp.poll()
            T = fp.stable()
            time.sleep(0.05)
    finally:
        perc.terminate()
        perc.wait(timeout=10)
    if T is None:
        sys.exit('no stable plate pose in 120 s (see /tmp/measure_held_perception.log)')
    if np.abs(robot_joints() - q0).max() > 0.5:
        sys.exit('the arm moved during the measurement: run it again with the arm still')
    Tb = pl.plate_upright(Tbc @ T)
    root, _, tilt = pl.plate_root_in_base(Tb)
    n = Tb[:3, 2]
    k = bp.pp.Kinematics()
    tool = np.eye(4)
    tool[2, 3] = bp.HAND_LENGTH_M
    H = held_H(k, q0, root, n, tool)
    A = yaml.safe_load((HERE / 'arm_poses.yaml').read_text())
    P = A['plate']
    H0 = bp.hand_plate(k, P['posj_deg'], np.array(P['obj_centre']) - [0.0, 0.0, 0.015], tool)   # the flat-plate model
    turn = np.degrees(np.arccos(np.clip(H0[:3, 2] @ H[:3, 2], -1, 1)))
    shift = (H[:3, 3] - H0[:3, 3]) * 1000
    print(f'plate in base: root {np.round(root, 3).tolist()}, tilt {tilt:.1f} deg from flat')
    print(f'vs the flat-plate model: normal turned {turn:.1f} deg in the hand, centre moved {shift.round(0).tolist()} mm (tool frame)')
    P['in_hand'] = dict(H=np.round(H, 5).tolist(), posj_deg=q0.round(3).tolist(), plate_root=np.round(root, 4).tolist(),
                        tilt_deg=round(tilt, 1), turn_vs_flat_deg=round(float(turn), 1),
                        measured=time.strftime('%Y-%m-%d %H:%M:%S'))
    (HERE / 'arm_poses.yaml').write_text(yaml.safe_dump(A, sort_keys=False, allow_unicode=True))
    print(f'saved plate.in_hand in {HERE / "arm_poses.yaml"}: basket_plan.py uses it from now on')


if __name__ == '__main__':
    main()
