"""Tesollo plate insert, sim side (rl/hand_basket.py): the planned descent as flange poses + the nominal plate-in-hand.

For each (quadrant, pin): basket_plan.py's plan (2F-85 planner, Tesollo grasp, --hang_deg nominal) -> the descend leg
densified to flange poses (FK), the hand tool, the nominal H, the basket pose and the target gap (basket frame).  The
Isaac script moves the plate along flange x tool x H_true (H_true = H_nom perturbed: the unknown hang / grip offset) and
releases it at the end.  CPU, foundationpose python:
    ~/miniconda3/envs/foundationpose/bin/python tesollo/hand_sim_job.py --out rl/logs/hand_basket/job.npz --hang 55 --pins Q3:6 Q3:9 Q4:6 Q4:9
"""
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'calibration'))
sys.path.insert(0, str(HERE))
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', required=True)
    p.add_argument('--hang', type=float, default=55.0, help='nominal in-hand hang the planner assumes [deg]')
    p.add_argument('--pins', nargs='+', default=['Q3:6', 'Q3:9', 'Q4:6', 'Q4:9'])
    p.add_argument('--step_mm', type=float, default=2.0, help='flange path resolution [mm]')
    a = p.parse_args()
    out = Path(a.out).resolve()
    import basket_plan as bp                                     # chdir()s into calibration/
    import yaml
    from scipy.spatial.transform import Rotation as R, Slerp
    pp = bp.pp
    k = pp.Kinematics()
    A = yaml.safe_load((HERE / 'arm_poses.yaml').read_text())['plate']
    tool = np.eye(4)
    tool[2, 3] = bp.HAND_LENGTH_M
    Tbb = np.array(json.loads((HERE.parent / 'calibration/data/basket/demo_basket_fixed.json').read_text())['T'])
    grasp = np.array(A['posj_deg'])
    H0 = bp.hand_plate(k, grasp, np.array(A['obj_centre']) - [0.0, 0.0, 0.015], tool)
    ax = np.array(bp.HANG_AXIS) / np.linalg.norm(bp.HANG_AXIS)
    Rh = np.eye(4)
    Rh[:3, :3] = R.from_rotvec(ax * np.radians(a.hang)).as_matrix()
    H_nom = Rh @ H0
    gaps = pp.basket_gaps()
    jobs = []
    for spec in a.pins:
        qd, pin = spec.split(':')
        env = dict(os.environ, PLATE_SHIFT='0 0')                # the real +y fix belongs to the flat model, not here
        r = subprocess.run([sys.executable, str(HERE / 'basket_plan.py'), '--quadrant', qd, '--pin', pin, '--model',
                            '--hang_deg', str(a.hang)], capture_output=True, text=True, env=env, cwd=str(HERE.parent))
        m = [l for l in r.stdout.splitlines() if l.startswith('plan: ')]
        if r.returncode or not m:
            print(f'{spec}: no plan ({r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr[-200:]})', flush=True)
            continue
        legs = json.loads(Path(m[0].split()[1]).read_text())['legs']
        wp = np.array(legs['descend']['waypoints_deg'])
        F = [k.fk(np.radians(q)) for q in wp]
        dense = []
        for Fa, Fb in zip(F[:-1], F[1:]):                         # flange poses every step_mm (slerp the small turns)
            n = max(2, int(np.ceil(np.linalg.norm(Fb[:3, 3] - Fa[:3, 3]) * 1000 / a.step_mm)) + 1)
            sl = Slerp([0, 1], R.from_matrix([Fa[:3, :3], Fb[:3, :3]]))
            for s in np.linspace(0, 1, n)[:-1]:
                T = np.eye(4)
                T[:3, :3] = sl(s).as_matrix()
                T[:3, 3] = (1 - s) * Fa[:3, 3] + s * Fb[:3, 3]
                dense.append(T)
        dense.append(F[-1])
        model_qd = pp.robot_quadrants(Tbb)[qd]
        gi = pp.pin_gap(Tbb, model_qd, int(pin))
        jobs.append(dict(name=spec, model_qd=model_qd, gap=gi, flange=np.array(dense),
                         gaps_b=np.array(gaps[model_qd]), above_deg=wp[0], insert_deg=wp[-1]))
        print(f'{spec}: model {model_qd} gap {gi}, {len(dense)} flange poses over the descent', flush=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    # no object arrays: Isaac's numpy 1.x cannot unpickle numpy 2 arrays -> ragged lists as one array + lengths
    np.savez(out, names=np.array([j['name'] for j in jobs]), model_qd=np.array([j['model_qd'] for j in jobs]),
             gap=np.array([j['gap'] for j in jobs]), flange=np.concatenate([j['flange'] for j in jobs]),
             flange_len=np.array([len(j['flange']) for j in jobs]), gaps_b=np.concatenate([j['gaps_b'] for j in jobs]),
             gaps_len=np.array([len(j['gaps_b']) for j in jobs]), tool=tool, H_nom=H_nom, hang=a.hang, hang_axis=ax, Tbb=Tbb)
    print(f'job {out}: {len(jobs)} pins, nominal hang {a.hang} deg')


if __name__ == '__main__':
    main()
