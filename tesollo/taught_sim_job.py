"""Tesollo plate insert, sim side for the pendant-taught insert (보스 10/29 night: learn a nice plate pose into the slot).

rl/hand_basket.py job from tesollo/taught_plan.py instead of the planner: for every (wrist_less, pin) the taught
over-basket -> 1 cm over the slot -> in the slot poses (turned back by wrist_less, moved to the pin) densified to flange
poses (joint lines, FK).  H_nom = the taught entry's plate_in_tcp (flange frame, tool = identity), so the sim learns
(dx, dy, dz) per job under the unknown in-hand pose and the jobs compare the wrist angles.  CPU, foundationpose python:
    ~/miniconda3/envs/foundationpose/bin/python tesollo/taught_sim_job.py --out rl/logs/hand_basket/job_taught.npz \
        --wrist 0 10 20 25 30 40 --pins Q3:3 Q3:6 Q3:9 Q3:12
Job names: 'w25:Q3:6'.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'calibration'))
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', required=True)
    p.add_argument('--wrist', type=float, nargs='+', default=[0, 10, 20, 25, 30, 40])
    p.add_argument('--pins', nargs='+', default=['Q3:3', 'Q3:6', 'Q3:9', 'Q3:12'])
    p.add_argument('--basket', default=str(HERE / 'demo_safe/basket_fixed.json'))
    p.add_argument('--index', type=int, default=0)
    p.add_argument('--step_mm', type=float, default=2.0, help='flange path resolution [mm]')
    p.add_argument('--calibrate_h', action='store_true', help='H_nom from the taught slot pose instead of the planner model: '
                   'the plate seated in the taught gap (lowest rim point on the gap line, plane on the tine lean) at teach 3 '
                   '(10/29: the model H put the plate ~9 cm off the slot, every rollout fell flat, while the real taught '
                   'insert went in)')
    a = p.parse_args()
    out = Path(a.out).resolve()
    import plate_plans as pp
    k = pp.Kinematics()
    e = json.loads((HERE / 'taught_inserts.json').read_text())[a.index]
    Tbb = np.array(json.loads(Path(a.basket).read_text())['T'])
    H_nom = np.array(e['plate_in_tcp'], float)
    if a.calibrate_h:
        from scipy.spatial.transform import Rotation as R
        Tt = np.array(e['basket_T'])                               # the basket at the teaching
        mq = pp.robot_quadrants(Tt)[e['quadrant']]
        g = pp.basket_gaps()[mq][pp.pin_gap(Tt, mq, e['pin'])]
        lean = np.radians(12.5 if mq in ('Q3', 'Q4') else -12.5)
        F3 = k.fk(np.radians(e['waypoints_deg'][-1]))
        n_model = np.linalg.inv(Tt)[:3, :3] @ (F3[:3, :3] @ H_nom[:3, 2])   # model plate normal, basket frame
        best = None
        for sg in (1, -1):
            n = np.array([np.cos(sg * lean), 0.0, np.sin(sg * lean)])
            n = n if n @ n_model >= 0 else -n
            u = np.array([0.0, 0.0, 1.0]) - n[2] * n
            u /= np.linalg.norm(u)
            bottom = np.array([g[0], g[1], -0.144 + 0.027])
            Tp = np.eye(4)
            Tp[:3, :3] = np.stack([u, np.cross(n, u), n], 1)
            Tp[:3, 3] = bottom + pp.PLATE_R * u
            Hc = np.linalg.inv(F3) @ Tt @ Tp
            d = np.degrees(np.linalg.norm(R.from_matrix(H_nom[:3, :3].T @ Hc[:3, :3]).as_rotvec()))
            if best is None or d < best[0]:
                best = (d, Hc)
        d, Hc = best
        print(f'H calibrated on the taught slot pose: {np.linalg.norm(Hc[:3, 3] - H_nom[:3, 3]) * 1000:.0f} mm / '
              f'{d:.0f} deg from the planner model plate_in_tcp', flush=True)
        H_nom = Hc
    sys.path.insert(0, str(HERE))
    from basket_plan import HANG_AXIS
    ax = np.array(HANG_AXIS) / np.linalg.norm(HANG_AXIS)
    gaps = pp.basket_gaps()
    jobs = []
    for w in a.wrist:
        for spec in a.pins:
            qd, pin = spec.split(':')
            r = subprocess.run([sys.executable, str(HERE / 'taught_plan.py'), '--index', str(a.index), '--quadrant', qd,
                                '--pin', pin, '--wrist_less', str(w), '--basket', str(Path(a.basket).resolve())],
                               capture_output=True, text=True, cwd=str(HERE.parent))
            m = [l for l in r.stdout.splitlines() if l.startswith('plan: ')]
            if r.returncode or not m:
                print(f'w{w:g}:{spec}: no plan ({(r.stdout + r.stderr).strip().splitlines()[-1][:160]})', flush=True)
                continue
            legs = json.loads(Path(m[0].split()[1]).read_text())['legs']
            wp = np.array(legs['post_grasp']['waypoints_deg'][-2:] + legs['insert']['waypoints_deg'][1:])   # teach 1, 2, mid, 3
            dense_q = []
            for qa, qb in zip(wp[:-1], wp[1:]):
                Fa, Fb = k.fk(np.radians(qa)), k.fk(np.radians(qb))
                n = max(2, int(np.ceil(np.linalg.norm(Fb[:3, 3] - Fa[:3, 3]) * 1000 / a.step_mm)) + 1)
                dense_q += [qa + s * (qb - qa) for s in np.linspace(0, 1, n)[:-1]]
            dense_q.append(wp[-1])
            F = np.array([k.fk(np.radians(q)) for q in dense_q])
            model_qd = pp.robot_quadrants(Tbb)[qd]
            gi = pp.pin_gap(Tbb, model_qd, int(pin))
            jobs.append(dict(name=f'w{w:g}:{spec}', model_qd=model_qd, gap=gi, flange=F, arm_q=np.radians(np.array(dense_q)),
                             gaps_b=np.array(gaps[model_qd])))
            print(f'w{w:g}:{spec}: model {model_qd} gap {gi}, {len(F)} flange poses', flush=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, names=np.array([j['name'] for j in jobs]), model_qd=np.array([j['model_qd'] for j in jobs]),
             gap=np.array([j['gap'] for j in jobs]), flange=np.concatenate([j['flange'] for j in jobs]),
             flange_len=np.array([len(j['flange']) for j in jobs]), arm_q=np.concatenate([j['arm_q'] for j in jobs]),
             arm_q_len=np.array([len(j['arm_q']) for j in jobs]), gaps_b=np.concatenate([j['gaps_b'] for j in jobs]),
             gaps_len=np.array([len(j['gaps_b']) for j in jobs]), tool=np.eye(4), H_nom=H_nom, hang=0.0, hang_axis=ax, Tbb=Tbb)
    print(f'job {out}: {len(jobs)} (wrist, pin) jobs')


if __name__ == '__main__':
    main()
