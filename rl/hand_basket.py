"""Tesollo hand plate insert in Isaac (10/27 night, 보스: learn the basket insert with the hand in sim).

Plate only: the hand is not simulated.  The plate follows the planned descent rigidly (pose = flange x tool x H_true each
physics step, like rl/basket/expert.py attach()), is released at the end and settles; the basket is the full parametric
basket (rl/basket_env).  H_true = the planner's nominal plate-in-hand turned by an unknown hang error about the hang axis,
shifted by an unknown grip offset and tilted a little (the real in-hand pose is not measured yet).  Per-pin corrections
(dx, dy along / across the rows, dz release height; basket frame) shift the whole descent; `cem` learns them per pin by
the cross-entropy method on the sim success rate under that randomisation.
    env -u CONDA_PREFIX ... ~/isaacsim-5.1.0/python.sh hand_basket.py eval --job logs/hand_basket/job_h55.npz
    ... hand_basket.py cem --job logs/hand_basket/job_h55.npz --iters 8 --pop 16 --reps 8
Success: settled plate centre over the target gap (|x - gap| < half a pitch), in its pin pair (|y - pair| < 6 cm),
standing in the rows (normal within 40 deg of the basket x axis), lowest point >= 7 cm under the rim.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

RL = Path(__file__).resolve().parent
sys.path.insert(0, str(RL))


def parse():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('mode', choices=('eval', 'cem'))
    p.add_argument('--job', required=True)
    p.add_argument('--reps', type=int, default=32, help='envs per pin (eval) / per candidate (cem)')
    p.add_argument('--pop', type=int, default=16)
    p.add_argument('--iters', type=int, default=8)
    p.add_argument('--hang_err', type=float, default=15.0, help='+- unknown hang error [deg]')
    p.add_argument('--grip_mm', type=float, default=15.0, help='+- unknown grip offset per axis [mm]')
    p.add_argument('--tilt_deg', type=float, default=3.0, help='+- unknown small in-hand tilt [deg]')
    p.add_argument('--corr', default='', help='eval: json {pin: [dx, dy, dz]} [m] (a cem result)')
    p.add_argument('--speed', type=float, default=0.10, help='descent speed of the plate [m/s]')
    p.add_argument('--settle_s', type=float, default=2.0)
    p.add_argument('--out', default='')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--viewer', action='store_true')
    return p.parse_args()


def main():
    a = parse()
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=not a.viewer).app
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.utils import configclass
    from scipy.spatial.transform import Rotation as R
    import basket_env as be
    import yaml

    J = np.load(a.job)
    names, gap_t = [str(n) for n in J['names']], [int(g) for g in J['gap']]
    flange = np.split(J['flange'], np.cumsum(J['flange_len'])[:-1])
    gaps_b = np.split(J['gaps_b'], np.cumsum(J['gaps_len'])[:-1])
    tool, H_nom, ax, Tbb = J['tool'], J['H_nom'], J['hang_axis'], J['Tbb']
    nj = len(names)
    per = a.reps if a.mode == 'eval' else a.pop * a.reps
    N = nj * per
    C = yaml.safe_load((RL / 'config.yaml').read_text())
    usd = be.full_basket_usd_path(be.marker_blocks_enabled())
    if not Path(usd).exists():
        be.build_full_basket_usd(usd)
    q_b = R.from_matrix(Tbb[:3, :3]).as_quat()                  # xyzw

    @configclass
    class Scene(InteractiveSceneCfg):
        basket = RigidObjectCfg(
            prim_path='{ENV_REGEX_NS}/Basket',
            init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(map(float, Tbb[:3, 3])),
                                                      rot=(float(q_b[3]), float(q_b[0]), float(q_b[1]), float(q_b[2]))),
            spawn=sim_utils.UsdFileCfg(usd_path=str(usd), rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=True, disable_gravity=True)))
        plate = RigidObjectCfg(
            prim_path='{ENV_REGEX_NS}/Plate',
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 1.5)),
            spawn=sim_utils.UsdFileCfg(usd_path=str(RL / C['paths']['plate_usd']), rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16, solver_velocity_iteration_count=2, max_depenetration_velocity=2.0)))
        plane = AssetBaseCfg(prim_path='/World/GroundPlane', init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0, -0.5)),
                             spawn=sim_utils.GroundPlaneCfg())
        light = AssetBaseCfg(prim_path='/World/light', spawn=sim_utils.DomeLightCfg(intensity=3000.0))

    dt = 1 / 120
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=dt, device='cuda:0'))
    scene = InteractiveScene(Scene(num_envs=N, env_spacing=2.0))
    sim.reset()
    dev = sim.device
    plate = scene['plate']
    org = scene.env_origins.cpu().numpy()
    verts = np.array([list(map(float, l.split()[1:4])) for l in (RL / 'assets/plate029_simplified.obj').open() if l.startswith('v ')])
    verts = verts[np.linspace(0, len(verts) - 1, min(len(verts), 400)).astype(int)]
    Tbi = np.linalg.inv(Tbb)
    root_off = np.eye(4)
    root_off[2, 3] = -0.01                                       # plate frame (centre, 1 cm up) -> mesh root (bottom centre)
    pitch = {i: float(np.median(np.diff([g[0] for g in gaps_b[i]]))) for i in range(nj)}
    rng = np.random.default_rng(a.seed)

    def h_true(n):
        """n unknown in-hand poses around H_nom."""
        out = []
        for _ in range(n):
            Rh = np.eye(4)
            Rh[:3, :3] = R.from_rotvec(ax * np.radians(rng.uniform(-a.hang_err, a.hang_err))).as_matrix()
            Rt = np.eye(4)
            v = rng.normal(size=3)
            Rt[:3, :3] = R.from_rotvec(v / np.linalg.norm(v) * np.radians(rng.uniform(-a.tilt_deg, a.tilt_deg))).as_matrix()
            Tg = np.eye(4)
            Tg[:3, 3] = rng.uniform(-1, 1, 3) * a.grip_mm / 1000
            out.append(Tg @ Rh @ Rt @ H_nom)
        return np.array(out)

    def rollout(job_of, corr, Hs):
        """job index, correction (dx, dy, dz basket frame) and H per env -> success (N,) and landed info."""
        L = max(len(flange[j]) for j in range(nj))
        step = np.array([np.linalg.norm(flange[j][1][:3, 3] - flange[j][0][:3, 3]) for j in range(nj)])
        n_move = int(math.ceil(max(len(flange[j]) * step[j] for j in range(nj)) / a.speed / dt))
        shift_base = np.einsum('ij,nj->ni', Tbb[:3, :3], corr)   # basket-frame correction -> base
        poses = np.zeros((N, L, 7))
        for e in range(N):
            j = job_of[e]
            F = flange[j].copy()
            F[:, :3, 3] += shift_base[e]
            P = F @ tool @ Hs[e] @ root_off
            P = np.concatenate([P, np.repeat(P[-1:], L - len(P), 0)])
            poses[e, :, :3] = P[:, :3, 3] + org[e]
            qx = R.from_matrix(P[:, :3, :3]).as_quat()
            poses[e, :, 3:] = np.c_[qx[:, 3], qx[:, :3]]
        poses_t = torch.tensor(poses, dtype=torch.float32, device=dev)
        lens = torch.tensor([len(flange[j]) for j in job_of], device=dev)
        zero = torch.zeros(N, 6, device=dev)
        for t in range(n_move + 30):
            i = torch.clamp((torch.tensor(t * a.speed * dt, device=dev) / torch.tensor(step[job_of], device=dev)).long(),
                            max=lens - 1)
            plate.write_root_pose_to_sim(poses_t[torch.arange(N, device=dev), i])
            plate.write_root_velocity_to_sim(zero)
            scene.write_data_to_sim()
            sim.step(render=a.viewer)
            scene.update(dt)
        for _ in range(int(a.settle_s / dt)):                    # released: the plate falls / settles in the pins
            scene.write_data_to_sim()
            sim.step(render=a.viewer)
            scene.update(dt)
        p = plate.data.root_pos_w.cpu().numpy() - org
        qw = plate.data.root_quat_w.cpu().numpy()
        Rm = R.from_quat(np.c_[qw[:, 1:], qw[:, :1]]).as_matrix()
        ok, landed = np.zeros(N, bool), np.full(N, -1)
        diag = np.zeros((N, 4))                                   # x error to the target gap [m], y error [m], upright, low [m]
        for e in range(N):
            j = job_of[e]
            c = Tbi[:3, :3] @ (p[e] + Rm[e][:, 2] * 0.01) + Tbi[:3, 3]
            n_b = Tbi[:3, :3] @ Rm[e][:, 2]
            low = (Tbi[:3, :3] @ (p[e][:, None] + Rm[e] @ verts.T) + Tbi[:3, 3:])[2].min()
            gx = np.array([g[0] for g in gaps_b[j]])
            k_ = int(np.argmin(np.abs(gx - c[0])))
            near = abs(gx[k_] - c[0]) < pitch[j] / 2
            landed[e] = k_ if near else -1
            diag[e] = [c[0] - gx[gap_t[j]], c[1] - gaps_b[j][0][1], abs(n_b[0]) > math.cos(math.radians(40)), low]
            ok[e] = (near and k_ == gap_t[j] and abs(c[1] - gaps_b[j][0][1]) < 0.06 and abs(n_b[0]) > math.cos(math.radians(40))
                     and low < -0.07)
        rollout.diag = diag
        return ok, landed

    t0 = time.time()
    if a.mode == 'eval':
        corr_d = json.loads(Path(a.corr).read_text()) if a.corr else {}
        job_of = np.repeat(np.arange(nj), per)
        corr = np.array([corr_d.get(names[j], [0.0, 0.0, 0.0]) for j in job_of], float)
        ok, landed = rollout(job_of, corr, h_true(N))
        d = rollout.diag
        res = {names[j]: dict(success=f'{int(ok[job_of == j].sum())}/{per}',
                              landed=np.bincount(landed[job_of == j] + 1, minlength=14).tolist(),
                              x_err_mm=round(float(np.median(d[job_of == j, 0])) * 1000, 1),
                              y_err_mm=round(float(np.median(d[job_of == j, 1])) * 1000, 1),
                              upright=round(float(d[job_of == j, 2].mean()), 2),
                              low_mm=round(float(np.median(d[job_of == j, 3])) * 1000, 1)) for j in range(nj)}
        for nm, r in res.items():
            print(f'{nm}: {r["success"]}  x err {r["x_err_mm"]} mm, y err {r["y_err_mm"]} mm, upright {r["upright"]}, '
                  f'lowest {r["low_mm"]} mm from the rim  (landed gap counts, -1 first: {r["landed"]})', flush=True)
        print(f'total {int(ok.sum())}/{N}  ({time.time() - t0:.0f} s)')
        out = dict(mode='eval', job=a.job, corr=corr_d, results=res, total=f'{int(ok.sum())}/{N}', args=vars(a))
    else:
        mu = np.zeros((nj, 3))
        sd = np.tile([0.02, 0.01, 0.01], (nj, 1))
        hist = []
        for it in range(a.iters):
            cand = mu[:, None, :] + sd[:, None, :] * rng.normal(size=(nj, a.pop, 3))
            cand[:, 0, :] = mu                                    # keep the current mean in the population
            job_of = np.repeat(np.arange(nj), a.pop * a.reps)
            cidx = np.tile(np.repeat(np.arange(a.pop), a.reps), nj)
            corr = cand[job_of, cidx]
            ok, _ = rollout(job_of, corr, h_true(N))
            rate = ok.reshape(nj, a.pop, a.reps).mean(2)
            for j in range(nj):
                elite = np.argsort(-rate[j])[:max(2, a.pop // 4)]
                mu[j] = cand[j, elite].mean(0)
                sd[j] = np.maximum(cand[j, elite].std(0), [0.002, 0.002, 0.002])
            hist.append(dict(iter=it, mean_rate=rate[:, 0].round(3).tolist(), best=rate.max(1).round(3).tolist(),
                             mu=mu.round(4).tolist()))
            print(f'iter {it}: rate at the mean ' + ', '.join(f'{names[j]} {rate[j, 0]:.2f}' for j in range(nj))
                  + f'  ({time.time() - t0:.0f} s)', flush=True)
        out = dict(mode='cem', job=a.job, corr={names[j]: mu[j].round(4).tolist() for j in range(nj)}, history=hist,
                   args=vars(a))
        print('corrections (dx, dy, dz basket frame, m): ' + json.dumps(out['corr']))
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps(out, indent=1))
        print(f'saved {a.out}')
    app.close()


if __name__ == '__main__':
    main()
