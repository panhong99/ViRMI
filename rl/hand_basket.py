"""Tesollo hand plate insert in Isaac (10/27 night, 보스: learn the basket insert with the hand in sim).

Plate only by default: the hand is not simulated.  With ``--hand`` the M1013 + DG-5F-S collision geometry follows the
same dense flange path using arm IK saved in the job; the hand uses plate_pre9 plus a 7 degree flexion closing
approximation.  Hand-to-basket contact force and PhysX contact separation (negative = penetration) are recorded.
The plate follows the planned descent rigidly (pose = flange x tool x H_true each
physics step, like rl/basket/expert.py attach()), is released at the end and settles; the basket is the full parametric
basket (rl/basket_env).  H_true = the planner's nominal plate-in-hand turned by an unknown hang error about the hang axis,
shifted by an unknown grip offset and tilted a little (the real in-hand pose is not measured yet).  Per-pin corrections
(dx, dy along / across the rows, dz release height; basket frame) shift the whole descent; `cem` learns them per pin by
the cross-entropy method on the sim success rate under that randomisation.
    env -u CONDA_PREFIX ... ~/isaacsim-5.1.0/python.sh hand_basket.py eval --job logs/hand_basket/job_h55.npz
    ... hand_basket.py cem --job logs/hand_basket/job_h55.npz --iters 8 --pop 16 --reps 8
Success (10/28, Codex CN: the gripper's rl/basket seated/exact, basket.yaml success): lowest rim point within
bottom_tol (8.5 mm) of the target gap line, plate centre within centre_tol (30 mm) of the pin pair, lowest point <= 55 mm
over the floor, plate plane within 20 deg of the tine plane (quadrant tine lean, either sign), moved < 1 cm in the last
1 s; landed gap = the gap nearest the lowest rim point.  The old loose test (centre within half a pitch, 6 cm, 40 deg,
7 cm under the rim) is kept as 'loose' for comparison.
"""
import argparse
import json
import math
import os
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
    p.add_argument('--compliant', action='store_true', help='(Claude 10/29, opt-in) drive the plate toward the path pose '
                   'with velocities (contacts resolved) instead of writing the pose every step (the dish is 2.5 cm deep: '
                   'a forced pose runs it through the tines and the release depenetration decides the result)')
    p.add_argument('--compliant_tau', type=float, default=0.05, help='--compliant: time constant [s]')
    p.add_argument('--settle_s', type=float, default=2.0)
    p.add_argument('--out', default='')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--viewer', action='store_true')
    p.add_argument('--viewer_hold', action='store_true', help='(opt-in, 보스 10/30 GUI) --viewer: keep the window open '
                   'on the settled plates after the results are saved, until it is closed')
    p.add_argument('--viewer_cam', type=float, nargs=3, default=None,
                   help='(opt-in, 보스 10/31 GIF) --viewer: eye offset [m] from env 0 basket centre, looking at it')
    p.add_argument('--hand', action='store_true', help='opt-in M1013 + DG-5F-S collision replay (job must contain arm_q)')
    p.add_argument('--hand_contact_n', type=float, default=1.0, help='hand-basket contact threshold [N]')
    p.add_argument('--dock_traj', default='', help='eval: dock_train.py exported residual trajectory bundle')
    return p.parse_args()


def main():
    a = parse()
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=not a.viewer).app
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.utils import configclass
    from scipy.spatial.transform import Rotation as R
    import basket_env as be
    import yaml

    J = np.load(a.job)
    model_qd = [str(m) for m in J['model_qd']]
    SUCC = be.B['success']
    TINE_LEAN = be.B['full']['tine_lean_deg']
    FLOOR_B = be.B['full']['floor_z']                            # basket frame (rim = 0)
    names, gap_t = [str(n) for n in J['names']], [int(g) for g in J['gap']]
    flange = np.split(J['flange'], np.cumsum(J['flange_len'])[:-1])
    arm_q = None
    if a.hand:
        if 'arm_q' not in J:
            raise SystemExit('--hand needs a newly generated job containing arm_q (tesollo/hand_sim_job.py)')
        arm_q = np.split(J['arm_q'], np.cumsum(J['arm_q_len'])[:-1])
    gaps_b = np.split(J['gaps_b'], np.cumsum(J['gaps_len'])[:-1])
    tool, H_nom, ax, Tbb = J['tool'], J['H_nom'], J['hang_axis'], J['Tbb']
    nj = len(names)
    dock = np.load(a.dock_traj) if a.dock_traj else None
    if dock is not None and (a.mode != 'eval' or a.corr):
        raise SystemExit('--dock_traj is eval-only and cannot be combined with --corr')
    per = a.reps if a.mode == 'eval' else a.pop * a.reps
    N = len(dock['job']) if dock is not None else nj * per
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

    if a.hand:
        import hand_env
        hand_bodies = ['link_mount', 'link_base'] + [f'link_{i}_{j}' for i in range(1, 6) for j in range(1, 5)] \
                      + [f'link_{i}_tip' for i in range(1, 6)]

        @configclass
        class HandScene(Scene):
            robot = hand_env.robot_cfg()
            # A filtered ContactSensor must address exactly one body per environment.
            # Keep one sensor per palm/finger link, then aggregate them below.
            for _body in hand_bodies:
                locals()[f'hand_contact_{_body}'] = ContactSensorCfg(
                    prim_path=f'{{ENV_REGEX_NS}}/Robot/{_body}', filter_prim_paths_expr=['{ENV_REGEX_NS}/Basket'],
                    history_length=1, track_contact_points=True, max_contact_data_count_per_prim=8)
            del _body
        SceneType = HandScene
    else:
        SceneType = Scene

    dt = 1 / 120
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=dt, device='cuda:0'))
    scene = InteractiveScene(SceneType(num_envs=N, env_spacing=2.0))
    sim.reset()
    if a.viewer and a.viewer_cam:
        tgt = np.asarray(scene.env_origins[0].cpu()) + Tbb[:3, 3]
        sim.set_camera_view(eye=(tgt + np.asarray(a.viewer_cam)).tolist(), target=tgt.tolist())
    dev = sim.device
    plate = scene['plate']
    robot = scene['robot'] if a.hand else None
    hand_sensors = [scene[f'hand_contact_{b}'] for b in hand_bodies] if a.hand else []
    org = scene.env_origins.cpu().numpy()
    verts = np.array([list(map(float, l.split()[1:4])) for l in (RL / 'assets/plate029_simplified.obj').open() if l.startswith('v ')])
    verts = verts[np.linspace(0, len(verts) - 1, min(len(verts), 400)).astype(int)]
    Tbi = np.linalg.inv(Tbb)
    root_off = np.eye(4)
    root_off[2, 3] = -0.01                                       # plate frame (centre, 1 cm up) -> mesh root (bottom centre)
    pitch = {i: float(np.median(np.diff([g[0] for g in gaps_b[i]]))) for i in range(nj)}
    rng = np.random.default_rng(a.seed)
    if a.hand:
        poses_y = yaml.safe_load((RL.parent / 'tesollo/poses.yaml').read_text())
        hand_q = np.array(poses_y['plate_pre9']['joints_rad'], dtype=np.float32)
        hand_q[1::4] += math.radians(7.0)                         # force-close approximation: flexion joint 2
        arm_ids, _ = robot.find_joints('joint_[1-6]')
        finger_ids, finger_names = robot.find_joints('joint_[1-5]_[1-4]')
        order = sorted(range(len(finger_names)), key=lambda i: tuple(map(int, finger_names[i].split('_')[1:])))
        finger_ids = [finger_ids[i] for i in order]
        if len(arm_ids) != 6 or len(finger_ids) != 20:
            raise RuntimeError(f'unexpected DG-5F-S joints: arm {len(arm_ids)}, hand {len(finger_ids)}')

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

    def rollout(job_of, corr, Hs, dock_residual=None, dock_zdown=None, dock_zoff=None):
        """job index, correction (dx, dy, dz basket frame) and H per env -> success (N,) and landed info.
        dock_zdown (N,) [m] (Claude 10/29, opt-in in the bundle): after the path end keep lowering the plate 2 mm a
        step along basket -z by that much before the release, as DockEnv(descend_extra_mm) does."""
        n_extra = np.zeros(N, int) if dock_zdown is None else np.ceil(np.asarray(dock_zdown) / 0.002 - 1e-6).astype(int)
        L = max(len(flange[j]) for j in range(nj)) + int(n_extra.max())
        step = np.array([np.linalg.norm(flange[j][1][:3, 3] - flange[j][0][:3, 3]) for j in range(nj)])
        n_move = int(math.ceil(max((len(flange[j]) + int(n_extra.max())) * step[j] for j in range(nj)) / a.speed / dt))
        shift_base = np.einsum('ij,nj->ni', Tbb[:3, :3], corr)   # basket-frame correction -> base
        poses = np.zeros((N, L, 7))
        for e in range(N):
            j = job_of[e]
            F = flange[j].copy()
            centre_path = F @ tool @ Hs[e]
            if dock_residual is None:
                centre_path[:, :3, 3] += shift_base[e]
            else:
                old = np.linspace(0, 1, dock_residual.shape[1])
                new = np.linspace(0, 1, len(F))
                rr = np.stack([np.interp(new, old, dock_residual[e, :, k]) for k in range(dock_residual.shape[2])], 1)
                centre_path[:, :3, 3] += (Tbb[:3, :3] @ np.c_[rr[:, :2], np.zeros(len(rr))].T).T
                for k in range(len(centre_path)):
                    centre_path[k, :3, :3] = R.from_rotvec(Tbb[:3, 1] * rr[k, 2]).as_matrix() @ centre_path[k, :3, :3]
                    if rr.shape[1] > 3:      # (Claude 10/30, dock_pc_train bundles) yaw about the basket vertical at the centre
                        centre_path[k, :3, :3] = R.from_rotvec(Tbb[:3, 2] * rr[k, 3]).as_matrix() @ centre_path[k, :3, :3]
            if dock_zoff is not None:                              # whole path raised: release that high (drop into the slot)
                centre_path[:, :3, 3] += float(dock_zoff[e]) * Tbb[:3, 2]
            if n_extra[e]:
                down = np.repeat(centre_path[-1:], n_extra[e], 0)
                down[:, :3, 3] -= np.outer(np.minimum(np.arange(1, n_extra[e] + 1) * 0.002, dock_zdown[e]), Tbb[:3, 2])
                centre_path = np.concatenate([centre_path, down])
            P = centre_path @ root_off
            P = np.concatenate([P, np.repeat(P[-1:], L - len(P), 0)])
            poses[e, :, :3] = P[:, :3, 3] + org[e]
            qx = R.from_matrix(P[:, :3, :3]).as_quat()
            poses[e, :, 3:] = np.c_[qx[:, 3], qx[:, :3]]
        poses_t = torch.tensor(poses, dtype=torch.float32, device=dev)
        lens = torch.tensor([len(flange[j]) + int(n_extra[e]) for e, j in enumerate(job_of)], device=dev)
        zero = torch.zeros(N, 6, device=dev)
        hand_hit = torch.zeros(N, dtype=torch.bool, device=dev)
        hand_max_force = torch.zeros(N, device=dev)
        hand_max_pen = torch.zeros(N, device=dev)
        if a.hand:
            q_path = np.zeros((N, L, 6), dtype=np.float32)
            for e in range(N):
                j = job_of[e]
                q_path[e, :len(arm_q[j])] = arm_q[j]
                q_path[e, len(arm_q[j]):] = arm_q[j][-1]
            q_path_t = torch.tensor(q_path, device=dev)
            hand_q_t = torch.tensor(hand_q, device=dev).repeat(N, 1)

        def measure_hand_contact():
            if not a.hand:
                return
            for sensor in hand_sensors:
                fm = sensor.data.force_matrix_w
                f = torch.linalg.vector_norm(fm, dim=-1).amax(dim=(1, 2))
                hand_max_force[:] = torch.maximum(hand_max_force, f)
                hand_hit[:] |= f >= a.hand_contact_n
                # Detailed PhysX contact data includes signed separation.  Each
                # single-link sensor has one sensor/filter row per environment.
                _, _, _, sep, counts, starts = sensor.contact_physx_view.get_contact_data(dt)
                counts = counts.reshape(N, -1).cpu().numpy()
                starts = starts.reshape(N, -1).cpu().numpy()
                sep = sep.reshape(-1).cpu().numpy()
                for e in np.nonzero(counts.sum(1))[0]:
                    deepest = 0.0
                    for c, s in zip(counts[e], starts[e]):
                        if c:
                            deepest = max(deepest, float(np.maximum(0.0, -sep[int(s):int(s + c)]).max()))
                    hand_max_pen[e] = max(float(hand_max_pen[e]), deepest)
        for t in range(n_move + 30):
            i = torch.clamp((torch.tensor(t * a.speed * dt, device=dev) / torch.tensor(step[job_of], device=dev)).long(),
                            max=lens - 1)
            tgt = poses_t[torch.arange(N, device=dev), i]
            if not a.compliant or t == 0:
                plate.write_root_pose_to_sim(tgt)
                plate.write_root_velocity_to_sim(zero)
            else:
                cur_p, cur_q = plate.data.root_pos_w, plate.data.root_quat_w          # wxyz
                v = ((tgt[:, :3] - cur_p) / a.compliant_tau).clamp(-0.5, 0.5)
                qt, qc = tgt[:, 3:], cur_q
                qc_conj = qc * torch.tensor([1.0, -1.0, -1.0, -1.0], device=dev)
                w1, x1, y1, z1 = qt.unbind(1)
                w2, x2, y2, z2 = qc_conj.unbind(1)
                qe = torch.stack((w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                                  w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2), 1)
                qe = qe * torch.where(qe[:, :1] < 0, -1.0, 1.0)
                vn = torch.linalg.vector_norm(qe[:, 1:], dim=1, keepdim=True).clamp_min(1e-9)
                rotvec = 2 * torch.atan2(vn, qe[:, :1]) * qe[:, 1:] / vn
                wv = (rotvec / a.compliant_tau).clamp(-3.0, 3.0)
                plate.write_root_velocity_to_sim(torch.cat((v, wv), 1))
                if __import__('os').environ.get('COMPLIANT_DEBUG') and t % 60 == 0:
                    pe = torch.linalg.vector_norm(tgt[:, :3] - cur_p, dim=1)
                    print(f'  compliant t {t}: pos err p50 {pe.median() * 1000:.1f} mm, p90 {pe.quantile(.9) * 1000:.1f} mm, '
                          f'rot err p50 {torch.rad2deg(2 * torch.atan2(vn[:, 0], qe[:, 0])).median():.1f} deg', flush=True)
            if a.hand:
                q = robot.data.default_joint_pos.clone()
                q[:, arm_ids] = q_path_t[torch.arange(N, device=dev), i]
                q[:, finger_ids] = hand_q_t
                robot.write_joint_state_to_sim(q, torch.zeros_like(q))
                robot.set_joint_position_target(q)
            scene.write_data_to_sim()
            sim.step(render=a.viewer)
            scene.update(dt)
            measure_hand_contact()
        if a.hand:
            # Release outcome stays comparable with the plate-only baseline: after
            # measuring insertion clearance, move the hand back to its default pose.
            q = robot.data.default_joint_pos.clone()
            robot.write_joint_state_to_sim(q, torch.zeros_like(q))
            robot.set_joint_position_target(q)
        n_settle = int(a.settle_s / dt)
        for t in range(n_settle):                                # released: the plate falls / settles in the pins
            if t == n_settle - int(1.0 / dt):
                p_still = plate.data.root_pos_w.cpu().numpy().copy()
            scene.write_data_to_sim()
            sim.step(render=a.viewer)
            scene.update(dt)
        p = plate.data.root_pos_w.cpu().numpy() - org
        qw = plate.data.root_quat_w.cpu().numpy()
        Rm = R.from_quat(np.c_[qw[:, 1:], qw[:, :1]]).as_matrix()
        ok, landed = np.zeros(N, bool), np.full(N, -1)
        loose = np.zeros(N, bool)
        still = np.linalg.norm(plate.data.root_pos_w.cpu().numpy() - p_still, axis=1) < 0.01
        nerr_all = np.zeros(N)
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
            loose[e] = (near and k_ == gap_t[j] and abs(c[1] - gaps_b[j][0][1]) < 0.06
                        and abs(n_b[0]) > math.cos(math.radians(40)) and low < -0.07)
            W = Tbi[:3, :3] @ (p[e][:, None] + Rm[e] @ verts.T) + Tbi[:3, 3:]
            bot = W[:, int(np.argmin(W[2]))]
            kb = int(np.argmin(np.abs(gx - bot[0])))
            lean = math.radians(TINE_LEAN[model_qd[j]])
            nerr = min(math.degrees(math.acos(min(1.0, abs(n_b @ np.array([math.cos(sg * lean), 0.0, math.sin(sg * lean)])))))
                       for sg in (1, -1))
            nerr_all[e] = nerr
            ok[e] = (kb == gap_t[j] and abs(bot[0] - gx[kb]) < SUCC['bottom_tol_m'] and abs(c[1] - gaps_b[j][0][1]) < SUCC['centre_tol_m']
                     and bot[2] - FLOOR_B < SUCC['max_height_m'] and nerr < SUCC['max_normal_err_deg'] and still[e])
            landed[e] = kb
        rollout.diag = diag
        rollout.nerr = nerr_all
        rollout.final_b = np.stack([Tbi @ np.r_[np.c_[Rm[e], p[e]], [[0, 0, 0, 1]]] for e in range(N)])   # plate root in basket
        rollout.loose = loose
        rollout.hand_hit = hand_hit.cpu().numpy()
        rollout.hand_max_force = hand_max_force.cpu().numpy()
        rollout.hand_max_pen = hand_max_pen.cpu().numpy()
        return ok, landed

    t0 = time.time()
    if a.mode == 'eval':
        corr_d = json.loads(Path(a.corr).read_text()) if a.corr else {}
        corr_d = corr_d.get('corr', corr_d)                      # a cem output file or a bare {pin: [dx, dy, dz]}
        job_of = dock['job'].astype(int) if dock is not None else np.repeat(np.arange(nj), per)
        corr = np.array([corr_d.get(names[j], [0.0, 0.0, 0.0]) for j in job_of], float)
        Hs = dock['H'] if dock is not None else h_true(N)
        ok, landed = rollout(job_of, corr, Hs, dock['residual'] if dock is not None else None,
                             dock['zdown'] if dock is not None and 'zdown' in dock else None,
                             dock['zoff'] if dock is not None and 'zoff' in dock else None)
        d = rollout.diag
        collision_free = ok & ~rollout.hand_hit
        res = {names[j]: dict(success=f'{int(ok[job_of == j].sum())}/{int((job_of == j).sum())}',
                              success_no_hand_contact=f'{int(collision_free[job_of == j].sum())}/{per}' if a.hand else None,
                              hand_contact=f'{int(rollout.hand_hit[job_of == j].sum())}/{per}' if a.hand else None,
                              hand_max_force_n=round(float(rollout.hand_max_force[job_of == j].max()), 2) if a.hand else None,
                              hand_max_penetration_mm=round(float(rollout.hand_max_pen[job_of == j].max()) * 1000, 2) if a.hand else None,
                              loose=f'{int(rollout.loose[job_of == j].sum())}/{int((job_of == j).sum())}',
                              landed=np.bincount(landed[job_of == j] + 1, minlength=14).tolist(),
                              x_err_mm=round(float(np.median(d[job_of == j, 0])) * 1000, 1),
                              y_err_mm=round(float(np.median(d[job_of == j, 1])) * 1000, 1),
                              upright=round(float(d[job_of == j, 2].mean()), 2),
                              low_mm=round(float(np.median(d[job_of == j, 3])) * 1000, 1))
               for j in range(nj) if np.any(job_of == j)}
        for nm, r in res.items():
            print(f'{nm}: {r["success"]} (loose {r["loose"]})  x err {r["x_err_mm"]} mm, y err {r["y_err_mm"]} mm, upright {r["upright"]}, '
                  f'lowest {r["low_mm"]} mm from the rim  (landed gap counts, -1 first: {r["landed"]})', flush=True)
        print(f'total {int(ok.sum())}/{N} (loose {int(rollout.loose.sum())}/{N})  ({time.time() - t0:.0f} s)')
        if a.hand:
            print(f'hand contact {int(rollout.hand_hit.sum())}/{N}; strict collision-free {int(collision_free.sum())}/{N}; '
                  f'max force {rollout.hand_max_force.max():.2f} N, max penetration {rollout.hand_max_pen.max() * 1000:.2f} mm')
        samples = ([dict(pin=names[int(job_of[e])], rep=int(e), strict_success=bool(ok[e]),
                         hand_contact=bool(rollout.hand_hit[e]), hand_max_force_n=round(float(rollout.hand_max_force[e]), 4),
                         hand_max_penetration_mm=round(float(rollout.hand_max_pen[e]) * 1000, 4)) for e in range(N)]
                   if a.hand else None)
        out = dict(mode='eval', job=a.job, corr=corr_d, dock_traj=a.dock_traj or None,
                   dock_method=(str(dock['method']) if dock is not None else None), results=res, total=f'{int(ok.sum())}/{N}',
                   total_no_hand_contact=f'{int(collision_free.sum())}/{N}' if a.hand else None,
                   hand_contact=f'{int(rollout.hand_hit.sum())}/{N}' if a.hand else None,
                   hand_max_force_n=float(rollout.hand_max_force.max()) if a.hand else None,
                   hand_max_penetration_mm=float(rollout.hand_max_pen.max() * 1000) if a.hand else None,
                   hand_pose=('plate_pre9 + 7 deg on each finger flexion joint_2' if a.hand else None),
                   samples=samples, args=vars(a))
        if dock is not None:                                   # (Claude 10/29) per env, to match kinematic vs physics
            out['dock_samples'] = [dict(env=int(e), job=int(job_of[e]), target_gap=int(gap_t[job_of[e]]), landed=int(landed[e]),
                                        strict=bool(ok[e]), plane_err_deg=round(float(rollout.nerr[e]), 2),
                                        low_mm=round(float(d[e, 3]) * 1000, 1),
                                        final_T_basket=np.round(rollout.final_b[e], 5).tolist()) for e in range(N)]
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
        print(f'saved {a.out}', flush=True)
    if a.viewer and a.viewer_hold:
        while app.is_running():
            app.update()
    import os
    os._exit(0)                                                  # (10/27) app.close() hung after the results were saved


if __name__ == '__main__':
    main()
