"""(보스 10/30) PPO on the point-cloud goal docking (rl/dock_env.py --pc_goal), with a PointNet encoder, the P
controller weaned off step by step and a tolerance curriculum (wide first, then tighter).

action = w * P(obs) + s * tanh(policy), w from --p_weights per stage (1 -> 0), s = 0.35 + 0.65 * (1 - w).
Stage k uses fit radius --tols[k] mm; it moves on when the point-cloud success (end fit ratio >= --fit_ok and kinematic
strict success) reaches --advance on --patience evals in a row.  Each stage leaves stage<k>.pt, a GUI snapshot png and
a line in progress.jsonl; the run stops after the last stage passes or --max_updates.
    ~/miniconda3/envs/foundationpose/bin/python rl/dock_pc_train.py --out outputs/dock_rl_20261030/pc_curr_v1"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import torch
import torch.nn as nn

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from dock_env import DockEnv, controller  # noqa: E402
from dock_train import wilson  # noqa: E402


class PointNetAC(nn.Module):
    """9 scalar obs + the plate's and the goal's points (M each, flag column for which set) -> shared point MLP, max pool."""

    def __init__(self, m, act, width=128):
        super().__init__()
        self.m, self.act = m, act
        self.point = nn.Sequential(nn.Linear(5, 64), nn.ReLU(), nn.Linear(64, width), nn.ReLU(), nn.Linear(width, width))
        self.body = nn.Sequential(nn.Linear(width + 9, 256), nn.Tanh(), nn.Linear(256, 128), nn.Tanh())
        self.actor = nn.Linear(128, act)
        self.critic = nn.Linear(128, 1)
        self.log_std = nn.Parameter(torch.full((act,), -1.5))
        nn.init.orthogonal_(self.actor.weight, 0.01); nn.init.zeros_(self.actor.bias)
        self.register_buffer('w', torch.tensor(1.0))

    def features(self, obs):
        n, m = len(obs), self.m
        cur = obs[:, 9:9 + 3 * m].reshape(n, m, 3)
        goal = obs[:, 9 + 3 * m:].reshape(n, m, 3)
        flag = torch.zeros(n, 2 * m, 2, device=obs.device)
        flag[:, :m, 0] = 1; flag[:, m:, 1] = 1
        f = self.point(torch.cat((torch.cat((cur, goal), 1), flag), 2)).max(1).values
        return self.body(torch.cat((obs[:, :9], f), 1))

    def dist_value(self, obs):
        h = self.features(obs)
        return torch.distributions.Normal(self.actor(h), self.log_std.exp()), self.critic(h).squeeze(-1)

    def _mix(self, obs, raw):
        p = controller(obs, 'p')
        p = torch.cat((p, torch.zeros(len(p), self.act - p.shape[1], device=p.device)), 1)
        return (self.w * p + (0.35 + 0.65 * (1 - self.w)) * torch.tanh(raw)).clamp(-1, 1)

    def deterministic(self, obs):
        return self._mix(obs, self.actor(self.features(obs)))

    def action_from_raw(self, obs, raw):
        return self._mix(obs, raw)


@torch.no_grad()
def evaluate(net, env_kw, fit_ok, episodes=1024, seed=1024):
    env = DockEnv(episodes, seed=seed, **env_kw)
    obs = env.reset()
    for _ in range(env.steps - 1):
        obs, _, _, info = env.step(net.deterministic(obs) if net is not None else controller(obs, 'p'))
    ok = info['success']
    pc_ok = ok & (info['fit'] >= fit_ok)
    lo, hi = wilson(int(pc_ok.sum()), episodes)
    extra = dict(phys_pred=float(info['phys_p'].mean())) if 'phys_p' in info else {}
    return dict(**extra, success=float(ok.float().mean()), pc_success=float(pc_ok.float().mean()), pc_lo95=lo, pc_hi95=hi,
                fit=float(info['fit'].mean()), chamfer_mm=float(info['chamfer'].median() * 1000))


def snapshot(net, env_kw, path, envs=4, seed=7):
    """The GUI's last frame (dock_pc_view) as a png, for 보스 to look at remotely."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import dock_pc_view as V
    frames, ok = V.rollout(net.deterministic, envs, seed, **env_kw)
    f = frames[-1]
    fig = plt.figure(figsize=(4.2 * envs, 4.4))
    for i in range(envs):
        ax = fig.add_subplot(1, envs, i + 1, projection='3d')
        ax.scatter(*f['goal'][i].T, s=8, c='tab:green'); ax.scatter(*f['cur'][i].T, s=8, c='tab:blue')
        m = f['goal'][i].mean(0)
        ax.set_xlim(m[0] - 160, m[0] + 160); ax.set_ylim(m[1] - 160, m[1] + 160); ax.set_zlim(m[2] - 160, m[2] + 160)
        ax.set_title(f'{"OK" if ok[i] else "fail"} fit {f["fit"][i]:.2f} Chamfer {f["cham"][i]:.1f} mm', fontsize=9)
    fig.tight_layout(); fig.savefig(path, dpi=90); plt.close(fig)


def kin_features(env):
    """DockEnv.kin_features (x relative to the gap root, not the shifted goal)."""
    return env.kin_features()


@torch.no_grad()
def export_physics(net, env_kw, prefix, episodes=1024, seed=51024):
    """P and policy residual paths for hand_basket.py --dock_traj (fixed basket, as dock_train.export_physics), with a
    4th column = in-hand yaw + the yaw action and zdown = the extra descent used."""
    import numpy as np
    prefix = Path(prefix); prefix.parent.mkdir(parents=True, exist_ok=True)
    kw = dict(env_kw, basket_xy_mm=0, basket_yaw_deg=0, pitch_frac=0, lean_deg=0)
    out = []
    for method in ('p', 'policy'):
        env = DockEnv(episodes, seed=seed, **kw)
        obs = env.reset()
        res = torch.zeros(episodes, env.path_steps, 4, device='cuda')
        ids = torch.arange(episodes, device='cuda')
        for _ in range(env.steps - 1):
            obs, _, _, info = env.step(net.deterministic(obs) if method == 'policy' else controller(obs, 'p'))
            r = torch.zeros(episodes, 4, device='cuda')
            r[:, :3] = env.residual[:, :3]
            r[:, 3] = env.yaw0 + (env.residual[:, 3] if env.act_dim > 3 else 0)
            res[ids, env.t] = r
        path = prefix.with_name(prefix.name + f'_{method}.npz')
        np.savez_compressed(path, method=method, seed=seed, job=env.job.cpu().numpy(), H=env.H.cpu().numpy(),
                            kin_feat=kin_features(env).cpu().numpy(),
                            residual=res.cpu().numpy(), zdown=env.zdown.cpu().numpy(),
                            kin_success=info['success'].cpu().numpy(), kin_fit=info['fit'].cpu().numpy(),
                            names=np.array([f'Q3_job_{i}' for i in range(len(env.paths))]), env_kwargs=json.dumps(kw))
        out.append(dict(path=str(path), kin_success=float(info['success'].float().mean()),
                        pc_success=float((info['success'] & (info['fit'] >= 0.8)).float().mean())))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', default='outputs/dock_rl_20261030/pc_curr_v1')
    p.add_argument('--envs', type=int, default=2048)
    p.add_argument('--tols', type=float, nargs='+', default=[15, 12, 10, 8, 6, 5])
    p.add_argument('--p_weights', type=float, nargs='+', default=[1.0, 0.8, 0.6, 0.4, 0.2, 0.0])
    p.add_argument('--fit_ok', type=float, default=0.8)
    p.add_argument('--advance', type=float, nargs='+', default=[0.5],
                   help='pc_success to move on, one for all stages or one per stage (~2/3 of the oracle bound at that radius)')
    p.add_argument('--eval_episodes', type=int, default=1024)
    p.add_argument('--disc_dist', action='store_true', help='exact point-to-goal-disc distance instead of the 32-point Chamfer')
    p.add_argument('--patience', type=int, default=2)
    p.add_argument('--max_updates', type=int, default=1500)
    p.add_argument('--yaw', action='store_true', help='in-hand yaw +/- --yaw_deg and a yaw action')
    p.add_argument('--yaw_deg', type=float, default=10.0)
    # (Claude 10/30 oracle check) the defaults cap even a true-geometry controller at pc 26-41 %; these opt-ins
    # (extra descent to the floor, pitch range, hover above the slot) give P 35 % / oracle 64 % at 30 mm
    p.add_argument('--descend_extra_mm', type=float, default=40.0)
    p.add_argument('--residual_pitch_deg', type=float, default=25.0)
    p.add_argument('--hover_until', type=float, default=0.15)
    p.add_argument('--target_x_mm', type=float, default=0.0, help='goal/seat x offset from the gap root (basket x)')
    p.add_argument('--phys_critic', default='', help='(opt-in) rl/phys_critic.py model: terminal reward += phys_w * P(physics)')
    p.add_argument('--phys_w', type=float, default=12.0)
    p.add_argument('--plateau', type=int, default=0, help='also move on after this many updates without a new best (0: never)')
    p.add_argument('--epochs', type=int, default=3); p.add_argument('--batch', type=int, default=8192)
    p.add_argument('--lr', type=float, default=3e-4); p.add_argument('--gamma', type=float, default=.99)
    p.add_argument('--lam', type=float, default=.95); p.add_argument('--clip', type=float, default=.2)
    p.add_argument('--vf', type=float, default=.5); p.add_argument('--ent', type=float, default=.002)
    p.add_argument('--eval_every', type=int, default=5); p.add_argument('--seed', type=int, default=0)
    p.add_argument('--init', default='')
    p.add_argument('--start_stage', type=int, default=0)
    p.add_argument('--export_physics', default='', help='with --init: only write the P/policy bundles to this prefix and exit')
    a = p.parse_args()
    assert len(a.tols) == len(a.p_weights)
    gates = a.advance * len(a.tols) if len(a.advance) == 1 else a.advance
    assert len(gates) == len(a.tols)
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    (out / 'args.json').write_text(json.dumps(vars(a), indent=1))
    stage = a.start_stage
    kw = dict(pc_goal=True, pc_tol_mm=a.tols[stage], pc_yaw=a.yaw, yaw_deg=a.yaw_deg if a.yaw else 0.0,
              descend_extra_mm=a.descend_extra_mm, residual_pitch_deg=a.residual_pitch_deg,
              hover_until=a.hover_until if a.hover_until > 0 else None, pc_disc_dist=a.disc_dist,
              target_x_mm=a.target_x_mm, phys_critic=a.phys_critic or None, phys_w=a.phys_w)
    env = DockEnv(a.envs, seed=a.seed, **kw)
    net = PointNetAC(env.pc_m, env.act_dim).cuda()
    if a.init:
        net.load_state_dict(torch.load(a.init, map_location='cuda', weights_only=True))
    net.w.fill_(a.p_weights[stage])
    if a.export_physics:
        print(json.dumps(export_physics(net, kw, a.export_physics), indent=1))
        return
    opt = torch.optim.Adam(net.parameters(), lr=a.lr)
    base = evaluate(None, kw, a.fit_ok)
    log = lambda rec: (print(json.dumps(rec), flush=True),  # noqa: E731
                       open(out / 'progress.jsonl', 'a').write(json.dumps(rec) + '\n'))
    log(dict(event='start', stage=stage, tol_mm=a.tols[stage], w=a.p_weights[stage], p_controller=base, time=time.strftime('%H:%M')))
    torch.cuda.reset_peak_memory_stats()
    streak, best, best_at = 0, -1.0, 0
    for update in range(a.max_updates):
        obs = env.reset()
        O = []; A = []; LP = []; R = []; V = []; D = []
        for _ in range(env.steps - 1):
            with torch.no_grad():
                dist, val = net.dist_value(obs)
                raw = dist.sample()
                sq = torch.tanh(raw)
                logp = (dist.log_prob(raw) - torch.log(1 - sq.square() + 1e-6)).sum(-1)
                action = net.action_from_raw(obs, raw)
            nxt, rew, done, _ = env.step(action)
            O.append(obs); A.append(raw); LP.append(logp); R.append(rew); V.append(val); D.append(done)
            obs = nxt
        with torch.no_grad():
            last_v = net.dist_value(obs)[1]
        O, A, LP, R, V, D = [torch.stack(x) for x in (O, A, LP, R, V, D)]
        adv = torch.zeros_like(R); gae = torch.zeros(a.envs, device='cuda')
        for t in reversed(range(len(R))):
            nv = last_v if t == len(R) - 1 else V[t + 1]
            nonterminal = (~D[t]).float()
            gae = R[t] + a.gamma * nv * nonterminal - V[t] + a.gamma * a.lam * nonterminal * gae
            adv[t] = gae
        ret = adv + V
        fo, fa, flp, fadv, fret = [x.reshape((-1,) + x.shape[2:]) for x in (O, A, LP, adv, ret)]
        fadv = (fadv - fadv.mean()) / (fadv.std() + 1e-8)
        for _ in range(a.epochs):
            for ix in torch.randperm(len(fo), device='cuda').split(a.batch):
                dist, val = net.dist_value(fo[ix]); sq = torch.tanh(fa[ix])
                lp = (dist.log_prob(fa[ix]) - torch.log(1 - sq.square() + 1e-6)).sum(-1)
                ratio = (lp - flp[ix]).exp()
                policy = -torch.minimum(ratio * fadv[ix], ratio.clamp(1 - a.clip, 1 + a.clip) * fadv[ix]).mean()
                value = 0.5 * (val - fret[ix]).square().mean()
                loss = policy + a.vf * value - a.ent * dist.entropy().sum(-1).mean()
                opt.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_(net.parameters(), 0.5); opt.step()
        if update % a.eval_every:
            continue
        ev = evaluate(net, kw, a.fit_ok, episodes=a.eval_episodes, seed=10000 + update)
        log(dict(update=update, stage=stage, tol_mm=a.tols[stage], w=a.p_weights[stage], **ev, reward=float(R.sum(0).mean()),
                 gpu_peak_gb=round(torch.cuda.max_memory_allocated() / 2**30, 2), time=time.strftime('%m-%d %H:%M')))
        torch.save(net.state_dict(), out / 'last.pt')
        streak = streak + 1 if ev['pc_success'] >= gates[stage] else 0
        if ev['pc_success'] > best:
            best, best_at = ev['pc_success'], update
            torch.save(net.state_dict(), out / f'stage{stage}_best.pt')
        stuck = a.plateau and update - best_at >= a.plateau
        if streak < a.patience and not stuck:
            continue
        if stuck:
            log(dict(event='plateau', stage=stage, update=update, best=best, best_at=best_at))
        torch.save(net.state_dict(), out / f'stage{stage}.pt')
        snapshot(net, kw, out / f'stage{stage}.png')
        log(dict(event='stage_passed', stage=stage, update=update, **ev, time=time.strftime('%m-%d %H:%M')))
        if stage == len(a.tols) - 1:
            break
        stage += 1; streak, best, best_at = 0, -1.0, update
        kw['pc_tol_mm'] = a.tols[stage]; env.pc_tol = a.tols[stage] / 1000
        net.w.fill_(a.p_weights[stage])
        log(dict(event='stage_start', stage=stage, tol_mm=a.tols[stage], w=a.p_weights[stage], gate=gates[stage],
                 p_controller=evaluate(None, kw, a.fit_ok, episodes=a.eval_episodes), time=time.strftime('%m-%d %H:%M')))
    torch.save(net.state_dict(), out / 'final.pt')
    final = {k: evaluate(net, dict(kw, pc_tol_mm=t), a.fit_ok, seed=777) for k, t in (('tol_final', a.tols[stage]), ('tol_5mm', 5.0))}
    log(dict(event='end', stage=stage, final=final, p_controller_5mm=evaluate(None, dict(kw, pc_tol_mm=5.0), a.fit_ok, seed=777),
             time=time.strftime('%m-%d %H:%M')))


if __name__ == '__main__':
    main()
