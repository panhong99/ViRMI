"""PPO for :mod:`rl.dock_env` and fixed/P/PPO comparison.

Training is entirely kinematic and GPU-vectorised.  Checkpoints and plots are
written under outputs/dock_rl_20261030 and are intentionally not staged.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from dock_env import DockEnv, controller, evaluate_controller  # noqa: E402


class ActorCritic(nn.Module):
    def __init__(self, obs, act):
        super().__init__()
        self.body = nn.Sequential(nn.Linear(obs, 128), nn.Tanh(), nn.Linear(128, 128), nn.Tanh())
        self.actor = nn.Linear(128, act)
        self.critic = nn.Linear(128, 1)
        self.log_std = nn.Parameter(torch.full((act,), -1.5))
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, math.sqrt(2)); nn.init.zeros_(m.bias)
        nn.init.orthogonal_(self.actor.weight, 0.01)
        nn.init.orthogonal_(self.critic.weight, 1.0)

    def dist_value(self, obs):
        h = self.body(obs)
        return torch.distributions.Normal(self.actor(h), self.log_std.exp()), self.critic(h).squeeze(-1)

    def deterministic(self, obs):
        residual = 0.35 * torch.tanh(self.actor(self.body(obs)))
        return (controller(obs, 'p') + residual).clamp(-1, 1)

    def action_from_raw(self, obs, raw):
        residual = 0.35 * torch.tanh(raw)
        return (controller(obs, 'p') + residual).clamp(-1, 1)


def wilson(k, n, z=1.959963984540054):
    p = k / n
    d = 1 + z*z/n
    c = (p + z*z/(2*n)) / d
    h = z * math.sqrt(p*(1-p)/n + z*z/(4*n*n)) / d
    return c-h, c+h


@torch.no_grad()
def evaluate_policy(net, episodes=1024, seed=1024, **env_kwargs):
    env = DockEnv(episodes, seed=seed, **env_kwargs)
    obs = env.reset()
    for _ in range(env.steps - 1):
        obs, _, _, info = env.step(net.deterministic(obs))
    ok = info['success']
    return float(ok.float().mean()), ok.cpu().numpy()


def train(a):
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    env = DockEnv(a.envs, seed=a.seed)
    net = ActorCritic(env.obs_dim, env.act_dim).cuda()
    opt = torch.optim.Adam(net.parameters(), lr=a.lr)

    # Short warm start from the image-space proportional controller.  PPO is
    # still free to depart from it; this prevents wasting the first updates on
    # discovering the camera/action sign convention.
    for _ in range(a.bc_steps):
        obs = env.reset()
        pred = net.deterministic(obs)
        loss = (pred - controller(obs, 'p')).square().mean()
        opt.zero_grad(); loss.backward(); opt.step()

    logs = []
    best_rate = -1.0
    for update in range(a.updates):
        obs = env.reset()
        O=[]; A=[]; LP=[]; R=[]; V=[]; D=[]
        for _ in range(env.steps - 1):
            with torch.no_grad():
                dist, val = net.dist_value(obs)
                raw = dist.sample(); squashed = torch.tanh(raw)
                action = net.action_from_raw(obs, raw)
                logp = (dist.log_prob(raw) - torch.log(1-squashed.square()+1e-6)).sum(-1)
            nxt, rew, done, _ = env.step(action)
            O.append(obs); A.append(raw); LP.append(logp); R.append(rew); V.append(val); D.append(done)
            obs = nxt
        with torch.no_grad(): last_v = net.dist_value(obs)[1]
        O,A,LP,R,V,D = [torch.stack(x) for x in (O,A,LP,R,V,D)]
        adv = torch.zeros_like(R); gae = torch.zeros(a.envs, device='cuda')
        for t in reversed(range(len(R))):
            nv = last_v if t == len(R)-1 else V[t+1]
            nonterminal = (~D[t]).float()
            delta = R[t] + a.gamma*nv*nonterminal - V[t]
            gae = delta + a.gamma*a.lam*nonterminal*gae
            adv[t] = gae
        ret = adv + V
        flat = [x.reshape((-1,) + x.shape[2:]) for x in (O,A,LP,adv,ret)]
        fo, fa, flp, fadv, fret = flat
        fadv = (fadv-fadv.mean())/(fadv.std()+1e-8)
        N=len(fo)
        for _ in range(a.epochs):
            perm=torch.randperm(N, device='cuda')
            for ix in perm.split(a.batch):
                dist,val=net.dist_value(fo[ix]); squashed=torch.tanh(fa[ix])
                lp=(dist.log_prob(fa[ix])-torch.log(1-squashed.square()+1e-6)).sum(-1)
                ratio=(lp-flp[ix]).exp(); s1=ratio*fadv[ix]; s2=ratio.clamp(1-a.clip,1+a.clip)*fadv[ix]
                policy=-torch.minimum(s1,s2).mean(); value=0.5*(val-fret[ix]).square().mean(); ent=dist.entropy().sum(-1).mean()
                loss=policy+a.vf*value-a.ent*ent
                opt.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_(net.parameters(),0.5); opt.step()
        if update % a.eval_every == 0 or update == a.updates-1:
            rate,_=evaluate_policy(net, min(1024,a.envs), seed=10000+update)
            rec=dict(update=update,samples=(update+1)*a.envs*(env.steps-1),success=rate,
                     reward=float(R.sum(0).mean()),policy=float(policy.detach()),
                     value=float(value.detach()),entropy=float(ent.detach()))
            logs.append(rec); print(json.dumps(rec),flush=True)
            torch.save(net.state_dict(),out/'last.pt')
            if rate > best_rate:
                best_rate = rate
                torch.save(net.state_dict(), out/'best.pt')
            (out/'train.jsonl').write_text('\n'.join(json.dumps(x) for x in logs)+'\n')
    net.load_state_dict(torch.load(out/'best.pt', map_location='cuda', weights_only=True))
    return net,logs


@torch.no_grad()
def evaluate_details(name, net, episodes, seed, env_kwargs):
    env=DockEnv(episodes,seed=seed,**env_kwargs); obs=env.reset()
    for _ in range(env.steps-1):
        action=net.deterministic(obs) if name=='ppo' else controller(obs,name)
        obs,_,_,info=env.step(action)
    ok=info['success']; x,y,h,a,ang=env.criteria()
    _,_,_,bb,rx,yc,_=env._geometry()
    lo,hi=wilson(int(ok.sum()),len(ok))
    return dict(method=name,success=float(ok.float().mean()),lo95=lo,hi95=hi,n=len(ok),
                pass_x=float(x.float().mean()),pass_y=float(y.float().mean()),
                pass_height=float(h.float().mean()),pass_angle=float(a.float().mean()),
                median_x_mm=float((bb[:,0]-rx).abs().median()*1000),
                median_y_mm=float((bb[:,1]-yc).abs().median()*1000),
                median_height_mm=float((bb[:,2]-env.floor_z).median()*1000),
                median_angle_deg=float(ang.median()*180/math.pi))


@torch.no_grad()
def pitch_reachability(episodes=4096,seed=41024):
    env=DockEnv(episodes,seed=seed); env.reset(); env.t.fill_(env.path_steps-1)
    minimum=torch.full((episodes,),float('inf'),device='cuda')
    for deg in torch.linspace(-15,15,61,device='cuda'):
        env.residual[:,2]=torch.deg2rad(deg)
        minimum=torch.minimum(minimum,env.criteria()[4])
    return dict(n=episodes,pass_angle_reachable=float((minimum<math.radians(20)).float().mean()),
                median_min_angle_deg=float(minimum.median()*180/math.pi),
                p90_min_angle_deg=float(torch.quantile(minimum,.9)*180/math.pi))


@torch.no_grad()
def export_physics_trajectories(net, prefix, episodes=1024, seed=51024):
    """Export matched P/PPO residual paths for hand_basket.py physics replay."""
    prefix=Path(prefix); prefix.parent.mkdir(parents=True,exist_ok=True)
    kwargs=dict(basket_xy_mm=0,basket_yaw_deg=0,pitch_frac=0,lean_deg=0)
    outputs=[]
    for method in ('p','ppo'):
        env=DockEnv(episodes,seed=seed,**kwargs); obs=env.reset()
        residual=torch.zeros(episodes,env.path_steps,3,device='cuda')
        residual[:,0]=env.residual
        for _ in range(env.steps-1):
            action=net.deterministic(obs) if method=='ppo' else controller(obs,'p')
            obs,_,_,_=env.step(action)
            residual[torch.arange(episodes,device='cuda'),env.t]=env.residual
        path=prefix.with_name(prefix.name+f'_{method}.npz')
        np.savez_compressed(path,method=method,seed=seed,job=env.job.cpu().numpy(),
                            H=env.H.cpu().numpy(),residual=residual.cpu().numpy(),
                            names=np.array([f'Q3_job_{i}' for i in range(len(env.paths))]),
                            env_kwargs=json.dumps(kwargs))
        outputs.append(str(path))
    return outputs


def report(net, logs, out, episodes=1024, seed=1024):
    out=Path(out); rows=[]
    conditions={
        'wide':{},
        'narrow_h':dict(hang_deg=5,grip_mm=5,tilt_deg=2,basket_xy_mm=0,basket_yaw_deg=0),
    }
    for condition,kwargs in conditions.items():
        for name in ('fixed','p','ppo'):
            row=evaluate_details(name,net,episodes,seed,kwargs); row['condition']=condition; rows.append(row)
    (out/'comparison.json').write_text(json.dumps(rows,indent=1))
    cols=('condition','method','success','lo95','hi95','n','pass_x','pass_y','pass_height','pass_angle',
          'median_x_mm','median_y_mm','median_height_mm','median_angle_deg')
    (out/'comparison.tsv').write_text('\t'.join(cols)+'\n'+''.join(
        '\t'.join(str(r[c]) for c in cols)+'\n' for r in rows))
    reach=pitch_reachability(); (out/'pitch_reachability.json').write_text(json.dumps(reach,indent=1))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(6.4,3.6)); ax.plot([r['samples'] for r in logs],[r['success'] for r in logs],lw=1.8)
    ax.set_xlabel('environment steps'); ax.set_ylabel('kinematic strict success')
    ax.spines[['top','right']].set_visible(False); fig.tight_layout(); fig.savefig(out/'learning_curve.png',dpi=180); plt.close(fig)
    print(json.dumps(dict(comparison=rows,pitch_reachability=reach),indent=1)); return rows


def main():
    p=argparse.ArgumentParser(); p.add_argument('--out',default='outputs/dock_rl_20261030')
    p.add_argument('--envs',type=int,default=2048); p.add_argument('--updates',type=int,default=40)
    p.add_argument('--epochs',type=int,default=3); p.add_argument('--batch',type=int,default=8192)
    p.add_argument('--bc_steps',type=int,default=100); p.add_argument('--lr',type=float,default=3e-4)
    p.add_argument('--gamma',type=float,default=.99); p.add_argument('--lam',type=float,default=.95)
    p.add_argument('--clip',type=float,default=.2); p.add_argument('--vf',type=float,default=.5); p.add_argument('--ent',type=float,default=.002)
    p.add_argument('--eval_every',type=int,default=2); p.add_argument('--seed',type=int,default=0)
    p.add_argument('--checkpoint',default=''); p.add_argument('--eval_only',action='store_true')
    p.add_argument('--export_physics',default='',help='prefix for matched _p.npz and _ppo.npz replay bundles')
    a=p.parse_args(); out=Path(a.out); out.mkdir(parents=True,exist_ok=True)
    if a.eval_only:
        net=ActorCritic(DockEnv.obs_dim,DockEnv.act_dim).cuda(); net.load_state_dict(torch.load(a.checkpoint,map_location='cuda',weights_only=True)); logs=[]
    else: net,logs=train(a)
    report(net,logs,out)
    if a.export_physics:
        print(json.dumps(dict(physics_trajectories=export_physics_trajectories(net,a.export_physics)),indent=1))


if __name__=='__main__': main()
