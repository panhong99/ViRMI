"""Basket insertion training (2026-10-13).

    python rl/basket/train.py bc --data IL/data_basket_v2_<date> [--seed 1] [--preflight-only]
    python rl/basket/train.py rl --bc <bc.pt>           (residual PPO, Isaac; added after env.py / expert.py)
    python rl/basket/train.py rl --bc <bc.pt> --starts <dir> --envs 9 --view <rl best.pt | bc>   (GUI, real time)

bc: behaviour cloning of the sim feedback expert (model.BCPolicy, MSE on the 6 TCP steps + BCE on 'open', AdamW,
cosine lr).  Safeguards (config.yaml preflight / bc): data check, tiny-batch overfit test, early stop on the
validation loss (patience), best-val checkpoint.  Curves: W&B (one run per seed, seeds grouped) + a PNG at the end.
"""
import argparse
import glob
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import model as M  # noqa: E402

CFG = yaml.safe_load((HERE / 'config.yaml').read_text())
ROOT = HERE.parents[1]


# ---------------------------------------------------------------- data
def load_demos(data_dir):
    """batch_*.npz with obs [E, T, 17], act [E, T, 7] (m / rad per step + open), mask [E, T], success [E].
    -> list of episodes (obs [t, 17], act [t, 7]) of the successful demos only."""
    eps = []
    for f in sorted(glob.glob(str(Path(data_dir) / 'batch_*.npz'))):
        z = np.load(f)
        for e in range(len(z['success'])):
            if z['success'][e]:
                t = z['mask'][e].astype(bool)
                eps.append((z['obs'][e][t].astype(np.float32), z['act'][e][t].astype(np.float32)))
    return eps


def check_data(eps):
    """Preflight 1: refuse data that cannot train (wrong dims, NaN, actions beyond the speed caps, too few demos)."""
    n_obs, n_act = len(CFG['obs']['names']), len(CFG['action']['names'])
    step_max = np.array(CFG['action']['step_max'])
    problems = []
    if len(eps) < 100:
        problems.append(f'only {len(eps)} successful episodes')
    for i, (o, a) in enumerate(eps):
        if o.shape[1] != n_obs or a.shape[1] != n_act:
            problems.append(f'episode {i}: obs {o.shape} act {a.shape}')
            break
        if not (np.isfinite(o).all() and np.isfinite(a).all()):
            problems.append(f'episode {i}: NaN / inf')
            break
        if (np.abs(a[:, :6]) > step_max * 1.001).any():
            problems.append(f'episode {i}: action beyond the speed caps')
            break
    frames = sum(len(o) for o, _ in eps)
    print(f'[data] {len(eps)} episodes, {frames} frames' + (f'  PROBLEMS: {problems}' if problems else '  ok'), flush=True)
    if problems:
        raise SystemExit('data check failed: ' + '; '.join(problems))


def tensors(eps, idx, device):
    o = np.concatenate([eps[i][0] for i in idx])
    a = np.concatenate([eps[i][1] for i in idx])
    step_max = np.array(CFG['action']['step_max'], np.float32)
    cont = np.clip(a[:, :6] / step_max, -1, 1)                      # targets in step units
    above = o[:, CFG['obs']['names'].index('above_rail')]
    w = np.where(above <= CFG['bc']['near_pin_m'], CFG['bc']['near_pin_weight'], 1.0).astype(np.float32)
    # (10/12 night) the recorded 'open' was the released STATE (1 only after the opening): label the decision instead,
    # the expert's rule = open once the lowest rim point is at target_low (obs low_z) or already open
    low = o[:, CFG['obs']['names'].index('low_z')]
    grip = np.maximum(a[:, 6], (low <= CFG['expert']['target_low_m']).astype(np.float32))
    T = lambda x: torch.as_tensor(x, device=device)
    return T(o), T(cont), T(grip), T(w)


# ---------------------------------------------------------------- loss / metrics
def loss_fn(policy, o, cont, grip, w):
    pc, pg = policy(o)
    mse = ((pc - cont) ** 2).mean(-1)
    bce = torch.nn.functional.binary_cross_entropy_with_logits(pg, grip, reduction='none')
    return ((mse + CFG['bc']['grip_loss_weight'] * bce) * w).sum() / w.sum(), mse.mean(), bce.mean()


@torch.no_grad()
def metrics(policy, o, cont, grip, w, mean_baseline):
    loss, mse, bce = loss_fn(policy, o, cont, grip, w)
    pc, pg = policy(o)
    tol = torch.as_tensor(CFG['bc']['acc_tol'], device=o.device) / policy.step_max
    acc = ((pc - cont).abs() <= tol).all(-1).float().mean()
    grip_acc = ((pg > 0).float() == grip).float().mean()
    r2 = 1 - mse / mean_baseline                                     # vs "always predict the mean action"
    return dict(loss=loss.item(), mse=mse.item(), bce=bce.item(), acc=acc.item(), grip_acc=grip_acc.item(), r2=r2.item())


# ---------------------------------------------------------------- tracking
def start_wandb(name, group, config):
    """One W&B run per training job (offline when not logged in).  Returns None if wandb is missing."""
    local = ROOT / '.runtime' / 'wandb'                       # Isaac python: wandb installed here (rl/train.py does the same)
    if local.exists() and str(local) not in sys.path:
        sys.path.insert(0, str(local))
    try:
        import wandb
        wandb.init                                             # a stray 'wandb' run folder on the path is not the package
    except (ImportError, AttributeError):
        print('[wandb] not available here: curves only as PNG / history.json', flush=True)
        return None
    mode = os.environ.get('WANDB_MODE', CFG['wandb']['mode'])
    if mode == 'auto':
        mode = 'online' if wandb.api.api_key else 'offline'
    return wandb.init(project=CFG['wandb']['project'], name=name, group=group, config=config, mode=mode,
                      dir=str(ROOT / 'rl' / 'logs' / 'basket'), reinit=True)


def plot_curves(hist, path):
    """Train vs val loss on one axis, accuracy / R2 and lr beside it (style: no grid, no top/right spines, no title)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
    s = [h['step'] for h in hist]
    ax[0].plot(s, [h['train/loss'] for h in hist], label='train')
    ax[0].plot(s, [h['val/loss'] for h in hist], label='val')
    ax[0].set_yscale('log'); ax[0].set_xlabel('step'); ax[0].set_ylabel('loss'); ax[0].legend(frameon=False)
    ax[1].plot(s, [h['val/acc'] for h in hist], label='action acc')
    ax[1].plot(s, [h['val/grip_acc'] for h in hist], label='grip acc')
    ax[1].plot(s, [h['val/r2'] for h in hist], label='R2')
    ax[1].set_xlabel('step'); ax[1].set_ylim(0, 1.02); ax[1].legend(frameon=False)
    ax[2].plot(s, [h['lr'] for h in hist]); ax[2].set_xlabel('step'); ax[2].set_ylabel('lr'); ax[2].set_yscale('log')
    for a in ax:
        a.spines[['top', 'right']].set_visible(False)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------- BC
def overfit_test(eps, device, seed):
    """Preflight 2: a fresh net must memorise a tiny batch; if it cannot, the pipeline (not the data size) is wrong."""
    c = CFG['bc']
    torch.manual_seed(seed)
    p = M.BCPolicy(len(CFG['obs']['names']), CFG['action']['step_max'], c['hidden'], c['activation']).to(device)
    o, cont, grip, w = tensors(eps, range(len(eps)), device)
    sel = torch.randperm(len(o), device=device)[:CFG['preflight']['overfit_frames']]
    p.norm.fit(o)
    opt = torch.optim.AdamW(p.parameters(), lr=c['lr'])
    for _ in range(CFG['preflight']['overfit_steps']):
        loss, mse, _ = loss_fn(p, o[sel], cont[sel], grip[sel], w[sel])
        opt.zero_grad(); loss.backward(); opt.step()
    ok = mse.item() <= CFG['preflight']['overfit_max_loss']
    print(f'[preflight] tiny-batch overfit: mse {mse.item():.4f} (need <= {CFG["preflight"]["overfit_max_loss"]}) '
          f'-> {"ok" if ok else "FAILED"}', flush=True)
    if not ok:
        raise SystemExit('preflight failed: the net cannot memorise 256 frames (check obs / action / loss)')


def run_bc(args):
    c = CFG['bc']
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    eps = load_demos(args.data)
    check_data(eps)
    overfit_test(eps, dev, args.seed)
    if args.preflight_only:
        return
    rng = np.random.default_rng(args.seed)
    order = rng.permutation(len(eps))
    n_val = max(1, int(len(eps) * c['val_fraction']))
    tr, va = order[n_val:], order[:n_val]
    o, cont, grip, w = tensors(eps, tr, dev)
    vo, vcont, vgrip, vw = tensors(eps, va, dev)
    base_val = ((vcont - cont.mean(0)) ** 2).mean()                  # R2 reference

    torch.manual_seed(args.seed)
    policy = M.BCPolicy(o.shape[1], CFG['action']['step_max'], c['hidden'], c['activation']).to(dev)
    policy.norm.fit(o)
    opt = torch.optim.AdamW(policy.parameters(), lr=c['lr'], weight_decay=c['weight_decay'])
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, c['max_steps'], eta_min=c['lr_min'])
    out = ROOT / 'rl' / 'logs' / 'basket' / f'bc_{Path(args.data).name}' / f'seed{args.seed}_{time.strftime("%Y%m%d_%H%M%S")}'
    out.mkdir(parents=True, exist_ok=True)
    meta = dict(obs_dim=o.shape[1], step_max=CFG['action']['step_max'], hidden=c['hidden'], activation=c['activation'],
                obs_names=CFG['obs']['names'], data=str(args.data), seed=args.seed)
    run = start_wandb(f'bc_seed{args.seed}', f'bc_{Path(args.data).name}', dict(bc=c, data=str(args.data), seed=args.seed,
                                                                                 train_eps=len(tr), val_eps=len(va)))
    hist, best, best_step, t0 = [], math.inf, 0, time.time()
    log_every = 250
    for step in range(1, c['max_steps'] + 1):
        idx = torch.randint(0, len(o), (c['batch'],), device=dev)
        loss, _, _ = loss_fn(policy, o[idx], cont[idx], grip[idx], w[idx])
        opt.zero_grad(); loss.backward()
        gn = torch.nn.utils.clip_grad_norm_(policy.parameters(), c['grad_clip'])
        opt.step(); sched.step()
        if step % log_every == 0 or step == c['max_steps']:
            policy.eval()
            tm = metrics(policy, o[idx], cont[idx], grip[idx], w[idx], base_val)
            vm = metrics(policy, vo, vcont, vgrip, vw, base_val)
            policy.train()
            rec = {'step': step, 'epoch': step * c['batch'] / len(o), 'lr': sched.get_last_lr()[0], 'grad_norm': gn.item(),
                   **{f'train/{k}': v for k, v in tm.items()}, **{f'val/{k}': v for k, v in vm.items()}}
            hist.append(rec)
            if run:
                run.log(rec, step=step)
            if vm['loss'] < best * (1 - c['min_delta']):
                best, best_step = vm['loss'], step
                M.save(out / 'best.pt', policy, dict(meta, step=step, val=vm))
            if step % 2500 == 0:
                print(f'[bc] step {step} ep {rec["epoch"]:.1f} train {tm["loss"]:.4f} val {vm["loss"]:.4f} '
                      f'acc {vm["acc"]:.3f} grip {vm["grip_acc"]:.3f} R2 {vm["r2"]:.3f} lr {rec["lr"]:.1e} '
                      f'({time.time() - t0:.0f} s)', flush=True)
            if step - best_step >= c['patience_steps']:
                print(f'[bc] early stop: no val improvement since step {best_step}', flush=True)
                break
    M.save(out / 'last.pt', policy, dict(meta, step=step))
    plot_curves(hist, out / 'curves.png')
    (out / 'history.json').write_text(json.dumps(hist))
    last = hist[-1]
    verdict = ('underfit (R2 < 0.5)' if last['val/r2'] < 0.5 else
               'overfit (val rose > 20 % above its best)' if last['val/loss'] > best * 1.2 else 'ok')
    summary = dict(best_val=best, best_step=best_step, stopped=step, verdict=verdict, out=str(out))
    (out / 'summary.json').write_text(json.dumps(summary, indent=1))
    print(f'[bc] done: {summary}', flush=True)
    if run:
        run.summary.update(summary)
        run.finish()


# ---------------------------------------------------------------- residual PPO (Isaac)
class StartBank:
    """RL start states saved by expert.py collect --save_starts: robot at `above`, plate attached (H), basket pose."""

    def __init__(self, folder, device):
        zs = [np.load(f) for f in sorted(glob.glob(str(Path(folder) / 'starts_*.npz')))]
        keep = [(z, z['live'].astype(bool)) for z in zs]
        cat = lambda k: np.concatenate([z[k][m] for z, m in keep])          # noqa: E731
        self.q, self.basket = cat('q'), cat('basket')
        self.att_p, self.att_q = torch.as_tensor(cat('att_p'), device=device), torch.as_tensor(cat('att_q'), device=device)
        self.n = len(self.q)
        print(f'[rl] {self.n} start states from {len(zs)} files', flush=True)


class RLRunner:
    """Vectorised episodes from the start bank: obs (env.observe) -> residual policy -> slot-frame step + open.
    Reward per step in [-1, 1], terminal +-10 (config reward)."""

    def __init__(self, env, bank, rng):
        import math
        import expert as X
        import env as E
        self.env, self.bank, self.rng, self.X, self.E = env, bank, rng, X, E
        self.n, self.dev = env.num_envs, env.device
        self.robot = env.scene['robot']
        self.arm = env.action_manager.get_term('arm_action')
        self.aj = self.arm._joint_ids
        self.arm.shield.safe = lambda q_: torch.ones(q_.shape[0], dtype=torch.bool, device=q_.device)
        import basket_env as be
        self.be = be
        self.lean = math.tan(math.radians(be.B['pins'].get('lean_deg', 0.0)))
        self.released = torch.zeros(self.n, dtype=torch.bool, device=self.dev)
        self.t_rel = torch.zeros(self.n, device=self.dev)
        self.t_ep = torch.zeros(self.n, device=self.dev)
        self.prev_low = torch.zeros(self.n, device=self.dev)
        env._attach = dict(on=torch.zeros(self.n, dtype=torch.bool, device=self.dev),
                           p=torch.zeros(self.n, 3, device=self.dev), q=torch.zeros(self.n, 4, device=self.dev))
        self.reset(torch.arange(self.n, device=self.dev))

    def reset(self, ids):
        if len(ids) == 0:
            return
        k = self.rng.integers(0, self.bank.n, len(ids))
        q = torch.as_tensor(self.bank.q[k], dtype=torch.float32, device=self.dev)
        self.robot.write_joint_state_to_sim(q, torch.zeros_like(q), env_ids=ids)
        self.robot.set_joint_position_target(q, env_ids=ids)
        self.X.set_basket(self.env, list(self.bank.basket[k]), ids=ids.tolist())
        a = self.env._attach
        a['p'][ids], a['q'][ids], a['on'][ids] = self.bank.att_p[k], self.bank.att_q[k], True
        self.arm.hold_init[ids] = 2
        self.arm.orientation_locked[ids] = False
        self.arm._safe_target[ids] = q[:, self.aj]
        self.released[ids], self.t_rel[ids], self.t_ep[ids] = False, 0.0, 0.0
        self.X.attach_update(self.env)
        st = self.be.basket_state(self.env)
        self.prev_low[ids] = st['low'][ids]

    def obs(self):
        o, st = self.E.observe(self.env, self.released)
        return o, st

    def step(self, action):
        """action (n, 7) in policy units (6 steps in [-1, 1] after tanh-free clip + open logit) -> reward, done, success."""
        import plate_env as pe
        R = CFG['reward']
        dt = self.env.step_dt
        sm = torch.as_tensor(CFG['action']['step_max'], device=self.dev)
        cont = action[:, :6].clamp(-1, 1)
        opening = (action[:, 6] > 0) & ~self.released
        exe = cont * sm
        exe[self.released] = 0.0
        exe[self.released & (self.t_rel > 1.0), 2] = sm[2]                         # after the release: rise
        open_ = (self.released | opening).float()
        act = self.E.to_env_action(self.env, exe, open_)
        act[self.env._attach['on'], 6] = 1.0
        self.env.step(act)
        self.X.attach_update(self.env)
        land = opening
        self.released |= land
        self.X.detach(self.env, land)
        self.t_rel = torch.where(self.released, self.t_rel + dt, self.t_rel)
        self.t_ep += dt
        st = self.be.basket_state(self.env)
        gap = (st['d_bottom'] + st['low'].clamp(min=0) * self.lean).abs()
        nz = pe.body_axis(self.env.scene['plate'].data.root_quat_w, 2)[:, 2].abs()
        aligned = gap < 0.003
        descent = (self.prev_low - st['low']).clamp(min=0) * aligned.float() * (~self.released).float()
        self.prev_low = st['low'].clone()
        r = (-R['w_gap'] * (gap + 0.5 * st['d_u'].abs()) - R['w_tilt'] * nz - R['w_action'] * (cont ** 2).mean(1)
             + R['w_progress'] * descent / dt * 0.1) * dt * 3.0
        r = r.clamp(-1.0, 1.0)
        done = torch.zeros(self.n, dtype=torch.bool, device=self.dev)
        success = torch.zeros(self.n, dtype=torch.bool, device=self.dev)
        judge = self.released & (self.t_rel >= 1.0)                                # settled 1 s after the release
        vel = self.env.scene['plate'].data.root_lin_vel_w.norm(dim=1)
        ok = judge & (gap < 0.0085) & (st['low'] < 0.060) & (st['d_u'].abs() < 0.03) & (nz < 0.4226) & (vel < 0.02)
        success |= ok
        fail = (judge & ~ok) | (self.t_ep > 25.0) | (st['low'] > 0.6)
        done = ok | fail
        r = torch.where(ok, torch.full_like(r, R['success']), r)
        r = torch.where(fail & ~ok, torch.full_like(r, R['failure']), r)
        return r, done, success


def run_rl(args):
    """Residual PPO on the frozen BC from saved start states.  Safeguards (config rl): iteration-0 deterministic eval =
    BC baseline, eval every eval_every, patience, one rollback, value-loss / KL / std / NaN stops, wall-clock cap."""
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=not args.view).app
    from isaaclab.envs import ManagerBasedRLEnv
    sys.path.insert(0, str(HERE.parent))
    import expert as X
    c = CFG['rl']
    dev = 'cuda'
    env = ManagerBasedRLEnv(X.make_cfg(args.envs or c['envs'], False, CFG['expert'].get('pad_torsion_radius_m', 0.01)))
    env._basket_tracking_mode = True  # actions/BC labels are live-state tracking corrections
    env.reset()

    # bc==behavior cloning
    bc, meta = M.load_bc(args.bc, device=dev)
    ac = M.ResidualActorCritic(bc, c['residual_scale'], meta['hidden'], meta['activation'], c['init_std']).to(dev)
    c['lr'] = args.lr or c['lr']
    c['patience_evals'] = args.patience or c['patience_evals']
    opt = torch.optim.Adam([p for p in ac.parameters() if p.requires_grad], lr=c['lr'])
    rng = np.random.default_rng(args.seed)
    runner = RLRunner(env, StartBank(args.starts, dev), rng)
    out = ROOT / 'rl' / 'logs' / 'basket' / f'rl_{time.strftime("%Y%m%d_%H%M%S")}'
    if not (args.view or args.eval_ckpt):                   # no folder / W&B run for viewing or re-evaluation
        out.mkdir(parents=True, exist_ok=True)
    run = None if (args.view or args.eval_ckpt) else start_wandb(f'rl_seed{args.seed}', 'rl_residual', dict(rl=c, reward=CFG['reward'], bc=args.bc))
    T, n = c['num_steps_per_env'], env.num_envs

    def evaluate(n_ep=None):
        """Deterministic (mean action) episodes until n_ep finished; returns success rate."""
        n_ep = n_ep or CFG['eval']['envs']
        runner.reset(torch.arange(n, device=env.device))
        done_n, succ_n, rets, acc = 0, 0, [], torch.zeros(n, device=env.device)
        while done_n < n_ep:
            o, _ = runner.obs()
            with torch.no_grad():
                a = ac.mean(o)
            r, d, s_ = runner.step(a)
            acc += r
            done_n += int(d.sum()); succ_n += int((d & s_).sum())
            rets += acc[d].tolist(); acc[d] = 0.0
            runner.reset(torch.nonzero(d).squeeze(1))
        evaluate.ret = float(np.mean(rets)) if rets else float('nan')   # mean episode return (undiscounted sum of r)
        return succ_n / max(done_n, 1)

    if args.view:                                              # GUI: BC (+ residual) episodes at real time, until closed
        if args.view != 'bc':
            ac.load_state_dict(torch.load(args.view, map_location=dev)['state'])
        runner.reset(torch.arange(n, device=env.device))
        done_n, succ_n = 0, 0
        while app.is_running():
            t_s = time.time()
            o, _ = runner.obs()
            with torch.no_grad():
                a = ac.mean(o)
            r, d, s_ = runner.step(a)
            for k in torch.nonzero(d).squeeze(1).tolist():
                done_n += 1; succ_n += int(s_[k])
                print(f'[view] env {k}: {"성공" if s_[k] else "실패"}  (누적 {succ_n}/{done_n} = {succ_n / done_n:.2f})', flush=True)
            runner.reset(torch.nonzero(d).squeeze(1))
            time.sleep(max(0.0, env.step_dt - (time.time() - t_s)))
        env.close()
        app.close()
        return
    if args.eval_ckpt:                                         # re-evaluate a saved residual policy (and the BC) only
        bc_rate = evaluate()
        bc_ret = evaluate.ret
        ac.load_state_dict(torch.load(args.eval_ckpt, map_location=dev)['state'])
        rl_rate = evaluate()
        print(f'[rl] re-eval ({CFG["eval"]["envs"]} episodes, seed {args.seed}): success BC {bc_rate:.3f}  RL {rl_rate:.3f}; '
              f'mean episode return BC {bc_ret:+.2f}  RL {evaluate.ret:+.2f}  ({args.eval_ckpt})', flush=True)
        env.close()
        app.close()
        return
    if args.init_ckpt:                                         # continue from a saved residual policy; the baseline stays the BC
        base = evaluate()
        ac.load_state_dict(torch.load(args.init_ckpt, map_location=dev)['state'])
        print(f'[rl] continue from {args.init_ckpt} (BC baseline {base:.3f})', flush=True)
    else:
        base = evaluate()
    print(f'[rl] iteration 0 deterministic success (= BC): {base:.3f}', flush=True)
    hist, best, best_it, rollbacks, v0, vema, t0 = [], base, 0, 0, None, 0.0, time.time()
    M.save(out / 'best.pt', ac, dict(meta, it=0, success=base))
    runner.reset(torch.arange(n, device=env.device))
    o, _ = runner.obs()
    ep_acc, ep_len = torch.zeros(n, device=dev), torch.zeros(n, device=dev)   # running return / length per env
    for it in range(1, c['max_iters'] + 1):
        O, A, LP, V, Rw, D = [], [], [], [], [], []
        ep_succ, ep_done, rets, lens = 0, 0, [], []
        for _ in range(T):
            with torch.no_grad():
                dist = ac.dist(o)
                a = dist.sample()
                lp, v = dist.log_prob(a).sum(-1), ac.value(o)
            r, d, s_ = runner.step(a)
            O.append(o); A.append(a); LP.append(lp); V.append(v); Rw.append(r); D.append(d.float())
            ep_done += int(d.sum()); ep_succ += int((d & s_).sum())
            ep_acc += r; ep_len += 1
            rets += ep_acc[d].tolist(); lens += ep_len[d].tolist()
            ep_acc[d], ep_len[d] = 0.0, 0.0
            runner.reset(torch.nonzero(d).squeeze(1))
            o, _ = runner.obs()
        with torch.no_grad():
            v_last = ac.value(o)
        O, A, LP, V, Rw, D = map(torch.stack, (O, A, LP, V, Rw, D))
        adv = torch.zeros_like(Rw)
        g = torch.zeros(n, device=dev)
        for t in reversed(range(T)):
            nv = v_last if t == T - 1 else V[t + 1]
            delta = Rw[t] + c['gamma'] * nv * (1 - D[t]) - V[t]
            g = delta + c['gamma'] * c['lam'] * (1 - D[t]) * g
            adv[t] = g
        ret = adv + V
        O, A, LP, adv, ret = (x.reshape(T * n, *x.shape[2:]) for x in (O, A, LP, adv, ret))
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        kl_m, pl_m, vl_m = 0.0, 0.0, 0.0
        mb = T * n // c['mini_batches']
        for ep in range(c['learning_epochs']):
            perm = torch.randperm(T * n, device=dev)
            for i in range(c['mini_batches']):
                j = perm[i * mb:(i + 1) * mb]
                dist = ac.dist(O[j])
                lp = dist.log_prob(A[j]).sum(-1)
                ratio = (lp - LP[j]).exp()
                pl = -torch.min(ratio * adv[j], ratio.clamp(1 - c['clip'], 1 + c['clip']) * adv[j]).mean()
                vl = ((ac.value(O[j]) - ret[j]) ** 2).mean()
                loss = pl + 0.5 * vl - c['entropy'] * dist.entropy().sum(-1).mean()
                opt.zero_grad(); loss.backward()
                torch.nn.utils.clip_grad_norm_(ac.parameters(), c['max_grad_norm'])
                opt.step()
                with torch.no_grad():
                    kl = (LP[j] - lp).mean().item()
                kl_m, pl_m, vl_m = kl, pl.item(), vl.item()
            if kl_m > 1.5 * c['desired_kl']:
                break
        # value loss: sparse +-10 terminals make single iterations spike (stopped twice on noise, eval was +4 pp over the
        # BC) -> watch a 10-iteration EMA against the warm-up (first 20 iterations) mean
        vema = vl_m if it == 1 else 0.9 * vema + 0.1 * vl_m
        if it <= 20:
            v0 = ((v0 or 0.0) * (it - 1) + vl_m) / it
        std = ac.log_std.exp().mean().item()
        rec = dict(it=it, reward=Rw.mean().item(), rollout_success=ep_succ / max(ep_done, 1), episodes=ep_done,
                   **(dict(episode_return=float(np.mean(rets)), episode_len_s=float(np.mean(lens)) * env.step_dt) if rets else {}),
                   value_loss=vl_m, policy_loss=pl_m, kl=kl_m, action_std=std, lr=c['lr'])
        stop = None
        if not all(np.isfinite(v) for v in (rec['reward'], vl_m, pl_m)):
            stop = 'NaN'
        elif it > 20 and vema > c['stop_value_loss_x'] * v0:
            stop = f'value loss EMA {vema:.3g} > {c["stop_value_loss_x"]} x warm-up mean {v0:.3g}'
        elif kl_m > c['stop_kl']:
            stop = f'KL {kl_m:.3f}'
        elif not (c['std_range'][0] <= std <= c['std_range'][1]):
            stop = f'action std {std:.3f}'
        elif time.time() - t0 > c['wall_hours'] * 3600:
            stop = 'wall-clock cap'
        if it % c['eval_every'] == 0 or stop:
            succ = evaluate()
            rec['eval_success'], rec['eval_episode_return'] = succ, evaluate.ret
            print(f'[rl] it {it}: eval {succ:.3f} (BC {base:.3f}, best {best:.3f} @ {best_it})', flush=True)
            if succ > best:
                best, best_it = succ, it
                M.save(out / 'best.pt', ac, dict(meta, it=it, success=succ))
            elif succ < base - c['max_drop_vs_bc']:
                rollbacks += 1
                if rollbacks > 1:
                    stop = stop or 'second drop below the BC'
                else:
                    ac.load_state_dict(torch.load(out / 'best.pt')['state'])
                    print('[rl] dropped below the BC: rolled back to the best checkpoint (once)', flush=True)
            if not stop and (it - best_it) >= c['patience_evals'] * c['eval_every']:
                stop = f'no new best for {c["patience_evals"]} evals'
            runner.reset(torch.arange(n, device=env.device))
            o, _ = runner.obs()
            ep_acc[:], ep_len[:] = 0.0, 0.0
        hist.append(rec)
        if run:
            run.log(rec, step=it)
        if it % 10 == 0:
            print(f'[rl] it {it} r {rec["reward"]:+.3f} succ {rec["rollout_success"]:.2f} ({ep_done} ep) vl {vl_m:.3g} '
                  f'kl {kl_m:.4f} std {std:.3f} ({time.time() - t0:.0f} s)', flush=True)
        if stop:
            print(f'[rl] STOP: {stop}', flush=True)
            break
    summary = dict(bc_success=base, best=best, best_it=best_it, stopped=it, reason=stop or 'max iters', out=str(out))
    (out / 'summary.json').write_text(json.dumps(summary, indent=1))
    (out / 'history.json').write_text(json.dumps(hist))
    print(f'[rl] done: {summary}', flush=True)
    if run:
        run.summary.update(summary)
        run.finish()
    env.close()
    app.close()


# ---------------------------------------------------------------- (10/15 보스) insert-correction PPO (one decision / insert)
def insert_reward(exact, seated, in_quadrant, landed, target, unseated_penalty=0.0):
    """1 = seated in the target gap; optional terminal penalty for not seating the plate."""
    miss = np.where(landed >= 0, np.minimum(np.abs(landed - target), 4), 4)
    return (exact.astype(float) + 0.2 * (seated & in_quadrant & ~exact) - 0.05 * miss
            - float(unseated_penalty) * (~seated))


FINAL_REPLAY_ARGS = ['--closed_loop', '--cl_top', '--cl_top_q', 'Q1:0', '--cl_iter', '2', '--cl_dx_q', 'Q1:0.025,Q2:0.038,Q3:-0.0464,Q4:-0.035',
                     '--cl_lean_q', 'Q1:0.5,Q2:0,Q3:-0.5,Q4:0.5']   # = rl/basket/replay_final.sh


def code_snapshot(out):
    """(10/15 22:0x) frozen copy of the COMMITTED rl/ code for the replays: a run died when expert.py was half-edited
    by another agent.  calibration/ and rl/assets, rl/logs are symlinks (data, not code)."""
    import subprocess
    snap = out / 'code'
    if not snap.exists():
        snap.mkdir(parents=True)
        files = subprocess.run(['git', 'ls-files', 'rl/*.py', 'rl/*.yaml', 'rl/basket'], cwd=ROOT, capture_output=True, text=True).stdout.split()
        arch = subprocess.run(['git', 'archive', 'HEAD', *files], cwd=ROOT, capture_output=True).stdout
        subprocess.run(['tar', '-x', '-C', str(snap)], input=arch, check=True)
        import shutil
        for d in ('rl/assets', 'rl/logs'):                  # data folders: git pathspecs also matched some files inside
            if (snap / d).exists():
                shutil.rmtree(snap / d)
        for top in ROOT.iterdir():                          # everything outside rl/ (calibration, assets, IL, ...) = live
            if top.name not in ('rl', '.git') and not (snap / top.name).exists():
                (snap / top.name).symlink_to(top)
        for sub in (ROOT / 'rl').iterdir():                 # rl/ data and anything not archived (assets, logs, ...)
            if not (snap / 'rl' / sub.name).exists():
                (snap / 'rl' / sub.name).symlink_to(sub)
        (snap / 'commit.txt').write_text(subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True).stdout)
    return snap


def insert_replay(plans, out_dir, tag, policy_path, seed, det, per_run, extra, code=None, unseated_penalty=0.0):
    """One Isaac replay (subprocess, final per-quadrant settings) with the policy -> transitions npz."""
    import subprocess
    side = (out_dir / f'{tag}.npz').resolve()
    io = ['--plans', str(Path(plans).resolve()), '--out', str((out_dir / f'{tag}.json').resolve()), '--per_run', str(per_run),
          '--seed', str(seed), '--cl_policy', str(Path(policy_path).resolve()), '--cl_policy_out', str(side)]
    if code:                                                # frozen snapshot (same flags as the final script)
        cmd = ['env', '-u', 'CONDA_PREFIX', '-u', 'CONDA_DEFAULT_ENV', '/home/pan/IsaacLab/isaaclab.sh', '-p',
               str(code / 'rl/basket/expert.py'), 'replay', '--headless', *FINAL_REPLAY_ARGS, *io]
    else:
        cmd = ['bash', str(ROOT / 'rl/basket/replay_final.sh'), io[1], io[3], *io[4:]]
    cmd += (['--cl_policy_det'] if det else []) + extra
    for attempt in range(3):                                # (10/15 23:4x) Isaac sometimes crashes at start-up (kit getenv
        with open(out_dir / f'{tag}.log', 'w' if not attempt else 'a') as log:   # segfault, telemetry regex): retry
            rc = subprocess.run(cmd, cwd=code or ROOT, stdout=log, stderr=subprocess.STDOUT).returncode
        if rc == 0 and side.exists():
            break
        print(f'[insert_rl] replay {tag} failed (rc {rc}), attempt {attempt + 1}/3', flush=True)
    if rc != 0 or not side.exists():
        raise RuntimeError(f'replay {tag} failed (rc {rc}), see {out_dir / f"{tag}.log"}')
    d = np.load(side)
    r = insert_reward(d['exact'], d['seated'], d['in_quadrant'], d['landed'], d['target'], unseated_penalty)
    return d, r


def run_insert_rl(a):
    """(10/15) PPO on the insert corrections (aim dx/dy, lean, dz, yaw) of the planner + per-quadrant settings, with the
    model-error randomisation (--cl_dr) and the FP noise model.  Iteration 0 = the hand-tuned method (zero mean head)."""
    import random
    if not math.isfinite(a.unseated_penalty) or a.unseated_penalty < 0:
        raise ValueError('--unseated-penalty must be finite and >= 0')
    out = Path(a.out or ROOT / f'rl/logs/basket/insert_rl_{time.strftime("%Y%m%d_%H%M%S")}')
    out.mkdir(parents=True, exist_ok=True)
    rows = [l for f in a.plans for l in open(f) if l.strip()]
    hold = [l for f in a.eval_plans for l in open(f) if l.strip()]
    rng = random.Random(a.seed)
    torch.manual_seed(a.seed)
    sys.path.insert(0, str(HERE))
    from expert import INSERT_ACT, INSERT_OBS, INSERT_OBS_SCALE
    pol = M.InsertPolicy(INSERT_OBS, INSERT_ACT, init_std=a.init_std, obs_scale=None if a.no_obs_scale else INSERT_OBS_SCALE)
    if a.init_ckpt:
        pol.load_state_dict(torch.load(a.init_ckpt, map_location='cpu'))
    opt = torch.optim.Adam(pol.parameters(), lr=a.lr or 3e-4)
    extra = (['--cl_dr'] + (['--cl_dr_full'] if a.dr_full else [])) if not a.no_dr else []
    extra += ['--cl_noise_fp', str(ROOT / 'rl/logs/basket/fp_noise_model_20261015.json')] if not a.no_fp_noise else []
    extra += ['--cl_policy_noseen'] if a.noseen else []
    extra += ['--cl_policy_seen_noise', str(a.seen_noise)] if a.seen_noise is not None else []
    extra += ['--cl_dr_gap_x', str(a.gap_x_dr)] if a.gap_x_dr is not None else []
    extra += ['--cl_policy_canon'] if a.canon else []      # (10/16) real-pipeline in-hand frame      # (10/16) policy never sees the believed gap error (no wrist gap detection)
    code = code_snapshot(out) if a.snapshot else None
    hist = []
    (out / 'config.json').write_text(json.dumps(dict(vars(a), plans_n=len(rows), eval_n=len(hold), extra=extra), indent=1, default=str))

    def evaluate(it):
        ev = out / 'eval_plans.jsonl'
        ev.write_text(''.join(hold))
        torch.save(pol.state_dict(), out / f'policy_{it:04d}.pt')
        d, r = insert_replay(ev, out, f'eval_{it:04d}', out / f'policy_{it:04d}.pt', 777, True, 1, extra, code,
                             a.unseated_penalty)
        res = dict(it=it, eval_exact=float(d['exact'].mean()), eval_reward=float(r.mean()),
                   **{f'eval_{q}': float(d['exact'][d['quadrant'] == q].mean()) for q in ('Q1', 'Q2', 'Q3', 'Q4') if (d['quadrant'] == q).any()})
        print(f'[insert_rl] eval it {it}: exact {res["eval_exact"]:.3f} ' + ' '.join(f'{q} {res.get("eval_" + q, float("nan")):.2f}' for q in ('Q1', 'Q2', 'Q3', 'Q4')), flush=True)
        return res

    best = -1.0
    for it in range(a.iters + 1):
        row = dict(it=it)
        if it % a.eval_every == 0:
            row.update(evaluate(it))
            if row['eval_exact'] > best:
                best = row['eval_exact']
                torch.save(pol.state_dict(), out / 'best.pt')
        if it == a.iters:
            hist.append(row)
            break
        batch = out / 'batch.jsonl'
        batch.write_text(''.join(rng.sample(rows, min(a.batch, len(rows)))))
        torch.save(pol.state_dict(), out / 'current.pt')
        t0 = time.time()
        d, r = insert_replay(batch, out, f'train_{it:04d}', out / 'current.pt', 1000 + it, False, 1, extra, code,
                             a.unseated_penalty)
        obs, act = torch.as_tensor(d['obs']), torch.as_tensor(d['act'])
        old_logp, ret = torch.as_tensor(d['logp'], dtype=torch.float32), torch.as_tensor(r, dtype=torch.float32)
        with torch.no_grad():
            adv = ret - pol.value(obs)
            adv = (adv - adv.mean()) / (adv.std() + 1e-6)
        for _ in range(a.epochs):                           # PPO (clip 0.2), one-step episodes: return = reward
            for idx in torch.randperm(len(obs)).split(a.minibatch):
                mu, std = pol(obs[idx])
                dist = torch.distributions.Normal(mu, std)
                logp = dist.log_prob(act[idx]).sum(1)
                ratio = (logp - old_logp[idx]).exp()
                pl = -torch.min(ratio * adv[idx], ratio.clamp(0.8, 1.2) * adv[idx]).mean()
                vl = ((pol.value(obs[idx]) - ret[idx]) ** 2).mean()
                loss = pl + 0.5 * vl - 0.003 * dist.entropy().sum(1).mean()
                opt.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(pol.parameters(), 1.0)
                opt.step()
                with torch.no_grad():
                    pol.log_std.clamp_(math.log(0.05), math.log(1.0))
        row.update(train_exact=float(d['exact'].mean()), train_reward=float(r.mean()), std=pol.log_std.exp().tolist(),
                   secs=round(time.time() - t0, 1), n=int(len(obs)))
        hist.append(row)
        print(f'[insert_rl] it {it}: train exact {row["train_exact"]:.3f} reward {row["train_reward"]:+.3f} '
              f'std {[round(s, 2) for s in row["std"]]} ({row["secs"]} s, {row["n"]} envs)', flush=True)
        (out / 'history.json').write_text(json.dumps(hist, indent=1))
    (out / 'history.json').write_text(json.dumps(hist, indent=1))
    plot_insert_curves(hist, out / 'curves.png')
    print(f'[insert_rl] done: best eval exact {best:.3f} -> {out}', flush=True)


def plot_insert_curves(hist, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 3.5))
    tr = [(h['it'], h['train_exact']) for h in hist if 'train_exact' in h]
    ev = [(h['it'], h['eval_exact']) for h in hist if 'eval_exact' in h]
    if tr:
        ax.plot(*zip(*tr), label='train (sampled)', lw=1)
    if ev:
        ax.plot(*zip(*ev), 'o-', label='eval (mean action, held-out)', lw=1.5)
    ax.set_xlabel('iteration')
    ax.set_ylabel('seated & exact')
    ax.spines[['top', 'right']].set_visible(False)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('mode', choices=('bc', 'rl', 'insert_rl'))
    p.add_argument('--plans', nargs='+', default=[], help='insert_rl: replay.jsonl plan files for training')
    p.add_argument('--eval_plans', nargs='+', default=[], help='insert_rl: held-out replay.jsonl plan files')
    p.add_argument('--out', default=None, help='insert_rl: output folder (new)')
    p.add_argument('--iters', type=int, default=200, help='insert_rl: PPO iterations')
    p.add_argument('--batch', type=int, default=96, help='insert_rl: plans per iteration (one env each)')
    p.add_argument('--epochs', type=int, default=8)
    p.add_argument('--minibatch', type=int, default=32)
    p.add_argument('--no_dr', action='store_true', help='insert_rl: no model-error randomisation')
    p.add_argument('--canon', action='store_true', help='insert_rl: in-hand pose input in the real pipeline frame (deployable)')
    p.add_argument('--gap_x_dr', type=float, default=None, help='insert_rl: (10/16) along-row gap error range +-m for every quadrant (default config)')
    p.add_argument('--seen_noise', type=float, default=None, help='insert_rl: (10/16) std [m] of the gap error the policy sees (default expert 0.002)')
    p.add_argument('--noseen', action='store_true', help='insert_rl: hide the believed gap error (real robot without wrist gap detection)')
    p.add_argument('--snapshot', action='store_true', help='insert_rl: replays run a frozen copy of the committed rl/ code')
    p.add_argument('--no_obs_scale', action='store_true', help='insert_rl: raw inputs (runs before 21:0x)')
    p.add_argument('--dr_full', action='store_true', help='insert_rl: also basket-yaw and in-hand-bias randomisation (unobserved)')
    p.add_argument('--no_fp_noise', action='store_true', help='insert_rl: exact in-hand pose (no FP noise model)')
    p.add_argument('--init_std', type=float, default=0.3, help='insert_rl: initial exploration std (pre-tanh; 0.3 ~ 9 mm aim)')
    p.add_argument('--unseated-penalty', type=float, default=0.0,
                   help='insert_rl: extra terminal reward penalty when the released plate is not seated (default preserves legacy reward)')
    p.add_argument('--data', help='bc: demo folder with batch_*.npz (expert.py collect)')
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--preflight-only', action='store_true', help='data check + tiny-batch overfit test, then exit')
    p.add_argument('--bc', help='rl: BC checkpoint (best.pt)')
    p.add_argument('--starts', help='rl: folder with starts_*.npz (expert.py collect --save_starts)')
    p.add_argument('--envs', type=int, default=None, help='rl: override config rl.envs')
    p.add_argument('--max_iters', type=int, default=None)
    p.add_argument('--eval_episodes', type=int, default=None)
    p.add_argument('--eval_every', type=int, default=None)
    p.add_argument('--init_ckpt', default=None, help='rl: continue from this residual checkpoint')
    p.add_argument('--lr', type=float, default=None)
    p.add_argument('--patience', type=int, default=None)
    p.add_argument('--view', default=None, help="rl: GUI at real time with this residual checkpoint ('bc' = BC only)")
    p.add_argument('--eval_ckpt', default=None, help='rl: only re-evaluate this residual checkpoint against the BC')
    p.add_argument('--max_steps', type=int, default=None, help='bc: override config (tests)')
    return p.parse_args()


if __name__ == '__main__':
    a = parse_args()
    if a.max_steps:
        CFG['bc']['max_steps'] = a.max_steps
    if a.max_iters:
        CFG['rl']['max_iters'] = a.max_iters
    if a.eval_episodes:
        CFG['eval']['envs'] = a.eval_episodes
    if a.eval_every:
        CFG['rl']['eval_every'] = a.eval_every
    if a.mode == 'bc':
        run_bc(a)
    elif a.mode == 'insert_rl':
        if a.eval_every is None:
            a.eval_every = 5
        run_insert_rl(a)
    else:
        run_rl(a)
