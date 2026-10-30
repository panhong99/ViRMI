"""(보스 10/30) GUI: the point-cloud goal docking in the kinematic dock env.  Blue = the held plate's points, green =
the seated-goal (GT) points, grey = the target gap's tine band; slider / arrow keys step through the episode, the
title shows the fit ratio (points within 5 mm) and the Chamfer distance.  Basket frame, mm.
    ~/miniconda3/envs/foundationpose/bin/python rl/dock_pc_view.py --ckpt outputs/dock_rl_20261030/pc_goal_v1/best.pt"""
import argparse
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dock_env import DockEnv, controller  # noqa: E402
from dock_train import ActorCritic  # noqa: E402


@torch.no_grad()
def rollout(policy, envs, seed):
    env = DockEnv(envs, seed=seed, pc_goal=True)
    obs = env.reset()
    frames = []
    for _ in range(env.steps):
        P, _, _, _, rx, yc, band = env._geometry()
        cur, goal = env._clouds(P, rx, yc)
        R = env.Tbb[:, :3, :3].transpose(1, 2)
        band_b = (R @ (band - env.Tbb[:, None, :3, 3]).transpose(1, 2)).transpose(1, 2)
        fit, cham = env.pc_fit()
        frames.append(dict(cur=cur.cpu().numpy() * 1000, goal=goal.cpu().numpy() * 1000, band=band_b.cpu().numpy() * 1000,
                           fit=fit.cpu().numpy(), cham=cham.cpu().numpy() * 1000))
        if env.terminated.all():
            break
        obs, _, _, info = env.step(policy(obs))
    return frames, info['success'].cpu().numpy()


def main():
    a = argparse.ArgumentParser()
    a.add_argument('--ckpt', default='outputs/dock_rl_20261030/pc_goal_v1/best.pt')
    a.add_argument('--policy', choices=['ppo', 'p', 'fixed'], default='ppo')
    a.add_argument('--envs', type=int, default=4)
    a.add_argument('--seed', type=int, default=7)
    a = a.parse_args()
    if a.policy == 'ppo':
        net = ActorCritic(DockEnv(1, pc_goal=True).obs_dim, DockEnv.act_dim).cuda()
        net.load_state_dict(torch.load(a.ckpt, map_location='cuda', weights_only=True))
        policy = net.deterministic
    else:
        policy = lambda o: controller(o, a.policy)  # noqa: E731
    frames, ok = rollout(policy, a.envs, a.seed)

    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider
    cols = min(a.envs, 4)
    rows = (a.envs + cols - 1) // cols
    fig = plt.figure(figsize=(4.2 * cols, 4.4 * rows + 0.6))
    axes = [fig.add_subplot(rows, cols, i + 1, projection='3d') for i in range(a.envs)]
    sl = Slider(fig.add_axes([0.15, 0.02, 0.7, 0.03]), 'step', 0, len(frames) - 1, valinit=len(frames) - 1, valstep=1)

    def draw(t):
        f = frames[int(t)]
        for i, ax in enumerate(axes):
            ax.cla()
            g, c, b = f['goal'][i], f['cur'][i], f['band'][i]
            ax.scatter(*g.T, s=8, c='tab:green', label='GT (seated)')
            ax.scatter(*c.T, s=8, c='tab:blue', label='plate')
            ax.plot(*np.vstack((b, b[:1])).T, c='0.5', lw=1.5)
            m = g.mean(0)
            for k, lim in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
                lim(m[k] - 160, m[k] + 160)
            ax.set_xlabel('x (along row)'); ax.set_ylabel('y'); ax.set_zlabel('z')
            ax.set_title(f'env {i} {"OK" if ok[i] else "fail"}  fit {f["fit"][i]:.2f}  Chamfer {f["cham"][i]:.1f} mm', fontsize=9)
        axes[0].legend(loc='upper left', fontsize=8)
        fig.canvas.draw_idle()

    sl.on_changed(draw)
    fig.canvas.mpl_connect('key_press_event', lambda e: sl.set_val(min(max(sl.val + {'right': 1, 'left': -1}.get(e.key, 0), 0),
                                                                          len(frames) - 1)))
    draw(sl.val)
    plt.show()


if __name__ == '__main__':
    main()
