"""(Claude 10/30) a learned physics-success model for the kinematic dock env: Isaac replays (hand_basket.py --dock_traj
dock_samples[].strict) labelled by the kinematic end state (dock_pc_train.kin_features, bottom/centre x relative to the
gap root).  Small MLP, so DockEnv can score every terminating episode on the GPU.
    ~/miniconda3/envs/foundationpose/bin/python rl/phys_critic.py --data outputs/dock_rl_20261030/phys_dataset_v1.npz \
        --out outputs/dock_rl_20261030/phys_critic_v1.pt"""
import argparse
import json

import numpy as np
import torch
import torch.nn as nn

SCALE = torch.tensor([1000.0] * 3 + [1.0] * 3 + [1000.0] * 5 + [57.3, 57.3])   # m -> mm, rad -> deg


class PhysCritic(nn.Module):
    def __init__(self, mean=None, std=None):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(13, 128), nn.ReLU(), nn.Linear(128, 128), nn.ReLU(), nn.Linear(128, 1))
        self.register_buffer('mean', torch.zeros(13) if mean is None else mean)
        self.register_buffer('std', torch.ones(13) if std is None else std)
        self.register_buffer('scale', SCALE.clone())

    def forward(self, feat):
        """kin_features (N,13), x relative to the gap root -> P(physics success)."""
        return torch.sigmoid(self.net((feat * self.scale - self.mean) / self.std).squeeze(-1))


def fit(X, y, epochs=400, seed=0):
    torch.manual_seed(seed)
    Xs = X * SCALE
    m = PhysCritic(Xs.mean(0), Xs.std(0).clamp_min(1e-6))
    opt = torch.optim.Adam(m.parameters(), lr=1e-3, weight_decay=1e-4)
    for _ in range(epochs):
        for ix in torch.randperm(len(X)).split(512):
            p = m(X[ix]).clamp(1e-5, 1 - 1e-5)
            loss = nn.functional.binary_cross_entropy(p, y[ix])
            opt.zero_grad(); loss.backward(); opt.step()
    return m


def main():
    from sklearn.metrics import roc_auc_score
    a = argparse.ArgumentParser()
    a.add_argument('--data', default='outputs/dock_rl_20261030/phys_dataset_v1.npz')
    a.add_argument('--out', default='outputs/dock_rl_20261030/phys_critic_v1.pt')
    a = a.parse_args()
    d = np.load(a.data)
    X = torch.tensor(d['X'], dtype=torch.float32); y = torch.tensor(d['y'], dtype=torch.float32); g = d['group']
    rows = []
    for hold in np.unique(g):
        tr = torch.tensor(g != hold)
        m = fit(X[tr], y[tr])
        with torch.no_grad():
            p = m(X[~tr]).numpy()
        rows.append(dict(held_out=int(hold), auc=round(float(roc_auc_score(d['y'][~tr.numpy()], p)), 3),
                         pred=round(float(p.mean()), 3), true=round(float(d['y'][~tr.numpy()].mean()), 3)))
        print(json.dumps(rows[-1]), flush=True)
    m = fit(X, y)
    torch.save(m.state_dict(), a.out)
    print(json.dumps(dict(saved=a.out, n=len(X), leave_one_bundle_out=rows)))


if __name__ == '__main__':
    main()
