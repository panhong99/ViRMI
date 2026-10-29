"""GPU kinematic docking environment for the taught Tesollo plate descent.

The nominal flange path comes from ``job_taught_1029_cal.npz``.  Each reset
perturbs the in-hand plate transform, basket pose and tine geometry.  The
policy only sees side-camera quantities normalised by the observed tine pitch;
it applies small basket-frame x/y translations and wrist pitch while z keeps
following the taught descent.  This file has no Isaac, robot, camera or ROS I/O.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_JOB = ROOT / "rl/logs/hand_basket/job_taught_1029_cal.npz"
DEFAULT_CAMERA = ROOT / "calibration/data/side_cam/T_base_side.json"


def _resample_pose(path: np.ndarray, steps: int) -> np.ndarray:
    old = np.linspace(0.0, 1.0, len(path))
    new = np.linspace(0.0, 1.0, steps)
    out = np.repeat(np.eye(4)[None], steps, axis=0)
    out[:, :3, 3] = np.stack([np.interp(new, old, path[:, i, 3]) for i in range(3)], axis=1)
    out[:, :3, :3] = Slerp(old, Rotation.from_matrix(path[:, :3, :3]))(new).as_matrix()
    return out


def _axis_angle(axis: torch.Tensor, angle: torch.Tensor) -> torch.Tensor:
    """Batched Rodrigues matrix; axis (...,3), angle (...)."""
    axis = axis / torch.linalg.vector_norm(axis, dim=-1, keepdim=True).clamp_min(1e-9)
    x, y, z = axis.unbind(-1)
    zero = torch.zeros_like(x)
    K = torch.stack((zero, -z, y, z, zero, -x, -y, x, zero), -1).reshape(axis.shape[:-1] + (3, 3))
    eye = torch.eye(3, device=axis.device, dtype=axis.dtype).expand_as(K)
    a = angle[..., None, None]
    return eye + torch.sin(a) * K + (1.0 - torch.cos(a)) * (K @ K)


class DockEnv:
    obs_dim = 9
    act_dim = 3

    def __init__(self, num_envs=2048, steps=48, device="cuda:0", seed=0,
                 job=DEFAULT_JOB, camera=DEFAULT_CAMERA):
        self.device = torch.device(device)
        self.n, self.steps = int(num_envs), int(steps)
        self.gen = torch.Generator(device=self.device).manual_seed(int(seed))
        J = np.load(job)
        names = [str(x) for x in J["names"]]
        keep = np.array(["Q3" in x for x in names])
        paths = np.split(J["flange"], np.cumsum(J["flange_len"])[:-1])
        paths = [_resample_pose(p, self.steps) for p, k in zip(paths, keep) if k]
        self.paths = torch.tensor(np.stack(paths), dtype=torch.float32, device=self.device)
        self.job_gap = torch.tensor(J["gap"][keep], dtype=torch.long, device=self.device)
        gaps = np.split(J["gaps_b"], np.cumsum(J["gaps_len"])[:-1])
        self.gaps = torch.tensor(np.stack([g for g, k in zip(gaps, keep) if k]), dtype=torch.float32,
                                 device=self.device)
        self.H_nom = torch.tensor(J["H_nom"], dtype=torch.float32, device=self.device)
        self.Tbb0 = torch.tensor(J["Tbb"], dtype=torch.float32, device=self.device)
        self.hang_axis = torch.tensor(J["hang_axis"], dtype=torch.float32, device=self.device)

        import json
        cam = json.loads(Path(camera).read_text())
        self.Tcb = torch.linalg.inv(torch.tensor(cam["T_base_cam"], dtype=torch.float32, device=self.device))
        self.K = torch.tensor(cam["K"], dtype=torch.float32, device=self.device)
        cfg = yaml.safe_load((ROOT / "rl/basket.yaml").read_text())
        self.floor_z = float(cfg["full"]["floor_z"])
        self.tine_h = float(cfg["pins"]["height"])
        self.row_half = float(abs(cfg["pins"]["rows_u"][1] - cfg["pins"]["rows_u"][0]) / 2)
        self.pitch0 = float(cfg["full"]["tine_pitch"]["Q3"])
        self.lean0 = math.radians(float(cfg["full"]["tine_lean_deg"]["Q3"]))
        self.plate_r = 0.134
        self.action_scale = torch.tensor([0.002, 0.002, math.radians(1.0)], device=self.device)
        self.residual_limit = torch.tensor([0.035, 0.035, math.radians(15.0)], device=self.device)
        self._alloc()

    def _alloc(self):
        n, d = self.n, self.device
        self.job = torch.zeros(n, dtype=torch.long, device=d)
        self.t = torch.zeros(n, dtype=torch.long, device=d)
        self.residual = torch.zeros(n, 3, device=d)
        self.prev_action = torch.zeros(n, 3, device=d)
        self.Tbb = torch.eye(4, device=d).repeat(n, 1, 1)
        self.H = torch.eye(4, device=d).repeat(n, 1, 1)
        self.pitch = torch.full((n,), self.pitch0, device=d)
        self.lean = torch.full((n,), self.lean0, device=d)
        self.band_hold = torch.zeros(n, 3, device=d)   # centre u/v and pitch px
        self.band_valid = torch.zeros(n, dtype=torch.bool, device=d)
        self.last_info = {}

    def _uniform(self, shape, lo=-1.0, hi=1.0):
        return lo + (hi - lo) * torch.rand(shape, generator=self.gen, device=self.device)

    def reset(self):
        n, d = self.n, self.device
        self.job = torch.randint(len(self.paths), (n,), generator=self.gen, device=d)
        self.t.zero_(); self.residual.zero_(); self.prev_action.zero_(); self.band_valid.zero_()
        # In-hand uncertainty: hang +/-15 deg, translation each axis +/-15 mm, small tilt +/-3 deg.
        Rh = _axis_angle(self.hang_axis.expand(n, -1), self._uniform((n,), -math.radians(15), math.radians(15)))
        axis = torch.randn(n, 3, generator=self.gen, device=d)
        Rt = _axis_angle(axis, self._uniform((n,), 0, math.radians(3)))
        A = torch.eye(4, device=d).repeat(n, 1, 1)
        A[:, :3, :3] = Rh @ Rt
        A[:, :3, 3] = self._uniform((n, 3), -0.015, 0.015)
        self.H = A @ self.H_nom
        # Actual basket relative to the nominal path: xy +/-1 cm, yaw +/-1 deg.
        yaw = self._uniform((n,), -math.radians(1), math.radians(1))
        Rz = _axis_angle(torch.tensor([0., 0., 1.], device=d).expand(n, -1), yaw)
        self.Tbb = self.Tbb0.repeat(n, 1, 1)
        self.Tbb[:, :3, :3] = Rz @ self.Tbb[:, :3, :3]
        self.Tbb[:, :2, 3] += self._uniform((n, 2), -0.01, 0.01)
        self.pitch = self.pitch0 * self._uniform((n,), 0.95, 1.05)
        self.lean = self.lean0 + self._uniform((n,), -math.radians(3), math.radians(3))
        return self._observe()

    def _plate_pose(self):
        ids = torch.arange(self.n, device=self.device)
        P = self.paths[self.job, self.t] @ self.H
        # Basket-frame translation residual, and a small pitch about basket y at the plate centre.
        shift = (self.Tbb[:, :3, :3] @ torch.cat((self.residual[:, :2], torch.zeros(self.n, 1, device=self.device)), 1)[..., None]).squeeze(-1)
        P[:, :3, 3] += shift
        Ry = _axis_angle(self.Tbb[:, :3, 1], self.residual[:, 2])
        P[:, :3, :3] = Ry @ P[:, :3, :3]
        return P

    def _bottom(self, P):
        normal = P[:, :3, 2]
        down = torch.tensor([0., 0., -1.], device=self.device).expand(self.n, -1)
        radial = down - (down * normal).sum(1, keepdim=True) * normal
        radial = radial / torch.linalg.vector_norm(radial, dim=1, keepdim=True).clamp_min(1e-6)
        return P[:, :3, 3] + self.plate_r * radial, normal

    def _project(self, X):
        """X (N,M,3) base -> pixels (N,M,2)."""
        Xc = (self.Tcb[:3, :3] @ X.transpose(1, 2) + self.Tcb[:3, 3, None]).transpose(1, 2)
        uvw = (self.K @ Xc.transpose(1, 2)).transpose(1, 2)
        return uvw[..., :2] / uvw[..., 2:].clamp_min(1e-4)

    def _geometry(self):
        P = self._plate_pose()
        bottom, normal = self._bottom(P)
        ids = torch.arange(self.n, device=self.device)
        gap_idx = self.job_gap[self.job]
        g = self.gaps[self.job, gap_idx]
        root_x, yc = g[:, 0], g[:, 1]
        # At the current rim height, leaning tine boundaries are between root and tip.
        Tbi_R = self.Tbb[:, :3, :3].transpose(1, 2)
        bottom_b = (Tbi_R @ (bottom - self.Tbb[:, :3, 3])[..., None]).squeeze(-1)
        frac = ((bottom_b[:, 2] - self.floor_z) / self.tine_h).clamp(0, 1)
        centre_x = root_x - torch.sin(self.lean) * self.tine_h * frac
        half = self.pitch / 2
        z = torch.full_like(root_x, self.floor_z) + self.tine_h * frac
        pts_b = torch.stack((
            torch.stack((centre_x - half, yc - self.row_half, z), 1),
            torch.stack((centre_x + half, yc - self.row_half, z), 1),
            torch.stack((centre_x + half, yc + self.row_half, z), 1),
            torch.stack((centre_x - half, yc + self.row_half, z), 1),
        ), 1)
        pts = (self.Tbb[:, :3, :3] @ pts_b.transpose(1, 2)).transpose(1, 2) + self.Tbb[:, None, :3, 3]
        return P, bottom, normal, bottom_b, root_x, yc, pts

    def _observe(self):
        P, bottom, normal, bottom_b, root_x, yc, band3 = self._geometry()
        uv = self._project(torch.cat((band3, bottom[:, None]), 1))
        # CNN-level point noise. Hold the last complete band on a 5% miss.
        uv[:, :4] += torch.randn(self.n, 4, 2, generator=self.gen, device=self.device) * 2.0
        centre = uv[:, :4].mean(1)
        along_v = ((uv[:, 1] - uv[:, 0]) + (uv[:, 2] - uv[:, 3])) / 2
        pitch_px = torch.linalg.vector_norm(along_v, dim=1).clamp_min(2.0)
        along = along_v / pitch_px[:, None]
        across_v = ((uv[:, 3] - uv[:, 0]) + (uv[:, 2] - uv[:, 1])) / 2
        across = across_v / torch.linalg.vector_norm(across_v, dim=1, keepdim=True).clamp_min(2.0)
        miss = torch.rand(self.n, generator=self.gen, device=self.device) < 0.05
        false = torch.rand(self.n, generator=self.gen, device=self.device) < 0.03
        centre = centre + false[:, None] * along_v * torch.sign(self._uniform((self.n,)))[:, None]
        fresh = ~miss | ~self.band_valid
        self.band_hold[fresh] = torch.cat((centre[fresh], pitch_px[fresh, None]), 1)
        self.band_valid |= ~miss
        centre, pitch_px = self.band_hold[:, :2], self.band_hold[:, 2].clamp_min(2.0)
        d = uv[:, 4] - centre
        err_along = (d * along).sum(1) / pitch_px
        err_across = (d * across).sum(1) / pitch_px
        # Normal error to either sign of the Q3 tine-plane normal, in radians.
        n_b = (self.Tbb[:, :3, :3].transpose(1, 2) @ normal[..., None]).squeeze(-1)
        desired = torch.stack((torch.cos(self.lean), torch.zeros_like(self.lean), torch.sin(self.lean)), 1)
        angle = torch.acos((n_b * desired).sum(1).abs().clamp(0, 1))
        remaining = 1.0 - self.t.float() / (self.steps - 1)
        low = (bottom_b[:, 2] - self.floor_z) / 0.15
        return torch.stack((err_along, err_across, angle / math.radians(20), remaining, low,
                            self.prev_action[:, 0], self.prev_action[:, 1], self.prev_action[:, 2],
                            self.band_valid.float()), 1).clamp(-5, 5)

    def step(self, action):
        action = action.clamp(-1, 1)
        old_action = self.prev_action.clone()
        self.residual = torch.maximum(torch.minimum(self.residual + action * self.action_scale,
                                                    self.residual_limit), -self.residual_limit)
        self.prev_action = action
        self.t += 1
        done = self.t >= self.steps - 1
        obs = self._observe()
        e, ey, ang = obs[:, 0], obs[:, 1], obs[:, 2]
        inside = e.abs() <= 0.5
        reward = -e.abs() - 0.25 * ey.abs() - 0.20 * ang.abs() - 0.01 * (action - old_action).square().sum(1)
        reward += inside.float() * (0.2 + 0.8 * (1.0 - obs[:, 3]))
        reward -= (e.abs() > 1.0).float() * 2.0
        success = self.success()
        reward += done.float() * (success.float() * 12.0 - (~success).float() * 6.0)
        self.last_info = dict(success=success, along=e, across=ey, angle=ang)
        return obs, reward, done, self.last_info

    def success(self):
        _, bottom, normal, bottom_b, root_x, yc, _ = self._geometry()
        ids = torch.arange(self.n, device=self.device)
        gap_idx = self.job_gap[self.job]
        # strict kinematic counterpart of hand_basket: bottom near target, centre across rows, low and aligned.
        n_b = (self.Tbb[:, :3, :3].transpose(1, 2) @ normal[..., None]).squeeze(-1)
        desired = torch.stack((torch.cos(self.lean), torch.zeros_like(self.lean), torch.sin(self.lean)), 1)
        angle = torch.acos((n_b * desired).sum(1).abs().clamp(0, 1))
        return ((bottom_b[:, 0] - root_x).abs() < 0.0085) & ((bottom_b[:, 1] - yc).abs() < 0.030) & \
               ((bottom_b[:, 2] - self.floor_z) < 0.055) & (angle < math.radians(20))


def controller(obs, mode="p"):
    if mode == "fixed":
        return torch.zeros(len(obs), 3, device=obs.device)
    a = torch.zeros(len(obs), 3, device=obs.device)
    # A small per-step gain is deliberate: actions are integrated over the
    # whole descent, so a conventional one-shot P gain oscillates in noise.
    a[:, 0] = -0.15 * obs[:, 0]
    # The fixed side view barely observes basket-y at the taught approach;
    # do not integrate saturated image-y noise. Positive pitch reduces the
    # measured normal error for this Q3 camera/hand convention.
    a[:, 1] = 0.0
    a[:, 2] = 0.20 * obs[:, 2]
    return a.clamp(-1, 1)


def evaluate_controller(mode="fixed", episodes=1024, seed=1024, device="cuda:0"):
    env = DockEnv(episodes, device=device, seed=seed)
    obs = env.reset()
    for _ in range(env.steps - 1):
        obs, _, _, info = env.step(controller(obs, mode))
    return float(info["success"].float().mean()), info["success"].cpu().numpy()


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=("fixed", "p"), default="fixed")
    p.add_argument("--episodes", type=int, default=1024)
    p.add_argument("--seed", type=int, default=1024)
    a = p.parse_args()
    rate, _ = evaluate_controller(a.mode, a.episodes, a.seed)
    print(f"{a.mode}: {rate:.6f} ({round(rate*a.episodes)}/{a.episodes})")
