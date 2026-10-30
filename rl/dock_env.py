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

    def __init__(self, num_envs=2048, steps=48, hover_steps=16, device="cuda:0", seed=0,
                 job=DEFAULT_JOB, camera=DEFAULT_CAMERA, hang_deg=15.0,
                 grip_mm=15.0, tilt_deg=3.0, basket_xy_mm=10.0,
                 basket_yaw_deg=1.0, pitch_frac=0.05, lean_deg=3.0, descend_extra_mm=0.0,
                 residual_xy_mm=80.0, residual_pitch_deg=15.0, hover_until=None, hover_max=120, hover_step_mm=2.0,
                 hover_tilt_deg=None, target_x_mm=0.0, lean_follow=0.0,
                 depth_obs=False, freeze_in_tines=False, pc_goal=False, pc_points=32):
        self.device = torch.device(device)
        self.n, self.path_steps, self.hover_steps = int(num_envs), int(steps), int(hover_steps)
        # (Claude 10/29 3rd review, opt-in) after the taught end keep going down 2 mm a step until floor contact,
        # at most descend_extra_mm: the in-hand z error otherwise leaves the plate above the floor (height 0.64)
        self.extra_steps = int(math.ceil(float(descend_extra_mm) / 2.0))
        # (opt-in, 보스 10/29 'docking') hover_until = |band error| in pitches: hold the descent above the slot until the
        # observed error is below it (at most hover_max steps, x/y steps up to hover_step_mm while holding)
        self.hover_until, self.hover_max, self.hover_step = hover_until, int(hover_max), float(hover_step_mm) / 1000
        self.hover_tilt = None if hover_tilt_deg is None else float(hover_tilt_deg) / 20.0   # obs[2] units (20 deg)
        # (Claude 10/29 Isaac oracle) the physical seat is ~+6 mm from the gap root in x (target_x_mm), and the last
        # 10 cm go in better with x += lean_follow * sin(lean) * bottom height (physics: +1 713 vs -1 596 of 1024)
        self.target_x, self.lean_follow = float(target_x_mm) / 1000, float(lean_follow)
        # (Claude 10/29, opt-in) the side camera's DEPTH: a plane fit to the plate mask gives the plate pitch (2 deg noise)
        # and its across-row offset (3 mm noise) instead of the mask PCA tilt (P with it ended 20 deg off the seat pitch)
        self.depth_obs = bool(depth_obs)
        # (opt-in) no correction once the plate bottom is below the tine tips: moving inside the tines drives the plate
        # through them (physics replay lands it 2-6 gaps off)
        self.freeze_in_tines = bool(freeze_in_tines)
        # (보스 10/30, opt-in) point-cloud goal: the GT is the plate seated in the target gap (bottom at the gap root on
        # the floor, face along the tine lean); obs += the plate's and the goal's points (basket frame, about the goal
        # centre, 2 mm noise), reward += fit ratio (points within 5 mm of the goal) - Chamfer distance in gap pitches
        self.pc_goal, self.pc_m = bool(pc_goal), int(pc_points)
        if self.pc_goal:
            self.obs_dim = 9 + 6 * self.pc_m
        self.steps = self.path_steps + self.hover_steps + self.extra_steps + (self.hover_max if hover_until else 0)
        self.gen = torch.Generator(device=self.device).manual_seed(int(seed))
        J = np.load(job)
        names = [str(x) for x in J["names"]]
        keep = np.array(["Q3" in x for x in names])
        paths = np.split(J["flange"], np.cumsum(J["flange_len"])[:-1])
        paths = [_resample_pose(p, self.path_steps) for p, k in zip(paths, keep) if k]
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
        self.hang_deg, self.grip_m, self.tilt_deg = float(hang_deg), float(grip_mm) / 1000, float(tilt_deg)
        self.basket_xy_m, self.basket_yaw_deg = float(basket_xy_mm) / 1000, float(basket_yaw_deg)
        self.pitch_frac, self.lean_deg = float(pitch_frac), float(lean_deg)
        self.action_scale = torch.tensor([0.002, 0.002, math.radians(1.0)], device=self.device)
        self.residual_limit = torch.tensor([residual_xy_mm / 1000, residual_xy_mm / 1000, math.radians(residual_pitch_deg)],
                                           device=self.device)
        th = torch.linspace(0, 2 * math.pi, 17, device=self.device)[:-1]
        self.rim_local = torch.stack((self.plate_r * torch.cos(th), self.plate_r * torch.sin(th),
                                      torch.zeros_like(th)), 1)
        if self.pc_goal:                                     # a disc: 16 rim points + the rest spread inside (sunflower)
            k = torch.arange(self.pc_m - 16, device=self.device, dtype=torch.float32)
            rr = self.plate_r * torch.sqrt((k + 0.5) / len(k)); aa = k * 2.39996
            inner = torch.stack((rr * torch.cos(aa), rr * torch.sin(aa), torch.zeros_like(rr)), 1)
            self.disc_local = torch.cat((self.rim_local, inner), 0)
        self._alloc()

    def _alloc(self):
        n, d = self.n, self.device
        self.job = torch.zeros(n, dtype=torch.long, device=d)
        self.t = torch.zeros(n, dtype=torch.long, device=d)
        self.phase = torch.zeros(n, dtype=torch.long, device=d)
        self.hold_count = torch.zeros(n, dtype=torch.long, device=d)
        self.last_err = torch.full((n,), 9.0, device=d)
        self.last_tilt = torch.full((n,), 9.0, device=d)
        self.zdown = torch.zeros(n, device=d)
        self.residual = torch.zeros(n, 3, device=d)
        self.prev_action = torch.zeros(n, 3, device=d)
        self.Tbb = torch.eye(4, device=d).repeat(n, 1, 1)
        self.H = torch.eye(4, device=d).repeat(n, 1, 1)
        self.pitch = torch.full((n,), self.pitch0, device=d)
        self.lean = torch.full((n,), self.lean0, device=d)
        self.band_hold = torch.zeros(n, 3, device=d)   # centre u/v and pitch px
        self.band_valid = torch.zeros(n, dtype=torch.bool, device=d)
        self.terminated = torch.zeros(n, dtype=torch.bool, device=d)
        self.terminal_success = torch.zeros(n, dtype=torch.bool, device=d)
        self.last_info = {}

    def _uniform(self, shape, lo=-1.0, hi=1.0):
        return lo + (hi - lo) * torch.rand(shape, generator=self.gen, device=self.device)

    def reset(self):
        n, d = self.n, self.device
        self.job = torch.randint(len(self.paths), (n,), generator=self.gen, device=d)
        self.t.zero_(); self.phase.zero_(); self.hold_count.zero_(); self.last_err.fill_(9.0); self.last_tilt.fill_(9.0); self.zdown.zero_(); self.residual.zero_(); self.prev_action.zero_(); self.band_valid.zero_()
        self.terminated.zero_(); self.terminal_success.zero_()
        # In-hand uncertainty: hang +/-15 deg, translation each axis +/-15 mm, small tilt +/-3 deg.
        Rh = _axis_angle(self.hang_axis.expand(n, -1), self._uniform((n,), -math.radians(self.hang_deg), math.radians(self.hang_deg)))
        axis = torch.randn(n, 3, generator=self.gen, device=d)
        Rt = _axis_angle(axis, self._uniform((n,), 0, math.radians(self.tilt_deg)))
        A = torch.eye(4, device=d).repeat(n, 1, 1)
        A[:, :3, :3] = Rh @ Rt
        A[:, :3, 3] = self._uniform((n, 3), -self.grip_m, self.grip_m)
        self.H = A @ self.H_nom
        # Actual basket relative to the nominal path: xy +/-1 cm, yaw +/-1 deg.
        yaw = self._uniform((n,), -math.radians(self.basket_yaw_deg), math.radians(self.basket_yaw_deg))
        Rz = _axis_angle(torch.tensor([0., 0., 1.], device=d).expand(n, -1), yaw)
        self.Tbb = self.Tbb0.repeat(n, 1, 1)
        self.Tbb[:, :3, :3] = Rz @ self.Tbb[:, :3, :3]
        self.Tbb[:, :2, 3] += self._uniform((n, 2), -self.basket_xy_m, self.basket_xy_m)
        self.pitch = self.pitch0 * self._uniform((n,), 1-self.pitch_frac, 1+self.pitch_frac)
        self.lean = self.lean0 + self._uniform((n,), -math.radians(self.lean_deg), math.radians(self.lean_deg))
        return self._observe()

    def _plate_pose_at(self, path_index):
        P = self.paths[self.job, path_index] @ self.H
        # Basket-frame translation residual, and a small pitch about basket y at the plate centre.
        shift = (self.Tbb[:, :3, :3] @ torch.cat((self.residual[:, :2], -self.zdown[:, None]), 1)[..., None]).squeeze(-1)
        P[:, :3, 3] += shift
        Ry = _axis_angle(self.Tbb[:, :3, 1], self.residual[:, 2])
        P[:, :3, :3] = Ry @ P[:, :3, :3]
        if self.lean_follow:
            bottom, _ = self._bottom(P)
            h = ((self.Tbb[:, :3, :3].transpose(1, 2) @ (bottom - self.Tbb[:, :3, 3])[..., None]).squeeze(-1)[:, 2]
                 - self.floor_z).clamp(0, 0.10)
            P[:, :3, 3] += self.Tbb[:, :3, 0] * (self.lean_follow * math.sin(self.lean0) * h)[:, None]
        return P

    def _plate_pose(self):
        return self._plate_pose_at(self.t)

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
        root_x, yc = g[:, 0] + self.target_x, g[:, 1]
        # At the current rim height, leaning tine boundaries are between root and tip.
        Tbi_R = self.Tbb[:, :3, :3].transpose(1, 2)
        bottom_b = (Tbi_R @ (bottom - self.Tbb[:, :3, 3])[..., None]).squeeze(-1)
        # A kinematic floor contact: do not let the diagnostic state pass
        # below the basket floor before the episode terminates.
        dz = (self.floor_z - bottom_b[:, 2]).clamp_min(0)
        shift = self.Tbb[:, :3, 2] * dz[:, None]
        P[:, :3, 3] += shift
        bottom += shift
        bottom_b[:, 2] += dz
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
        # Compare in one depth plane.  Above the tine tips, directly comparing
        # the high plate rim with the clamped tip-height band creates a camera
        # parallax error that points the controller the wrong way.  Project the
        # rim along the known nominal descent direction to the current band
        # height; below the tips the point is already in that plane.
        band0_b = (self.Tbb[:, :3, :3].transpose(1, 2) @
                   (band3[:, 0] - self.Tbb[:, :3, 3])[..., None]).squeeze(-1)
        projected_b = bottom_b.clone()
        projected_b[:, 2] = torch.minimum(bottom_b[:, 2], band0_b[:, 2])
        compare_bottom = (self.Tbb[:, :3, :3] @ projected_b[..., None]).squeeze(-1) + self.Tbb[:, :3, 3]
        uv = self._project(torch.cat((band3, compare_bottom[:, None]), 1))
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
        # Camera-observable plate tilt: PCA major axis of the projected plate
        # mask relative to the projected target band.  The -6 degree nominal
        # offset comes from the calibrated Q3 camera geometry, not true pose.
        rim = (P[:, :3, :3] @ self.rim_local.T).transpose(1, 2) + P[:, None, :3, 3]
        rim_uv = self._project(rim)
        rd = rim_uv - rim_uv.mean(1, keepdim=True)
        cov = rd.transpose(1, 2) @ rd / len(self.rim_local)
        major = torch.linalg.eigh(cov).eigenvectors[:, :, 1]
        major *= torch.where((major * across).sum(1) < 0, -1.0, 1.0)[:, None]
        tilt = torch.atan2(across[:, 0] * major[:, 1] - across[:, 1] * major[:, 0],
                           (major * across).sum(1))
        tilt += torch.randn(self.n, generator=self.gen, device=self.device) * math.radians(1.0)
        tilt_error = tilt - math.radians(-6.0)
        if self.depth_obs:
            n_b = (self.Tbb[:, :3, :3].transpose(1, 2) @ normal[..., None]).squeeze(-1)
            n_b = n_b * torch.sign(n_b[:, 0:1])
            pitch_err = torch.atan2(n_b[:, 2], n_b[:, 0]) - self.lean0
            tilt_error = pitch_err + torch.randn(self.n, generator=self.gen, device=self.device) * math.radians(2.0)
            err_across = ((bottom_b[:, 1] - yc) + torch.randn(self.n, generator=self.gen, device=self.device) * 0.003) / self.pitch0
        remaining = 1.0 - self.phase.float() / (self.steps - 1)
        low = (bottom_b[:, 2] - self.floor_z) / 0.15
        o = torch.stack((err_along, err_across, tilt_error / math.radians(20), remaining, low,
                         self.prev_action[:, 0], self.prev_action[:, 1], self.prev_action[:, 2],
                         self.band_valid.float()), 1).clamp(-5, 5)
        if not self.pc_goal:
            return o
        cur, goal = self._clouds(P, root_x, yc)
        c0 = goal.mean(1, keepdim=True)
        noisy = cur + torch.randn(cur.shape, generator=self.gen, device=self.device) * 0.002
        return torch.cat((o, ((noisy - c0) / 0.1).flatten(1).clamp(-5, 5), ((goal - c0) / 0.1).flatten(1)), 1)

    def _clouds(self, P, root_x, yc):
        """-> the plate's points and the seated-goal points (N,M,3) in the basket frame."""
        R = self.Tbb[:, :3, :3].transpose(1, 2)
        X = (P[:, :3, :3] @ self.disc_local.T).transpose(1, 2) + P[:, None, :3, 3]
        cur = (R @ (X - self.Tbb[:, None, :3, 3]).transpose(1, 2)).transpose(1, 2)
        s, c = torch.sin(self.lean), torch.cos(self.lean)
        z0 = torch.zeros_like(s)
        u = torch.stack((-s, z0, c), 1)                    # up in the seated face (tine lean about basket y)
        v = torch.stack((z0, torch.ones_like(s), z0), 1)
        centre = torch.stack((root_x, yc, torch.full_like(yc, self.floor_z)), 1) + self.plate_r * u
        L = self.disc_local
        goal = centre[:, None] + L[None, :, 0:1] * u[:, None] + L[None, :, 1:2] * v[:, None]
        return cur, goal

    def pc_fit(self):
        """-> (fit ratio: plate points within 5 mm of the goal cloud, symmetric Chamfer distance [m])."""
        P, _, _, _, root_x, yc, _ = self._geometry()
        cur, goal = self._clouds(P, root_x, yc)
        D = torch.cdist(cur, goal)
        a, b = D.min(2).values, D.min(1).values
        return (a < 0.005).float().mean(1), (a.mean(1) + b.mean(1)) / 2

    def step(self, action):
        action = action.clamp(-1, 1)
        active = ~self.terminated
        old_action = self.prev_action.clone()
        remaining = 1.0 - self.phase.float() / (self.steps - 1)
        taper = 0.25 + 0.75 * remaining
        scale = self.action_scale[None] * taper[:, None]
        if self.hover_until is not None:                      # holding: full-size steps, no taper
            holding = self._holding()
            big = self.action_scale.clone()
            big[:2] = self.hover_step
            scale = torch.where(holding[:, None], big[None].expand_as(scale), scale)
        if self.freeze_in_tines:
            P0 = self._plate_pose(); b0, _ = self._bottom(P0)
            h0 = (self.Tbb[:, :3, :3].transpose(1, 2) @ (b0 - self.Tbb[:, :3, 3])[..., None]).squeeze(-1)[:, 2] - self.floor_z
            action = torch.where((h0 < self.tine_h)[:, None], torch.zeros_like(action), action)
        candidate = self.residual + action * scale
        candidate = torch.maximum(torch.minimum(candidate, self.residual_limit), -self.residual_limit)
        self.residual[active] = candidate[active]
        self.prev_action[active] = action[active]
        self.phase[active] += 1
        advance = active & (self.phase > self.hover_steps)
        if self.hover_until is not None:
            advance &= ~self._holding()
            self.hold_count += (active & self._holding()).long()
        at_end = advance & (self.t >= self.path_steps - 1)
        self.zdown[at_end] = torch.clamp(self.zdown[at_end] + 0.002, max=self.extra_steps * 0.002)
        self.t[advance] = torch.minimum(self.t[advance] + 1,
                                        torch.full_like(self.t[advance], self.path_steps - 1))
        obs = self._observe()
        self.last_err = obs[:, 0].abs()
        self.last_tilt = obs[:, 2].abs()
        e, ey, tilt = obs[:, 0], obs[:, 1], obs[:, 2]
        inside = e.abs() <= 0.5
        _, _, height_ok, angle_ok, _ = self.criteria()
        reward = -e.abs() - 0.25 * ey.abs() - 0.20 * tilt.abs() - 0.01 * (action - old_action).square().sum(1)
        reward += inside.float() * (0.2 + 0.8 * (1.0 - obs[:, 3]))
        reward -= (e.abs() > 1.0).float() * 2.0
        if self.pc_goal:
            fit, cham = self.pc_fit()
            reward += fit - cham / self.pitch0
        contact = active & height_ok
        timeout = active & (self.phase >= self.steps - 1)
        newly_done = contact | timeout
        success = self.success()
        self.terminal_success[newly_done] = success[newly_done]
        self.terminated |= newly_done
        reward *= active.float()
        reward += newly_done.float() * (success.float() * 12.0 - (~success).float() * 6.0)
        self.last_info = dict(success=self.terminal_success, along=e, across=ey,
                              angle_ok=angle_ok, height_ok=height_ok)
        if self.pc_goal:
            self.last_info.update(fit=fit, chamfer=cham)
        return obs, reward, self.terminated, self.last_info

    def _holding(self):
        """Hover above the slot: after the fixed hover, at the start of the taught insert part of the path (t at
        hover_at), while the last observed band error is above hover_until and the hover budget is left."""
        if self.freeze_in_tines:                             # hold just above the tine tips, where moving is still free
            P0 = self._plate_pose(); b0, _ = self._bottom(P0)
            h0 = (self.Tbb[:, :3, :3].transpose(1, 2) @ (b0 - self.Tbb[:, :3, 3])[..., None]).squeeze(-1)[:, 2] - self.floor_z
            at = (h0 >= self.tine_h) & (h0 < self.tine_h + 0.04)
        else:
            at = self.t >= int(self.path_steps * 0.75)
        off = self.last_err > self.hover_until
        if self.hover_tilt is not None:                       # (Claude 10/29) also the plate plane before entering the tines
            off |= self.last_tilt > self.hover_tilt
        return at & (self.t < self.path_steps - 1) & off & (self.hold_count < self.hover_max)

    def success(self):
        x_ok, y_ok, height_ok, angle_ok, _ = self.criteria()
        return x_ok & y_ok & height_ok & angle_ok

    def criteria(self):
        _, bottom, normal, bottom_b, root_x, yc, _ = self._geometry()
        # strict kinematic counterpart of hand_basket: bottom near target, centre across rows, low and aligned.
        n_b = (self.Tbb[:, :3, :3].transpose(1, 2) @ normal[..., None]).squeeze(-1)
        desired = torch.stack((torch.cos(self.lean), torch.zeros_like(self.lean), torch.sin(self.lean)), 1)
        angle = torch.acos((n_b * desired).sum(1).abs().clamp(0, 1))
        x_ok = (bottom_b[:, 0] - root_x).abs() < 0.0085
        y_ok = (bottom_b[:, 1] - yc).abs() < 0.030
        height_ok = (bottom_b[:, 2] - self.floor_z) <= 0.002
        angle_ok = angle < math.radians(20)
        return x_ok, y_ok, height_ok, angle_ok, angle


def controller(obs, mode="p"):
    if mode == "fixed":
        return torch.zeros(len(obs), 3, device=obs.device)
    a = torch.zeros(len(obs), 3, device=obs.device)
    # A small per-step gain is deliberate: actions are integrated over the
    # whole descent, so a conventional one-shot P gain oscillates in noise.
    a[:, 0] = -0.40 * obs[:, 0]
    # The fixed side view barely observes basket-y at the taught approach;
    # do not integrate saturated image-y noise. Positive pitch reduces the
    # measured normal error for this Q3 camera/hand convention.
    a[:, 1] = 0.0
    a[:, 2] = -0.50 * obs[:, 2]
    return a.clamp(-1, 1)


def evaluate_controller(mode="fixed", episodes=1024, seed=1024, device="cuda:0", **env_kwargs):
    env = DockEnv(episodes, device=device, seed=seed, **env_kwargs)
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
