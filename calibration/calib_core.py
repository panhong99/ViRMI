"""Calibration math shared by every calibration/ tool (numpy/scipy; OpenCV only inside the marker / PnP functions).

  config     load_config, cpath, DATA                  calibration/config.yaml access
  transforms make_T, inv_T, rot_angle_deg, average_poses, posx_to_T, T_to_posx
  marker     make_detector, detect_marker               ArUco pose (IPPE_SQUARE) in an undistorted image
  solvers    solve_eye_to_hand (flange marker, fixed camera; the pre-gripper method, kept for reference/tests)
             solve_eye_in_hand (wrist camera T_flange_cam with a fixed marker)
             fixed_cam_average / fixed_cam_pnp (fixed camera from wrist-located markers; PnP is the one to use)
  files      save_json, load_json, latest_t_base_cam, newest_wrist_result, load_fixed_K
  selftest   selftest()                                 synthetic checks of all solvers (no hardware)

Conventions: 4x4 homogeneous, metres; T_a_b maps b coordinates into a (= pose of b seen from a).
Doosan posx = [x, y, z (mm), a, b, c (deg)], intrinsic ZYZ Euler.
"""
import json
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

HERE = Path(__file__).resolve().parent


# ---------------------------------------------------------------- config

def load_config(path=HERE / 'config.yaml'):
    return yaml.safe_load(Path(path).read_text())


def cpath(p):
    """config path -> absolute Path (relative to calibration/)."""
    p = Path(p).expanduser()
    return p if p.is_absolute() else (HERE / p).resolve()


DATA = cpath(load_config()['paths']['data'])


# ---------------------------------------------------------------- transforms

def make_T(Rm, t):
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def inv_T(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def rot_angle_deg(Rm):
    return float(np.degrees(np.arccos(np.clip((np.trace(Rm) - 1) / 2, -1, 1))))


def average_poses(Ts):
    """Mean of noisy poses: median translation, SVD-projected mean rotation."""
    Ts = np.asarray(Ts)
    U, _, Vt = np.linalg.svd(Ts[:, :3, :3].mean(axis=0))
    Rm = U @ Vt
    if np.linalg.det(Rm) < 0:
        U[:, -1] *= -1
        Rm = U @ Vt
    return make_T(Rm, np.median(Ts[:, :3, 3], axis=0))


def posx_to_T(posx):
    """Doosan [x,y,z (mm), a,b,c (deg, intrinsic ZYZ)] -> 4x4 in metres."""
    x, y, z, a, b, c = [float(v) for v in posx]
    return make_T(R.from_euler('ZYZ', [a, b, c], degrees=True).as_matrix(), np.array([x, y, z]) / 1000.0)


def T_to_posx(T):
    a, b, c = R.from_matrix(T[:3, :3]).as_euler('ZYZ', degrees=True)
    return [*(T[:3, 3] * 1000.0), a, b, c]


def rotation_spread_deg(Ts):
    Rs = [T[:3, :3] for T in Ts]
    return max(rot_angle_deg(a.T @ b) for a in Rs for b in Rs)


# ---------------------------------------------------------------- ArUco marker

def make_detector(dictionary='DICT_4X4_50', pixels_per_cell=None):
    """pixels_per_cell: bit-sampling resolution (OpenCV default 4).  A printed 100 mm marker seen obliquely with glare
    (2026-10-06 wrist view) was rejected at 4 and decoded correctly at 8-12; None keeps the OpenCV default."""
    import cv2
    aruco = cv2.aruco
    params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    if pixels_per_cell:
        params.perspectiveRemovePixelPerCell = int(pixels_per_cell)
    return aruco.ArucoDetector(aruco.getPredefinedDictionary(getattr(aruco, dictionary)), params)


def detect_marker(bgr, K, detector, marker_id, marker_size):
    """Pose of one ArUco marker in the camera frame (image must be undistorted).
    Returns dict(T=T_cam_marker, corners, reproj_px, tilt_deg) or None; tilt = marker normal vs viewing ray."""
    import cv2
    corners, ids, _ = detector.detectMarkers(bgr)
    if ids is None or marker_id not in ids.flatten():
        return None
    c = corners[list(ids.flatten()).index(marker_id)].reshape(4, 2).astype(np.float64)
    h = marker_size / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, c, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, None)
    T = make_T(cv2.Rodrigues(rvec)[0], tvec)
    ray = T[:3, 3] / np.linalg.norm(T[:3, 3])
    tilt = float(np.degrees(np.arccos(np.clip(abs(T[:3, 2] @ ray), 0, 1))))
    return dict(T=T, corners=c, reproj_px=float(np.linalg.norm(proj.reshape(4, 2) - c, axis=1).mean()), tilt_deg=tilt)


# ---------------------------------------------------------------- AX = XB

def park_martin(A_list, B_list):
    """Closed-form AX = XB (Park & Martin): rotation from the rotation-vector correlation, then translation by lstsq."""
    M = np.zeros((3, 3))
    for A, B in zip(A_list, B_list):
        M += np.outer(R.from_matrix(B[:3, :3]).as_rotvec(), R.from_matrix(A[:3, :3]).as_rotvec())
    w, V = np.linalg.eigh(M.T @ M)
    Rx = V @ np.diag(1 / np.sqrt(w)) @ V.T @ M.T
    U, _, Vt = np.linalg.svd(Rx)                    # inconsistent data can give det -1: project onto SO(3)
    Rx = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    C = np.vstack([A[:3, :3] - np.eye(3) for A in A_list])
    d = np.concatenate([Rx @ B[:3, 3] - A[:3, 3] for A, B in zip(A_list, B_list)])
    return make_T(Rx, np.linalg.lstsq(C, d, rcond=None)[0])


def _x_to_pair(x):
    return make_T(R.from_rotvec(x[:3]).as_matrix(), x[3:6]), make_T(R.from_rotvec(x[6:9]).as_matrix(), x[9:12])


def _pair_to_x(T1, T2):
    return np.concatenate([R.from_matrix(T1[:3, :3]).as_rotvec(), T1[:3, 3], R.from_matrix(T2[:3, :3]).as_rotvec(), T2[:3, 3]])


def _motion_pairs(Tbf, Tcm, cam_motion, min_rot_deg=3.0):
    A_list, B_list = [], []
    for i in range(len(Tbf)):
        for j in range(i + 1, len(Tbf)):
            A, B = inv_T(Tbf[i]) @ Tbf[j], cam_motion(Tcm[i], Tcm[j])
            if rot_angle_deg(A[:3, :3]) >= min_rot_deg and rot_angle_deg(B[:3, :3]) >= min_rot_deg:
                A_list.append(A)
                B_list.append(B)
    if len(A_list) < 3:
        raise ValueError('poses are too similar: rotate the flange more between samples')
    return A_list, B_list


# eye-to-hand: fixed camera, marker on the flange.  T_cam_marker = inv(T_base_cam) T_base_flange T_flange_marker

def _residual_e2h(x, Tbf, Tcm, rot_w):
    Tbc, Tfm = _x_to_pair(x)
    Tcb = inv_T(Tbc)
    out = []
    for a, b in zip(Tbf, Tcm):
        d = inv_T(Tcb @ a @ Tfm) @ b
        out += [R.from_matrix(d[:3, :3]).as_rotvec() * rot_w, d[:3, 3]]
    return np.concatenate(out)


def solve_eye_to_hand(T_base_flange, T_cam_marker, refine=True):
    """(T_base_cam, T_flange_marker) from N >= 3 pose pairs."""
    Tbf, Tcm = [np.asarray(T) for T in T_base_flange], [np.asarray(T) for T in T_cam_marker]
    if len(Tbf) < 3:
        raise ValueError('Need at least 3 samples')
    # T_cm_i^-1 T_cm_j = X^-1 (T_bf_i^-1 T_bf_j) X  ->  A X = X B with X = T_flange_marker
    Tfm = park_martin(*_motion_pairs(Tbf, Tcm, lambda a, b: inv_T(a) @ b))
    Tbc = inv_T(average_poses([Tcm[i] @ inv_T(Tfm) @ inv_T(Tbf[i]) for i in range(len(Tbf))]))
    if refine:
        x = least_squares(_residual_e2h, _pair_to_x(Tbc, Tfm), args=(Tbf, Tcm, 0.1), loss='soft_l1', f_scale=0.005).x
        Tbc, Tfm = _x_to_pair(x)
    return Tbc, Tfm


def evaluate_eye_to_hand(T_base_flange, T_cam_marker, T_base_cam, T_flange_marker):
    """Per-sample marker disagreement via robot vs via camera (pos mm, rot deg)."""
    pos, rot = [], []
    for a, b in zip(T_base_flange, T_cam_marker):
        via_robot, via_cam = a @ T_flange_marker, T_base_cam @ b
        pos.append(np.linalg.norm(via_robot[:3, 3] - via_cam[:3, 3]) * 1000)
        rot.append(rot_angle_deg(via_robot[:3, :3].T @ via_cam[:3, :3]))
    return np.array(pos), np.array(rot)


# eye-in-hand: wrist camera, marker fixed in the world.  T_base_flange_i X T_cam_marker_i = Z (constant)

def _residual_e2i(x, Tbf, Tcm, rot_w):
    X, Z = _x_to_pair(x)
    Zi = inv_T(Z)
    out = []
    for a, b in zip(Tbf, Tcm):
        d = Zi @ a @ X @ b
        out += [R.from_matrix(d[:3, :3]).as_rotvec() * rot_w, d[:3, 3]]
    return np.concatenate(out)


def solve_eye_in_hand(Tbf, Tcm, refine=True):
    """(T_flange_cam, T_base_marker) from N >= 3 samples of a fixed marker: Park-Martin, then robust (soft-L1) refine."""
    Tbf, Tcm = [np.asarray(T, dtype=np.float64) for T in Tbf], [np.asarray(T, dtype=np.float64) for T in Tcm]
    if len(Tbf) < 3:
        raise ValueError('need at least 3 samples')
    if rotation_spread_deg(Tbf) < 15.0:
        raise ValueError('flange orientations are too similar: rotate the flange more between samples')
    # Tbf_i X Tcm_i = Tbf_j X Tcm_j  ->  (Tbf_i^-1 Tbf_j) X = X (Tcm_i Tcm_j^-1)
    X = park_martin(*_motion_pairs(Tbf, Tcm, lambda a, b: a @ inv_T(b)))
    Z = average_poses([a @ X @ b for a, b in zip(Tbf, Tcm)])
    if refine:
        x = least_squares(_residual_e2i, _pair_to_x(X, Z), args=(Tbf, Tcm, 0.1), loss='soft_l1', f_scale=0.005).x
        X, Z = _x_to_pair(x)
    return X, Z


def evaluate_eye_in_hand(Tbf, Tcm, X, Z):
    """Per-sample disagreement of T_base_marker with the solved constant Z (pos mm, rot deg)."""
    pos, rot = [], []
    for a, b in zip(Tbf, Tcm):
        m = a @ X @ b
        pos.append(np.linalg.norm(m[:3, 3] - Z[:3, 3]) * 1000)
        rot.append(rot_angle_deg(Z[:3, :3].T @ m[:3, :3]))
    return np.array(pos), np.array(rot)


def leave_one_out(solve, evaluate, Tbf, Tcm):
    """Residual of each sample under a solve WITHOUT that sample (pos mm, rot deg arrays)."""
    pos, rot = [], []
    for k in range(len(Tbf)):
        rest = [i for i in range(len(Tbf)) if i != k]
        A, B = solve([Tbf[i] for i in rest], [Tcm[i] for i in rest])
        p, r = evaluate([Tbf[k]], [Tcm[k]], A, B)
        pos.append(p[0])
        rot.append(r[0])
    return np.array(pos), np.array(rot)


# fixed camera from wrist-located loose markers (2026-10-06; gripper hides the flange)

def fixed_cam_average(positions, scale):
    """Per-position T_base_cam = T_base_marker(wrist) inv(T_cam_marker(fixed)) and their average.  Kept for comparison:
    one marker's orientation noise x ~1.2 m range becomes camera-position error (11-111 mm spread seen)."""
    est = []
    for q in positions:
        Tw, Tf = np.array(q['wrist_T_cam_marker']), np.array(q['fixed_T_cam_marker'])
        Tw[:3, 3] *= scale
        Tf[:3, 3] *= scale
        est.append(np.array(q['T_base_flange']) @ np.array(q['T_flange_cam']) @ Tw @ inv_T(Tf))
    T = average_poses(est)
    res = [(float(np.linalg.norm(e[:3, 3] - T[:3, 3]) * 1000), float(rot_angle_deg(T[:3, :3].T @ e[:3, :3]))) for e in est]
    return T, res


def fixed_cam_pnp(positions, scale, K, idx=None):
    """T_base_cam by PnP on ALL marker corners: 3D corners in base from the wrist measurement, 2D pixels the fixed
    camera saw (its stored marker pose re-projected with the nominal size = the detected corners, so the fixed side does
    not depend on the marker scale).  Returns (T_base_cam, mean reprojection px per position)."""
    import cv2
    idx = list(range(len(positions))) if idx is None else idx
    size = positions[0].get('marker_size_m', 0.1)
    h, hn = size * scale / 2, size / 2
    corn = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    corn_nom = np.array([[-hn, hn, 0], [hn, hn, 0], [hn, -hn, 0], [-hn, -hn, 0]])
    obj, img = [], []
    for i in idx:
        q = positions[i]
        Tw = np.array(q['wrist_T_cam_marker'])
        Tw[:3, 3] *= scale
        Tbm = np.array(q['T_base_flange']) @ np.array(q['T_flange_cam']) @ Tw
        obj.append((Tbm[:3, :3] @ corn.T).T + Tbm[:3, 3])
        Tf = np.array(q['fixed_T_cam_marker'])
        px = (K @ ((Tf[:3, :3] @ corn_nom.T).T + Tf[:3, 3]).T).T
        img.append(px[:, :2] / px[:, 2:3])
    obj, img = np.vstack(obj), np.vstack(img)
    _, rv, tv = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_EPNP)
    _, rv, tv = cv2.solvePnP(obj, img, K, None, rv, tv, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    Tcb = make_T(cv2.Rodrigues(rv)[0], tv.ravel())
    proj, _ = cv2.projectPoints(obj, rv, tv, K, None)
    return inv_T(Tcb), np.linalg.norm(proj.reshape(-1, 2) - img, axis=1).reshape(-1, 4).mean(1)


# ---------------------------------------------------------------- files

def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2))


def load_json(path):
    return json.loads(Path(path).read_text())


def latest_t_base_cam():
    """Newest real_handeye session; inside it prefer T_base_cam_pnp.json (2026-10-06 fixed-camera PnP, camera moved),
    then the 2026-10-02 cube-GT corrected variants (old camera placement only), then plain T_base_cam.json."""
    root = DATA / 'real_handeye'
    sessions = sorted({f.parent for pattern in ('*/T_base_cam_pnp.json', '*/T_base_cam.json') for f in root.glob(pattern)
                       if not (f.parent / 'INVALID').exists()})    # (10/16) a session marked INVALID is never picked
    if not sessions:
        raise SystemExit('no real hand-eye result in data/real_handeye/; pass --t_base_cam')
    for name in ('T_base_cam_pnp.json', 'T_base_cam_markerscale_xyoffset.json', 'T_base_cam_zfix.json', 'T_base_cam.json'):
        if (sessions[-1] / name).is_file():
            return sessions[-1] / name


def newest_wrist_result():
    found = sorted((DATA / 'wrist_handeye').glob('*/result.json'))
    if not found:
        raise SystemExit('no data/wrist_handeye/*/result.json: run calibrate.py wrist first')
    return found[-1]


def load_fixed_K(cfg=None):
    cfg = cfg or load_config()
    return np.array(load_json(cpath(cfg['paths']['intrinsics']))['K'])


def check_T(T, what):
    """Refuse a malformed / non-rigid 4x4 (files and network input)."""
    T = np.asarray(T, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1])
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-3)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-3)):
        raise RuntimeError(f'invalid transform: {what}')
    return T


# ---------------------------------------------------------------- self-test (no hardware)

def selftest():
    import cv2
    rng = np.random.default_rng(0)
    print('posx round trip ...', end=' ')
    for _ in range(50):
        p = [*rng.uniform(-800, 800, 3), rng.uniform(-180, 180), rng.uniform(10, 170), rng.uniform(-180, 180)]
        assert np.allclose(posx_to_T(p), posx_to_T(T_to_posx(posx_to_T(p))), atol=1e-9)
    print('ok')

    print('marker detection on a synthetic image ...', end=' ')
    K = np.array([[900., 0, 640], [0, 900., 360], [0, 0, 1]])
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 1, 200)
    marker = cv2.copyMakeBorder(marker, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    true = make_T(R.from_euler('xyz', [200, 15, 10], degrees=True).as_matrix(), [0.02, -0.01, 0.5])
    h = 0.10 / 2 * 280 / 200
    px = (K @ (true[:3, :3] @ np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]]).T + true[:3, 3:4])).T
    H = cv2.getPerspectiveTransform(np.float32([[0, 0], [279, 0], [279, 279], [0, 279]]), (px[:, :2] / px[:, 2:3]).astype(np.float32))
    img = cv2.warpPerspective(marker, H, (1280, 720), borderValue=255)
    det = detect_marker(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), K, make_detector(), 1, 0.10)
    assert det is not None, 'marker not detected'
    dt, dr = np.linalg.norm(det['T'][:3, 3] - true[:3, 3]) * 1000, rot_angle_deg(det['T'][:3, :3].T @ true[:3, :3])
    assert dt < 3 and dr < 3, f'marker pose off: {dt:.2f} mm, {dr:.2f} deg'
    print(f'ok  (pos err {dt:.2f} mm, rot err {dr:.2f} deg)')

    print('eye-to-hand solve (synthetic):')
    Tbc = make_T(R.from_euler('xyz', [-125, 5, 178], degrees=True).as_matrix(), [0.9, 0.1, 0.6])
    Tfm = make_T(R.from_euler('xyz', [10, -5, 30], degrees=True).as_matrix(), [0.02, -0.01, 0.03])
    for noise_rot, noise_pos in ((0.0, 0.0), (0.3, 1.0)):
        Tbf, Tcm = [], []
        for _ in range(20):
            a = make_T((R.from_euler('xyz', [180, 0, 0], degrees=True) * R.from_rotvec(rng.normal(0, 0.5, 3))).as_matrix(),
                       [rng.uniform(0.4, 0.8), rng.uniform(-0.3, 0.3), rng.uniform(0.3, 0.7)])
            b = inv_T(Tbc) @ a @ Tfm @ make_T(R.from_rotvec(rng.normal(0, np.radians(noise_rot) / 1.7, 3)).as_matrix(),
                                              rng.normal(0, noise_pos / 1000 / 1.7, 3))
            Tbf.append(a)
            Tcm.append(b)
        d = inv_T(Tbc) @ solve_eye_to_hand(Tbf, Tcm)[0]
        print(f'  noise {noise_rot:.1f} deg / {noise_pos:.1f} mm -> {np.linalg.norm(d[:3, 3]) * 1000:.2f} mm, {rot_angle_deg(d[:3, :3]):.2f} deg')
        if noise_rot == 0.0:
            assert np.linalg.norm(d[:3, 3]) < 1e-4 and rot_angle_deg(d[:3, :3]) < 1e-2, 'noise-free solve is inexact'

    print('eye-in-hand solve (synthetic) ...', end=' ')
    rng = np.random.default_rng(1)
    X = make_T(R.from_euler('xyz', [5, -8, 12], degrees=True).as_matrix(), [0.03, -0.05, 0.08])
    Z = make_T(R.from_euler('xyz', [0, 180, 20], degrees=True).as_matrix(), [1.0, 0.05, 0.03])
    Tbf, Tcm = [], []
    for _ in range(14):
        a = make_T(R.from_euler('xyz', rng.uniform(-35, 35, 3) + [0, 180, 0], degrees=True).as_matrix(),
                   Z[:3, 3] + rng.uniform(-0.1, 0.1, 3) + [0, 0, 0.35])
        Tbf.append(a)
        Tcm.append(inv_T(a @ X) @ Z)
    Xs, Zs = solve_eye_in_hand(Tbf, Tcm)
    ex = np.linalg.norm(Xs[:3, 3] - X[:3, 3]) * 1000
    assert ex < 0.1 and rot_angle_deg(Xs[:3, :3].T @ X[:3, :3]) < 0.01, f'eye-in-hand off by {ex:.3f} mm'
    print(f'ok  ({ex:.4f} mm)')

    print('fixed camera from wrist-located markers (synthetic, scale 0.98) ...', end=' ')
    rng = np.random.default_rng(1)
    T_bc = make_T(R.from_euler('zyx', [150, -5, -125], degrees=True).as_matrix(), [1.2, 0.3, 0.9])
    Xw = make_T(R.from_euler('xyz', [-3, 5, 90], degrees=True).as_matrix(), [0.03, -0.05, 0.08])
    positions, s = [], 0.98
    for _ in range(8):
        Tm = make_T(R.from_euler('z', rng.uniform(0, 360), degrees=True).as_matrix()
                    @ R.from_euler('x', 180 + rng.uniform(-25, 25), degrees=True).as_matrix(),
                    [rng.uniform(0.7, 1.1), rng.uniform(-0.3, 0.3), 0.05])
        Tbf = make_T(R.from_euler('zyx', [rng.uniform(-10, 10), rng.uniform(40, 80), rng.uniform(-10, 10)], degrees=True).as_matrix(),
                     [rng.uniform(0.4, 0.7), rng.uniform(-0.2, 0.2), rng.uniform(0.3, 0.5)])
        Tf, Tw = inv_T(T_bc) @ Tm, inv_T(Tbf @ Xw) @ Tm
        Tf[:3, 3] = Tf[:3, 3] / s + rng.normal(0, 0.001, 3) / s                  # printed marker smaller -> looks farther
        Tw[:3, 3] = Tw[:3, 3] / s + rng.normal(0, 0.001, 3) / s
        positions.append(dict(T_base_flange=Tbf.tolist(), T_flange_cam=Xw.tolist(), wrist_T_cam_marker=Tw.tolist(),
                              fixed_T_cam_marker=Tf.tolist()))
    T, _ = fixed_cam_average(positions, s)
    err, rot = np.linalg.norm(T[:3, 3] - T_bc[:3, 3]) * 1000, rot_angle_deg(T[:3, :3].T @ T_bc[:3, :3])
    assert err < 3 and rot < 0.3, f'fixed-camera average off: {err:.2f} mm {rot:.3f} deg'
    print(f'ok  ({err:.2f} mm, {rot:.3f} deg)')
    print('self-test passed')


if __name__ == '__main__':
    selftest()


def yoloe_set_classes(model, names):
    """model.set_classes(names) with the text embeddings cached on disk (data/yoloe_text_pe/): computing them loads the
    572 MB MobileCLIP text model, ~2-3 s per process (보스 10/29: the demo start is slow)."""
    import hashlib
    import torch
    names = list(names)
    f = Path(__file__).resolve().parent / 'data/yoloe_text_pe' / (hashlib.sha1('\n'.join(names).encode()).hexdigest()[:16] + '.pt')
    try:
        pe = torch.load(f)
    except (OSError, RuntimeError, EOFError):
        pe = model.get_text_pe(names)
        f.parent.mkdir(parents=True, exist_ok=True)
        torch.save(pe.detach().cpu(), f)
    model.set_classes(names, pe)
