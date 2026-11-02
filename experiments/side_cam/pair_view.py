"""Read-only side-camera GUI for the final L/R slot-pair check.

The slot CNN supplies the two numbered tine rows.  YOLOE supplies a plate
silhouette.  For each row, the closest plate-contour crossing to the requested
slot band is expressed in local tine pitches.  Their common shift is a slot
position error; their difference is the pair/yaw error.

This tool only reports a suggested residual.  It never sends a robot command.

    ~/miniconda3/envs/foundationpose/bin/python experiments/side_cam/pair_view.py --pin 6
    ~/miniconda3/envs/foundationpose/bin/python experiments/side_cam/pair_view.py --frame FRAME.jpg --pin 6

Keys: [ / ] target pin, s save overlay, q quit.
"""
import argparse
import json
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / 'calibration'))
import slot_reader as S  # noqa: E402
import calib_core as cc  # noqa: E402

SERIAL = '338122303684'


def plate_masks(model, bgr, conf=0.04):
    """YOLOE plate masks as (mask, confidence), including edge-on low-score plates."""
    r = model.predict(bgr[:, :, ::-1].copy(), conf=conf, verbose=False, retina_masks=True)[0]
    if r.masks is None:
        return []
    return [(m > 0.5, float(c)) for m, c in zip(r.masks.data.cpu().numpy(), r.boxes.conf.cpu().numpy())]


def row_crossing(contour, row, pin, band_px=0.45, search_pitches=3.0):
    """Closest contour crossing to this row's target band -> offset in pitches, point, score."""
    direction = row[-1] - row[0]
    direction /= np.linalg.norm(direction)
    normal = np.array([-direction[1], direction[0]])
    pitch = float(np.median(np.linalg.norm(np.diff(row, axis=0), axis=1)))
    centre = (row[pin - 1] + row[pin]) / 2
    d = contour - centre
    along, across = d @ direction, np.abs(d @ normal)
    ok = (across < max(10.0, band_px * pitch)) & (np.abs(along) < search_pitches * pitch)
    if not ok.any():
        return None
    ids = np.flatnonzero(ok)
    j = ids[int(np.argmin(np.linalg.norm(d[ids], axis=1)))]
    return float(along[j] / pitch), contour[j], float(np.linalg.norm(d[j]) / pitch)


def estimate_pair(bgr, reading, pin, masks):
    """Choose the plate mask nearest the target band and return top/bottom pair values."""
    if any(r is None for r in reading['rows']):
        return None
    candidates = []
    for mask, confidence in masks:
        cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not cs:
            continue
        contour = max(cs, key=cv2.contourArea)[:, 0].astype(float)
        hits = [row_crossing(contour, row, pin) for row in reading['rows']]
        if any(h is None for h in hits):
            continue
        score = sum(h[2] for h in hits)
        candidates.append((score, -confidence, hits, mask, confidence))
    if not candidates:
        return None
    score, _, hits, mask, confidence = min(candidates, key=lambda x: (x[0], x[1]))
    offsets = np.array([h[0] for h in hits])
    pair_float = pin + offsets
    pair_error = float(pair_float[0] - pair_float[1])
    # 11/02 real trials: changing yaw by -12 deg changed L-R by about two slots.
    yaw_raw = -6.0 * pair_error
    yaw_limited = float(np.clip(yaw_raw, -8.0, 8.0))
    valid = bool(confidence >= 0.05 and score <= 5.0 and np.abs(offsets).max() <= 3.0)
    return dict(offsets=offsets, pair_float=pair_float, pair=np.rint(pair_float).astype(int),
                pair_error=pair_error, yaw_raw_deg=yaw_raw, yaw_deg=yaw_limited,
                confidence=confidence, score=score, hits=hits, mask=mask, valid=valid)


def draw_overlay(bgr, reading, pin, estimate, history):
    img = bgr.copy()
    for ri, row in enumerate(reading['rows']):
        if row is None:
            continue
        for i, p in enumerate(row):
            cv2.circle(img, tuple(np.rint(p).astype(int)), 4, (0, 220, 255), -1)
            cv2.putText(img, str(i + 1), tuple(np.rint(p + [-5, -8]).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 255), 1)
    # Outline every gap so the filled target cannot be mistaken for a detected
    # pair.  Only the requested pin is filled.
    if all(r is not None for r in reading['rows']):
        for k in range(1, S.N_TINES):
            gap = S.band(reading, k)
            cv2.polylines(img, [gap.astype(np.int32)], True, (150, 150, 150), 1)
    band = S.band(reading, pin)
    if band is not None:
        over = img.copy()
        cv2.fillPoly(over, [band.astype(np.int32)], (30, 255, 30))
        img = cv2.addWeighted(over, 0.45, img, 0.55, 0)
        c = np.rint(band.mean(0)).astype(int)
        cv2.putText(img, f'TARGET {pin}', tuple(c + [-45, -14]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 255, 20), 2)
    lines = [f'Q3 target pin {pin}', 'waiting for a plate near the target band']
    colour = (0, 180, 255)
    if estimate is not None:
        img[estimate['mask']] = (0.65 * img[estimate['mask']] + 0.35 * np.array([255, 0, 255])).astype(np.uint8)
        for name, h in zip(('L CROSS', 'R CROSS'), estimate['hits']):
            p = np.rint(h[1]).astype(int)
            cv2.circle(img, tuple(p), 9, (255, 255, 255), 2)
            cv2.putText(img, name, tuple(p + [10, -8]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        history.append(estimate['offsets'])
        med = np.median(np.array(history), axis=0)
        pair_f = pin + med
        err = float(pair_f[0] - pair_f[1])
        yaw_raw = -6.0 * err
        yaw = float(np.clip(yaw_raw, -8.0, 8.0))
        pair = np.rint(pair_f).astype(int)
        ok = estimate['valid'] and len(history) >= 3
        colour = (60, 220, 60) if ok and abs(err) < 0.45 else ((0, 180, 255) if ok else (0, 0, 255))
        lines = [f'predicted pair: L(top) {pair[0]} / R(bottom) {pair[1]}',
                 f'continuous: {pair_f[0]:.2f} / {pair_f[1]:.2f}   pair error {err:+.2f} slots',
                 f'suggested yaw residual: {yaw:+.1f} deg  (raw {yaw_raw:+.1f}, limited +/-8)',
                 f'plate conf {estimate["confidence"]:.2f}   geometry score {estimate["score"]:.2f}   '
                 + ('VALID' if ok else 'WAIT / LOW CONFIDENCE')]
    panel = np.zeros((190, img.shape[1], 3), np.uint8)
    for i, line in enumerate(lines):
        cv2.putText(panel, line, (18, 34 + i * 36), cv2.FONT_HERSHEY_SIMPLEX, 0.75, colour, 2)
    cv2.putText(panel, '[ / ] pin    s save    q quit    read-only: no robot command',
                (18, 178), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (210, 210, 210), 1)
    return np.vstack((img, panel)), lines


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('--pin', type=int, default=6)
    a.add_argument('--frame', default='', help='saved image instead of the live side camera')
    a.add_argument('--quadrant', default='Q3', choices=['Q3', 'Q4'])
    a.add_argument('--serial', default=SERIAL)
    a.add_argument('--save', default='', help='save one overlay and exit (requires --frame)')
    a = a.parse_args()
    if not S.REF.exists():
        raise SystemExit(f'missing numbered empty-basket reference: {S.REF}')
    ref = json.loads(S.REF.read_text())
    tip_net = S.SE.load(S.CKPT)
    from ultralytics import YOLOE
    plate_net = YOLOE(str(ROOT / 'yoloe-v8l-seg.pt'))
    cc.yoloe_set_classes(plate_net, ['plate'])

    pipe, still = None, cv2.imread(a.frame) if a.frame else None
    if a.frame and still is None:
        raise SystemExit(f'cannot read {a.frame}')
    if still is None:
        import pyrealsense2 as rs
        pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_device(a.serial)
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        pipe.start(cfg)
    pin, history, last_text = a.pin, deque(maxlen=7), None
    cv2.namedWindow('side pair residual', cv2.WINDOW_NORMAL)
    out_dir = ROOT / 'outputs/side_cam_pair_20261102'
    try:
        while True:
            t0 = time.time()
            if still is None:
                bgr = np.asanyarray(pipe.wait_for_frames().get_color_frame().get_data())
            else:
                bgr = still
            reading = S.read_anchored(bgr, a.quadrant, ref, tip_net)
            estimate = estimate_pair(bgr, reading, pin, plate_masks(plate_net, bgr))
            view, lines = draw_overlay(bgr, reading, pin, estimate, history)
            cv2.putText(view, f'{(time.time() - t0) * 1000:.0f} ms', (view.shape[1] - 130, view.shape[0] - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
            text = ' | '.join(lines[:3])
            if text != last_text:
                print(text, flush=True)
                last_text = text
            if a.save:
                Path(a.save).parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(a.save, view)
                print('saved', a.save)
                break
            cv2.imshow('side pair residual', cv2.resize(view, (1280, 910)))
            key = cv2.waitKey(1 if still is None else 50) & 0xFF
            if key == ord('q'):
                break
            if key in (ord('['), ord(']')):
                pin = int(np.clip(pin + (-1 if key == ord('[') else 1), 1, S.N_TINES - 1))
                history.clear()
            if key == ord('s'):
                out_dir.mkdir(parents=True, exist_ok=True)
                path = out_dir / f'pair_{time.strftime("%H%M%S")}_pin{pin}.png'
                cv2.imwrite(str(path), view)
                print('saved', path, flush=True)
    finally:
        if pipe is not None:
            pipe.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
