"""(보스 10/30) live GUI of the side-wall camera slot CNN: full frame with the Q3/Q4 crop boxes + each quadrant crop with
the CNN tips (numbered from the far end, 13 per row) and the target pin band.  Camera read only.
    ~/miniconda3/envs/foundationpose/bin/python experiments/side_cam/live_view.py [--pin 6] [--frame F.png] [--record 5]
Keys: [ / ] pin -/+, s save the window to outputs/side_cam_q3_20261029/live/, q quit."""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import slot_reader as S  # noqa: E402

SERIAL = '338122303684'


def draw(bgr, r, pin, colour):
    img = bgr.copy()
    for row in r['rows']:
        if row is not None:
            for i, p in enumerate(row):
                cv2.circle(img, (int(p[0]), int(p[1])), 4, colour, -1)
                cv2.putText(img, str(i + 1), (int(p[0]) - 5, int(p[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1)
    bd = S.band(r, pin)
    if bd is not None:
        over = img.copy()
        cv2.fillPoly(over, [bd.astype(np.int32)], (60, 230, 230))
        img = cv2.addWeighted(over, 0.45, img, 0.55, 0)
    return img, bd is not None


def main():
    a = argparse.ArgumentParser()
    a.add_argument('--pin', type=int, default=6)
    a.add_argument('--frame', default='', help='a saved frame instead of the live camera')
    a.add_argument('--record', type=int, default=0, help='save every N-th raw camera frame (no overlay) to '
                   'outputs/side_cam_q3_20261029/live/rec_HHMMSS/ for later evaluation; 0 = off')
    a = a.parse_args()
    net = S.SE.load(S.CKPT)
    pipe = None
    if not a.frame:
        import pyrealsense2 as rs
        pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_device(SERIAL)
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        pipe.start(cfg)
    still = cv2.imread(a.frame) if a.frame else None
    out = S.ROOT / 'outputs/side_cam_q3_20261029/live'
    pin, colours = a.pin, dict(Q3=(0, 0, 255), Q4=(255, 80, 0))
    rec = out / f'rec_{time.strftime("%H%M%S")}' if a.record and still is None else None
    if rec is not None:
        rec.mkdir(parents=True, exist_ok=True)
        print('recording raw frames to', rec, flush=True)
    cv2.namedWindow('side CNN', cv2.WINDOW_NORMAL)
    n_frame = 0
    try:
        while True:
            t0 = time.time()
            bgr = still if still is not None else np.asanyarray(pipe.wait_for_frames().get_color_frame().get_data())
            if rec is not None and n_frame % a.record == 0:
                cv2.imwrite(str(rec / f"{n_frame:06d}.jpg"), bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
            n_frame += 1
            full, tiles, status = bgr.copy(), [], []
            for q in ('Q3', 'Q4'):
                r = S.read(bgr, q, net)
                img, ok = draw(bgr, r, pin, colours[q])
                x0, y0, w, h = r['crop']
                cv2.rectangle(full, (x0, y0), (x0 + w, y0 + h), colours[q], 3)
                cv2.putText(full, q, (x0 + 8, y0 + 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, colours[q], 2)
                n = sum(len(x) for x in r['rows'] if x is not None)
                msg = f'{q} pin {pin}: ' + ('band OK' if ok else f'no band ({r["why"]})')
                status.append(msg)
                tile = cv2.resize(img[y0:y0 + h, x0:x0 + w], (640, 360))
                cv2.putText(tile, msg, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                tiles.append(tile)
            info = np.zeros((360, 640, 3), np.uint8)
            lines = ['side camera 338122303684 - slot CNN run_side_v2/ep07' + (' - REC' if rec is not None else ''),
                     'sim tip recall 0.994 / precision 0.987', ''] + status + \
                    ['', f'{(time.time() - t0) * 1000:.0f} ms/frame', '[ / ] pin   s save   q quit',
                     'pin k = gap between tines k, k+1 from the far end']
            for i, s in enumerate(lines):
                cv2.putText(info, s, (12, 30 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)
            win = np.vstack([np.hstack([cv2.resize(full, (640, 360)), info]), np.hstack(tiles)])
            cv2.imshow('side CNN', win)
            k = cv2.waitKey(1 if still is None else 50) & 0xFF
            if k == ord('q'):
                break
            if k == ord('['):
                pin = max(1, pin - 1)
            if k == ord(']'):
                pin = min(S.N_TINES - 1, pin + 1)
            if k == ord('s'):
                out.mkdir(parents=True, exist_ok=True)
                f = out / f'live_{time.strftime("%H%M%S")}_pin{pin}.png'
                cv2.imwrite(str(f), win)
                print('saved', f, flush=True)
    finally:
        if pipe is not None:
            pipe.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
