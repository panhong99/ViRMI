"""Tesollo demo: which basket quadrants already hold a bowl / plate, from the fixed D455 + YOLOE (read-only, no robot command).

Same rule as the 11/3 spectator view (calibration/demo_watch.py: bowl / plate mask points per quadrant of the measured
basket, 150 points = taken); a quadrant counts as taken when it is taken in at least half of the frames.  The arm must be
out of the view (home).  The camera must be free.
    ~/miniconda3/envs/foundationpose/bin/python tesollo/empty_quadrant.py --basket /tmp/tesollo_basket_X.json
-> PLATES {"Q3": 12} (plates standing in the basket, from depth, pin nearest to it), last line RESULT {"Q1": "bowl" | "plate" | null, ...}
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--basket', required=True, help='basket_vision.json (T)')
    p.add_argument('--frames', type=int, default=5)
    p.add_argument('--wait', type=float, default=0.0, help='wait up to this many s for --basket to be written (the demo '
                   'starts this while the basket is still being measured, 보스 10/29: slow start)')
    a = p.parse_args()
    sys.path.insert(0, str(ROOT / 'calibration'))
    sys.path.insert(0, str(ROOT / 'real_time_project'))
    import calib_core as cc
    import demo_watch as dw
    import plate_plans as pp
    from fixed_frames import FixedFrames
    Tbc = np.array(cc.load_json(cc.latest_t_base_cam())['T_base_cam'])
    det = dw.Detector()                                    # the model loads while the basket may still be measured
    t_end = time.time() + a.wait
    while True:
        try:
            res = json.loads(Path(a.basket).read_text())
            break
        except (OSError, ValueError):
            if time.time() > t_end:
                sys.exit(f'no basket file {a.basket}')
            time.sleep(0.2)
    if not res.get('ok'):
        sys.exit(f'basket not measured: {res.get("why")}')
    Tb = np.array(res['T'])
    cam = FixedFrames(pp.C['cameras']['fixed_serial'])
    votes, pins = [], {}
    try:
        for _ in range(15 if cam.cam is not None else 0):     # auto exposure settles (a shared camera has)
            cam.read()
        for _ in range(a.frames):
            fr = cam.read()
            K = np.asarray(fr.K, float)
            dets = det(fr.rgb)
            counts = dw.bowl_quadrants(dets, fr.depth, K, Tbc, Tb)
            plates = dw.bowl_quadrants(dets, fr.depth, K, Tbc, Tb, min_conf=0.15, name='plate')
            occ = dw.occupied(counts, plates)
            # (보스 10/29) a plate standing in the basket: YOLOE scores it 0.10-0.16, depth sees it (arm home cut out)
            for q, info in dw.depth_occupancy(fr.depth, K, Tbc, Tb, posj=list(pp.HOME_JOINTS_DEG)).items():
                occ[q] = occ[q] or info['kind']
                if info.get('pin'):
                    pins.setdefault(q, []).append(info['pin'])
            votes.append(occ)
    finally:
        cam.stop()
    occ = {}
    for q in dw.QUADRANTS:
        seen = [v[q] for v in votes if v[q]]
        occ[q] = max(set(seen), key=seen.count) if len(seen) * 2 >= len(votes) else None
    print('quadrants: ' + ', '.join(f'{q} {occ[q] or "empty"} ({sum(1 for v in votes if v[q])}/{len(votes)})' for q in sorted(occ)))
    plate_pins = {q: int(np.median(v)) for q, v in pins.items() if len(v) * 2 >= len(votes) and occ[q] == 'plate'}
    if plate_pins:
        print('plates standing in the basket (depth): ' + ', '.join(f'{q} pin {p_}' for q, p_ in plate_pins.items()))
    print('PLATES ' + json.dumps(plate_pins))
    print('RESULT ' + json.dumps(occ))


if __name__ == '__main__':
    main()
