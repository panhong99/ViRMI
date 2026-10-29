"""Tesollo demo: which basket quadrants already hold a bowl / plate, from the fixed D455 + YOLOE (read-only, no robot command).

Same rule as the 11/3 spectator view (calibration/demo_watch.py: bowl / plate mask points per quadrant of the measured
basket, 150 points = taken); a quadrant counts as taken when it is taken in at least half of the frames.  The arm must be
out of the view (home).  The camera must be free.
    ~/miniconda3/envs/foundationpose/bin/python tesollo/empty_quadrant.py --basket /tmp/tesollo_basket_X.json
-> last line: RESULT {"Q1": "bowl" | "plate" | null, ...}
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--basket', required=True, help='basket_vision.json (T)')
    p.add_argument('--frames', type=int, default=5)
    a = p.parse_args()
    sys.path.insert(0, str(ROOT / 'calibration'))
    sys.path.insert(0, str(ROOT / 'real_time_project'))
    import calib_core as cc
    import demo_watch as dw
    import plate_plans as pp
    from real_time_utils import D455Source
    Tbc = np.array(cc.load_json(cc.latest_t_base_cam())['T_base_cam'])
    Tb = np.array(json.loads(Path(a.basket).read_text())['T'])
    det = dw.Detector()
    cam = D455Source(serial=str(pp.C['cameras']['fixed_serial']))
    votes = []
    try:
        for _ in range(15):                                      # auto exposure settles
            cam.read()
        for _ in range(a.frames):
            fr = cam.read()
            K = np.asarray(fr.K, float)
            dets = det(fr.rgb)
            counts = dw.bowl_quadrants(dets, fr.depth, K, Tbc, Tb)
            plates = dw.bowl_quadrants(dets, fr.depth, K, Tbc, Tb, min_conf=0.15, name='plate')
            votes.append(dw.occupied(counts, plates))
    finally:
        cam.pipeline.stop()
    occ = {}
    for q in dw.QUADRANTS:
        seen = [v[q] for v in votes if v[q]]
        occ[q] = max(set(seen), key=seen.count) if len(seen) * 2 >= len(votes) else None
    print('quadrants: ' + ', '.join(f'{q} {occ[q] or "empty"} ({sum(1 for v in votes if v[q])}/{len(votes)})' for q in sorted(occ)))
    print('RESULT ' + json.dumps(occ))


if __name__ == '__main__':
    main()
