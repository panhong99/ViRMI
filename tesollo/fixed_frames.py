"""The fixed D455 frames for the Tesollo tools: the perception share (/dev/shm/fixed_latest.npz, rgb/depth/K/t) when a
viewer (demo_watch --audience --write_share, real_time_project/main.py) owns the camera and wrote it in the last
`fresh_s` seconds, else the camera opened here.  read() -> object with rgb, depth, K (a share frame newer than the last)."""
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
SHARE = Path('/dev/shm/fixed_latest.npz')


def share_fresh(fresh_s=2.0):
    return SHARE.exists() and time.time() - SHARE.stat().st_mtime < fresh_s


class FixedFrames:
    def __init__(self, serial, fresh_s=2.0, timeout_s=10.0):
        self.cam, self.last, self.timeout_s = None, 0.0, timeout_s
        if share_fresh(fresh_s):
            print(f'fixed camera: frames from {SHARE} (a viewer owns the camera)', flush=True)
        else:
            sys.path.insert(0, str(ROOT / 'real_time_project'))
            from real_time_utils import D455Source
            self.cam = D455Source(serial=str(serial))

    def read(self):
        if self.cam is not None:
            return self.cam.read()
        t0 = time.time()
        while time.time() - t0 < self.timeout_s:
            try:
                with np.load(SHARE) as z:
                    t = float(z['t'])
                    if t > self.last:
                        self.last = t
                        return SimpleNamespace(rgb=z['rgb'], depth=z['depth'], K=z['K'])
            except Exception:                                    # being replaced right now
                pass
            time.sleep(0.05)
        raise RuntimeError(f'{SHARE}: no new frame in {self.timeout_s:.0f} s')

    def stop(self):
        if self.cam is not None:
            self.cam.pipeline.stop()
