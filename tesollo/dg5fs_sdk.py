"""Tesollo DG-5F-S left: force-controlled grasp through the vendor DGSDK (the DGManager GRASP button, from code).

(보스 runs it; Developer Mode DIP 2, ROS 2 hand driver and DGManager CLOSED: the hand takes one client.)
    ~/miniconda3/bin/python3 tesollo/dg5fs_sdk.py [--mode 5F_2FINGER_1_AND_2] [--force 3] [--pre pregrasp] [--fixed 3 4 5] [--lock]
    DGManager recipes (보스 builds them in DGManager, Developer mode; read from its config.json, not from the hand):
    ~/miniconda3/bin/python3 tesollo/dg5fs_sdk.py --object plate    # pose plate_pre9 + 5F_5FINGER_PARALLEL, spread joints held, 7 N
    ~/miniconda3/bin/python3 tesollo/dg5fs_sdk.py --object bowl     # pose bowl_grasp30 + 5F_5FINGER, 7 N
    ~/miniconda3/bin/python3 tesollo/dg5fs_sdk.py --object auto     # fixed camera + YOLOE pick plate/bowl and its shift
    (both copied into poses.yaml / OBJECT_GRASP: DGManager pose recipes 20/30 vanished from its config.json on 10/27 14:31)
    (보스 10/27: start = pose 0 (all 0), then branch on the YOLOE class: plate -> pose 20, else bowl -> pose 30)
    Order (tesollo/arm_poses.yaml): plate = arm down to the grasp pose with the hand OPEN (pose 0), then step 1 goes
    to pose 9 (grasp-ready), step 2 grasps, then lift; bowl = the hand may already be in pose 30 on the way down.
Steps, one Enter each (q = stop, the hand keeps its pose; release + stop always run at the end):
    0. (--object) hand to its approach pose (plate open = pose 0, bowl pose 30), then the arm home -> above -> down
       (tesollo/arm_plan.py + mover, Enter per leg; s skips when the arm is already there)
    1. move to --pre (tesollo/poses.yaml name, 'open', or dgm:N = DGManager pose recipe N) in --motion_ms
    2. grasp: the SDK force grasp with --mode / --force [N]; fingers in --fixed stay where they are (position mode);
       --grasp_recipe N takes the mode, per-joint position mode and smooth flag from DGManager grasp recipe N instead
    2b. (--lock) hold the grasping fingers at their current angles (position mode): stops the force loop's back-and-forth
        correction (the visible tremble); the squeezed silicone tips keep the grip.  UNTESTED: try without the plate first
    3. HOLD until 보스 types 'release'; 'home' runs lift + home; (plate) 'basket Q3 6' plans (basket_plan.py) and runs
       lift + to_above, 'descend' goes down into the pins; then 4. open
Joints [deg], currents [mA] and the hottest motor are printed after each step (watch the temperature: stall heats up).
Needs the vendor Python SDK outside this repo (USB 2.Software/.../SampleProgram/dg_python-main.zip, unzipped to
--sdk) and cffi (conda base python has it; the ROS / foundationpose pythons do not).
"""
import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

POSES = Path(__file__).with_name('poses.yaml')
REPO = Path(__file__).resolve().parents[1]
PLAN_PY = Path.home() / 'miniconda3/envs/foundationpose/bin/python'
# arm legs (tesollo/arm_plan.py): (leg suffix, deg/s = deg/s^2), slow next to the object.  APPROACH runs before step 1
# (hand in its approach pose: plate open = pose 0, bowl pose 30), HOME runs when 'home' is typed while holding
ARM_APPROACH = ('above', 'down')
ARM_HOME = ('lift', 'home')
VEL = [60.0]                # arm deg/s = deg/s^2 for every leg (보스 10/29: sparse demo at 60; --vel)
APPROACH_HAND = {'plate': 'open', 'bowl': 'bowl_grasp30'}
BASKET_LEGS = ('lift', 'to_above', 'descend')     # fallback when lift + to_above cannot be merged into one spline
DGM_CONFIG = Path.home() / '.config/DGManager/config.json'
JOINTS = 20
# YOLOE object class -> (poses.yaml pre-pose, grasp mode, position-mode joints 1-based, force N), 보스 10/27:
# plate: pose 0 -> arm to the plate -> pose 9 (grasp-ready) -> 5 finger parallel (as grasp recipe 2: spread joints held) 7 N;
# bowl: pose 30 + grasp recipe 1 (5 finger to the centre), 7 N
OBJECT_GRASP = {'plate': ('plate_pre9', '5F_5FINGER_PARALLEL', [1, 5, 9, 13, 17], 7.0),
                'bowl': ('bowl_grasp30', '5F_5FINGER', [], 7.0)}


def measure_basket():
    """(10/29 보스: no basket-position dependency) the basket pose from the fixed D455 without markers, arm at home and
    out of the view (calibration/basket_vision.py --camera fixed); the file every basket plan of this run uses."""
    import math
    out_json = Path('/tmp') / f'tesollo_basket_{time.strftime("%Y%m%d_%H%M%S")}.json'
    share = Path('/dev/shm/fixed_latest.npz')               # (10/29) a viewer owns the camera: its shared frames
    cam = 'share' if share.exists() and time.time() - share.stat().st_mtime < 2.0 else 'fixed'
    r = subprocess.run([str(PLAN_PY), str(REPO / 'calibration/basket_vision.py'), '--camera', cam, '--out', str(out_json),
                        '--cut', 'bowl', '--majority', '4'], capture_output=True, text=True)
    try:
        res = json.loads(out_json.read_text())
    except (OSError, ValueError):
        res = {}
    if r.returncode or not res.get('ok'):
        sys.exit(f'basket not measured ({res.get("why") or r.stderr.strip()[-300:]}): clear the view / move the arm home, run again')
    T = res['T']
    print(f'basket measured: x {T[0][3] * 1000:.0f} mm, y {T[1][3] * 1000:+.0f} mm, yaw '
          f'{math.degrees(math.atan2(T[1][0], T[0][0])):+.1f} deg (frames within {res["spread_mm"]:.1f} mm) -> {out_json}', flush=True)
    BASKET_JSON[0] = str(out_json)


def perceive():
    """tesollo/perceive.py (fixed D455 + YOLOE): -> (object, dx, dy) of the object nearest the grasp spot, or exit."""
    out = subprocess.run([str(PLAN_PY), str(REPO / 'tesollo/perceive.py')], capture_output=True, text=True)
    print('\n'.join(l for l in out.stdout.splitlines() if not l.startswith('RESULT')), flush=True)
    m = re.search(r'^RESULT (plate|bowl) (\S+) (\S+)$', out.stdout, re.M)
    if out.returncode or not m:
        sys.exit(f'perception refused:\n{out.stderr[-800:]}')
    return m.group(1), float(m.group(2)), float(m.group(3))


SHIFT = [0.0, 0.0]          # object shift from the taught spot (perceive), applied to every arm plan of this run
SIM_CORR = ['']             # (--sim_corr) rl/hand_basket.py CEM json passed to basket_plan.py for the plate
WRIST_LESS = [45.0]         # (보스 10/29 real Q3 photo) plate: the hand turns this much less than the flat-plate plan
BASKET_JSON = ['']          # (10/29) the basket measured at the start (calibration/basket_vision.py), for every basket plan
MOVER_GO = ['--opening_done', '--auto']   # (보스 10/29) this script's own Enter / command starts a step; its legs then run
                            # on without the mover's 'start' + Enter per leg (--leg_enter: the old prompts)
BASKET = {}                 # the last basket plan of this run (basket_plan.py): its path, for 'descend'
# (보스 10/29) 'basket' alone = the empty quadrant by itself: a bowl into any empty quadrant, plates into an empty Q3 / Q4,
# the 1st plate front (small pin numbers, far from the robot), the 2nd back in the same quadrant.  What went in is kept in
# DEMO_STATE across runs (one run = one object); a new day or --new_demo starts it empty.
DEMO_STATE = Path('/tmp/tesollo_demo_state.json')
GRIPPER = [False, None]     # (--gripper_path [--wrist d4 d5 d6], 보스 10/29) plate: the 2F-85 insert path + a wrist offset
TAUGHT_IX = [None]          # (--taught [N], 보스 10/29) plate: basket = the taught insert N of taught_inserts.json
TEACH = [False]             # (--teach, 보스 10/29) plate: the plan stops above the basket, the insert is taught by the pendant
TAUGHT = REPO / 'tesollo/taught_inserts.json'   # every taught insert: joints + basket pose + plate in the hand
SEEN = {}                   # quadrant -> 'bowl' / 'plate' / None, the fixed camera at the start of this run (empty_quadrant.py)
BOWL_QUADRANTS = ('Q1', 'Q2', 'Q4', 'Q3')       # far ones first: Q3 / Q4 stay free for the plates
PLATE_QUADRANTS = ('Q3', 'Q4')
PINS = 12                   # pins per Q3 / Q4 row; pin 1 = farthest from the robot
# as pipeline.region_pins (min spacing 4 as the gripper demo).  The gripper's first pins (config Q4 front 3, back 10) do
# not plan for the hand (10/29 live basket, --wrist_less 45: Q3 front 6, back 12-10; Q4 front 6 / 5, back 12 / 11 plan;
# Q3 1-5, Q4 1-4 and 10 refused): front starts at the middle (6), back at the nearest (12)
FRONT_FIRST, BACK_FIRST, SPACING = {}, {}, 4


def load_state():
    try:
        st = json.loads(DEMO_STATE.read_text())
    except (OSError, ValueError):
        st = {}
    return st if st.get('date') == time.strftime('%Y-%m-%d') else {'date': time.strftime('%Y-%m-%d'), 'bowls': [], 'plates': []}


def save_state(st):
    DEMO_STATE.write_text(json.dumps(st))


def see_quadrants():
    """Bowls / plates already in the basket quadrants (fixed camera, arm home) -> SEEN."""
    out = subprocess.run([str(PLAN_PY), str(REPO / 'tesollo/empty_quadrant.py'), '--basket', BASKET_JSON[0]],
                         capture_output=True, text=True)
    m = re.search(r'^RESULT (.+)$', out.stdout, re.M)
    if out.returncode or not m:
        print(f'   quadrants not seen (only the remembered ones count):\n{out.stderr[-400:]}', flush=True)
        return
    SEEN.update(json.loads(m.group(1)))
    print(out.stdout.strip().splitlines()[-2], flush=True)


def front_pins(q, taken):
    top = min(taken) - SPACING if taken else FRONT_FIRST.get(q, PINS // 2)
    cand = list(range(top, 0, -1)) + ([] if taken else list(range(top + 1, PINS // 2 + 1)))
    return [c for c in cand if all(abs(c - t) >= SPACING for t in taken)]


def back_pins(q, taken):
    cand = list(range(PINS, PINS // 2, -1))
    if q in BACK_FIRST:
        cand = [c for c in cand if c <= BACK_FIRST[q]] + sorted(c for c in cand if c > BACK_FIRST[q])
    return [c for c in cand if all(abs(c - t) >= SPACING for t in taken)]


def auto_targets(obj):
    """[(quadrant, pin, why)] to try in order for 'basket' alone."""
    st = load_state()
    plate_q = [p_['q'] for p_ in st['plates']]
    full = {q for q, v in SEEN.items() if v} | set(st['bowls'])
    if obj == 'bowl':
        return [(q, 6, 'empty') for q in BOWL_QUADRANTS if q not in full and q not in plate_q]
    if plate_q and plate_q.count(plate_q[-1]) == 1:            # the 2nd plate: back of the 1st one's quadrant
        q = plate_q[-1]
        taken = [p_['pin'] for p_ in st['plates'] if p_['q'] == q]
        return [(q, pin, f'back (front plate at pin {taken[0]})') for pin in back_pins(q, taken)]
    return [(q, pin, 'front') for q in PLATE_QUADRANTS if q not in full and q not in plate_q for pin in front_pins(q, [])]


def record(obj, quadrant, pin):
    st = load_state()
    if obj == 'bowl':
        st['bowls'].append(quadrant)
    else:
        st['plates'].append({'q': quadrant, 'pin': pin})
    save_state(st)
    print(f'   remembered: bowls {st["bowls"]}, plates {[(p_["q"], p_["pin"]) for p_ in st["plates"]]} ({DEMO_STATE})', flush=True)


def auto_basket(obj, centre_h=None):
    """'basket' alone: the first target that plans, then lift + carry + descend."""
    targets = auto_targets(obj)
    if not targets:
        print(f'   no empty quadrant left for a {obj} (seen {SEEN}, remembered {load_state()}); '
              '"basket Q.. pin" picks one by hand', flush=True)
        return
    for q, pin, why in targets[:8]:
        print(f'   -> {obj} into {q}' + (f' pin {pin} ({why})' if obj == 'plate' else f' ({why})'), flush=True)
        if basket(obj, q, pin, centre_h) is not None:
            return
    print('   none of these planned; "basket Q.. pin" picks one by hand', flush=True)


def mover(plan, leg, v):
    """One mover leg (Enter to go); True if it ran."""
    r = subprocess.run(['bash', str(REPO / 'calibration/run.sh'), 'mover', '--plan', plan, '--leg', leg, '--smooth',
                        '--go_per', 'leg', '--vel_deg_s', str(v), '--acc_deg_s2', str(v), '--execute', '--go_enter']
                       + MOVER_GO)
    if r.returncode:
        print(f'   mover stopped at {leg} (exit {r.returncode})', flush=True)
    return r.returncode == 0


def basket(obj, quadrant, pin, centre_h=None):
    """While the object is held: basket_plan.py (plate: plate_plans' basket planner with the Tesollo grasp, into the pins;
    bowl: upright into the middle of a quadrant), then lift, to_above and descend (Enter per leg); BASKET['down'] once
    the descend has run, so 'release' knows the object is in the basket.  -> None: no plan, False: a leg stopped, True."""
    if TAUGHT_IX[0] is not None and obj == 'plate':      # (보스 10/29) the pendant-taught insert, one sparse spline after
        out = subprocess.run([str(PLAN_PY), str(REPO / 'tesollo/taught_plan.py'), '--index', str(TAUGHT_IX[0]),   # the grasp
                              '--quadrant', quadrant, '--pin', str(pin), '--dx', str(SHIFT[0]), '--dy', str(SHIFT[1])]
                             + (['--basket', BASKET_JSON[0]] if BASKET_JSON[0] else []), capture_output=True, text=True)
    else:
        out = None
    out = out or subprocess.run([str(PLAN_PY), str(REPO / 'tesollo/basket_plan.py'), '--object', obj, '--quadrant', quadrant,
                          '--pin', str(pin), '--dx', str(SHIFT[0]), '--dy', str(SHIFT[1])]
                         + (['--centre_h', str(centre_h)] if centre_h is not None else [])
                         + (['--sim_corr', SIM_CORR[0]] if SIM_CORR[0] and obj == 'plate' else [])
                         + (['--wrist_less', str(WRIST_LESS[0])] if WRIST_LESS[0] and obj == 'plate' and not GRIPPER[0] else [])
                         + (['--gripper_path', '--plates'] + [f'{p_["q"]}:{p_["pin"]}' for p_ in load_state()['plates']]
                            + (['--wrist'] + [str(v) for v in GRIPPER[1]] if GRIPPER[1] else []) if GRIPPER[0] and obj == 'plate' else [])
                         + (['--basket', BASKET_JSON[0]] if BASKET_JSON[0] else []), capture_output=True, text=True)
    print('\n'.join(l for l in out.stdout.splitlines() if l.startswith(('basket', 'OK', '  ', 'merged', 'wrist', 'carry'))), flush=True)
    m = re.search(r'^plan: (\S+)', out.stdout, re.M)
    if out.returncode or not m:
        print(f'   basket plan failed:\n{out.stdout[-600:]}{out.stderr[-600:]}', flush=True)
        return None
    BASKET['plan'], BASKET['target'] = m.group(1), (obj, quadrant, pin)
    BASKET.pop('out_leg', None)
    planned_legs = json.loads(Path(BASKET['plan']).read_text()).get('legs', {})
    if 'return_home' in planned_legs:
        BASKET['out_leg'] = 'return_home'
    print(f'   basket plan {m.group(1)} (after the release: vertical rise + home in one spline)', flush=True)
    BASKET['down'] = False
    mg = re.search(r'^merged: (\S+plan\.json)', out.stdout, re.M)
    # (보스 10/27, as the 2F-85 pipeline) grasp -> above the basket in ONE spline (lift + to_above), then the descent
    legs = [(mg.group(1), 'lift_carry'), (m.group(1), 'descend')] if mg else [(m.group(1), lg) for lg in BASKET_LEGS]
    if 'post_grasp' in planned_legs:                    # (보스 10/29) grasp -> slot as ONE spline (taught_plan.py)
        legs = [(m.group(1), 'post_grasp')]
    if TEACH[0] and obj == 'plate' and TAUGHT_IX[0] is None:   # (보스 10/29) only up to above the basket; the pendant inserts
        legs = [(teach_stop(m.group(1), mg.group(1) if mg else None), 'lift_carry')]
    for plan, leg in legs:
        if not mover(plan, leg, VEL[0]):
            return False
    if TEACH[0] and obj == 'plate' and TAUGHT_IX[0] is None and not teach_insert(m.group(1), quadrant, pin):
        return False
    BASKET['down'] = True
    record(*BASKET['target'])
    print('   in the basket: type "release"', flush=True)
    return True


def robot_joints():
    """The M1013 joints [deg] now (read-only ROS service, tesollo/read_posj.py), or None."""
    cmd = ['env', '-u', 'PYTHONPATH', '-u', 'PYTHONHOME', '-u', 'LD_LIBRARY_PATH', '-u', 'CONDA_PREFIX', '-u',
           'CONDA_DEFAULT_ENV', 'PATH=/usr/bin:/bin', 'bash', '-c',
           f'source /opt/ros/jazzy/setup.bash; source {REPO.parent / "doosan_ws/install/setup.bash"}; '
           f'exec /usr/bin/python3 {REPO / "tesollo/read_posj.py"}']
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=40)
    if out.returncode:
        print(f'   joints not read: {out.stderr.strip()[-300:]}', flush=True)
        return None
    return json.loads(out.stdout.strip().splitlines()[-1])


def teach_stop(plan_path, merged_path):
    """(보스 10/29: --teach went on down) to_above ends with the rule carry's straight drop onto the above pose (from
    quadrant_plan drop_start): for teaching the arm stops at the carry height over the slot, before that drop."""
    plan = json.loads(Path(plan_path).read_text())
    ds = int(plan.get('quadrant_plan', {}).get('drop_start') or len(plan['legs']['to_above']['waypoints_deg']) - 1)
    lift = plan['legs']['lift']['waypoints_deg']
    if merged_path:
        mp = json.loads(Path(merged_path).read_text())
        mp['legs']['lift_carry']['waypoints_deg'] = mp['legs']['lift_carry']['waypoints_deg'][:len(lift) + ds]
    else:
        mp = plan
        mp['legs']['lift_carry'] = dict(plan['legs']['to_above'], waypoints_deg=lift + plan['legs']['to_above']['waypoints_deg'][1:ds + 1])
    out = Path(plan_path).with_name('plan_teach_stop.json')
    out.write_text(json.dumps(mp, indent=1))
    print(f'   teach: the arm stops over the slot at the carry height (to_above {ds}/{len(plan["legs"]["to_above"]["waypoints_deg"]) - 1}, '
          'no drop)', flush=True)
    return str(out)


def teach_insert(plan_path, quadrant, pin):
    """(보스 10/29) the arm waits above the basket; 보스 teaches the insert with the pendant.  Enter records the joints
    (each pendant point), 'done' ends (the last record = the insert pose), 'undo' drops the last one.  Saved to TAUGHT
    with the basket pose and the plate in the hand; the way back for step 5 = the records reversed -> above -> to_home."""
    above = robot_joints()
    if above is None:
        print('   ROS joints needed for teaching (Doosan bringup on?): fix it and type "basket" again', flush=True)
        return False
    pts = []
    print(f'\n   TEACH {quadrant} pin {pin}: move the plate in with the pendant.  Enter = record the joints now, '
          '"undo" = drop the last, "done" = finished (the last record is the insert pose)', flush=True)
    while True:
        cmd = input(f'   teach [{len(pts)} recorded]> ').strip().lower()
        if cmd == 'undo' and pts:
            print(f'   dropped {pts.pop()}', flush=True)
        elif cmd == 'done':
            q = robot_joints()
            if not pts or (q and max(abs(x - y) for x, y in zip(q, pts[-1])) > 0.5):
                print('   record the final insert pose first (Enter at the insert), then done', flush=True)
                continue
            break
        elif cmd == '':
            q = robot_joints()
            if q:
                pts.append([round(v, 3) for v in q])
                print(f'   {len(pts)}: {pts[-1]}', flush=True)
        else:
            print('   Enter / undo / done', flush=True)
    plan = json.loads(Path(plan_path).read_text())
    entry = dict(time=time.strftime('%Y-%m-%d %H:%M:%S'), quadrant=quadrant, pin=pin, plan=plan_path,
                 basket_T=plan.get('basket_T'), plate_in_tcp=plan.get('plate_in_tcp'), above_deg=above, waypoints_deg=pts)
    try:
        allt = json.loads(TAUGHT.read_text())
    except (OSError, ValueError):
        allt = []
    TAUGHT.write_text(json.dumps(allt + [entry], indent=1))
    print(f'   taught insert saved: {TAUGHT} (#{len(allt) + 1}, {len(pts)} points)', flush=True)
    plan['created'] = time.strftime('%Y-%m-%d %H:%M:%S')
    plan['source'] = plan.get('source', '') + ' + taught insert (pendant)'
    back, keys = [], [pts[-1]] + pts[-2::-1] + [above]
    for qa, qb in zip(keys[:-1], keys[1:]):                       # joint-space line between the records, <= 2 deg steps
        steps = max(1, int(max(abs(x - y) for x, y in zip(qa, qb)) / 2.0) + 1)
        back += [[x + (y - x) * i / steps for x, y in zip(qa, qb)] for i in range(steps)]
    back.append(above)
    plan['legs']['taught_out'] = dict(waypoints_deg=back, planner='taught reversed', clearance_m=None,
                                      min_clearance_straight_m=plan.get('margin_m', 0.0))
    plan['legs']['to_home']['waypoints_deg'][0] = above
    out = Path(plan_path).with_name('plan_taught_out.json')
    out.write_text(json.dumps(plan, indent=1))
    BASKET['plan'], BASKET['out_leg'] = str(out), 'taught_out'
    return True


def arm_legs(obj, legs):
    """Fresh arm plan for the taught grasp pose (arm_plan.py), then the mover for these legs; True if all ran.
    The mover refuses a leg if the arm is not at its start, and asks for Enter before each leg."""
    out = subprocess.run([str(PLAN_PY), str(REPO / 'tesollo/arm_plan.py'), 'plate' if obj == 'plate' else 'bowl',
                          '--dx', str(SHIFT[0]), '--dy', str(SHIFT[1])], capture_output=True, text=True)
    m = re.search(r'^plan: (\S+)', out.stdout, re.M)
    if out.returncode or not m:
        print(f'   arm plan failed:\n{out.stdout[-800:]}{out.stderr[-800:]}', flush=True)
        return False
    for leg in legs:
        v = VEL[0]
        name = f'{"plate" if obj == "plate" else "bowl"}_{leg}'
        r = subprocess.run(['bash', str(REPO / 'calibration/run.sh'), 'mover', '--plan', m.group(1), '--leg', name,
                            '--smooth', '--go_per', 'leg', '--vel_deg_s', str(v), '--acc_deg_s2', str(v),
                            '--execute', '--go_enter'] + MOVER_GO)
        if r.returncode:
            print(f'   mover stopped at {name} (exit {r.returncode})', flush=True)
            return False
    return True


def object_grasp(obj):
    """Hand pre-pose / grasp mode / held joints / force for a detected class: plate -> pinch, anything else -> bowl grasp."""
    return OBJECT_GRASP['plate' if obj == 'plate' else 'bowl']


def dgm_recipe(kind, n):
    """DGManager recipe (kind = 'poses' | 'grasps') saved for this hand in Developer mode."""
    import json
    recipes = json.loads(DGM_CONFIG.read_text())['DG-5F-S20-L']['Developer']['recipes'][kind]
    if str(n) not in recipes:
        sys.exit(f'no DGManager {kind} recipe {n} in {DGM_CONFIG}: {sorted(recipes, key=int)}')
    return recipes[str(n)]


def load_pose(name):
    import math
    import yaml
    if name == 'open':
        return [0.0] * JOINTS
    if name.startswith('dgm:'):
        return [float(v) for v in dgm_recipe('poses', name[4:])['targetJoint'][:JOINTS]]
    saved = yaml.safe_load(POSES.read_text())
    if name not in saved:
        sys.exit(f'no pose "{name}" in {POSES}: {["open", *saved]}')
    return [math.degrees(v) for v in saved[name]['joints_rad']]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--sdk', default='/home/pan/pan/tesollo_ws/dg_python-main/src')
    p.add_argument('--ip', default='169.254.186.73')
    p.add_argument('--port', type=int, default=502)
    p.add_argument('--mode', default='5F_2FINGER_1_AND_2', help='DGGraspMode name without the leading _ (5F_3FINGER, ...)')
    p.add_argument('--force', type=float, default=3.0, help='grasp force [N] (start low; DGManager range 0-20 N)')
    p.add_argument('--pre', default='pregrasp', help='pose before the grasp (poses.yaml name or open)')
    p.add_argument('--fixed', type=int, nargs='*', default=[3, 4, 5], help='fingers kept in position mode during the grasp')
    p.add_argument('--object', help='auto = see it first (perceive.py: class + shift of the taught arm pose); '
                                    'YOLOE class: plate -> pose 9 + 5 finger parallel, else bowl (pose 30, 5 finger), '
                                    '7 N (overrides --pre, --mode, --force, --grasp_recipe)')
    p.add_argument('--grasp_recipe', type=int, help='DGManager grasp recipe: mode, position mode, smooth (not its force)')
    p.add_argument('--lock', action='store_true', help='after the grasp, hold the grasping fingers in position mode')
    p.add_argument('--basket_json', default='', help='use this basket_vision.json instead of measuring the basket now')
    p.add_argument('--fixed_basket', action='store_true', help='(old) the fixed 11/3 basket file + the 10/27 by-eye shifts')
    p.add_argument('--sim_corr', default='', help='(opt-in, plate) learned insert correction json from rl/hand_basket.py cem '
                                                  '(absolute path); use after tesollo/measure_held.py')
    p.add_argument('--wrist_less', type=float, default=45.0, help='(plate) turn the hand this much less from the grasp to the '
                   'insert than the flat-plate plan [deg] (보스 10/29: 45; 0 = the old plan)')
    p.add_argument('--gripper_path', action='store_true', help='(plate) after the grasp follow the 2F-85 gripper insert path '
                   '(its planner at the measured basket) with a J4-J6 offset; no hand collision check')
    p.add_argument('--wrist', type=float, nargs=3, help='--gripper_path: J4 J5 J6 offset [deg] (default hand - gripper at the lift end)')
    p.add_argument('--planner', action='store_true', help='(plate) the basket planner (basket_plan.py) instead of the taught '
                   'insert, which is the default once tesollo/taught_inserts.json exists (보스 10/29)')
    p.add_argument('--taught', type=int, nargs='?', const=-1, default=None,
                   help='(plate) "basket" runs the pendant-taught insert N of tesollo/taught_inserts.json (default the last): '
                        'lift -> teach 1 (over the basket) -> 2 (1 cm over the slot) -> 3 (in the slot); back 3 -> 2 -> 1 -> home')
    p.add_argument('--teach', action='store_true', help='(plate) the arm stops above the basket; teach the insert with the '
                   'pendant, Enter records the joints (tesollo/taught_inserts.json); release, then back the same way + home')
    p.add_argument('--new_demo', action='store_true', help='forget the bowls / plates put in the basket earlier today')
    p.add_argument('--leg_enter', action='store_true', help='(old) the mover asks "start" and Enter before every arm leg')
    p.add_argument('--vel', type=float, default=60.0, help='arm speed deg/s (= deg/s^2) for every arm leg (default 60)')
    p.add_argument('--motion_ms', type=int, default=3000, help='time per pose move [ms] (slow)')
    a = p.parse_args()
    VEL[0] = a.vel
    TEACH[0] = a.teach
    TAUGHT_IX[0] = a.taught if a.taught is not None else (None if a.planner or not TAUGHT.exists() else -1)
    if a.object in ('plate', 'auto') and TAUGHT_IX[0] is not None:
        print(f'plate insert: taught #{TAUGHT_IX[0]} ({TAUGHT.name}), one spline after the grasp (--planner = the planner)', flush=True)
    GRIPPER[:] = [a.gripper_path, a.wrist]
    MOVER_GO[:] = [] if a.leg_enter else MOVER_GO
    WRIST_LESS[0] = a.wrist_less
    SIM_CORR[0] = str(Path(a.sim_corr).resolve()) if a.sim_corr else ''
    held = None
    if a.basket_json:
        BASKET_JSON[0] = str(Path(a.basket_json).resolve())
    elif a.object and not a.fixed_basket:
        measure_basket()
    if a.new_demo:
        DEMO_STATE.unlink(missing_ok=True)
    if a.object and BASKET_JSON[0]:
        see_quadrants()                                          # arm home, the basket in view
        st = load_state()
        print(f'remembered in the basket today: bowls {st["bowls"]}, plates {[(p_["q"], p_["pin"]) for p_ in st["plates"]]}'
              ' (--new_demo clears)', flush=True)
    if a.object == 'auto':
        a.object, SHIFT[0], SHIFT[1] = perceive()
        if input(f'\nseen: {a.object}, {SHIFT[0] * 1000:+.0f} / {SHIFT[1] * 1000:+.0f} mm from the taught spot.  '
                 'Enter = go on, q = stop: ').strip().lower() == 'q':
            return
    if a.object:
        a.pre, a.mode, held, a.force = object_grasp(a.object)
        a.grasp_recipe = None
    # manual: Grasp Force 0-200 x 0.1 N; fingertip max 19.4 N, long stall near max torque heats the motors
    if not 0.5 <= a.force <= 20.0:
        sys.exit('--force must be 0.5 .. 20 N')
    if a.motion_ms < 1500:
        sys.exit('--motion_ms must be >= 1500')
    sys.path.insert(0, a.sdk)
    from dgsdk import (DGSDK, CommunicationMode, ControlMode, DGGraspMode, DGModel, DGResult, GripperSetting,
                       GripperSystemSetting, ReceivedDataType)
    mode = getattr(DGGraspMode, '_' + a.mode, None)
    hold, smooth = [1 if f + 1 in a.fixed else 0 for f in range(5) for _ in range(4)], 1
    if held is not None:
        hold, smooth = [1 if j + 1 in held else 0 for j in range(JOINTS)], 0
    if a.grasp_recipe is not None:
        r = dgm_recipe('grasps', a.grasp_recipe)
        mode, hold, smooth = DGGraspMode(r['graspMode']), [int(v) for v in r['positionMode'][:JOINTS]], r['smoothGrasping']
        print(f'DGManager grasp recipe {a.grasp_recipe}: {mode.name.lstrip("_")}, position-mode joints '
              f'{[j + 1 for j in range(JOINTS) if hold[j]]}, smooth {smooth}, force {a.force} N (from --force)')
    if mode is None:
        sys.exit(f'unknown --mode {a.mode}: {[m.name.lstrip("_") for m in DGGraspMode if m.name.startswith("_5F")]}')
    pre = load_pose(a.pre)

    # (10/27 real) ConnectToGripper segfaults unless the connected/disconnected callbacks are registered first (as the
    # vendor examples do); data is polled with get_gripper_data, no struct-by-value data callback
    g = DGSDK()

    def check(step, r):
        print(f'  {step}: {r.name}', flush=True)
        if r != DGResult.NONE:
            raise SystemExit(f'{step} failed: {r.name}')

    def show():
        time.sleep(0.3)
        try:
            d = g.get_gripper_data()
        except RuntimeError as e:
            print(f'  (no data: {e})')
            return
        joint, current, temp = list(d.joint[:JOINTS]), list(d.current[:JOINTS]), list(d.temperature[:JOINTS])
        for f in range(5):
            s = slice(4 * f, 4 * f + 4)
            print(f'  finger {f + 1}: ' + '  '.join(f'{v:+6.1f}' for v in joint[s]) + '  deg | '
                  + ' '.join(f'{c:6.0f}' for c in current[s]) + ' mA')
        t = max(temp)
        print(f'  hottest motor {t:.1f} C (J{temp.index(t) + 1})', flush=True)

    def ask(what):
        if input(f'\n{what}  Enter = go, q = stop: ').strip().lower() == 'q':
            raise KeyboardInterrupt

    check('system', g.set_gripper_system(GripperSystemSetting.create(
        ip=a.ip, port=a.port, control_mode=ControlMode.DEVELOPER, communication_mode=CommunicationMode.ETHERNET)))
    g.on_connected(lambda: print('  -> connected', flush=True))
    g.on_disconnected(lambda: print('  -> disconnected', flush=True))
    check('connect', g.connect_to_gripper())
    started = False
    try:
        check('option', g.set_gripper_option(GripperSetting.create(
            # (10/27 real) joint/finger count 20/5 -> SystemStart NOT_FOUND_MODEL; DGManager sends 0/0 and starts fine
            model=DGModel.DG_5F_S_LEFT, joint_count=0, finger_count=0,
            received_data_type=[ReceivedDataType.JOINT, ReceivedDataType.CURRENT, ReceivedDataType.TEMPERATURE, 0, 0, 0, 0])))
        # (10/27 real) start right after the option -> NOT_FOUND_MODEL; the hand needs a moment to report its model
        for _ in range(20):
            time.sleep(0.1)
            r = g.system_start()
            if r != DGResult.NOT_FOUND_MODEL:
                break
        check('start', r)
        started = True
        check('motion time', g.set_motion_time_all_equal(a.motion_ms))
        # (10/27 real) in Developer mode the hand has no gains until the PC sends them (DGManager does it on connect:
        # initUpdateRecipe + LoadRecipeGainData); without them every move returned NONE but the currents stayed 0 mA
        # and nothing moved.  Target = the measured angles first, so switching the gains on holds the hand still.
        check('hold target', g.move_joint_all(list(g.get_gripper_data().joint[:JOINTS])))
        check('PD mode', g.set_control_pid_mode(0))
        check('gains (DGManager gain recipe 1: P 3, D 5)', g.set_joint_gain_pid_all([3.0] * JOINTS, [5.0] * JOINTS,
                                                                                   [0.0] * JOINTS, [0.0] * JOINTS))
        show()
        if a.object:
            obj = 'plate' if a.object == 'plate' else 'bowl'
            ask(f'0. hand to the approach pose "{APPROACH_HAND[obj]}" in {a.motion_ms} ms.')
            check('move', g.move_joint_all(load_pose(APPROACH_HAND[obj])))
            time.sleep(a.motion_ms / 1000 + 0.5)
            show()
            if input(f'\n0b. arm: home -> above -> down to the taught {obj} grasp pose?  Enter = go, s = skip (arm already '
                     'there): ').strip().lower() != 's' and not arm_legs(a.object, ARM_APPROACH):
                raise KeyboardInterrupt
        ask(f'1. move to "{a.pre}" in {a.motion_ms} ms.')
        check('move', g.move_joint_all(pre))
        time.sleep(a.motion_ms / 1000 + 0.5)
        show()
        ask(f'2. grasp {mode.name.lstrip("_")} at {a.force} N (joints {[j + 1 for j in range(JOINTS) if hold[j]]} hold position).')
        check('position mode', g.set_position_mode_all(hold))
        check('grasp data', g.set_grasp_data(mode, a.force, 0, smooth))
        check('grasp', g.grasp(1))
        time.sleep(2.0)
        show()
        if a.lock:
            ask('2b. lock the grasping fingers at their current angles (position mode).')
            # target = the measured angles first, so the switch to position mode has nothing to move to
            check('hold target', g.move_joint_all(list(g.get_gripper_data().joint[:JOINTS])))
            for f in range(1, 6):
                check(f'position mode finger {f}', g.set_position_mode_finger([1] * 4, f))
            for _ in range(3):
                time.sleep(1.0)
                show()
        # (보스 10/27) keep the grasp while the arm lifts and goes home in the other terminal: only the typed word
        # 'release' opens the hand; Enter shows joints / currents / temperature, Ctrl+C and q do NOT let go here
        print('\n3. HOLDING.  "home" = lift + home;  "basket" = the empty quadrant by itself (bowl: any empty one; plate: '
              '1st front, 2nd back of the same Q3 / Q4), "basket Q3 6" / "basket Q1" = by hand: lift + carry + down into '
              'the basket (--teach, plate: stops above it, teach the insert with the pendant);  then "release" = let go -> hand '
              'open -> ascend (taught: the records reversed) + home.  '
              'Enter = joints / temperature.', flush=True)
        while True:
            try:
                cmd = input('   holding> ').strip().lower()
                if cmd == 'release':
                    if BASKET and not BASKET.get('down') and input('   not lowered into the basket yet (descend did not '
                                                                   'run): release anyway?  yes / Enter = no: ').strip().lower() != 'yes':
                        continue
                    break
                if cmd == 'home' and a.object:
                    arm_legs(a.object, ARM_HOME)
                elif cmd == 'basket' and a.object:                          # (보스 10/29) the empty quadrant by itself
                    auto_basket('plate' if a.object == 'plate' else 'bowl')
                elif cmd.startswith('basket') and a.object:                 # plate: basket Q3 6, bowl: basket Q1
                    w, obj = cmd.split(), 'plate' if a.object == 'plate' else 'bowl'
                    basket(obj, w[1].upper() if len(w) > 1 else ('Q3' if obj == 'plate' else 'Q1'),
                           int(w[2]) if len(w) > 2 else 6, float(w[3]) if len(w) > 3 else None)   # basket Q3 6 0.15 = deeper
                elif cmd == 'descend' and BASKET:                          # only after a stopped basket run
                    BASKET['down'] = mover(BASKET['plan'], 'descend', VEL[0])
                    if BASKET['down']:
                        record(*BASKET['target'])
                elif cmd:                                       # (10/27) 'basket' with a bowl printed joints silently
                    print(f'   "{cmd}" does nothing here (object {a.object}): "home", "release", '
                          + '"basket", ' + ('"basket Q3 6"' if a.object == 'plate' else '"basket Q1"') + ', "descend"; Enter = joints',
                          flush=True)
                    continue
                show()
            except KeyboardInterrupt:
                print('\n   still holding (Ctrl+C does not release); type "release" to let go', flush=True)
            except EOFError:
                time.sleep(1.0)
    except KeyboardInterrupt:
        print('\nstopped by 보스', flush=True)
    finally:
        if started:
            check('release', g.grasp(0))
            time.sleep(1.0)
            try:
                if input('\n4. open the hand (all 0)?  Enter = go, n = leave it: ').strip().lower() != 'n':
                    g.move_joint_all([0.0] * JOINTS)
                    time.sleep(a.motion_ms / 1000 + 0.5)
                    show()
            except EOFError:
                pass
            try:                                                 # after a basket drop: back up and home with that plan
                if BASKET and input('\n5. arm: ascend + home with the basket plan?  Enter = go, n = no: ').strip().lower() != 'n':
                    out_leg = BASKET.get('out_leg', 'ascend')
                    moved = mover(BASKET['plan'], out_leg, VEL[0])
                    if moved and out_leg != 'return_home':
                        mover(BASKET['plan'], 'to_home', VEL[0])
            except EOFError:
                pass
            g.system_stop()
        g.disconnect_to_gripper()


if __name__ == '__main__':
    main()
