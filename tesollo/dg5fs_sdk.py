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
VEL = [80.0]                # arm deg/s = deg/s^2 for every leg (보스 10/27: 80 unless told otherwise; --vel)
APPROACH_HAND = {'plate': 'open', 'bowl': 'bowl_grasp30'}
BASKET_LEGS = ('lift', 'to_above', 'descend')     # fallback when lift + to_above cannot be merged into one spline
DGM_CONFIG = Path.home() / '.config/DGManager/config.json'
JOINTS = 20
# YOLOE object class -> (poses.yaml pre-pose, grasp mode, position-mode joints 1-based, force N), 보스 10/27:
# plate: pose 0 -> arm to the plate -> pose 9 (grasp-ready) -> 5 finger parallel (as grasp recipe 2: spread joints held) 7 N;
# bowl: pose 30 + grasp recipe 1 (5 finger to the centre), 7 N
OBJECT_GRASP = {'plate': ('plate_pre9', '5F_5FINGER_PARALLEL', [1, 5, 9, 13, 17], 7.0),
                'bowl': ('bowl_grasp30', '5F_5FINGER', [], 7.0)}


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
BASKET = {}                 # the last basket plan of this run (basket_plan.py): its path, for 'descend'


def mover(plan, leg, v):
    """One mover leg (Enter to go); True if it ran."""
    r = subprocess.run(['bash', str(REPO / 'calibration/run.sh'), 'mover', '--plan', plan, '--leg', leg, '--smooth',
                        '--go_per', 'leg', '--vel_deg_s', str(v), '--acc_deg_s2', str(v), '--execute', '--go_enter'])
    if r.returncode:
        print(f'   mover stopped at {leg} (exit {r.returncode})', flush=True)
    return r.returncode == 0


def basket(obj, quadrant, pin, centre_h=None):
    """While the object is held: basket_plan.py (plate: plate_plans' basket planner with the Tesollo grasp, into the pins;
    bowl: upright into the middle of a quadrant), then lift, to_above and descend (Enter per leg); BASKET['down'] once
    the descend has run, so 'release' knows the object is in the basket."""
    out = subprocess.run([str(PLAN_PY), str(REPO / 'tesollo/basket_plan.py'), '--object', obj, '--quadrant', quadrant,
                          '--pin', str(pin), '--dx', str(SHIFT[0]), '--dy', str(SHIFT[1])]
                         + (['--centre_h', str(centre_h)] if centre_h is not None else [])
                         + (['--sim_corr', SIM_CORR[0]] if SIM_CORR[0] and obj == 'plate' else []), capture_output=True, text=True)
    print('\n'.join(l for l in out.stdout.splitlines() if l.startswith(('basket', 'OK', '  ', 'merged'))), flush=True)
    m = re.search(r'^plan: (\S+)', out.stdout, re.M)
    if out.returncode or not m:
        print(f'   basket plan failed:\n{out.stdout[-600:]}{out.stderr[-600:]}', flush=True)
        return
    BASKET['plan'] = m.group(1)
    print(f'   basket plan {m.group(1)} (after the release: ascend, to_home with run.sh mover)', flush=True)
    BASKET['down'] = False
    mg = re.search(r'^merged: (\S+plan\.json)', out.stdout, re.M)
    # (보스 10/27, as the 2F-85 pipeline) grasp -> above the basket in ONE spline (lift + to_above), then the descent
    legs = [(mg.group(1), 'lift_carry'), (m.group(1), 'descend')] if mg else [(m.group(1), lg) for lg in BASKET_LEGS]
    for plan, leg in legs:
        if not mover(plan, leg, VEL[0]):
            return
    BASKET['down'] = True
    print('   in the basket: type "release"', flush=True)


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
                            '--execute', '--go_enter'])
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
    p.add_argument('--sim_corr', default='', help='(opt-in, plate) learned insert correction json from rl/hand_basket.py cem '
                                                  '(absolute path); use after tesollo/measure_held.py')
    p.add_argument('--vel', type=float, default=80.0, help='arm speed deg/s (= deg/s^2) for every arm leg (default 80)')
    p.add_argument('--motion_ms', type=int, default=3000, help='time per pose move [ms] (slow)')
    a = p.parse_args()
    VEL[0] = a.vel
    SIM_CORR[0] = str(Path(a.sim_corr).resolve()) if a.sim_corr else ''
    held = None
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
        print('\n3. HOLDING.  "home" = lift + home;  "basket Q3 6" (plate, quadrant / pin) or "basket Q1" (bowl) = lift + '
              'carry + down into the basket (Enter per leg);  then "release" = let go -> hand open -> ascend + home.  '
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
                elif cmd.startswith('basket') and a.object:                 # plate: basket Q3 6, bowl: basket Q1
                    w, obj = cmd.split(), 'plate' if a.object == 'plate' else 'bowl'
                    basket(obj, w[1].upper() if len(w) > 1 else ('Q3' if obj == 'plate' else 'Q1'),
                           int(w[2]) if len(w) > 2 else 6, float(w[3]) if len(w) > 3 else None)   # basket Q3 6 0.15 = deeper
                elif cmd == 'descend' and BASKET:                          # only after a stopped basket run
                    BASKET['down'] = mover(BASKET['plan'], 'descend', VEL[0])
                elif cmd:                                       # (10/27) 'basket' with a bowl printed joints silently
                    print(f'   "{cmd}" does nothing here (object {a.object}): "home", "release", '
                          + ('"basket Q3 6"' if a.object == 'plate' else '"basket Q1"') + ', "descend"; Enter = joints',
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
                    mover(BASKET['plan'], 'ascend', VEL[0]) and mover(BASKET['plan'], 'to_home', VEL[0])
            except EOFError:
                pass
            g.system_stop()
        g.disconnect_to_gripper()


if __name__ == '__main__':
    main()
