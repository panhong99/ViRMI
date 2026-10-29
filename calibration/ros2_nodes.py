"""ROS2 side of the real robot (SYSTEM Python with ROS sourced - NOT conda; `run.sh` sets the environment).

  python3 calibration/ros2_nodes.py flange      read-only: stream flange posx + joints to UDP 5006 (calibration, pipeline)
  python3 calibration/ros2_nodes.py publisher   publish a plan from UDP 5010 as NON-control topics /sixd/dryrun/*
  python3 calibration/ros2_nodes.py mover --plan <plan_001.json> --leg approach [--execute]
                                                SUPERVISED point-to-point mover: moves ONLY after a typed `go`
  python3 calibration/ros2_nodes.py viz         read-only RViz view (run.sh viz): real robot, FP plate, plan preview

flange    : calls get_current_tool_flange_posx (DR_BASE) and get_current_posj at --rate; never moves anything.
            JSON {"posx": [x, y, z (mm), a, b, c (deg, ZYZ)], "posj": [j1..j6 (deg)], "t": unix_time}.
publisher : /sixd/dryrun/target_pose (PoseStamped), target_joints_deg, waypoints_deg (N x 6 row-major), status (JSON);
            the Doosan driver does not read these topics.
mover     : executes one leg of a plan as single MoveJoint calls.  Safety (each item can only refuse / stop):
            * without --execute it only validates and prints the table (works without ROS);
            * speed / accel must be > 0 and finite (10/13 보스: no upper limit; default 2);
            * the plan must be accepted, a real (non-offline) dry-run plan, younger than --max_plan_age_min, every move
              <= --max_step_deg, every waypoint >= 2 deg inside the URDF joint limits;
            * the robot's CURRENT joints must match the previous waypoint (--max_start_error_deg), re-checked after the
              typed `go` (the Human may have jogged the robot) and before every move; reached pose checked after;
            * an opening statement once, then `go` per waypoint (other input re-asks, q stops); Ctrl+C requests a quick stop;
              a controller refusal stops the sequence (never retried).  The physical E-stop / pendant has priority.
            Only ever started by the Human; the assistant does not run it against the robot.
viz       : the bringup's /dsr01/joint_states stays at 0 in this build, so RViz shows a straight robot.  viz reads the
            joints with get_current_posj (like flange) and draws, under the fixed frame base_link:
              real/*  the real robot (TF + /sixd/viz/robot_description) with a 2F-85 block, 10 Hz
              plan/*  a looping preview of the plan legs (to_grasp, lift) as a ghost robot + TCP path lines
              markers /sixd/viz/markers: work cell boxes, FoundationPose plate mesh, allowed placement area
            Inputs from pipeline.py on UDP 5011 (JSON): {"plate": 4x4 base<-mesh, "ok": bool}, {"plan": "<plan.json>"},
            {"area": [[x, y, z] lo, [x, y, z] hi]}.  Never moves anything.
Service prefix: /<robot_id>/<controller_ns>/... (this Doosan build: dsr01/dsr_controller2).
"""
import argparse
import json
import socket
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

URDF = Path('/home/pan/pan/doosan_ws/install/dsr_description2/share/dsr_description2/urdf/m1013.urdf')
# (10/13 보스) speed hard limit removed; speeds come from calibration/config.yaml plate_ik_bc (only > 0 and finite checked)
OPENING = 'start'
DR_BASE = 0


def service_prefix(robot_id, controller_ns):
    return '/' + '/'.join(x for x in (robot_id, controller_ns) if x)


def call(rclpy, node, client, request, timeout):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result() if future.done() else None


def force_limit_sample(tool_wrench, limit_n, request_stop):
    """Apply the opt-in tool-force guard to one Doosan [Fx,Fy,Fz,Mx,My,Mz] sample.

    The norm uses only the first three base-frame components (N); torque components are
    validated but never mixed into the force threshold. `request_stop` is called once
    on a strict threshold exceedance. Pure/fake samples can exercise this offline.
    """
    if not np.isfinite(limit_n) or limit_n <= 0:
        raise ValueError('force limit must be finite and positive')
    wrench = np.asarray(tool_wrench, dtype=float)
    if wrench.shape != (6,) or not np.isfinite(wrench).all():
        raise ValueError('GetToolForce must return six finite force/torque values')
    force_n = float(np.linalg.norm(wrench[:3]))
    tripped = force_n > float(limit_n)
    return dict(force_n=force_n, tripped=tripped,
                stop_requested=bool(request_stop()) if tripped else False)


def monitor_spline_force(spline_future, spin_once, read_tool_force, request_stop, limit_n,
                         timeout_s, poll_hz=20.0, clock=time.monotonic):
    """Monitor while a spline future runs; on stale/bad telemetry or force trip, stop and abort.

    `read_tool_force` must be bounded (the ROS adapter uses a 0.25 s service timeout).
    A monitor fault never lets the caller continue to release or retreat automatically.
    """
    if not np.isfinite(poll_hz) or poll_hz <= 0 or not np.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError('monitor rate and timeout must be finite and positive')
    deadline = clock() + timeout_s
    next_poll = clock()
    period = 1.0 / poll_hz
    while not spline_future.done():
        now = clock()
        if now >= deadline:
            return dict(status='timeout', stop_requested=bool(request_stop()))
        try:
            spin_once(min(0.01, deadline - now))
        except Exception as exc:
            return dict(status='monitor_error', error=str(exc), stop_requested=bool(request_stop()))
        if spline_future.done():
            break
        now = clock()
        if now >= next_poll:
            try:
                wrench = read_tool_force()
                if wrench is None:
                    raise RuntimeError('GetToolForce returned no successful sample')
                sample = force_limit_sample(wrench, limit_n, request_stop)
            except Exception as exc:
                return dict(status='sensor_error', error=str(exc), stop_requested=bool(request_stop()))
            if sample['tripped']:
                return dict(status='force_limit', **sample)
            next_poll = clock() + period
    if not spline_future.done():
        return dict(status='timeout', stop_requested=bool(request_stop()))
    return dict(status='complete', result=spline_future.result())


# ================================================================ flange: read-only pose stream

def run_flange(a):
    import rclpy
    from dsr_msgs2.srv import GetCurrentPosj, GetCurrentToolFlangePosx
    rclpy.init()
    node = rclpy.create_node('flange_pose_udp')
    prefix = service_prefix(a.robot_id, a.controller_ns)
    flange = node.create_client(GetCurrentToolFlangePosx, f'{prefix}/aux_control/get_current_tool_flange_posx')
    posj = node.create_client(GetCurrentPosj, f'{prefix}/aux_control/get_current_posj')
    for client in (flange, posj):
        if not client.wait_for_service(timeout_sec=10.0):
            raise SystemExit(f'service {client.srv_name} not available (is the Doosan bringup running?)')
    from dsr_msgs2.srv import GetExternalTorque, GetToolForce          # (10/12) read-only force logging for the basket insert
    tforce = node.create_client(GetToolForce, f'{prefix}/aux_control/get_tool_force')
    etorque = node.create_client(GetExternalTorque, f'{prefix}/aux_control/get_external_torque')
    has_force = tforce.wait_for_service(timeout_sec=2.0) and etorque.wait_for_service(timeout_sec=2.0)
    if not has_force:
        print('force services not available: streaming poses only', flush=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f'streaming tool-flange posx of {prefix} to 127.0.0.1:{a.port} ...', flush=True)
    try:
        while rclpy.ok():
            req = GetCurrentToolFlangePosx.Request()
            req.ref = DR_BASE
            fx = call(rclpy, node, flange, req, 1.0)
            jx = call(rclpy, node, posj, GetCurrentPosj.Request(), 1.0)
            if fx is not None and jx is not None and fx.success and jx.success and len(fx.pos) == 6 and len(jx.pos) == 6:
                msg = {'posx': [float(v) for v in fx.pos], 'posj': [float(v) for v in jx.pos], 't': time.time()}
                if has_force:
                    freq = GetToolForce.Request()
                    freq.ref = DR_BASE
                    tf = call(rclpy, node, tforce, freq, 0.5)
                    et = call(rclpy, node, etorque, GetExternalTorque.Request(), 0.5)
                    if tf is not None and tf.success:
                        msg['tool_force'] = [float(v) for v in tf.tool_force]     # N (base) + Nm (tool)
                    if et is not None and et.success:
                        msg['ext_torque'] = [float(v) for v in et.ext_torque]     # Nm per joint
                sock.sendto(json.dumps(msg).encode(), ('127.0.0.1', a.port))
            time.sleep(1.0 / a.rate)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ================================================================ publisher: plan -> observation topics

def run_publisher(a):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from scipy.spatial.transform import Rotation
    from std_msgs.msg import Float64MultiArray, String

    class DryRunPublisher(Node):
        def __init__(self, port, ack_file=None):
            super().__init__('sixd_dryrun_publisher')
            self.ack_file = ack_file
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pose = self.create_publisher(PoseStamped, '/sixd/dryrun/target_pose', qos)
            self.joints = self.create_publisher(Float64MultiArray, '/sixd/dryrun/target_joints_deg', qos)
            self.waypoints = self.create_publisher(Float64MultiArray, '/sixd/dryrun/waypoints_deg', qos)
            self.status = self.create_publisher(String, '/sixd/dryrun/status', qos)
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(('127.0.0.1', port))
            self.sock.setblocking(False)
            self.create_timer(0.05, self.poll)
            self.get_logger().info(f'DRY RUN publisher: UDP {port} -> /sixd/dryrun/* (no robot commands)')

        def poll(self):
            try:
                res = json.loads(self.sock.recv(1 << 20))
            except (BlockingIOError, ValueError):
                return
            legs = res.get('legs', {})
            self.status.publish(String(data=json.dumps(dict(
                accepted=res.get('accepted'), reasons=res.get('reasons'), cube_centre_m=res.get('cube_centre_m'),
                pregrasp_clearance_m=res.get('pregrasp_clearance_m'),
                approach_min_clearance_m=legs.get('approach', {}).get('min_clearance_straight_m'),
                return_min_clearance_m=legs.get('return', {}).get('min_clearance_straight_m'), dry_run=True))))
            if 'T_base_flange_pregrasp' in res and res.get('accepted'):     # refused plans publish status only
                T = np.array(res['T_base_flange_pregrasp'])
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, T[:3, 3])
                q = Rotation.from_matrix(T[:3, :3]).as_quat()
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, q)
                self.pose.publish(msg)
                self.joints.publish(Float64MultiArray(data=[float(v) for v in res['q_pregrasp_deg']]))
                self.waypoints.publish(Float64MultiArray(data=np.array(legs['approach']['waypoints_deg'], dtype=float).ravel().tolist()))
            if self.ack_file:
                self.ack_file.touch()

    rclpy.init()
    node = DryRunPublisher(a.port, a.ack_file)
    if a.ready_file:                                      # only after the UDP listener and publishers exist
        a.ready_file.touch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ================================================================ mover: supervised real motion

def mover_args(argv=None):
    p = argparse.ArgumentParser(prog='ros2_nodes.py mover')
    p.add_argument('--plan', type=Path, required=True)
    p.add_argument('--leg', required=True, help='a leg of the plan: approach/return (pipeline.py) or '
                   'to_ready/grasp/lift/lower/retreat/to_home (rl/eval.py export)')
    p.add_argument('--execute', action='store_true', help='really send the moves (otherwise check only)')
    p.add_argument('--vel_deg_s', type=float, default=2.0)
    p.add_argument('--acc_deg_s2', type=float, default=2.0)
    p.add_argument('--max_step_deg', type=float, default=45.0)
    p.add_argument('--max_start_error_deg', type=float, default=3.0)
    p.add_argument('--max_reach_error_deg', type=float, default=1.0)
    p.add_argument('--max_plan_age_min', type=float, default=30.0)
    p.add_argument('--from_step', type=int, default=1, help='first waypoint index to move to (1 = first move)')
    p.add_argument('--opening_done', action='store_true',
                   help='the calling pipeline already took the opening statement for this session (go is still asked)')
    p.add_argument('--smooth', action='store_true', help='execute the whole leg as ONE joint-spline motion (movesj through '
                   'all waypoints, no stop between them); start/finish checks as usual')
    p.add_argument('--auto', action='store_true', help='with --opening_done and --smooth: no go prompt (pipeline decides)')
    p.add_argument('--go_enter', action='store_true', help='(10/26 보스, look4) an empty Enter counts as go (typed-ahead keys dropped)')
    p.add_argument('--go_per', choices=('waypoint', 'leg'), default='waypoint',
                   help='leg: one go starts the whole leg (moves still stop at each waypoint and every check still runs)')
    p.add_argument('--tool-force-limit-n', type=float, default=None,
                   help='OPT-IN: during this smooth leg, Q-stop if base-frame GetToolForce norm exceeds this N; no release/retreat follows')
    p.add_argument('--robot_id', default='dsr01')
    p.add_argument('--controller_ns', default='dsr_controller2')
    return p.parse_args(argv)


CONTROLLER_LIMITS_DEG = {'joint_3': (-135.0, 135.0),    # (10/15 실기) alarm 1207 'JNT(3) POS LIMIT - INPUT(137.414), MAX(135.000)' (URDF 160)
                         'joint_5': (-135.0, 135.0)}    # (10/13) controller alarm 1207 'JNT(5) POS LIMIT' at 135.7 (URDF says 360)


def joint_limits_deg():
    limits = {}
    for joint in ET.parse(URDF).getroot().iter('joint'):
        lim = joint.find('limit')
        if joint.get('type') == 'revolute' and lim is not None and joint.get('name', '').startswith('joint_'):
            limits[joint.get('name')] = (np.degrees(float(lim.get('lower'))), np.degrees(float(lim.get('upper'))))
    limits.update(CONTROLLER_LIMITS_DEG)
    return [limits[f'joint_{i}'] for i in range(1, 7)]


def validate(args, plan):
    """Return (waypoints Nx6 deg, list of problems).  Never raises on a bad plan."""
    problems = []
    if not np.isfinite(args.vel_deg_s) or args.vel_deg_s <= 0:
        problems.append(f'--vel_deg_s {args.vel_deg_s} must be > 0')
    if not np.isfinite(args.acc_deg_s2) or args.acc_deg_s2 <= 0:
        problems.append(f'--acc_deg_s2 {args.acc_deg_s2} must be > 0')
    if args.tool_force_limit_n is not None:
        if not np.isfinite(args.tool_force_limit_n) or args.tool_force_limit_n <= 0:
            problems.append('--tool-force-limit-n must be finite and > 0')
        if not args.smooth:
            problems.append('--tool-force-limit-n is only supported for one smooth spline leg')
        if args.leg not in ('insert', 'descend'):
            problems.append('--tool-force-limit-n is only supported on insert/descend legs')
    if not plan.get('accepted'):
        problems.append('plan is not accepted: ' + '; '.join(plan.get('reasons', [])))
    if not plan.get('dry_run'):
        problems.append('file is not a dry-run plan')
    if plan.get('offline_test') or plan.get('execution_allowed') is False:
        problems.append('offline fixture cannot be used for real motion')
    try:
        age = (time.time() - time.mktime(time.strptime(plan['created'], '%Y-%m-%d %H:%M:%S'))) / 60
        if age < -1 or age > args.max_plan_age_min:
            problems.append(f'plan is {age:.0f} min old (> {args.max_plan_age_min:.0f}): scene may have changed, re-plan')
    except (KeyError, ValueError):
        problems.append('plan has no creation time')
    leg = plan.get('legs', {}).get(args.leg)
    if not leg:
        return np.zeros((0, 6)), problems + [f'plan has no leg {args.leg}']
    wp = np.array(leg['waypoints_deg'], dtype=float)
    if wp.ndim != 2 or wp.shape[1] != 6 or len(wp) < 2 or not np.isfinite(wp).all():
        return np.zeros((0, 6)), problems + ['waypoint table is malformed']
    lo, hi = np.array(joint_limits_deg()).T
    for k, w in enumerate(wp):
        if (w < lo + 2).any() or (w > hi - 2).any():
            problems.append(f'waypoint {k} is within 2 deg of / beyond a joint limit: {np.round(w, 1).tolist()}')
    for k in range(1, len(wp)):
        turn = np.abs(wp[k] - wp[k - 1]).max()
        if turn > args.max_step_deg:
            problems.append(f'move {k} turns a joint {turn:.1f} deg (> {args.max_step_deg})')
    if leg['min_clearance_straight_m'] < plan['margin_m']:
        problems.append('straight-segment clearance is below the margin')
    return wp, problems


def print_table(args, wp, plan):
    print(f'plan {args.plan} (created {plan.get("created")}), leg {args.leg}, speed {args.vel_deg_s} deg/s, accel {args.acc_deg_s2} deg/s^2')
    obj = plan.get('cube_centre_m')                         # taught / library plans have no single object point
    print(f'object in base [m]: {np.round(obj, 4).tolist() if obj is not None else "-"}   '
          f'margin {plan.get("margin_m", 0.0) * 1000:.0f} mm, cell {plan.get("cell")}, source {plan.get("source", "pipeline")}')
    for k, w in enumerate(wp):
        turn = '' if k == 0 else (f'   move {k}: largest joint turn {np.abs(w - wp[k - 1]).max():5.1f} deg, '
                                  f'~{np.abs(w - wp[k - 1]).max() / args.vel_deg_s:5.0f} s')
        print(f'  {k:2d} ' + ' '.join(f'{v:8.2f}' for v in w) + turn)


def run_mover(argv=None):
    args = mover_args(argv)
    plan = json.loads(args.plan.read_text())
    wp, problems = validate(args, plan)
    print_table(args, wp, plan)
    if problems:
        print('\nREFUSED:\n  - ' + '\n  - '.join(problems))
        sys.exit(2)
    print('\nCHECK OK.' + ('' if args.execute else '  (check only: nothing is sent; add --execute to move)'))
    if not args.execute:
        return

    import rclpy
    from dsr_msgs2.srv import GetCurrentPosj, MoveJoint, MoveSplineJoint, MoveStop
    from std_msgs.msg import Float64MultiArray
    rclpy.init()
    node = rclpy.create_node('supervised_pendant_mover')
    prefix = service_prefix(args.robot_id, args.controller_ns)
    get_posj = node.create_client(GetCurrentPosj, f'{prefix}/aux_control/get_current_posj')
    move_joint = node.create_client(MoveJoint, f'{prefix}/motion/move_joint')
    move_spline = node.create_client(MoveSplineJoint, f'{prefix}/motion/move_spline_joint')
    stop = node.create_client(MoveStop, f'{prefix}/motion/move_stop')
    get_tool_force = None
    GetToolForce = None
    if args.tool_force_limit_n is not None:
        from dsr_msgs2.srv import GetToolForce
        get_tool_force = node.create_client(GetToolForce, f'{prefix}/aux_control/get_tool_force')
    for client in (get_posj, move_joint):
        if not client.wait_for_service(timeout_sec=10.0):
            sys.exit(f'service {client.srv_name} not available (bringup running? control granted to this PC?)')
    if get_tool_force is not None and not get_tool_force.wait_for_service(timeout_sec=3.0):
        sys.exit(f'service {get_tool_force.srv_name} not available; force-guarded spline refused')
    if get_tool_force is not None and not stop.wait_for_service(timeout_sec=3.0):
        sys.exit(f'service {stop.srv_name} not available; force-guarded spline refused')

    def current():
        res = call(rclpy, node, get_posj, GetCurrentPosj.Request(), 3.0)
        if res is None or not res.success or len(res.pos) != 6:
            sys.exit('cannot read the current joints - stopping')
        return np.array(res.pos, dtype=float)

    def request_stop():
        try:
            req = MoveStop.Request()
            req.stop_mode = 1                       # DR_QSTOP: quick stop (never starts a motion)
            res = call(rclpy, node, stop, req, 2.0)
            return bool(res is not None and res.success)
        except Exception:
            return False

    def read_tool_force():
        req = GetToolForce.Request()
        req.ref = DR_BASE
        res = call(rclpy, node, get_tool_force, req, 0.25)
        if res is None or not res.success:
            return None
        return res.tool_force

    print(f'\nThis will move the REAL robot, {"ONE smooth spline through the leg" if args.smooth else "one confirmed move at a time"}, '
          f'at {args.vel_deg_s} deg/s.')
    if not args.opening_done and args.go_enter:   # (10/29 보스) Enter starts; a typo asks again instead of ending the sequence
        while (ans := input(f'Enter (or "{OPENING}") to continue, "q" to stop: ').strip()) not in ('', OPENING):
            if ans.lower() in ('q', 'quit', 'stop', 'abort', 'exit'):
                sys.exit('aborted')
            print(f'  "{ans}" - nothing sent; Enter or q', flush=True)
    elif not args.opening_done and input(f'Type exactly "{OPENING}" to continue: ').strip() != OPENING:
        sys.exit('aborted')
    if args.smooth:
        if args.auto and not args.opening_done:
            sys.exit('--auto needs --opening_done (the opening statement must have been taken)')
        if not move_spline.wait_for_service(timeout_sec=10.0):
            sys.exit(f'service {move_spline.srv_name} not available')
        try:
            k0 = max(1, args.from_step)
            now = current()
            err = np.abs(now - wp[k0 - 1]).max()
            if err > args.max_start_error_deg:
                sys.exit(f'REFUSED: robot is {err:.1f} deg from waypoint {k0 - 1} (> {args.max_start_error_deg})')
            targets = wp[k0:]
            if len(targets) > 100:
                sys.exit('REFUSED: movesj takes at most 100 waypoints')
            span = np.abs(np.diff(np.vstack([now, targets]), axis=0)).max(axis=1).sum()
            if not args.auto:
                while True:
                    if args.go_enter:
                        try:                          # no typed-ahead Enter starts the leg
                            import termios
                            termios.tcflush(sys.stdin, termios.TCIFLUSH)
                        except (ImportError, OSError, ValueError, termios.error):
                            pass
                    answer = input(f'type {"Enter" if args.go_enter else chr(34) + "go" + chr(34)} to run the whole leg '
                                   f'({len(targets)} waypoints, ~{span / args.vel_deg_s:.0f} s), "q" to stop here: ').strip()
                    if answer == 'go' or (args.go_enter and answer == ''):
                        break
                    if answer.lower() in ('q', 'quit', 'stop', 'abort', 'exit'):
                        sys.exit('aborted by user - robot left where it is')
            _, problems = validate(args, plan)
            if problems:
                sys.exit('REFUSED: ' + '; '.join(problems))
            if np.abs(current() - wp[k0 - 1]).max() > args.max_start_error_deg:
                sys.exit('REFUSED: robot moved before the start; make a new plan')
            if len(targets) < 2:                                           # 10/12: the controller takes it but does not move
                sys.exit('REFUSED: a smooth leg needs >= 2 target waypoints (make the plan again)')
            req = MoveSplineJoint.Request()
            req.pos = [Float64MultiArray(data=[float(v) for v in w]) for w in targets]
            req.pos_cnt = len(targets)
            req.vel, req.acc = [float(args.vel_deg_s)] * 6, [float(args.acc_deg_s2)] * 6
            req.time, req.mode, req.sync_type = 0.0, 0, 0                  # absolute, synchronous
            print(f'\nspline: {len(targets)} waypoints, joint travel {span:.1f} deg, ~{span / args.vel_deg_s:.0f} s', flush=True)
            move_timeout = span / args.vel_deg_s * 3 + 30
            if get_tool_force is None:
                res = call(rclpy, node, move_spline, req, move_timeout)
            else:
                # Baseline before motion: reject a pre-existing overload without moving.
                try:
                    baseline = force_limit_sample(read_tool_force(), args.tool_force_limit_n, lambda: False)
                except (TypeError, ValueError, RuntimeError) as exc:
                    sys.exit(f'force guard has no valid baseline; spline refused: {exc}')
                if baseline['tripped']:
                    sys.exit(f'force guard baseline {baseline["force_n"]:.1f} N exceeds '
                             f'{args.tool_force_limit_n:.1f} N; spline refused without motion')
                spline_future = move_spline.call_async(req)
                monitored = monitor_spline_force(
                    spline_future,
                    lambda timeout: rclpy.spin_once(node, timeout_sec=timeout),
                    read_tool_force, request_stop, args.tool_force_limit_n, move_timeout)
                if monitored['status'] != 'complete':
                    if monitored['status'] == 'force_limit':
                        print(f'FORCE LIMIT: {monitored["force_n"]:.1f} N > {args.tool_force_limit_n:.1f} N; '
                              f'DR_QSTOP requested (accepted={monitored["stop_requested"]}). '
                              'Automatic gripper release/retreat is disabled; Human must decide hold vs raise.', flush=True)
                    else:
                        print(f'FORCE GUARD {monitored["status"]}: {monitored.get("error", "")}; '
                              f'DR_QSTOP requested (accepted={monitored["stop_requested"]}). '
                              'Automatic gripper release/retreat is disabled.', flush=True)
                    sys.exit('force-guarded spline aborted; no automatic continuation')
                res = monitored['result']
            if res is None or not res.success:
                request_stop()
                sys.exit('controller did not accept / complete the spline - stopped, nothing retried')
            reached = np.abs(current() - wp[-1]).max()
            print(f'  reached, max joint error {reached:.2f} deg', flush=True)
            if reached > args.max_reach_error_deg:
                sys.exit(f'STOP: reached pose is {reached:.2f} deg off the target (> {args.max_reach_error_deg})')
            print('\nleg finished.')
        except KeyboardInterrupt:
            request_stop()
            print('\ninterrupted - stop requested')
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return
    try:
        for k in range(max(1, args.from_step), len(wp)):
            now = current()
            err = np.abs(now - wp[k - 1]).max()
            if err > args.max_start_error_deg:
                sys.exit(f'REFUSED before move {k}: robot is {err:.1f} deg from waypoint {k - 1} '
                         f'(> {args.max_start_error_deg}). Current {np.round(now, 1).tolist()}')
            turn = np.abs(wp[k] - now).max()
            print(f'\nmove {k}/{len(wp) - 1}: {np.round(now, 1).tolist()} -> {np.round(wp[k], 1).tolist()} '
                  f'(largest turn {turn:.1f} deg, ~{turn / args.vel_deg_s:.0f} s at {args.vel_deg_s} deg/s)')
            while not (args.go_per == 'leg' and k > max(1, args.from_step)):
                try:                                  # drop anything typed during the previous move (no typed-ahead go)
                    import termios
                    termios.tcflush(sys.stdin, termios.TCIFLUSH)
                except (ImportError, OSError, ValueError, termios.error):   # not a terminal (tests): nothing to flush
                    pass
                answer = input(('type Enter' if args.go_enter else 'type "go"') + ' to move' +
                               (' (runs the WHOLE leg)' if args.go_per == 'leg' else '') + ', "q" to stop here: ').strip()
                if answer == 'go' or (args.go_enter and answer == ''):
                    break
                if answer.lower() in ('q', 'quit', 'stop', 'abort', 'exit'):
                    sys.exit('aborted by user - robot left where it is')
                print(f'  "{answer}" is not "go" - nothing sent; type go or q', flush=True)
            now = current()                           # the Human may have waited or jogged the robot
            if np.abs(now - wp[k - 1]).max() > args.max_start_error_deg:
                sys.exit('REFUSED: robot moved while waiting for go; make a new plan')
            _, problems = validate(args, plan)
            if problems:
                sys.exit('REFUSED: ' + '; '.join(problems))
            turn = np.abs(wp[k] - now).max()
            if turn > args.max_step_deg:
                sys.exit('REFUSED: actual joint turn exceeds the segment limit')
            req = MoveJoint.Request()
            req.pos = [float(v) for v in wp[k]]
            req.vel, req.acc, req.time, req.radius = float(args.vel_deg_s), float(args.acc_deg_s2), 0.0, 0.0
            req.mode, req.blend_type, req.sync_type = 0, 0, 0     # absolute, no blending, synchronous
            res = call(rclpy, node, move_joint, req, turn / args.vel_deg_s * 2 + 20)
            if res is None or not res.success:
                request_stop()
                sys.exit('controller did not accept / complete the move (control authority on the pendant?) - stopped, nothing retried')
            reached = np.abs(current() - wp[k]).max()
            print(f'  reached, max joint error {reached:.2f} deg')
            if reached > args.max_reach_error_deg:
                sys.exit(f'STOP: reached pose is {reached:.2f} deg off the target (> {args.max_reach_error_deg})')
        print('\nleg finished.')
    except KeyboardInterrupt:
        request_stop()
        print('\ninterrupted - stop requested')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ================================================================ viz: read-only RViz view

def run_viz(a):
    import rclpy
    from dsr_msgs2.srv import GetCurrentPosj
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from scipy.spatial.transform import Rotation
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from calib_core import cpath, load_config
    from robot_model import Kinematics, frame_layout, measured_cell
    cfg = load_config()
    kin = Kinematics(URDF)
    tcp_m = cfg['robot']['coupling_m'] + 0.145                      # flange -> 2F-85 pinch point (rl/env_setup.py)
    plate_mesh = 'file://' + str(cpath(cfg['paths']['plate_obj']))
    # (10/12) basket model (rl/assets/basket_full.obj, basket frame) where the slot path assumes it: taught pose + nudge
    basket_mesh = 'file://' + str(Path(__file__).resolve().parents[1] / 'rl' / 'assets' / 'basket_full.obj')
    try:
        T_basket = np.array(json.loads((Path(__file__).resolve().parent / 'data' / 'basket' / 'slot_01_in_basket.json')
                                       .read_text())['T_base_basket_at_teach'])
        T_basket[:3, 3] += cfg.get('plate_ik_bc', {}).get('basket_shift_m', [0.0, 0.0, 0.0])
    except (OSError, KeyError, ValueError):
        T_basket = None

    rclpy.init()
    node = rclpy.create_node('sixd_viz')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    desc = node.create_publisher(String, '/sixd/viz/robot_description', latched)
    desc.publish(String(data=URDF.read_text()))
    tf_pub = node.create_publisher(TFMessage, '/tf', 10)
    mk_pub = node.create_publisher(MarkerArray, '/sixd/viz/markers', latched)
    posj = node.create_client(GetCurrentPosj, f'{service_prefix(a.robot_id, a.controller_ns)}/aux_control/get_current_posj')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', a.port))
    sock.setblocking(False)
    state = dict(real=None, pending=None, plate=None, plate_ok=False, area=None, plan=None, preview=None, t0=time.time(),
                 basket=None, held=None, release_k=0)

    def stamp():
        return node.get_clock().now().to_msg()

    def tf_msgs(prefix, q_deg):
        out = []
        frames = kin.frames(np.radians(q_deg))
        names = ['base_link'] + [j[0] for j in kin.joints]
        for i, name in enumerate(names):
            T = frames[name] if i == 0 else np.linalg.inv(frames[names[i - 1]]) @ frames[name]
            t = TransformStamped()
            t.header.stamp = stamp()
            t.header.frame_id = 'base_link' if i == 0 else f'{prefix}/{names[i - 1]}'
            t.child_frame_id = f'{prefix}/{name}'
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, T[:3, 3])
            qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
            out.append(t)
        return out

    def marker(ns, mid, mtype, frame='base_link', pose=None, scale=(1, 1, 1), rgba=(1, 1, 1, 1), **kw):
        m = Marker()
        m.header.frame_id, m.header.stamp, m.ns, m.id, m.type, m.action = frame, stamp(), ns, mid, mtype, Marker.ADD
        T = np.eye(4) if pose is None else pose
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, T[:3, 3])
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = \
            map(float, Rotation.from_matrix(T[:3, :3]).as_quat())
        m.scale.x, m.scale.y, m.scale.z = map(float, scale)
        m.color.r, m.color.g, m.color.b, m.color.a = map(float, rgba)
        m.frame_locked = frame != 'base_link'
        for k, v in kw.items():
            setattr(m, k, v)
        return m

    def box_pose(c):
        T = np.eye(4)
        T[:3, 3] = c
        return T

    def gripper(prefix, mid, rgba):                                     # 2F-85 + coupling as a block on link_6
        T = np.eye(4)
        T[2, 3] = tcp_m / 2
        return marker('gripper', mid, Marker.CUBE, f'{prefix}/link_6', T, (0.08, 0.04, tcp_m), rgba)

    def path_points(q_list):
        pts = []
        for q in q_list:
            T = kin.fk(np.radians(q)) @ box_pose([0, 0, tcp_m])
            pts.append(Point(x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3])))
        return pts

    def publish_markers():
        ms = [marker('cell', i, Marker.CUBE, pose=box_pose(c), scale=size,
                     rgba=(0.55, 0.45, 0.35, 0.35) if name == 'table' else (0.7, 0.7, 0.75, 0.6))
              for i, (name, c, size) in enumerate(frame_layout(measured_cell()))]
        ms.append(gripper('real', 0, (0.15, 0.15, 0.15, 1.0)))
        T_b = state['basket'] if state['basket'] is not None else T_basket   # (10/13) the plan's basket pose first
        if T_b is not None:
            ms.append(marker('basket', 0, Marker.MESH_RESOURCE, pose=T_b, rgba=(0.25, 0.25, 0.3, 0.9),
                             mesh_resource=basket_mesh))
        if state['held'] is not None and state['preview'] is not None:   # (10/13) plate in the hand until the release
            k = int((time.time() - state['t0']) * a.rate) % (len(state['preview']) + int(2 * a.rate))
            if k < state.get('grasp_k', 0):
                pass
            elif k < state['release_k']:
                ms.append(marker('held', 0, Marker.MESH_RESOURCE, 'plan/link_6', state['held'], rgba=(0.9, 0.75, 0.2, 0.9),
                                 mesh_resource=plate_mesh))
            else:
                T_rel = kin.fk(np.radians(state['preview'][state['release_k'] - 1])) @ state['held']
                ms.append(marker('held', 0, Marker.MESH_RESOURCE, pose=T_rel, rgba=(0.9, 0.75, 0.2, 0.9), mesh_resource=plate_mesh))
        if state['plate'] is not None:
            ok = state['plate_ok']
            ms.append(marker('plate', 0, Marker.MESH_RESOURCE, pose=state['plate'], rgba=(0.2, 0.9, 0.3, 1) if ok else (0.95, 0.3, 0.2, 1),
                             mesh_resource=plate_mesh))
            ms.append(marker('plate', 1, Marker.TEXT_VIEW_FACING, pose=state['plate'] @ box_pose([0, 0, 0.12]), scale=(0, 0, 0.04),
                             rgba=(1, 1, 1, 1), text='FP plate ' + ('OK' if ok else 'move it')))
        if state['area'] is not None:
            lo, hi = np.array(state['area'][0]), np.array(state['area'][1])
            ms.append(marker('area', 0, Marker.CUBE, pose=box_pose((lo + hi) / 2), scale=np.maximum(hi - lo, 0.002),
                             rgba=(0.2, 0.6, 1.0, 0.18)))
        if state['plan'] is not None:
            legs = state['plan']['legs']
            for i, (leg, rgba) in enumerate((('to_grasp', (0.2, 0.6, 1, 1)), ('lift', (1, 0.6, 0.1, 1)))):
                if leg in legs:
                    ms.append(marker('plan_path', i, Marker.LINE_STRIP, scale=(0.004, 0, 0), rgba=rgba,
                                     points=path_points(legs[leg]['waypoints_deg'])))
            for i, (leg, rgba) in enumerate((('to_slot', (0.2, 0.9, 0.3, 1)), ('slot_out', (0.7, 0.7, 0.7, 1)),
                                             ('to_above', (0.2, 0.9, 0.3, 1)), ('descend', (1, 0.2, 0.2, 1)),
                                             ('ascend', (1, 0.6, 0.1, 1)), ('to_home', (0.7, 0.7, 0.7, 1)),
                                             ('descend_pre', (1, 0.2, 0.2, 1)))):
                if leg in legs:                                     # basket plans (10/12: four stages)
                    ms.append(marker('plan_path', 10 + i, Marker.LINE_STRIP, scale=(0.004, 0, 0), rgba=rgba,
                                     points=path_points(legs[leg]['waypoints_deg'])))
            for i, leg in enumerate(('to_above', 'descend')):  # (10/23 보스 장애물 안내) the plan without the obstacles, grey
                if leg in state['plan'].get('compare_legs', {}):
                    ms.append(marker('plan_compare', i, Marker.LINE_STRIP, scale=(0.003, 0, 0), rgba=(0.6, 0.6, 0.6, 0.8),
                                     points=path_points(state['plan']['compare_legs'][leg]['waypoints_deg'])))
            if 'to_grasp' in legs:
                end = kin.fk(np.radians(legs['to_grasp']['waypoints_deg'][-1])) @ box_pose([0, 0, tcp_m])
                ms.append(marker('plan_path', 2, Marker.SPHERE, pose=box_pose(end[:3, 3]), scale=(0.02, 0.02, 0.02),
                                 rgba=(1, 0.1, 0.1, 1)))
                ms.append(marker('plan_path', 3, Marker.TEXT_VIEW_FACING, pose=box_pose(end[:3, 3] + [0, 0, 0.06]),
                                 scale=(0, 0, 0.035), rgba=(1, 1, 1, 1), text='close here'))
            ms.append(gripper('plan', 1, (0.3, 0.6, 1.0, 0.45)))
            for i, (_, c, size, R) in enumerate(o[:4] for o in state['plan'].get('obstacles', [])):   # (10/22 장애물 데모)
                T = box_pose(c)
                T[:3, :3] = np.asarray(R)
                ms.append(marker('obstacles', 2 * i, Marker.CUBE, pose=T, scale=size, rgba=(1.0, 0.15, 0.1, 0.55)))
                ms.append(marker('obstacles', 2 * i + 1, Marker.TEXT_VIEW_FACING, pose=box_pose(np.add(c, [0, 0, 0.18])),
                                 scale=(0, 0, 0.04), rgba=(1, 0.4, 0.3, 1), text=f'obstacle {i + 1}'))
        mk_pub.publish(MarkerArray(markers=ms))

    def load_plan(path):
        plan = json.loads(Path(path).read_text())
        seq = [np.array(plan['legs'][leg]['waypoints_deg'], dtype=float) for leg in ('to_grasp', 'lift', 'to_slot', 'slot_out', 'to_above', 'descend_pre', 'descend', 'ascend', 'to_home')
               if leg in plan['legs']]
        path_q = np.concatenate(seq)
        dense, at = [path_q[0]], [1]                                    # preview at a_viz_deg_s (faster than the real 6)
        for a_, b_ in zip(path_q[:-1], path_q[1:]):
            n = max(1, int(np.ceil(np.abs(b_ - a_).max() / a.preview_deg_s * a.rate)))
            dense += [a_ + (b_ - a_) * k / n for k in range(1, n + 1)]
            at.append(len(dense))                                       # preview index reached at each waypoint
        state['plan'], state['preview'], state['t0'] = plan, np.array(dense), time.time()
        state['basket'] = np.array(plan['basket_T']) if 'basket_T' in plan else None
        state['held'] = None
        if 'plate_in_tcp' in plan:                                      # mesh root = plate bottom, 1 cm under the centre
            off = np.eye(4)
            off[2, 3] = -0.01
            state['held'] = box_pose([0, 0, tcp_m]) @ np.array(plan['plate_in_tcp']) @ off
            cnt = lambda lgs: sum(len(plan['legs'][lg]['waypoints_deg']) for lg in lgs if lg in plan['legs'])   # noqa: E731
            state['grasp_k'] = at[max(cnt(('to_grasp',)) - 1, 0)]
            state['release_k'] = at[max(cnt(('to_grasp', 'lift', 'to_above', 'descend')) - 1, 0)]
        node.get_logger().info(f'plan preview: {path} ({len(dense) / a.rate:.0f} s loop)')

    def on_posj(future):
        r = future.result()
        if r is not None and r.success and len(r.pos) == 6:
            state['real'] = [float(v) for v in r.pos]
        state['pending'] = None

    def tick():
        while True:                                                     # pipeline messages
            try:
                msg = json.loads(sock.recv(1 << 16))
            except BlockingIOError:
                break
            except ValueError:
                continue
            try:
                if 'plate' in msg:
                    state['plate'], state['plate_ok'] = np.array(msg['plate'], dtype=float), bool(msg.get('ok'))
                if 'area' in msg:
                    state['area'] = msg['area']
                if 'plan' in msg:
                    load_plan(msg['plan'])
            except (OSError, KeyError, ValueError) as e:
                node.get_logger().warn(f'bad viz message: {e}')
        if state['pending'] is None and posj.service_is_ready():
            state['pending'] = posj.call_async(GetCurrentPosj.Request())
            state['pending'].add_done_callback(on_posj)
        tfs = []
        if state['real'] is not None:
            tfs += tf_msgs('real', state['real'])
        if state['preview'] is not None:
            n = len(state['preview'])
            k = int((time.time() - state['t0']) * a.rate) % (n + int(2 * a.rate))   # 2 s pause at the end
            tfs += tf_msgs('plan', state['preview'][min(k, n - 1)])
        if tfs:
            tf_pub.publish(TFMessage(transforms=tfs))
        if tick.n % int(a.rate) == 0:
            publish_markers()
        tick.n += 1
    tick.n = 0

    node.create_timer(1.0 / a.rate, tick)
    if a.plan:
        load_plan(a.plan)
    print(f'viz: UDP {a.port} -> /sixd/viz/* (read-only).  RViz: rviz2 -d {Path(__file__).resolve().parent / "viz.rviz"}', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ================================================================ main

if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] not in ('flange', 'publisher', 'mover', 'viz'):
        sys.exit(__doc__)
    mode, rest = sys.argv[1], sys.argv[2:]
    if mode == 'mover':
        run_mover(rest)
        sys.exit(0)
    p = argparse.ArgumentParser(prog=f'ros2_nodes.py {mode}')
    if mode == 'flange':
        p.add_argument('--robot_id', default='dsr01')
        p.add_argument('--controller_ns', default='dsr_controller2', help="'' = no sub-namespace")
        p.add_argument('--port', type=int, default=5006)
        p.add_argument('--rate', type=float, default=20.0)
        run_flange(p.parse_args(rest))
    elif mode == 'viz':
        p.add_argument('--robot_id', default='dsr01')
        p.add_argument('--controller_ns', default='dsr_controller2', help="'' = no sub-namespace")
        p.add_argument('--port', type=int, default=5011)
        p.add_argument('--rate', type=float, default=10.0)
        p.add_argument('--preview_deg_s', type=float, default=30.0, help='ghost robot speed (preview only, 5x the real)')
        p.add_argument('--plan', type=Path, help='show this plan.json right away')
        run_viz(p.parse_args(rest))
    else:
        p.add_argument('--port', type=int, default=5010)
        p.add_argument('--ready-file', type=Path)
        p.add_argument('--ack-file', type=Path)
        run_publisher(p.parse_args(rest))
