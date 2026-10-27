"""Read-only: the M1013 joints [deg] once, as JSON on stdout (system python3 + ROS; tesollo/measure_held.py calls it)."""
import json
import sys

import rclpy
from dsr_msgs2.srv import GetCurrentPosj

rclpy.init()
node = rclpy.create_node('tesollo_read_posj')
cli = node.create_client(GetCurrentPosj, '/dsr01/dsr_controller2/aux_control/get_current_posj')
if not cli.wait_for_service(timeout_sec=8.0):
    sys.exit('get_current_posj not available (Doosan bringup running?)')
fut = cli.call_async(GetCurrentPosj.Request())
rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
r = fut.result()
if r is None or not r.success:
    sys.exit('get_current_posj failed')
print(json.dumps([float(v) for v in r.pos]))
