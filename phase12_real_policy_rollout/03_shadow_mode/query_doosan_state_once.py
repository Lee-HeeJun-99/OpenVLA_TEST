#!/usr/bin/env python3
"""One-shot allowlisted getter calls. Never imports or creates motion clients."""
from __future__ import annotations
import argparse,json,time
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import GetCurrentPosx,GetRobotState,GetRobotMode,GetLastAlarm,GetControlMode,GetControlSpace
from read_only_state_adapter import require_allowed_service,tcp_record

SPECS=(
('/dsr01/aux_control/get_current_posx','dsr_msgs2/srv/GetCurrentPosx',GetCurrentPosx),
('/dsr01/system/get_robot_state','dsr_msgs2/srv/GetRobotState',GetRobotState),
('/dsr01/system/get_robot_mode','dsr_msgs2/srv/GetRobotMode',GetRobotMode),
('/dsr01/system/get_last_alarm','dsr_msgs2/srv/GetLastAlarm',GetLastAlarm),
('/dsr01/aux_control/get_control_mode','dsr_msgs2/srv/GetControlMode',GetControlMode),
('/dsr01/aux_control/get_control_space','dsr_msgs2/srv/GetControlSpace',GetControlSpace),
)
def call(node,name,type_name,srv_type,timeout_sec=3.0):
    require_allowed_service(name,type_name)
    client=node.create_client(srv_type,name)
    if not client.wait_for_service(timeout_sec=2.0): return {'service':name,'status':'TIMEOUT_WAITING_FOR_SERVICE'}
    req=srv_type.Request()
    if name.endswith('get_current_posx'): req.ref=0
    started=time.monotonic_ns();future=client.call_async(req)
    rclpy.spin_until_future_complete(node,future,timeout_sec=float(timeout_sec))
    ended=time.monotonic_ns(); ros_ns=node.get_clock().now().nanoseconds
    if not future.done() or future.result() is None:return {'service':name,'status':'TIMEOUT','latency_seconds':(ended-started)/1e9}
    res=future.result(); common={'service':name,'status':'OK' if bool(res.success) else 'FAILED','success':bool(res.success),'receive_ros_timestamp_ns':ros_ns,'receive_monotonic_timestamp_ns':ended,'latency_seconds':(ended-started)/1e9}
    if name.endswith('get_current_posx'):
        values=list(res.task_pos_info[0].data) if res.task_pos_info else []
        common['tcp']=tcp_record(values,success=res.success,receive_ros_ns=ros_ns,receive_monotonic_ns=ended,latency_sec=common['latency_seconds'])
    elif name.endswith('get_robot_state'):common['robot_state']=int(res.robot_state)
    elif name.endswith('get_robot_mode'):common['robot_mode']=int(res.robot_mode)
    elif name.endswith('get_control_mode'):common['control_mode']=int(res.control_mode)
    elif name.endswith('get_control_space'):common['control_space']=int(res.space)
    elif name.endswith('get_last_alarm'):
        a=res.log_alarm;common['last_alarm']={'level':int(a.level),'group':int(a.group),'index':int(a.index),'param':list(a.param)}
    return common
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--only',choices=[s[0] for s in SPECS]);parser.add_argument('--timeout',type=float,default=3.0);args=parser.parse_args()
    rclpy.init();node=Node('phase12_read_only_state_once')
    selected=[s for s in SPECS if args.only is None or s[0]==args.only]
    try: print(json.dumps({'classification':'READ_ONLY_ONE_SHOT','results':[call(node,*s,timeout_sec=args.timeout) for s in selected],'command_issued':False,'executed_action':None,'robot_delivered_command':None},ensure_ascii=False))
    finally: node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
