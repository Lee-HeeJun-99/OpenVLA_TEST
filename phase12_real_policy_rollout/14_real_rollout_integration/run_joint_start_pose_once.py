"""Approved single joint reset through the existing DSR_ROBOT2.movej wrapper.

No Cartesian command, mode setter, home, gripper or model action is used.
"""
import argparse
import json
import math
import os
import threading
import time
from pathlib import Path
from jointstate_runtime_startup import JointStateStartup
from real_sink import Authorization, RealDoosanCommandSink, RosServiceTransport

TARGET = [-182.55277, -1.07182, -41.00585, 1.25662, -113.50009, 1.49975]
# Existing real-home validation tolerance, not a production action limit.
TOLERANCE_DEG = 0.5


def ordered_degrees(row):
    positions = dict(zip(row['names'], row['position']))
    return [math.degrees(positions['joint_' + str(i)]) for i in range(1, 7)]


def check_approval(approval, now):
    required = ('operator_present', 'workspace_clear', 'estop_accessible',
                'robot_stationary', 'authority', 'servo',
                'protective_stop_clear', 'emergency_stop_clear', 'explicit_joint_reset_approval')
    if not all(approval.get(k) is True for k in required):
        raise PermissionError('actual_manual_confirmation_missing')
    if approval.get('robot_model') != 'a0509' or approval.get('target_joint_deg') != TARGET:
        raise PermissionError('model_or_target_mismatch')
    if not 0 <= now - approval.get('approved_wall_time', 0) <= 60:
        raise PermissionError('explicit_reset_approval_expired')


def run(output, approval):
    import rclpy
    import dsr_msgs2.srv as services
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    check_approval(approval, time.time())
    output.mkdir(parents=True, exist_ok=False)
    def save(name, value):
        (output / name).write_text(json.dumps(value, indent=2))
    save('manual_confirmation.json', approval)
    rclpy.init()
    sensor = rclpy.create_node('a0509_single_reset_watchdog')
    command_node = rclpy.create_node('a0509_approved_single_joint_reset', namespace='dsr01')
    state = JointStateStartup(time.monotonic())
    lock = threading.RLock()
    rows = []
    halt = threading.Event()
    dispatched = threading.Event()
    stop_once = threading.Event()
    faults = []
    receipts = []
    auth = Authorization(True, True, True, 'MOTION_ENABLED')
    sink = RealDoosanCommandSink(lambda: RosServiceTransport(sensor), auth)
    log = (output / 'jointstate_during_reset.jsonl').open('x')
    def sample(msg):
        received = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        valid = (len(msg.name) == 6 and set(msg.name) == {'joint_'+str(i) for i in range(1, 7)}
                 and len(msg.position) == len(msg.velocity) == 6
                 and all(math.isfinite(x) for x in list(msg.position)+list(msg.velocity)))
        row = dict(receive=received, source=stamp,
                   header_age=sensor.get_clock().now().nanoseconds/1e9-stamp,
                   valid=valid, names=list(msg.name), position=list(msg.position), velocity=list(msg.velocity))
        with lock:
            state.sample(row)
            rows.append(row)
            try:
                log.write(json.dumps(row)+'\n')
                log.flush()
            except Exception as exc:
                faults.append('logger_failure:'+str(exc))
                state.fail('logger_failure')
    sensor.create_subscription(JointState, '/dsr01/joint_states', sample, qos_profile_sensor_data)
    def spin():
        try:
            while not halt.is_set():
                rclpy.spin_once(sensor, timeout_sec=.005)
        except Exception as exc:
            with lock:
                faults.append('executor_failure:'+str(exc))
                state.fail('executor_failure')
    def watch():
        while not halt.is_set():
            with lock:
                status = state.status(time.monotonic())
            if status['fault']:
                if dispatched.is_set() and not stop_once.is_set():
                    stop_once.set()
                    try:
                        receipts.append(sink.stop())
                    except Exception as exc:
                        receipts.append({'stop_failure': str(exc)})
                return
            halt.wait(.005)
    spinner = threading.Thread(target=spin, daemon=True)
    watcher = threading.Thread(target=watch, daemon=True)
    spinner.start(); watcher.start()
    result = dict(status='START_POSE_NOT_EXECUTED', physical_joint_commands=0,
                  target_joint_deg=TARGET, gripper_commands=0, mode_setters=0)
    worker = None
    try:
        while True:
            with lock: status = state.status(time.monotonic())
            if status['fault']: raise RuntimeError(status['fault'])
            if status['phase'] == 'RUNTIME_READY': break
            time.sleep(.01)
        hardware = {}
        for key, cls, path, expected in [('mode', services.GetRobotMode, '/dsr01/system/get_robot_mode', 1),
                                       ('state', services.GetRobotState, '/dsr01/system/get_robot_state', 1),
                                       ('system', services.GetRobotSystem, '/dsr01/system/get_robot_system', 0)]:
            client = sensor.create_client(cls, path)
            try:
                if not client.wait_for_service(timeout_sec=1.): raise RuntimeError('hardware_service_missing:'+key)
                future = client.call_async(cls.Request()); until=time.monotonic()+3
                while not future.done() and time.monotonic()<until: time.sleep(.005)
                if not future.done(): raise RuntimeError('hardware_getter_timeout:'+key)
                response=future.result(); value=getattr(response, 'robot_'+key)
                hardware[key]={'value':value, 'success':response.success, 'time':time.time()}
                save('hardware_current.json',hardware)
                if not response.success or value != expected: raise RuntimeError('hardware_preflight:'+key+':'+str(value))
            finally: sensor.destroy_client(client)
        save('hardware_current.json', hardware)
        # Reuse existing joint API; importing creates clients but does not send commands.
        import DR_init
        DR_init.__dsr__id='dsr01'; DR_init.__dsr__model='a0509'; DR_init.__dsr__node=command_node
        import DSR_ROBOT2 as driver
        with lock:
            before=dict(rows[-1]); status=state.status(time.monotonic())
        check_approval(approval, time.time())
        if status['phase'] != 'RUNTIME_READY' or status['fault']: raise RuntimeError('runtime_not_ready')
        if any(abs(v)>1e-6 for v in before['velocity']): raise RuntimeError('robot_not_stationary')
        save('jointstate_before_reset.json', before)
        save('joint_reset_command.json', {'api':'DSR_ROBOT2.movej', 'robot_model':'a0509',
             'target_joint_deg':TARGET, 'vel':10, 'acc':10, 'current_joint_deg':ordered_degrees(before),
             'error_before_deg':[a-b for a,b in zip(ordered_degrees(before),TARGET)], 'requested_at':time.time()})
        original_client=driver._ros2_movej
        class SingleDispatchClient:
            def wait_for_service(self, **kwargs):
                return original_client.wait_for_service(**kwargs)
            def call_async(self, request):
                from rosidl_runtime_py.convert import message_to_ordereddict
                with lock:
                    check_approval(approval,time.time())
                    current=state.status(time.monotonic())
                    if current['fault'] or current['phase']!='RUNTIME_READY' or dispatched.is_set():
                        raise PermissionError('single_joint_dispatch_inhibited')
                    save('actual_joint_service_request.json',dict(service='/dsr01/motion/move_joint',
                        request=message_to_ordereddict(request),requested_at=time.time()))
                    future=original_client.call_async(request)
                    dispatched.set();result['physical_joint_commands']=1
                    return future
        driver._ros2_movej=SingleDispatchClient()
        outcome={}
        def move():
            try: outcome['return_code']=driver.movej(driver.posj(*TARGET), vel=10, acc=10)
            except Exception as exc: outcome['error']=str(exc)
            outcome['returned_at']=time.time()
        # Exactly one pose command, with independent subscriber/watchdog throughout the blocking wrapper.
        with lock:
            status=state.status(time.monotonic())
            if status['fault']: raise RuntimeError(status['fault'])
        started=time.monotonic(); worker=threading.Thread(target=move,daemon=True);worker.start()
        worker.join(timeout=30)
        if worker.is_alive():
            with lock: state.fail('joint_command_ack_timeout')
            time.sleep(.1)
            raise RuntimeError('IN_FLIGHT_STATUS_UNKNOWN_ACK_TIMEOUT')
        if outcome.get('return_code') != 0: raise RuntimeError('movej_ack_failed:'+str(outcome))
        # Existing home protocol uses a two-second settling period.
        until=time.monotonic()+2
        while time.monotonic()<until:
            with lock: status=state.status(time.monotonic())
            if status['fault']: raise RuntimeError(status['fault'])
            time.sleep(.01)
        with lock: after=dict(rows[-1]); status=state.status(time.monotonic())
        actual=ordered_degrees(after); error=[a-b for a,b in zip(actual,TARGET)]
        result.update(status='START_POSE_PASS' if max(map(abs,error))<=TOLERANCE_DEG else 'START_POSE_FAIL',
            ack=outcome, latency_s=time.monotonic()-started, actual_joint_deg=actual,error_deg=error,
            max_abs_error_deg=max(map(abs,error)),tolerance_deg=TOLERANCE_DEG,
            observed_joint_delta_deg=[a-b for a,b in zip(actual,ordered_degrees(before))],
            physical_direction_confirmation='PENDING_OPERATOR',cartesian_contract_verified=False)
        save('jointstate_after_reset.json',after)
    except Exception as exc:
        result.update(status='START_POSE_FAIL' if dispatched.is_set() else 'START_POSE_NOT_EXECUTED', reason=str(exc))
    finally:
        with lock:
            status=state.status(time.monotonic());events=list(state.events)
        result.update(runtime=status,events=events,faults=faults,stop_receipts=receipts,
                      physical_stop_commands_requested=len(receipts),command_thread_alive=bool(worker and worker.is_alive()))
        save('start_pose_validation.json',result)
        halt.set();spinner.join(timeout=2);watcher.join(timeout=4);log.close()
        if not worker or not worker.is_alive(): command_node.destroy_node()
        sensor.destroy_node();rclpy.shutdown()
        print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--approval',type=Path,required=True);args=parser.parse_args()
    run(args.output,json.loads(args.approval.read_text()))
