"""Experimental ServoL episode; arm-only by default, full-policy by explicit mode.

Independent 20 Hz target stream; original inference timestamps stay attached.
The interpolation state is COMMANDED state, never claimed as robot feedback.
Targets use last commanded pose; CurrentPosX is independent following evidence.
No MoveLine/MoveJ. Gripper IO exists ONLY in explicitly approved full-policy mode.
Normal/fault exit requests HOLD once.
"""
import argparse
from contextlib import ExitStack
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from urllib.request import urlopen

import yaml
from run_arm_only_once import JointStateStartup, TARGET, FsyncJsonlLogger, validate_identity
from run_real_rollout import SafetyPipeline, RuntimeState, CanonicalAction
from rollout_latency import select_fresh_frame, predict_frame, archive_request_jpeg
from camera_inference_timing import camera_stream_lost
from servo_experimental.live_segment import LiveCommandSegment, require_release, stream_planning_limits, release_blockers
from servo_experimental.live_guard import StreamGuard, require_stream_liveness
from concurrent_logger import ConcurrentLogger

ROOT = Path(__file__).resolve().parent


def episode_limits(requested=None):
    config = yaml.safe_load((ROOT/'configs/full_task.yaml').read_text())
    duration = float(config['maximum_duration_s'])
    servo_config = yaml.safe_load((ROOT/'configs/servo_episode.yaml').read_text())
    steps = int(servo_config['maximum_predictions'])
    if requested is not None:
        if not math.isfinite(requested) or not 0 < requested <= duration:
            raise ValueError('duration_must_be_positive_and_no_greater_than_existing_cap')
        duration = requested
    if not math.isfinite(duration) or duration <= 0 or steps <= 0:
        raise ValueError('invalid_episode_limits')
    return steps, duration


def start_error(row):
    q = dict(zip(row['names'], row['position']))
    return max(abs(math.degrees(q[f'joint_{i+1}'])-v) for i, v in enumerate(TARGET))


def run(output, requested_duration=None, *, _full_policy_mode=None, _approval=None):
    full = _full_policy_mode is not None
    pending_status = 'OPERATOR_RESULT_CONFIRMATION_PENDING' if full else 'ARM_ONLY_SERVO_EPISODE_OPERATOR_RESULT_PENDING'
    invalid_status = 'INVALID' if full else 'ARM_ONLY_SERVO_EPISODE_INVALID'
    scope = 'FULL_POLICY_ROLLOUT' if full else 'ARM_ONLY_SERVO_FULL_EPISODE_SYSTEM_INTEGRATION_TEST'
    if full:
        _full_policy_mode.validate(_approval)
        if (_approval.get('physical_controller')!='servo' or
                _approval.get('servo_risks_accepted') is not True or
                _approval.get('concurrent_arm_gripper_accepted') is not True or
                _approval.get('authority_and_stops_confirmed') is not True or
                _approval.get('no_other_motion_executor') is not True):
            raise PermissionError('explicit_servo_full_policy_approval_required')
    max_predictions, duration = episode_limits(requested_duration)
    servo_config = yaml.safe_load((ROOT/'configs/servo_episode.yaml').read_text())
    servo_transport = servo_config['servo_transport']
    protection = servo_config.get('segment_release', {})
    # Refuse BEFORE ROS initialization, IO, prompts or any hardware call.
    require_release(protection)
    raw_rotation_limit = float(servo_config['raw_rotation_step_limit_deg'])
    if not math.isfinite(raw_rotation_limit) or not 0 < raw_rotation_limit < 180:
        raise ValueError('invalid_raw_rotation_step_limit')
    print(f"실제 {'FULL-POLICY' if full else 'ARM-ONLY'} ServoL episode 1회: 최대 {duration:g}초 / {max_predictions} predictions.")
    print(f"20Hz stream (최대 약 {math.ceil(duration*20)} messages). Gripper {'ENABLED' if full else 'DISABLED'}. MoveLine/자동 반복/Home/recovery 없음.")
    print('ServoL 명령별 ACK 없음. 정상/오류 종료 시 MoveStop HOLD(3) ACK 확인.')
    print('실험 경로입니다. 전체 trajectory/정지 성능은 아직 검증 완료가 아닙니다.')
    print('ServoL은 speed slider/안전영역 감속에 의존하지 않습니다. E-stop 즉시 접근 필수.')
    print(f"ServoL transport: vel={servo_transport['velocity_mm_s_deg_s']} [mm/s, deg/s], "
          f"acc={servo_transport['acceleration_mm_s2_deg_s2']} [mm/s², deg/s²], "
          f"target_time={servo_transport['target_time_s']}s. 20Hz 궤적 생성기 속도/가속도 제한 유지.")
    print('튜닝 후보는 물리 미검증입니다. 직전 관절 한계 초과/HOLD 실패는 이 변경으로 해결되지 않습니다.')
    labels = ['Operator present', 'Workspace clear', 'E-stop immediately accessible',
              'Robot stationary', 'Authority/servo 정상, protective/emergency stop clear',
              '다른 rollout/motion/servo 실행기 없음',
              '기존 최소 ServoL 이동에 이상 동작/E-stop 사용 없었음',
              'formal TCP 미검증 및 zero-offset engineering execution basis 승인',
              '튜닝 후보 물리 미검증 및 관절 한계/정지 경로 미해결 사항 확인',
              'Gripper 없이 위 ServoL episode 1회 명시적 승인']
    confirmations = {}
    if not full:
        for label in labels:
            confirmations[label] = input(label+' [yes/NO]: ').strip().lower() == 'yes'
            if not confirmations[label]:
                print('NOT_EXECUTED: 승인 없음')
                return
    output.mkdir(parents=True, exist_ok=False)
    def save(name, obj):
        FsyncJsonlLogger._atomic_json(output/name, obj)
    confirmation = dict(_approval) if full else dict(items=confirmations, confirmed_wall_time=time.time())
    save('operator_confirmation.json', confirmation)
    limits_path = ROOT.parent/'01_configs/safety_limits.yaml'
    limits = yaml.safe_load(limits_path.read_text())
    limits = stream_planning_limits(limits, servo_config)
    print(f"현재 hard gate: translation {limits['translation_step_m']*1000:g}mm, "
          f"raw rotation {raw_rotation_limit:g}deg. 이동 delta 축소 OFF; 회전 limiter 및 20Hz 속도/가속도 제한 유지.", flush=True)
    config_paths = [limits_path, ROOT/'configs/full_task.yaml', ROOT/'configs/servo_episode.yaml']
    config_paths += list((ROOT.parent/'13_rollout_readiness').rglob('command_disabled_runtime.yaml'))
    config_paths += list((ROOT.parent/'01_configs').glob('*.yaml'))
    config_paths += [Path(__file__), ROOT/'servo_experimental/controller.py',
                     ROOT/'servo_experimental/live_segment.py', ROOT/'servo_experimental/bounded_segment.py',
                     ROOT/'servo_experimental/live_guard.py', ROOT/'camera_inference_timing.py',
                     ROOT/'rollout_latency.py', ROOT/'servo_experimental/stop_evidence.py',
                     ROOT/'settle_window.py']
    config_paths += [ROOT.parent/'02_safety'/name for name in (
        'safety_pipeline.py','command_disabled_runtime.py','runtime_safety_supervisor.py',
        'action_rate_limiter.py','action_step_limits.py')]
    if full:config_paths += [ROOT/'configs/gripper_hardware.yaml',ROOT/'full_policy_gripper.py',
                            ROOT/'servo_experimental/full_policy.py',ROOT/'real_sink.py']
    frozen = {str(p):dict(sha256=hashlib.sha256(p.read_bytes()).hexdigest(), content=p.read_text())
              for p in config_paths if p.exists()}
    git = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True)
    save('config_snapshot.json', dict(files=frozen, git_head=git.stdout.strip(),
        tracked_changes=subprocess.run(['git','status','--short'], cwd=ROOT, capture_output=True,text=True).stdout))
    save('contract.json', dict(scope=scope, controller_type='SERVO_L',
        max_duration_s=duration, max_predictions=max_predictions, rate_hz=20,
        gripper_enabled=full, task_success_evaluation=full, formal_tcp_verified=False,
        gripper_concurrent_with_arm=full,gripper_command_policy='STATE_TRANSITION_ONLY' if full else 'NONE',
        tcp_basis='ZERO_OFFSET_CONSISTENCY_AND_PHYSICAL_DIRECTION_SCALE_TEST',
        command_state_is_not_feedback=True, policy_target_source='LAST_PUBLISHED_COMMAND',
        generator='COMMAND_REFERENCE_C2_BRAKE_SLERP_NORMALIZED_ZYZ', segment_release=protection,
        target_replacement='LATEST_ONLY_AFTER_BOUNDED_C2_BRAKE',
        current_posx_polled_once_per_prediction=True, vendor_ack_available=False,
        stop_mode=3, stop_retry=False, auto_retry=False, auto_home=False, auto_recovery=False,
        policy_limiter_rate_hz=limits['control_rate_hz'], production_limits=limits,
        translation_delta_limiter_applied=False, rotation_delta_limiter_applied=True,
        translation_dynamic_profile_owner='SERVO_20HZ_TRACKER',
        effective_raw_rotation_limit_deg=raw_rotation_limit,
        servo_transport=servo_transport, tuning_physical_validation='NOT_EXECUTED'))
    result = dict(status='NOT_EXECUTED', scope=scope,evaluation_scope=scope,controller_type='SERVO_L',
        task_success=None, task_performance_denominator_included=False,
        predictions=0, accepted_targets=0, servo_publish_attempts=0,
        gripper_commands=0, move_line_commands=0, stop_commands=0,
        unexpected_motion=None, estop_used=None, runtime_valid=False,
        auto_retry=False, auto_home=False, auto_recovery=False, auto_next_episode=False)

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, JointState
    from dsr_msgs2.msg import ServolStream
    from dsr_msgs2.srv import GetRobotSystem, GetRobotMode, GetRobotState, GetCurrentPosx, MoveStop
    from rosidl_runtime_py.convert import message_to_ordereddict
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    # Keep ROS alive for HOLD/OFF cleanup on Ctrl+C; Python/wrapper owns abort.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('approved_full_policy_servo_episode' if full else 'approved_arm_only_servo_episode')
    stop_client = node.create_client(MoveStop, '/dsr01/motion/move_stop')
    stop_future = [None]
    def request_stop():
        result['stop_requested_monotonic'] = time.monotonic()
        try:
            if not stop_client.service_is_ready():
                raise RuntimeError('stop_service_unavailable')
            req = MoveStop.Request(); req.stop_mode = 3
            stop_future[0] = stop_client.call_async(req)
            result['stop_commands'] += 1
        except Exception as exc:
            result['stop_error'] = str(exc)
    guard = StreamGuard(request_stop)
    state_lock = threading.RLock()
    halt = threading.Event(); producer_done = threading.Event()
    gate = JointStateStartup(time.monotonic())
    camera = [None]; rows = []; tracker = [None]; running = [False]; pending_target = [None]
    started = [None]; producer = None
    feedback = [None]; last_publish = [None]
    gripper_session = [None]; gripper_log_context = None; pending_gripper_intent = [None]
    def fail(reason):
        guard.finish(reason)
        if gripper_session[0] is not None:gripper_session[0].abort_async(reason)
    def joint(msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        valid = (len(msg.name)==len(msg.position)==len(msg.velocity)==6 and
                 set(msg.name)=={f'joint_{i}' for i in range(1,7)} and
                 all(math.isfinite(v) for v in list(msg.position)+list(msg.velocity)))
        row = dict(receive=time.monotonic(), source=stamp,
            header_age=node.get_clock().now().nanoseconds/1e9-stamp, valid=valid,
            names=list(msg.name), position=list(msg.position), velocity=list(msg.velocity))
        with state_lock:
            gate.sample(row); rows.append(row)
    def image(msg):
        with state_lock: camera[0] = (msg, time.monotonic())
    node.create_subscription(JointState, '/dsr01/joint_states', joint, qos_profile_sensor_data)
    node.create_subscription(Image, '/zed/zed_node/rgb/color/rect/image', image, qos_profile_sensor_data)
    def snapshot():
        with state_lock: return camera[0]
    def health():
        with state_lock:
            s = gate.status(time.monotonic()); cam = camera[0]; fb = feedback[0]
        if s['fault']: raise RuntimeError('jointstate:'+s['fault'])
        if (running[0] or gripper_session[0] is not None) and camera_stream_lost(None if cam is None else cam[1], time.monotonic()):
            raise RuntimeError('camera_stream_loss')
        if running[0] and fb and time.monotonic()-fb['received']>=limits['max_action_age_sec']:
            raise RuntimeError('controller_feedback_stale')
        require_stream_liveness(time.monotonic(), last_publish[0],
                                active=running[0] and not guard.closed)
        if (output/'operator_abort.request').exists(): raise RuntimeError('operator_abort')
        # Detect config edits, not merely persist a snapshot.
        for path, cfg in frozen.items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=cfg['sha256']:
                raise RuntimeError('configuration_changed_during_episode')
        return s
    def check():
        with guard.lock: guard.check()
        return health()
    def spin():
        try:
            while not halt.is_set():
                rclpy.spin_once(node, timeout_sec=.005)
                health()
        except Exception as exc: fail(str(exc))
    spinner = threading.Thread(target=spin, name='servo-watchdog'); spinner.start()
    def getter(kind, endpoint, *, _after_stop=False, _with_timing=False, **fields):
        client = node.create_client(kind, endpoint)
        # After inhibit, read-only stop evidence must remain available even
        # when the originating sensor/model fault is still latched.
        validate = (lambda: None) if _after_stop else check
        try:
            validate()
            if not client.wait_for_service(timeout_sec=2): raise RuntimeError('getter_unavailable:'+endpoint)
            request = kind.Request()
            for key, value in fields.items(): setattr(request, key, value)
            request_sent = time.monotonic()
            future = client.call_async(request); end = time.monotonic()+2
            while not future.done() and time.monotonic()<end:
                validate(); time.sleep(.002)
            if not future.done(): raise RuntimeError('getter_timeout:'+endpoint)
            response_received = time.monotonic()
            response = message_to_ordereddict(future.result()); validate()
            if response.get('success') is not True: raise RuntimeError('getter_failure:'+endpoint)
            if _with_timing:
                return response,dict(request_sent_monotonic=request_sent,
                    response_received_monotonic=response_received,
                    request_latency_s=response_received-request_sent,
                    controller_source_timestamp=None,controller_measurement_time_verified=False)
            return response
        finally: node.destroy_client(client)

    def pose_read():
        response, timing = getter(GetCurrentPosx, '/dsr01/aux_control/get_current_posx', _with_timing=True, ref=0)
        pose = list(response['task_pos_info'][0]['data'][:6])
        if len(pose)!=6 or not all(math.isfinite(v) for v in pose): raise RuntimeError('invalid_current_pose')
        with state_lock: feedback[0] = dict(pose=pose, received=timing['response_received_monotonic'], **timing)
        return pose

    try:
        while check()['phase']!='RUNTIME_READY': time.sleep(.005)
        hardware = {}
        for name, kind, value in [('system',GetRobotSystem,0), ('mode',GetRobotMode,1), ('state',GetRobotState,1)]:
            response = getter(kind, '/dsr01/system/get_robot_'+name); hardware[name] = response
            if response['robot_'+name]!=value: raise RuntimeError('hardware_not_REAL_AUTO_STANDBY')
        with state_lock: before = dict(rows[-1])
        error = start_error(before)
        if error>.5 or any(v!=0 for v in before['velocity']): raise RuntimeError('start_pose_or_stationary_failed')
        with urlopen('http://127.0.0.1:8766/health', timeout=3) as response: model = json.load(response)
        validate_identity(model, 'openvla')
        if model.get('status')!='ready': raise RuntimeError('model_not_ready')
        save('model_health.json', model)
        if not stop_client.wait_for_service(timeout_sec=2): raise RuntimeError('stop_service_unavailable')
        if node.count_publishers('/dsr01/servol_stream'): raise RuntimeError('another_servo_publisher_exists')
        initial = pose_read()
        tracker[0] = LiveCommandSegment(initial, limits, transport=servo_transport, protection=protection)
        pipeline = SafetyPipeline('openvla', operator_confirmed_initial_open=False, arm_only=True,
                                  translation_endpoint_delta=True,raw_rotation_step_limit_deg=raw_rotation_limit)
        publisher = node.create_publisher(ServolStream, '/dsr01/servol_stream',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        end = time.monotonic()+2
        while publisher.get_subscription_count()<1 and time.monotonic()<end:
            check(); time.sleep(.01)
        if publisher.get_subscription_count()!=1: raise RuntimeError('servo_subscription_count_not_one')
        cam = snapshot()
        if cam is None or time.monotonic()-cam[1]>=.5: raise RuntimeError('camera_preflight_failed')
        if not 0<=time.time()-confirmation['confirmed_wall_time']<=60: raise RuntimeError('operator_approval_expired')
        check()
        if full:
            from real_sink import RosServiceTransport
            from servo_experimental.full_policy import ServoFullPolicyGripper
            cfg = _full_policy_mode.approved_gripper_config(_approval)
            gripper_log_context = FsyncJsonlLogger(output/'gripper_commands.jsonl')
            gripper_logger = ConcurrentLogger(gripper_log_context.__enter__())
            gripper_session[0] = ServoFullPolicyGripper(lambda:RosServiceTransport(node),cfg,
                gripper_logger,guard,check,fail,age_limit=limits['max_action_age_sec'])
            save('gripper_contract.json',dict(cfg,physical_jaw_abort_state_verified=False,
                worker='SERIAL_LATEST_INTENT',concurrent_with_arm=True))
            _full_policy_mode.initialize(gripper_session[0],check,save)
            _full_policy_mode.validate(_approval)
            with state_lock: before = dict(rows[-1])
            error = start_error(before)
            if error>.5 or any(v!=0 for v in before['velocity']):raise RuntimeError('start_pose_changed_during_gripper_initialization')
            if getter(GetRobotState,'/dsr01/system/get_robot_state')['robot_state']!=1:
                raise RuntimeError('robot_not_STANDBY_after_gripper_initialization')
            initial=pose_read()
            tracker[0]=LiveCommandSegment(initial,limits,transport=servo_transport,protection=protection)
            gripper_session[0].start()
        save('preflight.json',dict(hardware=hardware, initial_pose=initial, jointstate=before,
            start_pose_error_deg=error, readiness=check(), physical_gripper_disabled=not full))
        with ExitStack() as stack:
            predictions = stack.enter_context(FsyncJsonlLogger(output/'predictions.jsonl'))
            commands = stack.enter_context(FsyncJsonlLogger(output/'servo_commands.jsonl'))
            started[0] = time.monotonic(); deadline = started[0]+duration
            running[0] = True
            def predict():
                last_source = None
                next_decision = 0.
                try:
                    for step in range(max_predictions):
                        if time.monotonic()>=deadline or guard.closed: break
                        check()
                        robot_state = getter(GetRobotState, '/dsr01/system/get_robot_state')['robot_state']
                        if robot_state not in (1,2): raise RuntimeError('robot_abnormal_state:'+str(robot_state))
                        if getter(GetRobotMode, '/dsr01/system/get_robot_mode')['robot_mode']!=1:
                            raise RuntimeError('robot_not_AUTO')
                        selected = select_fresh_frame(snapshot, check)
                        stamp = selected[0].header.stamp
                        key = (stamp.sec, stamp.nanosec)
                        if key==last_source: raise RuntimeError('duplicate_camera_frame')
                        last_source = key
                        sample = predict_frame(selected, lambda:node.get_clock().now().nanoseconds/1e9, check)
                        result['predictions'] += 1
                        # A faster server must not exceed the unchanged policy
                        # rate gate; the 20Hz stream is a separate clock.
                        while time.monotonic()<next_decision:
                            check(); time.sleep(.002)
                        # Actual controller feedback, not interpolation/virtual pose.
                        current = pose_read()
                        with state_lock: current_feedback = dict(feedback[0])
                        with guard.lock:
                            guard.check()
                            tracker[0].verify_feedback(current_feedback, time.monotonic())
                            command_reference = tracker[0].pose()
                        action = CanonicalAction.from_vector(sample['raw'], timestamp_monotonic=sample['t0'],
                            sequence_id=f'servo-live-{step}', source_model='openvla')
                        decision_now = time.monotonic()
                        decision = pipeline.inspect(action, RuntimeState(decision_now,
                            tuple(v/1000 for v in command_reference[:3]), tuple(command_reference[3:]), 'alignment'))
                        next_decision = decision_now+pipeline.runtime.limiter.dt
                        archive_request_jpeg(output/f'camera_{step:03d}.jpg', sample['jpeg'])
                        g = sample['raw'][6]
                        predictions.append(dict(step=step, raw_action=sample['raw'], camera_timing=sample['timing'],
                            camera_source_timestamp=key, current_controller_pose=current, robot_state=robot_state,
                            controller_feedback=current_feedback,
                            command_reference_pose=command_reference,
                            decision=decision.as_dict(), action_pipeline=decision.candidate,
                            logical_gripper_intent='OPEN' if g<.3 else 'CLOSED' if g>.7 else 'HOLD',
                            physical_gripper_command='ASYNC_STATE_TRANSITION_ONLY' if full else None,
                            logical_gripper_commanded_state=gripper_session[0].full_policy_gripper.state if full else None,
                            jointstate=check()))
                        if not decision.accepted:
                            raise RuntimeError('safety_rejection:'+','.join(decision.reason))
                        with guard.lock:
                            guard.check()
                            if time.monotonic()>=deadline: break
                            # Only the stream thread mutates interpolation state.
                            # Never anchor a new segment to an unsubmitted tick.
                            pending_target[0] = (decision.candidate['limited_canonical_action'], sample['t0'])
                            if full:pending_gripper_intent[0]=(g,sample['t0'],step)
                            result['accepted_targets'] += 1
                except Exception as exc:
                    if not guard.closed: fail(str(exc))
                finally: producer_done.set()
            producer = threading.Thread(target=predict, name='servo-policy-producer'); producer.start()
            next_tick = time.monotonic()
            while time.monotonic()<deadline and not producer_done.is_set():
                check(); now = time.monotonic()
                if now<next_tick:
                    time.sleep(min(next_tick-now,.002)); continue
                with guard.lock:
                    guard.check()
                    if pending_target[0] is not None:
                        limited, source = pending_target[0]
                        tracker[0].set_action(limited, source, now)
                        pending_target[0] = None
                    row = tracker[0].tick(now)
                if row:
                    row.update(requested_monotonic=now, command_issued=False, ack=None)
                    with state_lock: row['latest_controller_feedback'] = dict(feedback[0])
                    commands.append(row)  # Persist request before any submission.
                    check()
                    if time.monotonic()-now>=.1: raise RuntimeError('servo_logger_or_tick_stall')
                    def send():
                        health()
                        if publisher.get_subscription_count()!=1: raise RuntimeError('servo_subscription_lost')
                        if not 0<=time.monotonic()-row['source_monotonic']<=limits['max_action_age_sec']:
                            raise RuntimeError('stale_action_before_publish')
                        with state_lock: latest_feedback = dict(feedback[0])
                        row['following_error'] = tracker[0].verify_feedback(latest_feedback, time.monotonic())
                        msg = ServolStream(); msg.pos=row['pos']; msg.vel=row['vel']; msg.acc=row['acc']; msg.time=row['time']
                        publisher.publish(msg)
                        last_publish[0] = time.monotonic()
                        tracker[0].mark_published(last_publish[0], row['pos'])
                    guard.publish(send)
                    commands.append(dict(event='PUBLISH_RETURNED_NOT_VENDOR_ACK',
                        source_monotonic=row['source_monotonic'], published_monotonic=last_publish[0],
                        command_issued=True, physical_delivery_confirmed=False, ack=None))
                    if full:
                        # First stream submission for this source precedes its
                        # gripper intent. A publish return is NOT arm completion.
                        with guard.lock:
                            guard.check();intent=pending_gripper_intent[0]
                            if intent is not None and intent[1]==row['source_monotonic']:
                                gripper_session[0].offer(*intent);pending_gripper_intent[0]=None
                next_tick = now+tracker[0].period
            check()
            result['termination'] = 'EXISTING_EPISODE_DURATION_LIMIT' if time.monotonic()>=deadline else 'EXISTING_PREDICTION_COUNT_LIMIT'
            if not guard.attempts: raise RuntimeError('no_servo_publish')
            with guard.lock:
                if full:
                    unfinished=gripper_session[0].prepare_finish()
                    intent=pending_gripper_intent[0]
                    if intent is not None:
                        value=intent[0];state=gripper_session[0].full_policy_gripper.state
                        desired='OPEN' if value<.3 else 'CLOSED' if value>.7 else state
                        unfinished=unfinished or desired!=state
                    if unfinished:raise RuntimeError('episode_limit_during_gripper_transition')
                guard.finish()
            producer.join(4)
            if producer.is_alive(): raise RuntimeError('producer_did_not_terminate')
            result['status'] = pending_status
    except KeyboardInterrupt:
        fail('operator_abort'); result.update(status='OPERATOR_ABORT', reason='operator_abort')
    except Exception as exc:
        fail(str(exc)); result.update(status=invalid_status if started[0] is not None else 'NOT_EXECUTED', reason=guard.reason)
    finally:
        guard.finish()
        episode_end = time.monotonic()
        if producer is not None: producer.join(4)
        if full and gripper_session[0] is not None:
            session=gripper_session[0]
            if guard.reason:session.abort_async(guard.reason)
            elif session.prepare_finish():
                fail('gripper_transition_not_finished');result['status']=invalid_status
            session.join()
        end = time.monotonic()+3
        while stop_future[0] is not None and not stop_future[0].done() and time.monotonic()<end: time.sleep(.005)
        if guard.attempts:
            try:
                if stop_future[0] is None or not stop_future[0].done(): raise RuntimeError('stop_ack_timeout_or_unavailable')
                result['stop_response'] = message_to_ordereddict(stop_future[0].result())
                if result['stop_response'].get('success') is not True: raise RuntimeError('stop_ack_failure')
                result['stop_ack'] = True
            except Exception as exc:
                fail(str(exc))
                result.update(stop_ack=False, stop_error=str(exc), status=invalid_status)
                print('Stop ACK 미확인: 로봇을 확인하고 이상 시 즉시 E-stop을 사용하세요.', flush=True)
        running[0] = False
        if guard.attempts and result.get('stop_ack'):
            try:
                result['final_robot_state'] = getter(GetRobotState,
                    '/dsr01/system/get_robot_state', _after_stop=True)
                result['final_robot_state_received_monotonic'] = time.monotonic()
                if result['final_robot_state']['robot_state']!=1:
                    raise RuntimeError('final_robot_not_STANDBY')
            except Exception as exc:
                fail('post_stop_check:'+str(exc))
                result['status'] = invalid_status
        if guard.attempts:
            # ACK is not physical stopping evidence. Require a *post-request*
            # fresh zero-velocity window and current STANDBY; unknown is failure.
            from servo_experimental.stop_evidence import evaluate_stop
            proof_deadline = time.monotonic()+2
            while True:
                with state_lock: stop_rows = list(rows)
                proof = evaluate_stop(ack=result.get('stop_ack') is True,
                    requested_at=result.get('stop_requested_monotonic'),
                    robot_state=result.get('final_robot_state', {}),
                    state_received_at=result.get('final_robot_state_received_monotonic'),
                    joint_rows=stop_rows, now=time.monotonic())
                if (proof['confirmed'] or time.monotonic() >= proof_deadline
                        or not result.get('stop_ack')):
                    break
                # Refresh state evidence rather than repeatedly testing a stale
                # cached response while the post-stop joint window accumulates.
                try:
                    result['final_robot_state'] = getter(GetRobotState,
                        '/dsr01/system/get_robot_state', _after_stop=True)
                    result['final_robot_state_received_monotonic'] = time.monotonic()
                except Exception as exc:
                    result['post_stop_refresh_error'] = str(exc)
                    break
                time.sleep(.005)
            result['stop_confirmation'] = guard.confirm_stop(
                ack=result.get('stop_ack') is True,
                requested_at=result.get('stop_requested_monotonic'),
                robot_state=result.get('final_robot_state', {}),
                state_received_at=result.get('final_robot_state_received_monotonic'),
                joint_rows=stop_rows, now=time.monotonic())
            save('stop_confirmation.json', result['stop_confirmation'])
            if not result['stop_confirmation']['confirmed']:
                fail('stop_confirmation_failed:'+result['stop_confirmation']['reason'])
                result['status'] = invalid_status
        if guard.reason:
            result['reason'] = guard.reason
            if result['status']==pending_status: result['status']=invalid_status
        # Stop ACK/post-stop failures also require the same OFF cleanup. Keep
        # ROS and the logger alive until that idempotent cleanup has finished.
        if full and gripper_session[0] is not None:
            session=gripper_session[0]
            if guard.reason:session.abort_async(guard.reason)
            session.join()
        if gripper_log_context is not None:
            try:gripper_log_context.__exit__(RuntimeError if guard.reason else None,None,None)
            except Exception as exc:
                fail('gripper_logger:'+str(exc))
                result['gripper_logger_error']=str(exc);result['status']=invalid_status
                if gripper_session[0] is not None:gripper_session[0].join()
        if full and gripper_session[0] is not None:
            session=gripper_session[0];gsummary=session.summary()
            result.update(gripper_commands=gsummary['physical_gripper_commands'],
                physical_gripper_commands=gsummary['physical_gripper_commands'],gripper_summary=gsummary)
            save('gripper_io_events.json',session.sink.events)
            if session.cleanup_result is not None:
                save('gripper_abort_result.json',session.cleanup_result)
                if session.cleanup_result.get('state')!='completed':
                    result['gripper_cleanup_error']='BOTH_OUTPUTS_OFF_FAILED';result['status']=invalid_status
        result.update(servo_publish_attempts=guard.attempts, servo_publish_returned=guard.submitted,
            duration_s=None if started[0] is None else episode_end-started[0],
            runtime_valid=result['status']==pending_status and result.get('stop_ack') is True
                and result.get('stop_confirmation', {}).get('confirmed') is True)
        if full:result.update(arm_commands=guard.attempts,episode_ended_wall_time=time.time())
        halt.set(); spinner.join(3)
        with state_lock:
            save('jointstate.json',dict(samples=rows, readiness=gate.status(time.monotonic()), events=gate.events))
            save('last_controller_feedback.json', feedback[0])
            runtime_rows = [row for row in rows if gate.ready_at is not None and row['receive']>=gate.ready_at]
            result['jointstate_max_source_gap_s'] = max((row.get('source_gap') or 0 for row in runtime_rows), default=None)
            result['jointstate_max_receive_gap_s'] = max((row.get('receive_gap') or 0 for row in runtime_rows), default=None)
        node.destroy_node(); rclpy.shutdown()
        save('trial_summary.json', result)
        print(json.dumps(result, indent=2), flush=True)
        print('Result:', output, flush=True)


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'real_trials'/time.strftime('%Y%m%d_%H%M%S_arm_only_servo_episode'))
    parser.add_argument('--max-duration-s', type=float, help='기존 episode 상한보다 짧게만 지정 가능')
    parser.add_argument('--full-policy',action='store_true',help='별도 승인 및 실제 OPEN 확인 필수; gripper 활성화')
    parser.add_argument('--approval',type=Path)
    parser.add_argument('--open-confirmation',type=Path)
    parser.add_argument('--check-readiness',action='store_true',
        help='설정상 실기 차단 사유만 출력. ROS/IO/로봇 명령 없음.')
    args = parser.parse_args()
    if args.check_readiness:
        cfg=yaml.safe_load((ROOT/'configs/servo_episode.yaml').read_text())
        blocked=release_blockers(cfg.get('segment_release',{}))
        print(json.dumps(dict(status='BLOCKED' if blocked else 'CONFIGURED_NOT_LIVE_VERIFIED',
            missing_evidence_or_settings=blocked,stream_planning=cfg['stream_planning'],
            servo_transport=cfg['servo_transport'],physical_commands=0,
            ros_initialized=False,live_hardware_verified=False),indent=2))
        raise SystemExit(0)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    mode=None;approval=None
    if args.full_policy:
        if args.approval is None or args.open_confirmation is None:parser.error('full-policy requires --approval and --open-confirmation')
        from full_policy_live_mode import FullPolicyLiveMode
        mode=FullPolicyLiveMode(args.open_confirmation);approval=json.loads(args.approval.read_text())
    elif args.approval is not None or args.open_confirmation is not None:parser.error('approval files require --full-policy')
    try:
        run(args.output, args.max_duration_s,_full_policy_mode=mode,_approval=approval)
    except PermissionError as exc:
        if not str(exc).startswith('servo_release_blocked:'):
            raise
        # Explicit preflight rejection, not a Python/ROS installation failure.
        print(json.dumps(dict(status='NOT_EXECUTED',reason=str(exc),physical_commands=0,
            next='VERIFY_CONTROLLER_EULER_STOP_JOINT_PROTECTION_AND_FOLLOWING_LIMITS'),indent=2))
        raise SystemExit(2)
