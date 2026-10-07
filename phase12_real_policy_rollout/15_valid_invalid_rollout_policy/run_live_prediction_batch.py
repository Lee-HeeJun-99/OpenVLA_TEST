"""Live camera/JointState/HTTP only. No robot command client or sink exists."""
import argparse
import base64
import io
import json
import math
import sys
import threading
import time
from pathlib import Path
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT.parent/name) for name in
                ('14_real_rollout_integration', '02_safety', '03_shadow_mode')]
from prediction_only_trial import PredictionOnlyTrial, summarize_runtime_trials
from rollout_trial_classifier import json_safe
from jointstate_runtime_startup import JointStateStartup
from live_observation import camera_rgb_image
from verify_live_model_server import validate_identity
from canonical_action import CanonicalAction
import yaml


def write_json(path, value):
    path.write_text(json.dumps(json_safe(value), indent=2, allow_nan=False))


def distribution(rows):
    import numpy as np
    def stats(key):
        values = [row[key] for row in rows]
        return {name: float(function(values)) if values else None for name, function in
                [('median', np.median), ('p95', lambda x: np.percentile(x, 95)),
                 ('max', np.max), ('min', np.min), ('mean', np.mean)]}
    return dict(predictions=len(rows), translation_m=stats('translation_norm_m'),
                rotation_deg=stats('rotation_norm_deg'), gripper=stats('gripper_closedness'),
                close_candidates=sum(row['gripper_closedness'] >= .7 for row in rows),
                translation_over_4mm=sum(row['translation_norm_m'] > .004 for row in rows))


def run(args):
    import rclpy
    from sensor_msgs.msg import Image, JointState
    from rclpy.qos import qos_profile_sensor_data
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load((ROOT.parent/'14_real_rollout_integration/configs/real_openvla.yaml').read_text())
    url = config['server_url']
    rclpy.init()
    node = rclpy.create_node('phase12_prediction_only_batch')
    lock = threading.RLock()
    stop = threading.Event()
    state = {'readiness': None, 'active': None, 'camera': None, 'joint': None,
             'sample_sequence': 0, 'model_requests': 0, 'dispatch_after_invalid': 0}
    summaries = []; all_valid_predictions = []; model_failures = 0

    def joints(message):
        now = time.monotonic()
        source = message.header.stamp.sec + message.header.stamp.nanosec/1e9
        valid = (len(message.name) == 6 and set(message.name) == {f'joint_{i}' for i in range(1,7)}
                 and len(message.position) == len(message.velocity) == 6
                 and all(math.isfinite(v) for v in list(message.position)+list(message.velocity)))
        with lock:
            state['sample_sequence'] += 1
            row = dict(receive=now, source=source, header_age=node.get_clock().now().nanoseconds/1e9-source,
                       valid=valid, names=list(message.name), position=list(message.position),
                       velocity=list(message.velocity), sample_id=state['sample_sequence'])
            readiness = state['readiness']
            if readiness:
                readiness.sample(row)
                state['joint_rows'].append(dict(row))
                trial = state['active']
                if trial and not trial.runtime_completed and trial.status is None and readiness.fault:
                    reason = ('JOINTSTATE_SOURCE_GAP' if row.get('source_gap', 0) >= .1 else
                              'JOINTSTATE_RECEIVE_GAP' if row.get('receive_gap', 0) >= .1 else
                              readiness.fault)
                    trial.invalidate(reason)
            state['joint'] = row

    def camera(message):
        with lock: state['camera'] = (message, time.monotonic())

    node.create_subscription(JointState, '/dsr01/joint_states', joints, qos_profile_sensor_data)
    node.create_subscription(Image, '/zed/zed_node/rgb/color/rect/image', camera, qos_profile_sensor_data)
    def spin():
        try:
            while not stop.is_set(): rclpy.spin_once(node, timeout_sec=.01)
        except Exception as exc:
            with lock:
                state['spin_error'] = str(exc)
                if state['active']: state['active'].invalidate('RUNTIME_EXECUTOR_ERROR:'+str(exc))
    observer = threading.Thread(target=spin, name='persistent-sensor-executor')
    observer.start()

    def monitor():
        while not stop.wait(.01):
            with lock:
                trial = state['active']
                if not trial or trial.runtime_completed or trial.status is not None: continue
                status = state['readiness'].status(time.monotonic())
                if status['fault']: trial.invalidate(status['fault'])
                elif not state['camera'] or time.monotonic()-state['camera'][1] >= .5:
                    trial.invalidate('CAMERA_STREAM_LOSS')
    watcher = threading.Thread(target=monitor, name='independent-runtime-watchdog')
    watcher.start()
    fatal = None
    try:
        with urlopen(url+'/health', timeout=3) as response: health = json.load(response)
        validate_identity(health, 'openvla')
        write_json(output/'model_health.json', dict(status='MODEL_HEALTH_IDENTITY_PASS', health=health))
        for attempt in range(1, args.max_attempts+1):
            if sum(row['runtime_valid'] for row in summaries) >= args.target_valid_trials: break
            directory = output/f'trial_{attempt:03d}'; directory.mkdir()
            trial = PredictionOnlyTrial(attempt)
            records = []; discarded = []; dispatches = 0; peak_camera = 0.
            with urlopen(url+'/health', timeout=3) as response: current_health=json.load(response)
            for key in ('server_version','model','checkpoint','variant','action_dim','chunk_size','requires_proprio','preprocessing'):
                if current_health.get(key)!=health.get(key):raise ValueError('MODEL_HEALTH_CHANGED:'+key)
            with lock:
                state['active'] = None
                state['readiness'] = JointStateStartup(time.monotonic())
                state['joint_rows'] = []
                readiness = state['readiness']
            # SAME node/subscribers; new first fresh sample + ten-second warm-up.
            deadline = readiness.started+71
            while time.monotonic() < deadline:
                with lock: status = readiness.status(time.monotonic()); image = state['camera']
                if status['fault']: break
                if status['phase'] == 'RUNTIME_READY' and image and time.monotonic()-image[1] < .5: break
                time.sleep(.02)
            write_json(directory/'readiness.json', status)
            with lock:
                startup_rows = list(state['joint_rows'])
                if status['phase'] != 'RUNTIME_READY' or status['fault']:
                    trial.invalidate(status['fault'] or 'JOINTSTATE_DISCOVERY_TIMEOUT')
                else:
                    trial.start({**status, 'first_fresh_after_rearm':readiness.first_fresh >= readiness.started,
                                 'warmup_seconds':time.monotonic()-readiness.first_fresh})
                    state['active'] = trial
            write_json(directory/'startup_warmup_samples.json', startup_rows)
            runtime_t0 = time.monotonic()
            with (directory/'live_predictions.jsonl').open('x') as stream:
                try:
                    while trial.status is None and time.monotonic()-runtime_t0 < args.duration:
                        frame_deadline = time.monotonic()+.5
                        while trial.status is None:
                            with lock: selected = state['camera']
                            if selected and time.monotonic()-selected[1] <= .01: break
                            if time.monotonic() >= frame_deadline:
                                trial.invalidate('CAMERA_FRESH_FRAME_TIMEOUT'); break
                            time.sleep(.001)
                        if trial.status is not None: break
                        image, received = selected
                        if [image.width,image.height] != [1280,720]:
                            trial.invalidate('CAMERA_INVALID_RESOLUTION'); break
                        try:
                            rgb = camera_rgb_image(image); buffer = io.BytesIO()
                            rgb.save(buffer, format='JPEG', quality=95); jpeg = buffer.getvalue()
                            request = Request(url+'/predict', data=json.dumps(dict(image_jpeg_base64=base64.b64encode(jpeg).decode(),
                                instruction=config['instruction'])).encode(), headers={'Content-Type':'application/json'})
                        except Exception as exc:
                            trial.invalidate('MODEL_INPUT_CONSTRUCTION_FAILURE:'+str(exc)); break
                        # Linearize request reservation against asynchronous INVALID.
                        with lock:
                            if trial.status is not None: break
                            dispatches += 1; state['model_requests'] += 1; tick = time.monotonic()
                        try:
                            with urlopen(request, timeout=1.2) as response: raw = json.load(response)
                            if raw.get('fixture') is True: raise ValueError('fixture_forbidden')
                        except Exception as exc:
                            trial.invalidate('MODEL_REQUEST_FAILURE:'+str(exc)); break
                        now = time.monotonic(); latency = now-tick; selected_age = now-received
                        peak_camera = max(peak_camera, selected_age)
                        with lock:
                            joint = dict(state['joint']); runtime = readiness.status(now)
                        observation = dict(timestamp_wall=time.time(), frame_id=len(records),
                            image_timestamp=image.header.stamp.sec+image.header.stamp.nanosec/1e9,
                            camera={'selected_age':selected_age, 'encoding':image.encoding,
                                    'resolution':[image.width,image.height], 'resolution_valid':True,'stream_alive':True},
                            jointstate={**joint, 'latest_age':now-joint['receive']},
                            joint_state_phase=runtime['phase'], tcp_source='UNAVAILABLE',
                            tcp_contract_verified=False, instruction=config['instruction'], command_requested=False,
                            command_issued=False, delivered_action=None, executed_action=None,
                            **{key:None for key in ('episode_id','condition','matched_pair_id','real_observation_id',
                                'sim_observation_id','observation_gap_score','action_gap_translation','action_gap_rotation','action_gap_gripper')})
                        evidence = dict(raw_model_output=raw, inference_latency_s=latency, camera_selected_age_s=selected_age)
                        if trial.status is not None:
                            discarded.append({**evidence,'reason':'IN_FLIGHT_RESPONSE_AFTER_INVALID'}); break
                        trial.prediction(evidence)
                        if not trial.observe(observation): break
                        try:
                            action = CanonicalAction.from_vector(raw['action'],timestamp_monotonic=tick,
                                sequence_id=f'{attempt}-{len(records)}',source_model='openvla')
                        except Exception as exc:
                            trial.invalidate('MODEL_INPUT_NONFINITE_OR_ACTION_SCHEMA:'+str(exc)); break
                        translation = math.dist(action.translation_m,(0,0,0))
                        rotation = math.degrees(math.dist(action.rotation_rotvec_rad,(0,0,0)))
                        blockers = (['raw_translation_step_limit'] if translation > .004+1e-12 else [])
                        if rotation > 4+1e-12: blockers.append('raw_rotation_step_limit')
                        record = {**observation, **evidence,'canonical_action':action.as_dict(),
                                  'translation_norm_m':translation,'rotation_norm_deg':rotation,
                                  'gripper_closedness':action.gripper_closedness,'safety_blockers':blockers,
                                  'motion_safety_validated':False,'runtime_status':'INVALID' if blockers else 'IN_PROGRESS'}
                        # Save exact selected frame, outside inference critical path.
                        (directory/f'frame_{len(records):04d}.jpg').write_bytes(jpeg)
                        stream.write(json.dumps(json_safe(record), allow_nan=False)+'\n');stream.flush()
                        records.append(record)
                        if blockers: trial.invalidate('|'.join(blockers)); break
                    with lock:
                        if trial.status is None: trial.finish_runtime()
                except Exception as exc:
                    trial.invalidate('LOGGER_OR_RUNTIME_ERROR:'+str(exc))
            with lock:
                state['active'] = None
                runtime_rows = [row for row in state['joint_rows'] if row['receive'] >= runtime_t0]
            summary = trial.summary()
            summary.update(prediction_dispatch_count=dispatches, predictions_after_invalid=0,
                command_issued=False,physical_commands=0, max_camera_selected_age_s=peak_camera,
                max_jointstate_source_gap_s=max((row.get('source_gap') or 0 for row in runtime_rows), default=None),
                max_jointstate_receive_gap_s=max((row.get('receive_gap') or 0 for row in runtime_rows), default=None),
                jointstate_runtime_sample_count=len(runtime_rows),nan_inf_count=sum(not all(math.isfinite(v) for v in r['raw_model_output'].get('action',[])) for r in trial.predictions),
                motion_safety_validated=False)
            trial.save(directory);write_json(directory/'trial_summary.json',summary)
            write_json(directory/'jointstate_runtime_samples.json',runtime_rows)
            write_json(directory/'discarded_inflight_responses.json',discarded)
            summaries.append(summary)
            model_failures += summary['model_failure_count']
            if summary['runtime_valid']: all_valid_predictions.extend(records)
            write_json(output/'batch_progress.json',summarize_runtime_trials(summaries))
            print(json.dumps(dict(attempt=attempt, runtime_status=summary['runtime_status'],
                predictions=summary['prediction_count'], invalid_reason=summary['invalid_reason'],
                valid_trials=sum(row['runtime_valid'] for row in summaries))),flush=True)
    except Exception as exc:
        fatal = str(exc)
    finally:
        stop.set(); observer.join(timeout=3);watcher.join(timeout=3)
        node.destroy_node(); rclpy.shutdown()
    summary = summarize_runtime_trials(summaries)
    passed = summary['valid_trials'] >= args.target_valid_trials and model_failures == 0 and not fatal
    summary.update(verdict='PREDICTION_ONLY_BATCH_PASS' if passed else
                   'PREDICTION_ONLY_BATCH_BLOCKED' if not summaries else 'PREDICTION_ONLY_BATCH_UNSTABLE',
                   target_valid_trials=args.target_valid_trials,max_attempts=args.max_attempts,trial_duration_s=args.duration,
                   model_failure_count=model_failures,fatal_error=fatal,command_clients_created=0,
                   dispatch_after_invalid=0,task_success=None,classification_errors=0,
                   next_stage='BLOCKED',motion_authorized=False, trials=summaries,
                   remaining_motion_gates=['TCP_TOOL_CONTRACT','HARDWARE_STATE','MANUAL_SAFETY_CONFIRMATIONS'])
    write_json(output/'batch_summary.json',summary)
    write_json(output/'invalid_reason_counts.json',summary['invalid_reason_counts'])
    write_json(output/'valid_action_distribution.json',distribution(all_valid_predictions))
    (output/'PREDICTION_ONLY_BATCH_REPORT.md').write_text('# Live prediction-only batch\n\n'+
        'Real camera, JointState and OpenVLA; persistent subscribers. No command clients or sinks.\n\n'+
        'Normal completion has runtime_status=VALID, status=null and task_success=null; no physical task outcome.\n'+
        'INVALID is terminal; no further inference is dispatched. Already in-flight replies are retained separately.\n'+
        'Raw translation/rotation hard-limit exceedances terminate the trial. TCP-dependent workspace, gripper phase and\n'+
        'motion safety remain unverified; runtime VALID is not motion readiness. Startup/warm-up samples are preserved.\n\n'+
        '```json\n'+json.dumps({k:v for k,v in summary.items() if k!='trials'},indent=2)+'\n```\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='trials'},indent=2),flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--target-valid-trials',type=int,default=10)
    parser.add_argument('--max-attempts',type=int,default=20)
    parser.add_argument('--duration',type=float,default=20)
    args=parser.parse_args()
    if not 0<args.target_valid_trials<=args.max_attempts or not 20<=args.duration<=30:
        parser.error('require target <= attempts and 20–30 second trials')
    run(args)
