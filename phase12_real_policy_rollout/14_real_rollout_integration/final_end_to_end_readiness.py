"""Long, minimal, command-incapable live audit. No driver/server lifecycle changes."""
import base64, hashlib, io, json, math, os, queue, subprocess, sys, threading, time
from pathlib import Path
from urllib.request import Request, urlopen
ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT.parent / p) for p in ('02_safety', '03_shadow_mode')]
from jointstate_runtime_startup import JointStateStartup
from integrated_logger import FsyncJsonlLogger
from jointstate_flange_fk import A0509FlangeFK, reorder_joint_state
from live_observation import camera_rgb_image
from verify_live_model_server import validate_identity
from final_hardware_tcp_precheck import source_audit
from canonical_action import CanonicalAction
from safety_pipeline import SafetyPipeline, RuntimeState


def distribution(values):
    import numpy as np
    a = np.asarray(values, dtype=float)
    if not len(a): return {'count': 0}
    return dict(count=len(a), mean=float(a.mean()), std=float(a.std()),
                **{k: float(np.percentile(a, p)) for k, p in [('median',50),('p90',90),('p95',95),('p99',99)]}, max=float(a.max()))


def failure_counts(predictions):
    input_errors = sum(not p.get('valid') and p.get('error') == 'no_frame_within_10ms' for p in predictions)
    model_errors = sum(not p.get('valid') and p.get('error') != 'no_frame_within_10ms' for p in predictions)
    age_errors = sum(p.get('valid',False) and p.get('camera_age',1)>=.5 for p in predictions)
    return dict(input_failures=input_errors, model_failures=model_errors,
                selected_age_failures=age_errors, camera_failure=input_errors+age_errors)


def training_audit(directory):
    episodes, actions, seen = [], [], set()
    for meta in sorted(directory.rglob('episode.json')):
        steps = meta.with_name('steps.jsonl')
        if not steps.exists(): continue
        digest = hashlib.sha256(steps.read_bytes()).hexdigest()
        if digest in seen: continue
        seen.add(digest)
        m = json.loads(meta.read_text()); rows = [json.loads(s) for s in steps.read_text().splitlines() if s.strip()]
        hz = m.get('record_frequency_hz'); close = next((i for i,r in enumerate(rows) if r['action'][6]>=.7), None)
        episodes.append(dict(path=str(meta), steps_sha256=digest, count=len(rows), hz=hz,
                             demonstration_type=m.get('demonstration_type'), first_close_step=close,
                             first_close_seconds=close/hz if close is not None and hz else None))
        actions.extend(r['action'] for r in rows)
    norms = [math.sqrt(sum(x*x for x in a[:3]))*1000 for a in actions]
    return dict(episodes=episodes, unique_episodes=len(episodes), steps=len(actions),
                translation_norm_mm=distribution(norms),
                xyz_mm={k:distribution([a[i]*1000 for a in actions]) for i,k in enumerate(('dx','dy','dz'))},
                over_4mm=sum(n>4 for n in norms), gripper=distribution([a[6] for a in actions]),
                checkpoint_training_membership='UNVERIFIED', representativeness='LIMITED_SAMPLE',
                units='world-relative translation meters per dataset action schema; norms reported mm')


def window_result(rows, start, end, state):
    data = [r for r in rows if start<=r['receive']<end] if start is not None else []
    sg = [r['source_gap'] for r in data if r['source_gap'] is not None]
    rg = [r['receive_gap'] for r in data if r['receive_gap'] is not None]
    coverage = data[-1]['receive']-data[0]['receive'] if len(data)>1 else 0
    passed = bool(data) and coverage>=end-start-.1 and all(r['valid'] for r in data) and all(0<x<.1 for x in sg+rg) and not state['fault'] and state['latest_receive_age_s']<.5
    return dict(status='PASS' if passed else 'FAIL', duration_s=end-start if start is not None else 0,
                message_count=len(data), coverage_s=coverage, rate_hz=(len(data)-1)/coverage if coverage else 0,
                max_source_gap_s=max(sg,default=None), max_receive_gap_s=max(rg,default=None),
                source_events_100ms=sum(x>=.1 for x in sg), receive_events_100ms=sum(x>=.1 for x in rg),
                invalid=sum(not r['valid'] for r in data), missing=sum(r.get('missing',False) for r in data),
                duplicate=sum(x==0 for x in sg), regression=sum(x<0 for x in sg), state=state)


def main():
    import rclpy, yaml
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.utilities import get_rmw_implementation_identifier
    from sensor_msgs.msg import JointState, Image
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    out=ROOT/'real_trials'/(time.strftime('%Y%m%d_%H%M%S')+'_final_end_to_end_readiness');out.mkdir()
    (out/'images').mkdir()
    def save(name,value): (out/(name+'.json')).write_text(json.dumps(value,indent=2))
    def query(cmd):
        try:
            p=subprocess.run(cmd,capture_output=True,text=True,timeout=10,cwd=ROOT)
            return dict(command=cmd,exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
        except subprocess.TimeoutExpired: return dict(command=cmd,timeout=True)
    save('00_environment',dict(wall_time=time.time(),ros={k:os.getenv(k,'UNSET') for k in ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','ROS_LOCALHOST_ONLY')},
         rmw=get_rmw_implementation_identifier(),git=[query(['git','branch','--show-current']),query(['git','rev-parse','HEAD']),query(['git','status','--short'])],
         processes=query(['ps','-eo','pid,ppid,pcpu,pmem,nlwp,args']),load=os.getloadavg(),network=Path('/proc/net/dev').read_text()))
    save('08_action_distribution_training',training_audit(ROOT.parents[1]/'train_dataset'))
    config=yaml.safe_load((ROOT/'configs/real_openvla.yaml').read_text());url=config['server_url']
    health={};health_ok=False
    try:
        with urlopen(url+'/health',timeout=5) as response: health=json.load(response)
        validate_identity(health,'openvla');health_ok=True
        save('04_model_health',dict(status='MODEL_HEALTH_PASS',payload=health,wall_time=time.time()))
    except Exception as exc: save('04_model_health',dict(status='FAIL',reason=str(exc)))
    rclpy.init();node=rclpy.create_node('phase12_minimal_long_readiness')
    lock=threading.RLock();halt=threading.Event();state=JointStateStartup(time.monotonic());rows=[];camera=[None];cam_rows=[];host=[];hardware={};activity=['INFERENCE_OFF'];logger_fault=[];watch_events=[]
    def joints(m):
        now=time.monotonic();source=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        names=list(m.name);pos=list(m.position);vel=list(m.velocity)
        missing=set(names)!={f'joint_{i}' for i in range(1,7)}
        valid=not missing and len(names)==6 and len(pos)==len(vel)==6 and all(math.isfinite(v) for v in pos+vel)
        with lock:
            row=dict(wall_time=time.time(),receive=now,source=source,header_age=node.get_clock().now().nanoseconds/1e9-source,
                     valid=valid,missing=missing,names=names,position=pos,velocity=vel,http_activity=activity[0],
                     camera_callback=cam_rows[-1] if cam_rows else None,host=host[-1] if host else None,graph_cli_activity=False)
            state.sample(row);rows.append(row)
    def cam(m):
        now=time.monotonic()
        with lock:
            camera[0]=(m,now)
            cam_rows.append(dict(receive=now,wall_time=time.time(),source=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                                 callback_gap=now-cam_rows[-1]['receive'] if cam_rows else None,encoding=m.encoding,resolution=[m.width,m.height]))
    def hardware_sample(m,topic):
        with lock:
            prior=hardware.get(topic,{})
            hardware[topic]=dict(receive=time.monotonic(),wall_time=time.time(),count=prior.get('count',0)+1,message=message_to_ordereddict(m))
    node.create_subscription(JointState,'/dsr01/joint_states',joints,qos_profile_sensor_data)
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',cam,qos_profile_sensor_data)
    def spin():
        while not halt.is_set():rclpy.spin_once(node,timeout_sec=.01)
    def status():
        with lock:return state.status(time.monotonic())
    def watchdog():
        last=None
        while not halt.wait(.02):
            s=status()
            if s['fault'] and s['fault']!=last:
                watch_events.append(dict(wall_time=time.time(),receive=time.monotonic(),reason=s['fault'],commands_inhibited=True));last=s['fault']
    def host_monitor():
        while not halt.wait(1):
            host.append(dict(wall_time=time.time(),receive=time.monotonic(),activity=activity[0],load=os.getloadavg(),
                             cpu=Path('/proc/stat').read_text().splitlines()[0],network=Path('/proc/net/dev').read_text(),
                             memory=Path('/proc/meminfo').read_text(),processes={str(pid):Path('/proc/'+str(pid)+'/stat').read_text() for pid in (3061204,3075971,3076021) if Path('/proc/'+str(pid)+'/stat').exists()}))
    threads=[threading.Thread(target=f) for f in (spin,watchdog,host_monitor)]
    for t in threads:t.start()
    logger=FsyncJsonlLogger(out/'05_live_shadow.jsonl');logger.__enter__();log_queue=queue.Queue(maxsize=8)
    def writer():
        while True:
            item=log_queue.get()
            if item is None:log_queue.task_done();return
            record,jpeg,m=item
            try:
                if jpeg is not None:
                    path=out/'images'/f"{record['frame_id']:06d}.jpg";path.write_bytes(jpeg)
                    record.update(model_input_hash=hashlib.sha256(jpeg).hexdigest(),source_image_hash=hashlib.sha256(bytes(m.data)).hexdigest(),image_path=str(path.relative_to(out)))
                logger.append(record)
            except Exception as exc:logger_fault.append(str(exc))
            finally:log_queue.task_done()
    writer_thread=threading.Thread(target=writer);writer_thread.start()
    predictions=[];pipeline=SafetyPipeline('openvla',operator_confirmed_initial_open=False)
    fk=A0509FlangeFK('/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf')
    def predict(tag):
        deadline=time.monotonic()+.5;c=None
        while time.monotonic()<deadline:
            with lock:c=camera[0]
            if c and time.monotonic()-c[1]<=.01:break
            time.sleep(.001)
        record=dict(frame_id=len(predictions),segment=tag,timestamp=time.time(),instruction=config['instruction'],
                    tcp_source='FK_ESTIMATED_FLANGE',tcp_contract_verified=False,tool_offset_verified=False,
                    command_requested=False,command_issued=False,delivered_action=None,executed_action=None,
                    episode_id=out.name,real_observation_id=f'{out.name}_{len(predictions)}',condition=None,matched_pair_id=None,
                    sim_observation_id=None,observation_gap_score=None,action_gap_translation=None,action_gap_rotation=None,action_gap_gripper=None)
        jpeg=None;m=None;begin=time.monotonic()
        try:
            if not c or begin-c[1]>.01:raise RuntimeError('no_frame_within_10ms')
            m,received=c;rgb=camera_rgb_image(m);converted=time.monotonic();b=io.BytesIO();rgb.save(b,format='JPEG',quality=95);jpeg=b.getvalue();encoded=time.monotonic()
            request=Request(url+'/predict',data=json.dumps(dict(image_jpeg_base64=base64.b64encode(jpeg).decode(),instruction=config['instruction'])).encode(),headers={'Content-Type':'application/json'})
            tick=time.monotonic();activity[0]='HTTP_'+str(record['frame_id'])
            with urlopen(request,timeout=3) as response:raw=json.load(response)
            end=time.monotonic();activity[0]='POST_PREDICTION';a=raw.get('action')
            if raw.get('fixture') is True:raise ValueError('fixture_response_not_real_model')
            canonical=CanonicalAction.from_vector(a,timestamp_monotonic=begin,sequence_id=str(record['frame_id']),source_model='openvla')
            with lock:last=rows[-1] if rows else None
            flange=None
            if last and last['valid']:flange=fk.compute(reorder_joint_state(last['names'],last['position']))
            position=tuple(flange['position_m']) if isinstance(flange,dict) else (0.,0.,0.)
            decision=pipeline.inspect(canonical,RuntimeState(end,position,(0.,0.,0.),'APPROACH',camera_ok=end-received<.5,joint_state_ok=status()['phase']=='RUNTIME_READY',tcp_ok=False,logger_ok=not logger_fault))
            record.update(valid=True,raw_action=a,canonical_action=canonical.as_dict(),translation_mm=math.sqrt(sum(v*v for v in a[:3]))*1000,
                          rotation_deg=math.degrees(math.sqrt(sum(v*v for v in a[3:6]))),gripper=a[6],camera_age=end-received,
                          camera_timestamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,encoding=m.encoding,resolution=[m.width,m.height],
                          jointstate=status(),joint_state=last,fk_flange=flange,safety=decision.as_dict(),
                          latency=dict(snapshot_receive_age=begin-received,rgb_conversion=converted-begin,jpeg_encode=encoded-converted,http_request_and_decode=end-tick,
                                       end_to_end=end-begin),inference_latency=end-tick)
        except Exception as exc:
            record.update(valid=False,error=str(exc),jointstate=status())
        activity[0]='INFERENCE_IDLE';predictions.append(record);log_queue.put((record,jpeg,m))
        return record
    try:
        print('TRIAL '+str(out),flush=True)
        # Startup-only graph inspection, never CLI/graph polling during runtime.
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:time.sleep(.1)
        graph=dict(topics=node.get_topic_names_and_types(),services=node.get_service_names_and_types())
        save('startup_graph',graph)
        for topic,types in graph['topics']:
            if types and types[0] in ('dsr_msgs2/msg/RobotState','dsr_msgs2/msg/RobotStateRt'):
                node.create_subscription(get_message(types[0]),topic,lambda m,t=topic:hardware_sample(m,t),qos_profile_sensor_data)
        while state.first_fresh is None and not status()['fault']:time.sleep(.02)
        if state.first_fresh is not None:
            t0=state.first_fresh+10
            while time.monotonic()<t0:time.sleep(.02)
            print('BASELINE_180_BEGIN',flush=True)
            while time.monotonic()<t0+180:time.sleep(.05)
            baseline=window_result(rows,t0,t0+180,status())
        else:baseline=dict(status='FAIL',reason='JOINTSTATE_DISCOVERY_TIMEOUT',duration_s=0,state=status())
        save('01_jointstate_180s',baseline);print('BASELINE '+json.dumps(baseline),flush=True)
        if health_ok:
            cold=predict('FIRST_TRIAL_PREDICTION_NOT_PROVEN_SERVER_COLD');save('cold_prediction',cold)
            activity[0]='MODEL_WARMUP_10S';time.sleep(10)
            start=time.monotonic();print('SHADOW_120_BEGIN',flush=True)
            while time.monotonic()<start+120:predict('RUNTIME_SHADOW')
            shadow_joint=window_result(rows,start,start+120,status())
        else:shadow_joint=dict(status='FAIL',reason='MODEL_HEALTH_FAILED')
        log_queue.join()
        runtime=[p for p in predictions if p['segment']=='RUNTIME_SHADOW'];valid=[p for p in runtime if p.get('valid')]
        failures=failure_counts(runtime);camera_fail=failures['camera_failure']
        summary=dict(predictions=len(runtime),**failures,
                     max_selected_age=max((p['camera_age'] for p in valid),default=None),jointstate=shadow_joint,
                     translation_mm=distribution([p['translation_mm'] for p in valid]),rotation_deg=distribution([p['rotation_deg'] for p in valid]),
                     gripper=distribution([p['gripper'] for p in valid]),close_candidates=sum(p['gripper']>=.7 for p in valid),
                     over_4mm=sum(p['translation_mm']>4 for p in valid),over_4deg=sum(p['rotation_deg']>4 for p in valid),
                     physical_commands=0,tcp_dependent_safety_validation='INCOMPLETE',camera_pass=bool(runtime) and camera_fail==0)
        save('03_camera_120s',summary);save('06_live_shadow_summary',summary);save('07_action_distribution_real',summary)
        save('09_action_outliers',[p for p in valid if p['translation_mm']>4 or p['rotation_deg']>4])
        deltas=[math.sqrt(sum((b['raw_action'][i]-a['raw_action'][i])**2 for i in range(3)))*1000 for a,b in zip(valid,valid[1:])]
        runs=[];current=[]
        for p in valid:
            if p['translation_mm']>4:current.append(p['frame_id'])
            elif current:runs.append(current);current=[]
        if current:runs.append(current)
        save('10_action_temporal_consistency',dict(delta_translation_mm=distribution(deltas),over_4mm_runs=runs,isolated_runs=sum(len(r)==1 for r in runs),visual_causality='UNRESOLVED'))
        save('02_jointstate_events',dict(events=state.events,rows=[r for r in rows if r['source_gap'] is not None and (r['source_gap']>=.1 or r['receive_gap']>=.1 or r['source_gap']<=0)],startup_warmup_preserved=True))
        save('jointstate_all_samples',rows);save('host_stats',host);save('camera_callbacks',cam_rows)
        audit=source_audit()
        for entry in audit:
            if entry['service_type']=='GetLastAlarm':entry['classification']='UNSAFE_OR_NULL_RISK'
        save('11_tcp_sources',dict(source_audit=audit,graph=graph,live_robotstate_samples=hardware,getter_calls=0,
             cache_note='fCurrentToolPosx updated by OnMonitoringDataExCB; no exposed freshness/sequence in flange getter, hence not called',
             active_name_binding='UNVERIFIED',config_candidates='historical only'))
        with lock:last=rows[-1] if rows else None
        try:flange=fk.compute(reorder_joint_state(last['names'],last['position'])) if last and last['valid'] else None
        except Exception as exc:flange={'error':str(exc)}
        save('12_flange_pose',dict(source='FK_ESTIMATED_FLANGE',pose=flange,not_active_tcp=True))
        save('13_tcp_contract',dict(classification='TCP_FLANGE_ONLY',active_tcp='UNKNOWN',active_tool='UNKNOWN',offset_verified=False,current_tcp_verified=False))
        for name in ('14_robot_mode','15_robot_state','16_connection','17_authority','18_servo','19_protective_stop','20_emergency_stop'):
            save(name,dict(value='UNKNOWN',provenance='UNKNOWN',fresh_affirmative_evidence=False,reason='No validated fresh continuous source; prior enum getters not current evidence'))
        save('21_gripper',dict(state='UNKNOWN',physical_command_disabled=True,pulse_disabled=True,model_intent_not_executed=True))
        save('22_watchdog_logger',dict(watchdog_events=watch_events,watchdog_status='PASS' if not watch_events else 'FAIL',logger_errors=logger_fault,
                                     logger_status='PASS' if not logger_fault else 'FAIL',threads_joined_after_run=True,command_capability_absent=True))
        save('23_manual_requirements',dict(operator='MANUAL_CONFIRMATION_REQUIRED',workspace='MANUAL_CONFIRMATION_REQUIRED',estop='MANUAL_CONFIRMATION_REQUIRED'))
        blockers=['TCP_CONTRACT_UNVERIFIED','HARDWARE_STATE_UNKNOWN']
        if baseline['status']!='PASS' or shadow_joint['status']!='PASS':blockers.append('JOINTSTATE_RUNTIME_FAIL')
        if not summary['camera_pass']:blockers.append('CAMERA_RUNTIME_FAIL')
        if summary['over_4mm'] or summary['over_4deg']:blockers.append('RAW_ACTION_STEP_LIMIT_EXCEEDED')
        if not health_ok or summary['model_failures']:blockers.append('MODEL_FAILURE')
        if logger_fault:blockers.append('LOGGER_FAILURE')
        final=dict(verdict='BLOCKED_MULTIPLE',blockers=blockers,manual_confirmations='REQUIRED',physical_commands=0,next_allowed_stage='BLOCKED',summary=summary,baseline=baseline)
        save('24_final_readiness',final)
        (out/'FINAL_END_TO_END_READINESS_REPORT.md').write_text('# Final end-to-end read-only readiness\n\n```json\n'+json.dumps(final,indent=2)+'\n```\n\nSame JointState and camera subscribers throughout discovery, 10 s warm-up, fixed 180 s baseline and 120 s Shadow. Failure latches; no window reset. Read-only diagnostics continue after a fault, never motion. Source events and cold/first-trial prediction are preserved. No command/service clients, publishers or driver restart. TCP context is flange only; actual TCP unknown. Raw 4 mm/4 deg thresholds are configured software safety step blockers, not manufacturer physical limits. Training corpus sample is not proven to belong to this checkpoint. Different sampling rates prevent naive velocity comparison. Manual safety and hardware unknowns remain independent.\n')
        print('FINAL '+str(out)+' '+json.dumps(final),flush=True)
    finally:
        log_queue.put(None);writer_thread.join(timeout=5);logger.__exit__(None,None,None)
        halt.set()
        for t in threads:t.join(timeout=3)
        node.destroy_node();rclpy.shutdown()
        completed=out/'22_watchdog_logger.json'
        if completed.exists():
            payload=json.loads(completed.read_text())
            payload.update(threads_joined_after_run=not any(t.is_alive() for t in threads+[writer_thread]))
            save('22_watchdog_logger',payload)


if __name__=='__main__':main()
