"""Vision-only diagnostic Shadow. No measured TCP claim or command transport."""
import base64,collections,hashlib,io,json,math,threading,time
from pathlib import Path
import sys
sys.path[:0]=[str(Path(__file__).resolve().parent.parent/name) for name in ('02_safety','03_shadow_mode')]
from urllib.request import Request,urlopen
from PIL import Image
import yaml
from live_observation import camera_rgb_array,camera_rgb_image
from verify_live_model_server import validate_identity
from model_health import validate_runtime_health
from canonical_action import CanonicalAction
from safety_pipeline import SafetyPipeline,RuntimeState
from command_sinks import NullCommandSink
from integrated_logger import FsyncJsonlLogger
from jointstate_flange_fk import A0509FlangeFK,reorder_joint_state
from jointstate_runtime_startup import JointStateStartup

def joint_gate(rows,now,window=20):
    recent=[r for r in rows if r['receive']>=now-window]
    gs=[b['source']-a['source'] for a,b in zip(recent,recent[1:])]
    rg=[b['receive']-a['receive'] for a,b in zip(recent,recent[1:])]
    coverage=recent[-1]['receive']-recent[0]['receive'] if len(recent)>1 else 0
    passed=coverage>=window-.1 and all(r['valid'] for r in recent) and bool(gs) and all(0<g<.1 for g in gs) and max(rg)<.1 and now-recent[-1]['receive']<.5
    return dict(status='PASS' if passed else 'FAIL',count=len(recent),coverage_s=coverage,
        latest_receive_age_s=now-recent[-1]['receive'] if recent else None,
        receive_rate_hz=(len(recent)-1)/coverage if coverage else None,
        max_source_gap_s=max(gs) if gs else None,max_receive_gap_s=max(rg) if rg else None,
        invalid_values=sum(not r['valid'] for r in recent))

def run(args):
    if not (args.dry_run and args.live and args.model=='openvla' and args.protocol=='short_horizon'):
        raise PermissionError('flange_shadow_requires_live_dry_run_openvla_only')
    import rclpy
    from sensor_msgs.msg import Image as ImageMsg,JointState
    from rclpy.qos import qos_profile_sensor_data
    root=Path(__file__).resolve().parent;output=args.output
    if output.exists():raise FileExistsError('preserve_existing_shadow')
    output.parent.mkdir(parents=True,exist_ok=True)
    config=yaml.safe_load((root/'configs/real_openvla.yaml').read_text());url=config['server_url']
    def health(strict=True):
        with urlopen(url+'/health',timeout=3) as f:h=json.load(f)
        if strict:validate_identity(h,'openvla')
        return h
    h=health();rclpy.init();node=rclpy.create_node('phase12_flange_prediction_only')
    lock=threading.RLock();samples=[];camera=[None];camera_rows=[];fault=[None];halt=threading.Event()
    readiness=JointStateStartup(time.monotonic())
    fk=A0509FlangeFK('/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf')
    def joint(m):
        now=time.monotonic();valid=len(m.name)==6 and len(set(m.name))==6 and len(m.position)==6 and len(m.velocity)==6 and all(math.isfinite(v) for v in list(m.position)+list(m.velocity)) and set(m.name)==set('joint_'+str(i) for i in range(1,7))
        source=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        with lock:
            row=dict(receive=now,source=source,header_age=node.get_clock().now().nanoseconds/1e9-source,valid=valid,names=list(m.name),position=list(m.position),velocity=list(m.velocity))
            readiness.sample(row);samples.append(row)
    def cam(m):
        with lock:
            camera[0]=(m,time.monotonic());camera_rows.append(dict(receive=camera[0][1],source=m.header.stamp.sec+m.header.stamp.nanosec/1e9,encoding=m.encoding,width=m.width,height=m.height))
    node.create_subscription(JointState,'/dsr01/joint_states',joint,qos_profile_sensor_data)
    node.create_subscription(ImageMsg,'/zed/zed_node/rgb/color/rect/image',cam,qos_profile_sensor_data)
    def spin():
        while not halt.is_set():rclpy.spin_once(node,timeout_sec=.02)
    observer=threading.Thread(target=spin);observer.start();started=time.monotonic();predictions=[];pre=None;watcher=None
    def watchdog():
        while not halt.wait(.02):
            with lock:
                now=time.monotonic();jg=readiness.status(now)
                if jg['fault']:fault[0]=jg['fault'];return
                if not camera[0] or now-camera[0][1]>.5:fault[0]='camera_stale';return
    try:
        # The same node/subscribers survive discovery, warm-up and inference.
        deadline=readiness.started+120
        while time.monotonic()<deadline:
            with lock:
                pre=readiness.status(time.monotonic());c=camera[0]
            if pre['fault'] or (pre['phase']=='RUNTIME_READY' and c and time.monotonic()-c[1]<.5):break
            time.sleep(.05)
        (output.parent/'jointstate_pre_shadow.json').write_text(json.dumps(pre,indent=2))
        with lock:c=camera[0]
        (output.parent/'camera_pre_shadow.json').write_text(json.dumps(camera_rows[-1] if camera_rows else {'status':'NOT_RECEIVED'},indent=2))
        if pre['phase']!='RUNTIME_READY' or pre['fault']:fault[0]=pre['fault'] or 'pre_shadow_jointstate_gate_failed'
        elif not c or time.monotonic()-c[1]>.5:fault[0]='pre_shadow_camera_gate_failed'
        if not fault[0]:
            watcher=threading.Thread(target=watchdog);watcher.start()
        pipeline=SafetyPipeline('openvla',operator_confirmed_initial_open=False);sink=NullCommandSink()
        shadow_start=time.monotonic()
        with FsyncJsonlLogger(output) as log:
            while not fault[0] and time.monotonic()-shadow_start<60:
                # Never hold a camera frame across an HTTP health request.
                health_start=time.monotonic()
                try:
                    currenthealth=health(strict=False)
                    for key in ('server_version','model','checkpoint','variant','action_dim','chunk_size','requires_proprio'):
                        if currenthealth.get(key)!=h.get(key):raise ValueError('model_identity_changed:'+key)
                except Exception as exc:fault[0]='model_health_failure:'+str(exc);break
                health_latency=time.monotonic()-health_start
                # Wait for a newly received frame; do not consume a cached one
                # near its age budget. Watchdog still runs during this wait.
                frame_deadline=time.monotonic()+.5
                while not fault[0]:
                    with lock:image,received=camera[0];j=samples[-1];jg=readiness.status(time.monotonic())
                    if time.monotonic()-received<=.01:break
                    if time.monotonic()>frame_deadline:fault[0]='fresh_camera_snapshot_timeout';break
                    time.sleep(.001)
                if fault[0]:break
                snapshot_time=time.monotonic();encode_start=snapshot_time
                rgb=camera_rgb_image(image);buffer=io.BytesIO();rgb.save(buffer,format='JPEG',quality=95);jpeg=buffer.getvalue()
                encode_latency=time.monotonic()-encode_start
                tick=time.monotonic()
                try:
                    request=Request(url+'/predict',data=json.dumps({'image_jpeg_base64':base64.b64encode(jpeg).decode(),'instruction':config['instruction']}).encode(),headers={'Content-Type':'application/json'})
                    with urlopen(request,timeout=1.2) as response:raw=json.load(response)
                    latency=time.monotonic()-tick
                    if raw.get('fixture') is True:raise ValueError('fixture_forbidden')
                    if latency>1.2:raise TimeoutError('model_timeout')
                    action=CanonicalAction.from_vector(raw['action'],timestamp_monotonic=tick,sequence_id='live-'+str(len(predictions)),source_model='openvla')
                except Exception as exc:fault[0]='model_or_action_failure:'+str(exc);break
                # Vision-only input has no joints/proprio. Refresh safety-only
                # flange context after inference, not an old pre-inference pose.
                with lock:j=samples[-1];jg=readiness.status(time.monotonic())
                positions=reorder_joint_state(j['names'],j['position']);flange=fk.compute(positions)
                now=time.monotonic()
                from scipy.spatial.transform import Rotation
                flange_abc=tuple(Rotation.from_matrix(flange['rotation_matrix']).as_euler('ZYZ',degrees=True))
                decision=pipeline.inspect(action,RuntimeState(now,tuple(flange['position_m']),flange_abc,'unknown',
                    camera_ok=now-received<.5,joint_state_ok=jg['phase']=='RUNTIME_READY' and not jg['fault'],tcp_ok=False))
                safety=decision.as_dict()
                # Never serialize a flange-based pose candidate as robot TCP.
                record=dict(event='PREDICTION',timestamp_wall=time.time(),receive_monotonic=now,frame_id=len(predictions),
                    image_timestamp=dict(sec=image.header.stamp.sec,nanosec=image.header.stamp.nanosec),
                    jointstate_source_timestamp=j['source'],jointstate_age_s=now-j['receive'],camera_age_s=now-received,
                    joint_state_phase=jg['phase'],joint_state_latest_age=jg['latest_receive_age_s'],
                    joint_state_source_gap=jg['source_gap'],joint_state_receive_gap=jg['receive_gap'],
                    tcp_source='FK_ESTIMATED_FLANGE',tcp_contract_verified=False,tool_offset_verified=False,
                    fk_flange=flange,instruction=config['instruction'],raw_model_action=raw,canonical_action=action.as_dict(),
                    translation_norm_m=math.dist(action.translation_m,(0,0,0)),rotation_norm_deg=math.degrees(math.dist(action.rotation_rotvec_rad,(0,0,0))),
                    gripper_closedness=action.gripper_closedness,safety=safety,
                    safety_context='TCP_INVALID; absolute workspace/orientation validation unavailable; phase and gripper physical state UNKNOWN',
                    inference_latency_s=latency,end_to_end_latency_s=now-received,
                    health_latency_s=health_latency,jpeg_encode_latency_s=encode_latency,
                    selected_frame_age_at_snapshot_s=snapshot_time-received,
                    selected_frame_age_at_request_s=tick-received,
                    encoding=image.encoding,resolution=[image.width,image.height],source_image_sha256=hashlib.sha256(bytes(image.data)).hexdigest(),
                    rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),model_input_sha256=hashlib.sha256(jpeg).hexdigest(),
                    command_requested=False,command_issued=False,delivered_action=None,executed_action=None,robot_delivered_command=None,
                    **{k:None for k in ('episode_id','condition','matched_pair_id','real_observation_id','sim_observation_id','observation_gap_score','action_gap_translation','action_gap_rotation','action_gap_gripper')})
                sink.submit(action.as_dict(),safety);log.append(record);predictions.append(record)
                if len(predictions)>=30 and now-shadow_start>=30:break
            log.append(dict(event='TERMINAL',fault=fault[0],command_issued=False,executed_action=None,robot_delivered_command=None))
        blockers=collections.Counter(b for r in predictions for b in r['safety']['reason'])
        rejected=sum(not r['safety']['accepted'] for r in predictions)
        anomalies=sum(r['translation_norm_m']>.004 or r['rotation_norm_deg']>4 for r in predictions)
        input_failures=any(b in blockers for b in ('camera_failure','state_failure','communication_failure','inference_failure','logger_failure'))
        complete=len(predictions)>=30 and time.monotonic()-shadow_start>=30
        summary=dict(status=('LIVE_SHADOW_FAIL_JOINTSTATE_RUNTIME' if fault[0] and readiness.first_fresh is not None and readiness.fault else 'LIVE_SHADOW_FAIL') if fault[0] or anomalies or input_failures else ('LIVE_SHADOW_PASS' if complete else 'LIVE_SHADOW_INCONCLUSIVE'),
            prediction_count=len(predictions),shadow_duration_s=time.monotonic()-shadow_start,pre_shadow_gate=pre,
            safety_rejection_ratio=rejected/len(predictions) if predictions else None,blockers=dict(blockers),
            action_magnitude_threshold_exceedances=anomalies,watchdog_or_abort_reason=fault[0],
            tcp_source='FK_ESTIMATED_FLANGE',tcp_contract_verified=False,tool_offset_verified=False,
            motion_authorized=False,physical_commands=0,next_allowed_stage='TCP_CROSSCHECK_REQUIRED_BEFORE_MOTION')
        (output.parent/'model_health.json').write_text(json.dumps({'status':'MODEL_HEALTH_PASS' if predictions else 'HEALTH_IDENTITY_PASS_SAMPLE_NOT_EXECUTED','health':h},indent=2))
        (output.parent/'live_shadow_summary.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps(summary,indent=2),flush=True)
    finally:
        halt.set();observer.join(timeout=3)
        if watcher:watcher.join(timeout=3)
        with (output.parent/'jointstate_observed.json').open('x') as f:json.dump(samples,f)
        (output.parent/'jointstate_startup_runtime.json').write_text(json.dumps(readiness.status(time.monotonic()),indent=2))
        (output.parent/'jointstate_runtime_events.json').write_text(json.dumps(readiness.events,indent=2))
        (output.parent/'camera_runtime.json').write_text(json.dumps(camera_rows,indent=2))
        node.destroy_node();rclpy.shutdown()
