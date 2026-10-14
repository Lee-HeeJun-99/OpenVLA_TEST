"""Read-only sensor/model distribution collection; no physical command clients."""
import argparse,base64,hashlib,io,json,math,sys,threading,time
from pathlib import Path
from urllib.request import Request,urlopen
ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT.parent/x) for x in ('14_real_rollout_integration','02_safety')]
from jointstate_runtime_startup import JointStateStartup
from camera_inference_timing import inference_timing
from live_observation import camera_rgb_image
from verify_live_model_server import validate_identity
from action_step_limits import RAW_TRANSLATION_STEP_LIMIT_M,raw_translation_exceeds_limit

TARGET=[-182.55277,-1.07182,-41.00585,1.25662,-113.50009,1.49975]


def stats(values):
    import numpy as np
    if not values:return dict(count=0)
    return dict(count=len(values),min=float(np.min(values)),mean=float(np.mean(values)),median=float(np.median(values)),
                std=float(np.std(values)),p75=float(np.percentile(values,75)),p90=float(np.percentile(values,90)),
                p95=float(np.percentile(values,95)),p99=float(np.percentile(values,99)),max=float(np.max(values)))


def candidates(values):
    d=stats(values)
    if len(values)<30:return dict(status='INSUFFICIENT_VALID_SAMPLES',candidates=[])
    limits=[('current_10mm',10.),('observed_p95',d['p95']),('observed_p99',d['p99']),('observed_max_no_margin',d['max'])]
    return dict(status='DESCRIPTIVE_CANDIDATES_NOT_APPLIED',candidates=[dict(name=name,threshold_mm=limit,
                rejection_count=sum(v>limit+1e-9 for v in values),rejection_rate=sum(v>limit+1e-9 for v in values)/len(values)) for name,limit in limits],
                margin=None,reason='No physical stopping/workspace evidence establishes a safe extra margin. Percentiles describe this one scene, not a physical safety guarantee.')


def run(output,target=50,max_attempts=70):
    import os,rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState,Image
    from dsr_msgs2.srv import GetRobotMode,GetRobotState,GetRobotSystem
    from rosidl_runtime_py.convert import message_to_ordereddict
    output.mkdir(parents=True,exist_ok=False);(output/'images').mkdir()
    def save(name,value):(output/name).write_text(json.dumps(value,indent=2,allow_nan=False))
    def append(name,value):
        with (output/name).open('a') as stream:stream.write(json.dumps(value,allow_nan=False)+'\n');stream.flush()
    for name in ('predictions.jsonl','valid_predictions.jsonl','invalid_samples.jsonl','preflight_events.jsonl'):(output/name).touch()
    config_path=ROOT.parent/'01_configs/safety_limits.yaml';config_bytes=config_path.read_bytes()
    save('collection_config.json',dict(target_predictions=target,max_attempts=max_attempts,current_translation_limit_m=RAW_TRANSLATION_STEP_LIMIT_M,
        config_sha256=hashlib.sha256(config_bytes).hexdigest(),physical_commands=0,clipping=False,scaling=False,tcp_classification='TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED'))
    os.environ['ROS_LOCALHOST_ONLY']='1';rclpy.init();node=rclpy.create_node('real_action_distribution_prediction_only')
    lock=threading.RLock();stop=threading.Event();gate=JointStateStartup(time.monotonic());state=dict(joint=None,camera=None)
    samples=[];valid=[];invalid=[];used_hashes=set();used_stamps=set();attempt=0;error=None
    def joint(msg):
        received=time.monotonic();source=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        row=dict(receive=received,source=source,header_age=node.get_clock().now().nanoseconds/1e9-source,names=list(msg.name),position=list(msg.position),velocity=list(msg.velocity))
        row['valid']=len(msg.name)==6 and set(msg.name)=={f'joint_{i}' for i in range(1,7)} and len(msg.position)==len(msg.velocity)==6 and all(math.isfinite(v) for v in list(msg.position)+list(msg.velocity))
        with lock:gate.sample(row);state['joint']=row
    def camera(msg):
        with lock:state['camera']=(msg,time.monotonic())
    node.create_subscription(JointState,'/dsr01/joint_states',joint,qos_profile_sensor_data)
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',camera,qos_profile_sensor_data)
    def spin():
        try:
            while not stop.is_set():
                rclpy.spin_once(node,timeout_sec=.005)
                with lock:gate.status(time.monotonic())
        except Exception as exc:
            with lock:gate.fail('EXECUTOR_ERROR:'+str(exc))
    thread=threading.Thread(target=spin);thread.start()
    clients={key:node.create_client(kind,'/dsr01/system/get_robot_'+key) for key,kind in [('mode',GetRobotMode),('state',GetRobotState),('system',GetRobotSystem)]}
    def gate_status():
        with lock:return gate.status(time.monotonic())
    def read_hardware():
        result={}
        for key,kind in [('mode',GetRobotMode),('state',GetRobotState),('system',GetRobotSystem)]:
            deadline=time.monotonic()+2;client=clients[key]
            while not client.service_is_ready() and time.monotonic()<deadline:time.sleep(.005)
            if not client.service_is_ready():raise ValueError('hardware_endpoint_unavailable:'+key)
            if gate_status()['fault']:raise ValueError('jointstate_runtime_fault')
            sent=time.time();future=client.call_async(kind.Request());deadline=time.monotonic()+2
            while not future.done() and time.monotonic()<deadline:time.sleep(.002)
            if not future.done():raise ValueError('hardware_response_timeout:'+key)
            response=message_to_ordereddict(future.result());result[key]=dict(response=response,requested_wall_time=sent,provenance='DRIVER_REPORTED')
            if response.get('success') is not True or response.get('robot_'+key)!=dict(mode=1,state=1,system=0)[key]:raise ValueError('hardware_not_REAL_AUTO_STANDBY:'+key)
        return result
    url='http://127.0.0.1:8766'
    def health():
        with urlopen(url+'/health',timeout=2) as response:return json.load(response)
    def identity(value):return {key:value.get(key) for key in ('model','checkpoint','variant','chunk_size','action_dim','requires_proprio','server_version','preprocessing','center_crop')}
    try:
        while gate_status()['phase']!='RUNTIME_READY':
            if gate_status()['fault']:raise RuntimeError(gate_status()['fault'])
            time.sleep(.01)
        baseline=health();model_config=validate_identity(baseline,'openvla');save('model_health.json',baseline)
        while len(valid)<target and attempt<max_attempts:
            attempt+=1
            record=dict(sample_id=attempt,timestamp_wall=time.time(),command_requested=False,command_issued=False,delivered_action=None,executed_action=None,
                        current_translation_limit_m=RAW_TRANSLATION_STEP_LIMIT_M,tcp_contract_verified=False,tcp_source='FK_ESTIMATED_FLANGE',
                        tcp_classification='TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED',instruction=model_config['instruction'],clipping=False,scaling=False)
            try:
                if config_path.read_bytes()!=config_bytes:raise ValueError('safety_config_changed_during_collection')
                if gate_status()['fault']:raise ValueError('jointstate_runtime_fault')
                record['hardware']=read_hardware()
                if identity(health())!=identity(baseline):raise ValueError('model_health_identity_changed')
                with lock:j=dict(state['joint']);ready=gate.status(time.monotonic())
                ordered=dict(zip(j['names'],j['position']));error_deg=max(abs(math.degrees(ordered[f'joint_{i+1}'])-TARGET[i]) for i in range(6))
                record.update(jointstate_before=j,jointstate_phase=ready['phase'],start_pose_max_error_deg=error_deg)
                if ready['fault'] or ready['phase']!='RUNTIME_READY':raise ValueError('jointstate_not_ready')
                if error_deg>.5:raise ValueError('start_pose_outside_0.5deg')
                deadline=time.monotonic()+2;selected=None
                while time.monotonic()<deadline:
                    with lock:selected=state['camera']
                    if selected and 0<=time.monotonic()-selected[1]<=.01:
                        msg,received=selected;stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
                        if stamp not in used_stamps:break
                    time.sleep(.001)
                else:raise ValueError('camera_fresh_frame_timeout')
                if msg.encoding!='bgra8' or (msg.width,msg.height)!=(1280,720):raise ValueError('camera_encoding_resolution_invalid')
                source_hash=hashlib.sha256(bytes(msg.data)).hexdigest()
                if source_hash in used_hashes:raise ValueError('duplicate_source_image')
                rgb=camera_rgb_image(msg);buffer=io.BytesIO();rgb.save(buffer,format='JPEG',quality=95);jpeg=buffer.getvalue()
                request=Request(url+'/predict',data=json.dumps(dict(image_jpeg_base64=base64.b64encode(jpeg).decode(),instruction=model_config['instruction'])).encode(),headers={'Content-Type':'application/json'})
                ros_start=node.get_clock().now().nanoseconds/1e9;started=time.monotonic()
                if not 0<=started-received<.5:raise ValueError('camera_stale_at_inference_start')
                if gate_status()['fault']:raise ValueError('jointstate_runtime_fault')
                used_hashes.add(source_hash);used_stamps.add(stamp)
                record.update(source_image_sha256=source_hash,model_input_sha256=hashlib.sha256(jpeg).hexdigest(),camera_source_timestamp=stamp,camera_receive_monotonic=received,encoding=msg.encoding,resolution=[msg.width,msg.height])
            except Exception as exc:
                record.update(classification='PREFLIGHT_REJECTED',reason=str(exc));append('preflight_events.jsonl',record)
                if gate_status()['fault']:error='JOINTSTATE_RUNTIME_FAULT';break
                continue
            try:
                with urlopen(request,timeout=2) as response:raw=json.load(response)
                finished=time.monotonic();ros_end=node.get_clock().now().nanoseconds/1e9
                record.update(inference_timing(received,started,finished,source_stamp=stamp[0]+stamp[1]/1e9,ros_start=ros_start,ros_end=ros_end))
                record['raw_model_response']=raw;action=raw.get('action');record['raw_action']=action
                if raw.get('fixture') is True or not isinstance(action,list) or len(action)!=7 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in action):raise ValueError('model_invalid_nonfinite_or_schema')
                with lock:after=gate.status(time.monotonic());latest=dict(state['joint'])
                record.update(jointstate_after=latest,jointstate_runtime=after)
                if after['fault']:raise ValueError('jointstate_runtime_fault')
                tnorm=math.dist(action[:3],[0]*3);rnorm=math.dist(action[3:6],[0]*3)
                record.update(classification='SAMPLE_VALID',translation_xyz_m=action[:3],translation_norm_m=tnorm,rotation_xyz_rad=action[3:6],rotation_norm_rad=rnorm,
                    rotation_norm_deg=math.degrees(rnorm),gripper_closedness=action[6],translation_gate_pass=not raw_translation_exceeds_limit(action[:3]),rotation_gate_pass=rnorm<=math.radians(4)+1e-12,nan_inf_count=0)
                (output/'images'/f'{attempt:03d}.jpg').write_bytes(jpeg)
                valid.append(record);append('valid_predictions.jsonl',record)
            except Exception as exc:
                reason=str(exc);record.update(classification='SAMPLE_INVALID_JOINTSTATE' if 'jointstate' in reason else 'SAMPLE_INVALID_MODEL',reason=reason)
                invalid.append(record);append('invalid_samples.jsonl',record)
            samples.append(record);append('predictions.jsonl',record)
            print(f"attempt={attempt} valid={len(valid)} status={record['classification']}",flush=True)
            if gate_status()['fault']:error='JOINTSTATE_RUNTIME_FAULT';break
    except Exception as exc:error=str(exc)
    finally:
        with lock:save('jointstate_runtime_events.json',dict(status=gate.status(time.monotonic()),events=gate.events))
        stop.set();thread.join(3)
        for client in clients.values():node.destroy_client(client)
        node.destroy_node();rclpy.shutdown()
    translation=[r['translation_norm_m']*1000 for r in valid];rotation=[r['rotation_norm_deg'] for r in valid];grip=[r['gripper_closedness'] for r in valid]
    td=stats(translation);td['units']='mm'
    for limit in (10,12,15):td[f'over_{limit}mm_count']=sum(v>limit+1e-9 for v in translation);td[f'over_{limit}mm_rate']=sum(v>limit+1e-9 for v in translation)/len(valid) if valid else None
    save('translation_distribution.json',td);save('rotation_distribution.json',dict(units='deg',**stats(rotation)))
    save('gripper_distribution.json',dict(**stats(grip),open_like_count=sum(v<.3 for v in grip),mid_count=sum(.3<=v<=.7 for v in grip),close_like_count=sum(v>.7 for v in grip)))
    save('latency_distribution.json',dict(units='seconds',**stats([r['inference_latency'] for r in valid])))
    save('camera_timing_distribution.json',{key:stats([r[key] for r in valid]) for key in ('camera_age_at_inference_start','camera_age_at_inference_end','source_age_at_inference_start','source_age_at_inference_end')})
    save('threshold_candidate_analysis.json',candidates(translation))
    top=sorted(valid,key=lambda r:r['translation_norm_m'],reverse=True)[:5];save('top_translation_samples.json',top)
    from PIL import Image as PILImage
    for i,row in enumerate(top,1):
        PILImage.open(output/'images'/f"{row['sample_id']:03d}.jpg").save(output/f'top_translation_{i:02d}.png')
    summary=dict(attempts=attempt,inference_requests=len(samples),valid_predictions=len(valid),invalid_predictions=len(invalid),preflight_rejections=attempt-len(samples),error=error,
        physical_commands=0,tcp_classification='TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED',physical_rollout_ready=False,current_translation_limit_m=RAW_TRANSLATION_STEP_LIMIT_M,
        recommendation='KEEP_10MM_PENDING_REVIEW; empirical percentiles are candidates, not physical safety evidence',analysis_valid=len(valid)>=30,
        unique_source_hashes=len(used_hashes),independence='distinct source frames; same stationary scene, not IID or multi-scene policy coverage')
    save('collection_summary.json',summary);print(json.dumps(summary,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--target',type=int,default=50);parser.add_argument('--max-attempts',type=int,default=70)
    args=parser.parse_args();run(args.output,args.target,args.max_attempts)
