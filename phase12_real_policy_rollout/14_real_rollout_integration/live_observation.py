"""Subscribers and HTTP only. TCP getter explicitly absent."""
import base64,io,json,math,threading,time,hashlib
from urllib.request import urlopen,Request
from PIL import Image

def camera_rgb_array(image):
    """Honor ROS encoding and row padding; discard alpha, never assume RGB."""
    import numpy as np
    channels={'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4}.get(image.encoding)
    if channels is None:raise ValueError('unsupported_camera_encoding')
    if image.step<image.width*channels:raise ValueError('invalid_camera_step')
    arr=np.frombuffer(bytes(image.data),dtype=np.uint8).reshape(image.height,image.step)[:,:image.width*channels].reshape(image.height,image.width,channels)
    return arr[:,:,[2,1,0]] if image.encoding in ('bgr8','bgra8') else arr[:,:,:3]

class LiveObservation:
    def __init__(self,url,checkpoint,variant,evidence):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image as ImageMsg,JointState
        from std_msgs.msg import Float64MultiArray
        rclpy.init();self.node=rclpy.create_node('phase12_gated_rollout_observation')
        from robot_state_monitor import RobotStateMonitor
        self.hardware_monitor=RobotStateMonitor()
        self.executor=MultiThreadedExecutor(num_threads=4);self.executor.add_node(self.node)
        self.lock=threading.Lock();self.data={};self.url=url;self.evidence=evidence;self.rt_state={}
        self.node.create_subscription(ImageMsg,'/zed/zed_node/rgb/color/rect/image',self.camera,qos_profile_sensor_data)
        self.node.create_subscription(JointState,'/dsr01/joint_states',self.joints,qos_profile_sensor_data)
        self.node.create_subscription(Float64MultiArray,'/doosan/current_pose',self.tcp,10)
        if evidence.get('verified_robot_state_topic'):
            from dsr_msgs2.msg import RobotState
            self.node.create_subscription(RobotState,evidence['verified_robot_state_topic'],self.robot_state,10)
        from std_msgs.msg import Float32MultiArray
        for key in ('robot_state','robot_mode'):
            topic=evidence.get('verified_rt_'+key+'_topic')
            if topic:self.node.create_subscription(Float32MultiArray,topic,lambda msg,key=key:self.rt_robot_state(msg,key),10)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        with urlopen(url+'/health',timeout=3) as response:self.health=json.load(response)
        if self.health.get('variant')!=variant:raise RuntimeError('model_variant_mismatch')
        if checkpoint not in str(self.health.get('bundle',self.health.get('checkpoint',''))):raise RuntimeError('checkpoint_mismatch')
        from model_health import validate_runtime_health
        if 'verified_model_health_contract' not in evidence:raise RuntimeError('verified_model_health_contract_required')
        validate_runtime_health(self.health,evidence['verified_model_health_contract'])
        if evidence.get('source')!='FAKE_TEST_GRAPH':
            from verify_live_model_server import validate_identity
            validate_identity(self.health,evidence['verified_model_health_contract']['model'])
        self.health_checked=time.monotonic();self.health_ok=True
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            with self.lock:ready=all(k in self.data for k in ('camera','joints')) and ('tcp' in self.data or evidence.get('fk_tcp_approved') is True)
            if evidence.get('verified_robot_state_topic') or evidence.get('verified_rt_robot_state_topic'):ready=ready and self.hardware_monitor.value is not None
            if ready:break
            time.sleep(.02)

    def camera(self,msg):
        with self.lock:self.data['camera']=(msg,time.monotonic())
    def joints(self,msg):
        with self.lock:self.data['joints']=(msg,time.monotonic())
    def tcp(self,msg):
        with self.lock:self.data['tcp']=(list(msg.data),time.monotonic())
    def robot_state(self,msg):
        # Fake graph metadata is explicitly barred from motion approval.
        fake=self.evidence.get('source')=='FAKE_TEST_GRAPH'
        fresh=0<=time.time()-self.evidence.get('verified_wall_time',0)<=60
        confirmations=self.evidence.get('hardware_operator_confirmations') if fresh and not fake else None
        fake_state={}
        if fake and msg.robot_state_str:
            try:fake_state=json.loads(msg.robot_state_str)
            except ValueError:fake_state={}
        self.hardware_monitor.update(msg,self.evidence['verified_robot_state_topic'],
            robot_mode=fake_state.get('mode','AUTO') if fake else None,servo_enabled=fake_state.get('servo',True) if fake else None,
            authority=None,confirmations=confirmations)
    def rt_robot_state(self,msg,key):
        # Exact optional driver publishers /rt_topic/robot_state and robot_mode.
        # No RT connect/start or getter is invoked by this subscriber adapter.
        if len(msg.data)!=1 or not math.isfinite(msg.data[0]):return
        value=msg.data[0]
        if value!=int(value):return
        with self.lock:
            self.rt_state[key]=(int(value),time.monotonic());values=dict(self.rt_state)
        if not all(k in values for k in ('robot_state','robot_mode')):return
        from types import SimpleNamespace
        mode={0:'MANUAL',1:'AUTO'}.get(values['robot_mode'][0])
        fresh=0<=time.time()-self.evidence.get('verified_wall_time',0)<=60
        confirmations=self.evidence.get('hardware_operator_confirmations') if fresh else None
        if confirmations:confirmations={k:v for k,v in confirmations.items() if k!='operation_mode'}
        self.hardware_monitor.update(SimpleNamespace(robot_state=values['robot_state'][0],disconnected=False),
            'DRIVER_RT_TOPIC_RECEIVE',robot_mode=mode,now=min(v[1] for v in values.values()),confirmations=confirmations)

    def snapshot(self):
        with self.lock:data=dict(self.data)
        now=time.monotonic()
        if 'camera' not in data or 'joints' not in data:raise RuntimeError('observation_unavailable')
        image,ci=data['camera'];joints,ji=data['joints']
        positions=dict(zip(joints.name,joints.position));velocities=dict(zip(joints.name,joints.velocity))
        names=[f'joint_{i}' for i in range(1,7)]
        finite=all(n in positions and n in velocities and math.isfinite(positions[n]) and math.isfinite(velocities[n]) for n in names)
        tcp=data.get('tcp');source='MEASURED'
        if not tcp or now-tcp[1]>.5:
            if self.evidence.get('fk_tcp_approved') is not True:raise RuntimeError('tcp_unavailable')
            from jointstate_flange_fk import A0509FlangeFK
            from scipy.spatial.transform import Rotation
            import numpy as np
            result=A0509FlangeFK(self.evidence['fk_urdf_path']).compute(positions)
            # Approved flange->TCP transform is required even when offset is zero.
            transform=np.asarray(self.evidence['flange_to_tcp_matrix'],dtype=float)
            if transform.shape!=(4,4) or not np.isfinite(transform).all():raise ValueError('invalid_tcp_transform')
            current=np.eye(4);current[:3,:3]=result['rotation_matrix'];current[:3,3]=result['position_m']
            target=current@transform
            pose=target[:3,3].tolist()+Rotation.from_matrix(target[:3,:3]).as_euler('ZYZ',degrees=True).tolist()
            tcp=(pose,ji);source='FK_ESTIMATED'
        else:
            pose=tcp[0]
            if len(pose)!=6 or not all(math.isfinite(x) for x in pose):raise ValueError('invalid_tcp')
            pose=[x/1000 for x in pose[:3]]+pose[3:]
        arr=camera_rgb_array(image)
        jpeg=io.BytesIO();Image.fromarray(arr).save(jpeg,format='JPEG',quality=95)
        return jpeg.getvalue(),dict(receive_monotonic=now,tcp_m_abc=pose,tcp_source=source,
            image_sha256=hashlib.sha256(bytes(image.data)).hexdigest(),image_hash_scope='SOURCE_ROS_PIXEL_BYTES',
            rgb_pixel_sha256=hashlib.sha256(arr.tobytes()).hexdigest(),model_input_sha256=hashlib.sha256(jpeg.getvalue()).hexdigest(),
            camera_encoding=image.encoding,model_color_order='RGB',
            camera_ok=now-ci<=.5,joint_state_ok=finite and now-ji<=.5,tcp_ok=now-tcp[1]<=.5,
            model_ok=True,communication_ok=now-ji<=.5,
            **self.hardware_monitor.snapshot(),
            phase=self.evidence.get('phase','unknown'),joint_positions_by_name=positions,
            joint_velocity_by_name=velocities,effort='UNSUPPORTED',
            image_source_timestamp={'sec':image.header.stamp.sec,'nanosec':image.header.stamp.nanosec},
            joint_source_timestamp={'sec':joints.header.stamp.sec,'nanosec':joints.header.stamp.nanosec},
            tcp_source_timestamp=None,clock_domains={'source':'ROS_HEADER','receive':'HOST_MONOTONIC'})

    def predict(self,image,model):
        payload={'image_jpeg_base64':base64.b64encode(image).decode(),'instruction':'Pick up the orange cube.'}
        request=Request(self.url+'/predict',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
        started=time.monotonic()
        with urlopen(request,timeout=1.2) as response:result=json.load(response)
        self.last_prediction={'raw_model_output':result,'inference_latency_s':time.monotonic()-started,
            'model_input_sha256':hashlib.sha256(image).hexdigest()}
        return result['actions'] if model=='oft' else [result['action']]

    def check_health(self):
        if time.monotonic()-self.health_checked<.2:return self.health_ok
        try:
            from model_health import validate_runtime_health
            with urlopen(self.url+'/health',timeout=.2) as response:health=json.load(response)
            validate_runtime_health(health,self.evidence['verified_model_health_contract'])
            self.health_ok=True
        except Exception:self.health_ok=False
        self.health_checked=time.monotonic()
        return self.health_ok

    def close(self):
        import rclpy
        self.executor.shutdown();self.thread.join(timeout=2);self.node.destroy_node();rclpy.shutdown()
