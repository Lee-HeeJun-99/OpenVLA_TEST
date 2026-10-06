"""Subscribers and HTTP only. TCP getter explicitly absent."""
import base64,io,json,math,threading,time,hashlib
from urllib.request import urlopen,Request
from PIL import Image

class LiveObservation:
    def __init__(self,url,checkpoint,variant,evidence):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image as ImageMsg,JointState
        from std_msgs.msg import Float64MultiArray
        rclpy.init();self.node=rclpy.create_node('phase12_gated_rollout_observation')
        self.executor=MultiThreadedExecutor(num_threads=4);self.executor.add_node(self.node)
        self.lock=threading.Lock();self.data={};self.url=url;self.evidence=evidence
        self.node.create_subscription(ImageMsg,'/zed/zed_node/rgb/color/rect/image',self.camera,qos_profile_sensor_data)
        self.node.create_subscription(JointState,'/dsr01/joint_states',self.joints,qos_profile_sensor_data)
        self.node.create_subscription(Float64MultiArray,'/doosan/current_pose',self.tcp,10)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        with urlopen(url+'/health',timeout=3) as response:self.health=json.load(response)
        if self.health.get('variant')!=variant:raise RuntimeError('model_variant_mismatch')
        if checkpoint not in str(self.health.get('bundle',self.health.get('checkpoint',''))):raise RuntimeError('checkpoint_mismatch')

    def camera(self,msg):
        with self.lock:self.data['camera']=(msg,time.monotonic())
    def joints(self,msg):
        with self.lock:self.data['joints']=(msg,time.monotonic())
    def tcp(self,msg):
        with self.lock:self.data['tcp']=(list(msg.data),time.monotonic())

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
        if image.encoding not in ('rgb8','bgr8'):raise ValueError('unsupported_camera_encoding')
        import numpy as np
        arr=np.frombuffer(bytes(image.data),dtype=np.uint8).reshape(image.height,image.step)[:,:image.width*3].reshape(image.height,image.width,3)
        if image.encoding=='bgr8':arr=arr[:,:,::-1]
        jpeg=io.BytesIO();Image.fromarray(arr).save(jpeg,format='JPEG',quality=95)
        return jpeg.getvalue(),dict(receive_monotonic=now,tcp_m_abc=pose,tcp_source=source,
            image_sha256=hashlib.sha256(bytes(image.data)).hexdigest(),image_hash_scope='SOURCE_ROS_PIXEL_BYTES',
            camera_ok=now-ci<=.5,joint_state_ok=finite and now-ji<=.5,tcp_ok=now-tcp[1]<=.5,
            model_ok=True,communication_ok=now-ji<=.5,
            robot_state_ok=self.evidence.get('robot_mode_confirmed') is True and time.time()-self.evidence.get('verified_wall_time',0)<=60,
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

    def close(self):
        import rclpy
        self.executor.shutdown();self.thread.join(timeout=2);self.node.destroy_node();rclpy.shutdown()
