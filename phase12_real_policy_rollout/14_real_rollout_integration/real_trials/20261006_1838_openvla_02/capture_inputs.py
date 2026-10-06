"""Subscriber-only input capture; no command or getter capability."""
import sys,time,json,csv,hashlib
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,JointState
from PIL import Image as PILImage
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[1]))
from live_observation import camera_rgb_array
def main():
    rclpy.init();node=Node('phase12_model_input_validation');js=[];frames=[];start=time.monotonic()
    def camera(m):
        rgb=camera_rgb_array(m);now=time.monotonic()
        row=dict(receive_monotonic=now,source_ros=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
            encoding=m.encoding,width=m.width,height=m.height,source_sha256=hashlib.sha256(bytes(m.data)).hexdigest(),
            rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest())
        if not frames:PILImage.fromarray(rgb).save(ROOT/'live_rgb.png')
        frames.append(row)
    def joint(m):
        js.append(dict(receive_monotonic=time.monotonic(),source_ros=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
            names=list(m.name),position=list(m.position),velocity=list(m.velocity)))
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',camera,qos_profile_sensor_data)
    node.create_subscription(JointState,'/dsr01/joint_states',joint,qos_profile_sensor_data)
    while time.monotonic()-start<35:rclpy.spin_once(node,timeout_sec=.02)
    def stats(rows):
        gaps=[b['receive_monotonic']-a['receive_monotonic'] for a,b in zip(rows,rows[1:])]
        sg=[b['source_ros']-a['source_ros'] for a,b in zip(rows,rows[1:])]
        return dict(count=len(rows),first_delay_s=rows[0]['receive_monotonic']-start if rows else None,
            max_receive_gap_s=max(gaps) if gaps else None,max_source_gap_s=max(sg) if sg else None,
            latest_age_s=time.monotonic()-rows[-1]['receive_monotonic'] if rows else None,
            source_gaps_ge_100ms=sum(x>=.1 for x in sg))
    for name,rows in [('camera_frames',frames),('joint_samples',js)]:
        with (ROOT/(name+'.json')).open('x') as f:json.dump(rows,f)
    summary=dict(camera=stats(frames),joints=stats(js),camera_last=frames[-1] if frames else None,
        robot_commands=0,executed_action=None,robot_delivered_command=None)
    with (ROOT/'input_validation.json').open('x') as f:json.dump(summary,f,indent=2)
    print(json.dumps(summary,indent=2));node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
