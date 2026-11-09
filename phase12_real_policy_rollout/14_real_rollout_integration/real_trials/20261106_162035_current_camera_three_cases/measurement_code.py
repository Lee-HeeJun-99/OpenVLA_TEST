"""Camera load comparison. No motion, gripper, or stop transport exists here."""
import json, os, subprocess, threading, time
from pathlib import Path
import rclpy
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from rollout_latency import predict_frame, select_fresh_frame

root=Path('/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout')
p14=root/'14_real_rollout_integration'
out=p14/'real_trials'/time.strftime('%Y%m%d_%H%M%S_current_camera_three_cases')
out.mkdir(); print('OUTPUT',out,flush=True)
os.environ['ROS_LOCALHOST_ONLY']='1'
rclpy.init(); node=rclpy.create_node('current_camera_no_motion_load_probe')
camera=[None]; lock=threading.Lock(); halt=threading.Event()
def image(msg):
    with lock: camera[0]=(msg,time.monotonic())
node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',image,qos_profile_sensor_data)
def spin():
    while not halt.is_set(): rclpy.spin_once(node,timeout_sec=.005)
thread=threading.Thread(target=spin); thread.start()
def snapshot():
    with lock:return camera[0]
cases=[]
try:
    for case in ['A_camera_server_idle','B_camera_repeated_inference','C_inference_action_preparation_no_dispatch']:
        print('START',case,flush=True)
        log=(out/(case+'.log')).open('x')
        observer=subprocess.Popen(['/usr/bin/python3',str(root/'16_jointstate_transport_diagnostics/zed_capture_receive_audit.py'),'--duration','120','--output',str(out/case)],stdout=log,stderr=subprocess.STDOUT)
        rows=[]; start=time.monotonic(); child=None; child_log=None; count=0
        while observer.poll() is None:
            if case.startswith('B'):
                try:
                    s=predict_frame(select_fresh_frame(snapshot,lambda:None),lambda:node.get_clock().now().nanoseconds/1e9,lambda:None)
                    rows.append(dict(timing=s['timing'],raw_action=s['raw'],command_requested=False,command_issued=False))
                except Exception as exc:
                    rows.append(dict(wall=time.time(),error=str(exc),command_issued=False));time.sleep(.1)
            elif case.startswith('C'):
                if child is None or child.poll() is not None:
                    if child_log:child_log.close()
                    child_log=(out/f'{case}_probe_{count:02d}.log').open('x')
                    child=subprocess.Popen(['/usr/bin/python3',str(p14/'validate_action_latency.py'),'--samples','20','--output',str(out/f'{case}_probe_{count:02d}')],stdout=child_log,stderr=subprocess.STDOUT)
                    count+=1
                time.sleep(.1)
            else:time.sleep(.1)
        # Allow our bounded diagnostic child to finish, never kill any live process.
        if child:child.wait(timeout=180)
        if child_log:child_log.close()
        log.close()
        (out/(case+'_predictions.json')).write_text(json.dumps(rows,indent=2))
        summary=json.loads((out/case/'summary.json').read_text())
        cases.append(dict(case=case,observer=summary,inference_count=len(rows),probe_count=count,elapsed=time.monotonic()-start))
        (out/'comparison.json').write_text(json.dumps(dict(cases=cases,physical_commands=0,transport_created=False,limitation='C uses existing prediction/required logging/read-only state/settle probe, not actual motor dispatch or complete physical Controller execution. No configs or existing processes changed.'),indent=2))
        print('END',case,json.dumps(summary['streams']['image']),flush=True)
finally:
    halt.set();thread.join();node.destroy_node();rclpy.shutdown()
print('DONE',out,flush=True)
