"""Explicitly approved ZED-only FPS restart and controlled passive comparison."""
import json,os,signal,subprocess,time
from pathlib import Path
import yaml
LHJ=Path(__file__).resolve().parents[2]
OUT=LHJ/'phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261030_zed_fps30_001'
NODE='/zed/zed_node'
def save(name,x):(OUT/name).write_text(json.dumps(x,indent=2))
def params():
    r=subprocess.run(['ros2','param','dump',NODE],capture_output=True,text=True,timeout=15)
    if r.returncode:raise RuntimeError(r.stderr or r.stdout)
    return yaml.safe_load(r.stdout)[NODE]['ros__parameters'],r.stdout
def flat(d,prefix=''):
    out={}
    for k,v in d.items():
        key=prefix+'.'+k if prefix else k
        if isinstance(v,dict):out.update(flat(v,key))
        else:out[key]=v
    return out
pid=71185;container=71202
expected=['/usr/bin/python3','/opt/ros/humble/bin/ros2','launch','zed_wrapper','zed_camera.launch.py','camera_model:=zed2i']
cmd=Path(f'/proc/{pid}/cmdline').read_bytes().decode().strip('\0').split('\0')
if cmd!=expected:raise RuntimeError('ZED_launch_PID_or_command_changed')
before,raw=params();saved=yaml.safe_load((OUT/'parameters_before.yaml').read_text())[NODE]['ros__parameters']
if before!=saved:raise RuntimeError('parameters_changed_since_snapshot')
env=dict(entry.split('=',1) for entry in Path(f'/proc/{pid}/environ').read_text().split('\0') if '=' in entry)
env['ROS_LOCALHOST_ONLY']='1'
buffer_before=Path('/sys/module/usbcore/parameters/usbfs_memory_mb').read_text().strip()
save('restart_scope.json',dict(target='ZED_ONLY',old_launch_pid=pid,old_container_pid=container,original_command=cmd,
    driver_restart=False,physical_commands=0,usbfs_memory_mb_before=buffer_before,
    ros_environment={k:env.get(k) for k in ('ROS_LOCALHOST_ONLY','ROS_DOMAIN_ID','RMW_IMPLEMENTATION')},
    parameter_dump_note='Raw dump contains empty-name image_transport branch rejected by ROS YAML parser. Override excludes that branch; its regenerated values are included in exact before/after comparison.'))
os.kill(pid,signal.SIGINT)
end=time.monotonic()+20
while time.monotonic()<end and Path(f'/proc/{container}').exists():time.sleep(.1)
if Path(f'/proc/{container}').exists():raise RuntimeError('old_ZED_container_did_not_exit_no_force_kill')
override=str(OUT/'parameters_fps30_override.yaml')
with (OUT/'zed_launch_30fps.log').open('x') as log:
    process=subprocess.Popen(cmd+['ros_params_override_path:='+override],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
save('new_launch.json',dict(pid=process.pid,command=cmd+['ros_params_override_path:='+override]))
end=time.monotonic()+60;after=None
while time.monotonic()<end:
    if process.poll() is not None:raise RuntimeError('new_ZED_launch_exited')
    try:
        after,raw=params()
        if after.get('general',{}).get('grab_frame_rate')==30:break
    except Exception:pass
    time.sleep(1)
if after is None:raise RuntimeError('new_ZED_parameters_unavailable')
(OUT/'parameters_after.yaml').write_text(raw)
b,a=flat(before),flat(after)
changes=[dict(parameter=k,before=b.get(k),after=a.get(k)) for k in sorted(set(b)|set(a)) if b.get(k)!=a.get(k)]
buffer_after=Path('/sys/module/usbcore/parameters/usbfs_memory_mb').read_text().strip()
valid=changes==[dict(parameter='general.grab_frame_rate',before=60,after=30)] and buffer_before==buffer_after
save('parameter_comparison.json',dict(only_fps_changed=valid,changes=changes,usbfs_memory_mb_before=buffer_before,usbfs_memory_mb_after=buffer_after,
    depth_mode=after.get('depth',{}).get('depth_mode'),physical_commands=0))
print(json.dumps(dict(only_fps_changed=valid,changes=changes,buffer_mb=buffer_after)),flush=True)
if not valid:raise RuntimeError('non_FPS_parameter_difference_measurement_blocked')
time.sleep(5)
subprocess.run(['/usr/bin/python3',str(Path(__file__).parent/'zed_capture_receive_audit.py'),'--duration','120','--output',str(OUT/'measurement')],env=env,check=True)
subprocess.run(['/usr/bin/python3',str(Path(__file__).parent/'summarize_zed_capture_receive.py'),str(OUT/'measurement')],env=env,check=True)
