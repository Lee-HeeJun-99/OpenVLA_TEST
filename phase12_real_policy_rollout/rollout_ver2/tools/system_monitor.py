#!/usr/bin/python3
"""Out-of-process 2 Hz host/GPU/ROS receive monitor.  Never imports wandb."""
from __future__ import annotations
import argparse,json,os,signal,threading,time,traceback
from collections import deque
from pathlib import Path
from urllib.parse import urlparse
import psutil


def cpu_frequency_mhz(freq):
    """Normalize psutil's documented-MHz result on mixed-unit Linux builds.

    psutil documents scpufreq fields as MHz.  On this host psutil 5.9.0 returns
    current≈3.6 while min/max are 800/5200, i.e. current is effectively GHz.
    Use the MHz-scale bounds to detect that inconsistency; ordinary MHz values
    pass through unchanged.
    """
    if freq is None:return None
    current=float(freq.current)
    bounds=[float(value) for value in (getattr(freq,"min",0),getattr(freq,"max",0))
            if value is not None and float(value)>0]
    if current>0 and current<100 and any(value>=100 for value in bounds):current*=1000.0
    return current


def find_listening_pid(port):
    try:
        for row in psutil.net_connections(kind="inet"):
            if row.laddr and row.laddr.port==int(port) and row.status==psutil.CONN_LISTEN:return row.pid
    except (psutil.AccessDenied,psutil.Error):pass
    return None


_PROCESS_CACHE={}
def process_metrics(pid):
    if not pid:return None
    try:
        p=_PROCESS_CACHE.setdefault(int(pid),psutil.Process(int(pid)));mem=p.memory_info()
        return {"pid":int(pid),"cpu":p.cpu_percent(None),"rss_mb":mem.rss/1024**2,
                "threads":p.num_threads()}
    except (psutil.NoSuchProcess,psutil.AccessDenied,psutil.Error):
        _PROCESS_CACHE.pop(int(pid),None);return None


class RosReceiveMetrics:
    """Minimal callbacks plus sampling-side ROS interval aggregation."""
    def __init__(self,node,joint_topic=None,camera_topic=None,clock=time.monotonic):
        self.clock=clock;self.lock=threading.Lock();self.joint_events=deque();self.camera_times=deque()
        self.joint_last_recv=None;self.joint_last_header_ns=None
        self.recv_interval_max=0.0;self.header_interval_max=0.0;self.joint_age_max=0.0
        if node is None:return
        from rclpy.qos import DurabilityPolicy,HistoryPolicy,QoSProfile,ReliabilityPolicy
        joint_qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,
          reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        camera_qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,
          reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        from sensor_msgs.msg import Image,JointState
        self.sub_joint=node.create_subscription(JointState,joint_topic,self.on_joint,joint_qos)
        self.sub_camera=node.create_subscription(Image,camera_topic,self.on_camera,camera_qos)
    def on_joint(self,message):
        now=self.clock();stamp=getattr(getattr(message,"header",None),"stamp",None)
        header_ns=None if stamp is None else int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
        with self.lock:self.joint_events.append((now,header_ns))
    def on_camera(self,_):
        now=self.clock()
        with self.lock:self.camera_times.append(now)
    def snapshot(self,now):
        with self.lock:
            events=list(self.joint_events);self.joint_events.clear()
            while self.camera_times and self.camera_times[0]<now-1.0:self.camera_times.popleft()
            camera_count=len(self.camera_times)
        for recv,header_ns in events:
            if self.joint_last_recv is not None:
                self.recv_interval_max=max(self.recv_interval_max,max(0.0,recv-self.joint_last_recv))
            if header_ns is not None and self.joint_last_header_ns is not None:
                interval=(header_ns-self.joint_last_header_ns)/1e9
                if interval>=0:self.header_interval_max=max(self.header_interval_max,interval)
            self.joint_last_recv=recv
            if header_ns is not None:self.joint_last_header_ns=header_ns
        age=None if self.joint_last_recv is None else max(0.0,now-self.joint_last_recv)
        if age is not None:self.joint_age_max=max(self.joint_age_max,age)
        return {"joint_states_age_max_ms":None if self.joint_last_recv is None else self.joint_age_max*1000,
          "header_interval_max_ms":None if self.joint_last_recv is None else self.header_interval_max*1000,
          "recv_interval_max_ms":None if self.joint_last_recv is None else self.recv_interval_max*1000,
          "joint_states_interval_max_ms":None if self.joint_last_recv is None else self.recv_interval_max*1000,
          "camera_hz":float(camera_count)}


def start_ros_executor_thread(executor,errors=None):
    errors=[] if errors is None else errors
    def spin():
        try:executor.spin()
        except Exception as error:
            # Context shutdown is an expected wake-up path on ROS Humble.  Any
            # other executor failure is retained and printed, but must not kill
            # the resource-monitor process.
            if type(error).__name__!="ExternalShutdownException":
                detail=traceback.format_exc();errors.append(detail);print(detail,file=os.sys.stderr,flush=True)
    thread=threading.Thread(target=spin,name="system-monitor-ros-spin",daemon=True)
    thread.start();return thread


def safe_metric(errors,name,call,default=None):
    try:return call()
    except Exception as error:
        errors.append({"metric":name,"error":f"{type(error).__name__}: {error}"})
        return default


class GpuMetrics:
    def __init__(self,index=0):
        self.nv=None;self.handle=None;self.error=None
        try:
            import pynvml
            pynvml.nvmlInit();self.nv=pynvml;self.handle=pynvml.nvmlDeviceGetHandleByIndex(index)
        except Exception as error:self.error=f"{type(error).__name__}: {error}"
    def snapshot(self):
        if self.handle is None:return {key:None for key in ("gpu_util","vram_used_gb","gpu_temp_c",
          "gpu_power_w","gpu_clock_mhz","gpu_throttle")}
        n=self.nv;h=self.handle
        try:
            util=n.nvmlDeviceGetUtilizationRates(h);mem=n.nvmlDeviceGetMemoryInfo(h)
            return {"gpu_util":float(util.gpu),"vram_used_gb":mem.used/1024**3,
              "gpu_temp_c":float(n.nvmlDeviceGetTemperature(h,n.NVML_TEMPERATURE_GPU)),
              "gpu_power_w":n.nvmlDeviceGetPowerUsage(h)/1000.0,
              "gpu_clock_mhz":float(n.nvmlDeviceGetClockInfo(h,n.NVML_CLOCK_SM)),
              "gpu_throttle":int(n.nvmlDeviceGetCurrentClocksThrottleReasons(h))}
        except Exception:return {key:None for key in ("gpu_util","vram_used_gb","gpu_temp_c",
          "gpu_power_w","gpu_clock_mhz","gpu_throttle")}
    def close(self):
        if self.nv:
            try:self.nv.nvmlShutdown()
            except Exception:pass


def main():
    p=argparse.ArgumentParser();p.add_argument("--run-dir",type=Path,required=True)
    p.add_argument("--ctrl-pid",type=int,required=True);p.add_argument("--obs-pid",type=int)
    p.add_argument("--server-pid",type=int);p.add_argument("--server",default="http://127.0.0.1:8766")
    p.add_argument("--joint-topic",default="/dsr01/joint_states");p.add_argument("--camera-topic",required=True)
    p.add_argument("--start-monotonic",type=float,required=True);p.add_argument("--hz",type=float,default=2.0)
    a=p.parse_args();a.run_dir.mkdir(parents=True,exist_ok=True)
    stop=False
    def stopping(*_):
        nonlocal stop;stop=True
    server_pid=a.server_pid or find_listening_pid(urlparse(a.server).port or 8766)
    psutil.cpu_percent(None,percpu=True)
    for pid in (a.ctrl_pid,a.obs_pid,server_pid):
        if pid:
            try:psutil.Process(pid).cpu_percent(None)
            except psutil.Error:pass
    gpu=GpuMetrics();meta={"started_wall":time.time(),"server_pid":server_pid,
      "gpu_init_error":gpu.error,"sample_hz":a.hz}
    (a.run_dir/"system_monitor_status.json").write_text(json.dumps(meta,indent=2)+"\n")
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node("v2_system_monitor_readonly")
    # Keep SIGTERM as a flag-only request.  The main thread owns the ordered
    # executor/node/context shutdown sequence.
    signal.signal(signal.SIGTERM,stopping);signal.signal(signal.SIGINT,stopping)
    ros=RosReceiveMetrics(node,a.joint_topic,a.camera_topic)
    executor=rclpy.executors.SingleThreadedExecutor();executor.add_node(node)
    ros_spin_errors=[];ros_spin=start_ros_executor_thread(executor,ros_spin_errors)
    stream=(a.run_dir/"system_metrics.jsonl").open("a",buffering=1);period=1.0/a.hz;next_sample=time.monotonic()
    try:
        while not stop:
            now=time.monotonic()
            if now+1e-9<next_sample:
                time.sleep(min(.05,next_sample-now));continue
            errors=[]
            per_core=safe_metric(errors,"cpu_percent",lambda:psutil.cpu_percent(None,percpu=True),[])
            vm=safe_metric(errors,"virtual_memory",psutil.virtual_memory)
            swap=safe_metric(errors,"swap_memory",psutil.swap_memory)
            freq=safe_metric(errors,"cpu_freq",psutil.cpu_freq)
            row={"t_monotonic":now,"t_wall":time.time(),"t_s":now-a.start_monotonic,
              "cpu_total":float(sum(per_core)/len(per_core)) if per_core else None,
              "cpu_core_max":float(max(per_core)) if per_core else None,"cpu_per_core":per_core,
              "loadavg_1m":safe_metric(errors,"loadavg_1m",lambda:os.getloadavg()[0]),
              "cpu_freq_mhz":safe_metric(errors,"cpu_freq_mhz",lambda:cpu_frequency_mhz(freq)),
              "ram_used_gb":None if vm is None else vm.used/1024**3,
              "ram_avail_gb":None if vm is None else vm.available/1024**3,
              "swap_used_gb":None if swap is None else swap.used/1024**3,
              "proc_ctrl":safe_metric(errors,"proc_ctrl",lambda:process_metrics(a.ctrl_pid)),
              "proc_obs":safe_metric(errors,"proc_obs",lambda:process_metrics(a.obs_pid)),
              "proc_server":safe_metric(errors,"proc_server",lambda:process_metrics(server_pid))}
            row.update(safe_metric(errors,"gpu",gpu.snapshot,{}))
            row.update(safe_metric(errors,"ros",lambda:ros.snapshot(now),{}))
            if ros_spin_errors:errors.append({"metric":"ros_executor","error":ros_spin_errors[-1].splitlines()[-1]})
            if errors:row["sample_errors"]=errors
            try:stream.write(json.dumps(row,separators=(",", ":"))+"\n")
            except Exception as error:
                print(f"SYSTEM_MONITOR_SAMPLE_WRITE_SKIPPED {type(error).__name__}: {error}",
                      file=os.sys.stderr,flush=True)
            next_sample+=period
            if next_sample<now-period:next_sample=now+period
    finally:
        stream.close()
        try:executor.shutdown(timeout_sec=1.0)
        except Exception as error:print(f"SYSTEM_MONITOR_EXECUTOR_SHUTDOWN_WARNING:{error}",file=os.sys.stderr)
        ros_spin.join(timeout=1.0)
        if ros_spin.is_alive():
            # Fallback only: wake a stuck executor through context shutdown.
            if rclpy.ok():rclpy.shutdown()
            ros_spin.join(timeout=1.0)
        try:executor.remove_node(node)
        except Exception:pass
        try:node.destroy_node()
        except Exception as error:print(f"SYSTEM_MONITOR_NODE_DESTROY_WARNING:{error}",file=os.sys.stderr)
        if rclpy.ok():rclpy.shutdown()
        gpu.close()

if __name__=="__main__":main()
