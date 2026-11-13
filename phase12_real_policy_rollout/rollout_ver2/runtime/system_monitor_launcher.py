"""Lifecycle helper for the external system monitor process."""
from __future__ import annotations
import atexit,json,subprocess,time
from pathlib import Path


class SystemMonitorProcess:
    def __init__(self,root,run_dir,ctrl_pid,obs_pid,server,camera_topic,start_monotonic,
                 server_pid=None,popen=subprocess.Popen):
        self.run_dir=Path(run_dir);self.warning=None
        command=["/usr/bin/python3",str(Path(root)/"tools/system_monitor.py"),"--run-dir",str(run_dir),
          "--ctrl-pid",str(ctrl_pid),"--server",str(server),
          "--start-monotonic",str(start_monotonic)]
        if obs_pid is not None:command += ["--obs-pid",str(obs_pid)]
        if camera_topic is not None:command += ["--camera-topic",str(camera_topic)]
        if server_pid is not None:command += ["--server-pid",str(server_pid)]
        self.log=(self.run_dir/"system_monitor.log").open("a")
        try:self.process=popen(command,stdout=self.log,stderr=self.log)
        except Exception as error:self.process=None;self.warning=f"SYSTEM_MONITOR_START_FAILED:{error}";self.log.close()
        atexit.register(self.stop)
    def _log_tail(self,count=5):
        try:
            if not self.log.closed:self.log.flush()
            lines=[line for line in (self.run_dir/"system_monitor.log").read_text(
              encoding="utf-8",errors="replace").splitlines() if line.strip()]
            return lines[-int(count):]
        except Exception:return []
    def status(self):
        if self.process is None:return {"running":False,"warning":self.warning}
        code=self.process.poll()
        tail=[] if code is None or code==0 else self._log_tail(5)
        warning=None if code is None or code==0 else f"SYSTEM_MONITOR_EXITED:{code}"+(
          "\n"+"\n".join(tail) if tail else "")
        return {"running":code is None,"returncode":code,"warning":warning,
                "traceback_tail":tail}
    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=1)
        status=self.status()
        if not self.log.closed:self.log.close()
        return status
