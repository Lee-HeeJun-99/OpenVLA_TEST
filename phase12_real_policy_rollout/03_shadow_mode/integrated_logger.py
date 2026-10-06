"""Crash-evident JSONL logger with atomic session metadata; no ROS dependency."""
import json,os,shutil,time
from pathlib import Path
class LoggerFailure(RuntimeError):pass
class FsyncJsonlLogger:
    def __init__(self,path,minimum_free_bytes=100*1024*1024,disk_usage_fn=shutil.disk_usage):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True);self._f=None;self.sequence=0
        self.minimum_free_bytes=int(minimum_free_bytes);self.disk_usage_fn=disk_usage_fn
        self.partial_path=self.path.with_suffix(self.path.suffix+'.partial')
        self.status_path=self.path.with_suffix(self.path.suffix+'.status.json')
    def _check_disk(self):
        if self.disk_usage_fn(self.path.parent).free < self.minimum_free_bytes:raise LoggerFailure('disk_free_below_threshold')
    @staticmethod
    def _atomic_json(path,payload):
        temporary=path.with_suffix(path.suffix+'.tmp')
        with temporary.open('w',encoding='utf-8') as f:
            json.dump(payload,f,separators=(',',':'));f.flush();os.fsync(f.fileno())
        os.replace(temporary,path)
        directory=os.open(path.parent,os.O_RDONLY)
        try:os.fsync(directory)
        finally:os.close(directory)
    def __enter__(self):
        self._check_disk();self._atomic_json(self.partial_path,{'state':'PARTIAL','opened_wall_time':time.time()})
        self._f=self.path.open('a',encoding='utf-8');return self
    def append(self,record):
        if self._f is None:raise LoggerFailure('logger_not_open')
        try:
            self._check_disk();payload=dict(record);payload.setdefault('sequence_number',self.sequence);payload.setdefault('logger_wall_time',time.time())
            self._f.write(json.dumps(payload,separators=(',',':'))+'\n');self._f.flush();os.fsync(self._f.fileno());self.sequence+=1
        except Exception as e:raise LoggerFailure(str(e)) from e
    def __exit__(self,exc_type,*_):
        if self._f:self._f.flush();os.fsync(self._f.fileno());self._f.close();self._f=None
        state='COMPLETE' if exc_type is None else 'PARTIAL'
        self._atomic_json(self.status_path,{'state':state,'records':self.sequence,'closed_wall_time':time.time()})
        if exc_type is None and self.partial_path.exists():self.partial_path.unlink()
