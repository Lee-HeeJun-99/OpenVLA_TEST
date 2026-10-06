"""Serialize all command/watchdog/model writes and capture health atomically."""
import threading

class ConcurrentLogger:
    def __init__(self,logger,fail_after=None):
        self.logger=logger;self.lock=threading.RLock();self.healthy=True;self.fail_after=fail_after;self.writes=0
    def append(self,row):
        with self.lock:
            if not self.healthy:raise RuntimeError('logger_unhealthy')
            try:
                if self.fail_after is not None and self.writes>=self.fail_after:raise OSError('FAKE_TEST_WRITE_FAILURE')
                self.logger.append(row);self.writes+=1
            except Exception:
                self.healthy=False;raise
