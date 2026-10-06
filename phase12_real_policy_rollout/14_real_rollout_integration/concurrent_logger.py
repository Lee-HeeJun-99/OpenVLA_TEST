"""Serialize all command/watchdog/model writes and capture health atomically."""
import threading

class ConcurrentLogger:
    def __init__(self,logger):self.logger=logger;self.lock=threading.RLock();self.healthy=True
    def append(self,row):
        with self.lock:
            if not self.healthy:raise RuntimeError('logger_unhealthy')
            try:self.logger.append(row)
            except Exception:
                self.healthy=False;raise
