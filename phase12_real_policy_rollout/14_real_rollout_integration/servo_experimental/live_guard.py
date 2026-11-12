"""ROS-free, serialized inhibit/publish/stop boundary for the experimental stream."""
import threading
from servo_experimental.stop_evidence import evaluate_stop


class StreamGuard:
    def __init__(self, stop):
        self.lock = threading.RLock()
        self.stop = stop
        self.reason = None
        self.closed = False
        self.attempts = 0
        self.submitted = 0
        self.stop_requested = False
        self.stop_confirmation = None

    def check(self):
        if self.reason:
            raise RuntimeError(self.reason)
        if self.closed:
            raise RuntimeError('stream_closed')

    def publish(self, send):
        with self.lock:
            self.check()
            # Count BEFORE submission: exceptions can leave delivery unknown.
            self.attempts += 1
            send()
            self.submitted += 1

    def finish(self, reason=None):
        with self.lock:
            if reason and self.reason is None:
                self.reason = str(reason)
            self.closed = True
            if self.attempts and not self.stop_requested:
                self.stop_requested = True
                self.stop()

    def confirm_stop(self, **evidence):
        with self.lock:
            if not self.closed or not self.stop_requested:
                raise RuntimeError('stop_confirmation_without_stop')
            self.stop_confirmation = evaluate_stop(**evidence)
            if not self.stop_confirmation['confirmed'] and self.reason is None:
                self.reason = 'stop_confirmation_failed:'+self.stop_confirmation['reason']
            # Closed stays latched even after a verified stop.
            return self.stop_confirmation


def require_stream_liveness(now, last_publish, *, active):
    if active and last_publish is not None and not 0 <= now-last_publish < .1:
        raise RuntimeError('servo_stream_timeout')
