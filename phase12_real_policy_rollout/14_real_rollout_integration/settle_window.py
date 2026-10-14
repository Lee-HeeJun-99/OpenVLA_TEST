"""Command-free exact-zero settle confirmation over distinct fresh samples."""
import math


# One existing 5 Hz control interval. A stricter temporal confirmation, not
# a new velocity tolerance or a relaxation of runtime gap/staleness gates.
SETTLE_CLEAN_SECONDS = .2
SETTLE_TIMEOUT_SECONDS = 2.


class ExactZeroSettleWindow:
    def __init__(self):
        self.last = None
        self.clean_since = None
        self.clean_count = 0
        self.reset_count = 0

    def sample(self, row):
        now, source = row['receive'], row['source']
        velocity = row['velocity']
        if (row.get('valid') is not True or len(velocity) != 6
                or not all(math.isfinite(v) for v in [now, source, *velocity])):
            raise RuntimeError('settle_invalid_jointstate')
        if self.last is not None:
            # Polling the identical sample must not grow the window.
            if now == self.last['receive'] and source == self.last['source']:
                return False
            if not (0 < now-self.last['receive'] < .1
                    and 0 < source-self.last['source'] < .1):
                raise RuntimeError('settle_jointstate_gap_or_regression')
        self.last = row
        if any(v != 0 for v in velocity):
            self.clean_since = None
            self.clean_count = 0
            self.reset_count += 1
            return False
        if self.clean_since is None:
            self.clean_since = now
        self.clean_count += 1
        return now-self.clean_since >= SETTLE_CLEAN_SECONDS-1e-12

    def evidence(self):
        return dict(criterion='SIX_FINITE_EXACT_ZERO_VELOCITIES',
                    required_clean_seconds=SETTLE_CLEAN_SECONDS,
                    timeout_seconds=SETTLE_TIMEOUT_SECONDS,
                    clean_sample_count=self.clean_count,reset_count=self.reset_count,
                    clean_since=self.clean_since,
                    last_receive=None if self.last is None else self.last['receive'])


def wait_for_settle(drain_samples, check_runtime, *, clock, sleep):
    """Caller supplies all newly received samples; no cached-window reuse."""
    window = ExactZeroSettleWindow()
    deadline = clock()+SETTLE_TIMEOUT_SECONDS
    while True:
        check_runtime()
        if clock() >= deadline:
            raise TimeoutError('settle_timeout')
        ready = False
        # Process the whole batch, including a later nonzero sample, before
        # deciding. A zero window earlier in the same batch is not sufficient.
        for row in drain_samples():
            ready = window.sample(row)
        check_runtime()
        if ready:
            if not 0 <= clock()-window.last['receive'] < .1:
                raise RuntimeError('settle_jointstate_stale')
            return dict(window.last),window.evidence()
        sleep(.005)
