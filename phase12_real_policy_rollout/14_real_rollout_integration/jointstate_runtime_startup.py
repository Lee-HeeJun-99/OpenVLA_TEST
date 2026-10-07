"""Recoverable pre-trial readiness; runtime faults remain terminal."""
import math

class JointStateStartup:
    READINESS_TIMEOUT = 120.
    CLEAN_SECONDS = 10.

    def __init__(self, started):
        self.started=started; self.first_fresh=None; self.last=None
        self.phase='WAIT_DISCOVERY'; self.fault=None; self.events=[]
        self.clean_since=None; self.ready_at=None

    def fail(self, reason):
        self.fault=reason; self.phase='RUNTIME_FAULT'

    def readiness_timeout(self):
        self.fault='JOINTSTATE_READINESS_TIMEOUT';self.phase='READINESS_TIMEOUT'

    def reset_clean(self, now, reason, **detail):
        self.events.append(dict(event='STARTUP_WARMUP_EVENT',phase=self.phase,
                                timestamp=now,reason=reason,clean_timer_reset=True,**detail))
        self.clean_since=None

    def update(self, now):
        if self.fault:return
        age=now-self.last['receive'] if self.last else None
        if self.phase=='RUNTIME_READY':
            if age is None or age>=.1:self.fail('joint_state_receive_timeout')
            return
        if now-self.started>=self.READINESS_TIMEOUT:
            self.readiness_timeout();return
        if self.clean_since is not None:
            if age is None or age<0 or age>=.1:
                self.reset_clean(now,'PRE_TRIAL_RECEIVE_STALE',latest_receive_age_s=age)
            elif now-self.clean_since>=self.CLEAN_SECONDS:
                self.phase='RUNTIME_READY';self.ready_at=now

    def sample(self, row):
        now=row['receive'];previous=self.last
        sg=row['source']-previous['source'] if previous else None
        rg=now-previous['receive'] if previous else None
        row.update(source_gap=sg,receive_gap=rg)
        valid=row['valid'] and all(math.isfinite(row[k]) for k in ('receive','source','header_age'))
        bad_gap=previous is not None and (not 0<sg<.1 or not 0<rg<.1)
        bad_header=not 0<=row['header_age']<.5
        if self.phase=='RUNTIME_READY':
            if bad_gap or not valid or bad_header:
                self.events.append(dict(event='RUNTIME_EVENT',phase=self.phase,before=previous['source'] if previous else None,
                    after=row['source'],receive_before=previous['receive'] if previous else None,receive_after=now,
                    source_gap=sg,receive_gap=rg,header_age=row['header_age']))
                self.fail('joint_state_runtime_invalid_or_gap')
        elif not self.fault:
            if now-self.started>=self.READINESS_TIMEOUT:self.readiness_timeout()
            else:
                if self.first_fresh is None:self.phase='WAIT_FIRST_FRESH_SAMPLE'
                if bad_gap or not valid or bad_header:
                    self.reset_clean(now,'PRE_TRIAL_INVALID_OR_GAP',before=previous['source'] if previous else None,
                        after=row['source'],receive_before=previous['receive'] if previous else None,receive_after=now,
                        source_gap=sg,receive_gap=rg,header_age=row['header_age'])
                fresh=valid and 0<=row['header_age']<.1 and (sg is None or sg>0) and (rg is None or rg>0)
                if fresh:
                    if self.first_fresh is None:self.first_fresh=now
                    self.phase='WARMUP'
                    if self.clean_since is None:self.clean_since=now
        row['phase']=self.phase;self.last=row
        self.update(now)
        row['phase']=self.phase

    def status(self, now):
        self.update(now)
        return dict(phase=self.phase,fault=self.fault,first_fresh_sample=self.first_fresh,
            discovery_delay_s=self.first_fresh-self.started if self.first_fresh is not None else None,
            latest_receive_age_s=now-self.last['receive'] if self.last else None,
            source_gap=self.last.get('source_gap') if self.last else None,
            receive_gap=self.last.get('receive_gap') if self.last else None,
            clean_since=self.clean_since,clean_window_s=now-self.clean_since if self.clean_since is not None else 0,
            ready_at=self.ready_at,readiness_timeout_s=self.READINESS_TIMEOUT)
