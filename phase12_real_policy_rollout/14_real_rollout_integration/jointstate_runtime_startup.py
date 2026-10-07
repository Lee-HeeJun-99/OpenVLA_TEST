"""Persistent subscriber readiness; startup never relaxes runtime limits."""
class JointStateStartup:
    def __init__(self, started):
        self.started=started; self.first_fresh=None; self.last=None
        self.phase='WAIT_DISCOVERY'; self.fault=None; self.events=[]

    def update(self, now):
        if self.fault:return
        if self.first_fresh is None:
            if now-self.started>=60:self.fail('JOINTSTATE_DISCOVERY_TIMEOUT')
        elif now-self.first_fresh>=10:
            self.phase='RUNTIME_READY'
            if self.last is None or now-self.last['receive']>=.1:
                self.fail('joint_state_receive_timeout')

    def fail(self, reason):
        self.fault=reason; self.phase='RUNTIME_FAULT'

    def sample(self, row):
        now=row['receive']; self.update(now)
        previous=self.last
        sg=row['source']-previous['source'] if previous else None
        rg=now-previous['receive'] if previous else None
        row.update(source_gap=sg,receive_gap=rg)
        regression=sg is not None and sg<0
        if previous and (sg<=0 or sg>=.1 or rg<=0 or rg>=.1):
            self.events.append(dict(phase=self.phase,before=previous['source'],after=row['source'],
                receive_before=previous['receive'],receive_after=now,source_gap=sg,receive_gap=rg,
                header_age=row['header_age']))
        if self.phase=='RUNTIME_READY' and (not row['valid'] or
                (sg is not None and not 0<sg<.1) or (rg is not None and not 0<rg<.1)):
            self.fail('joint_state_runtime_invalid_or_gap')
        if self.first_fresh is None and not self.fault:
            self.phase='WAIT_FIRST_FRESH_SAMPLE'
            if row['valid'] and not regression and 0<=row['header_age']<.1:
                self.first_fresh=now; self.phase='WARMUP'
        row['phase']=self.phase; self.last=row

    def status(self, now):
        self.update(now)
        return dict(phase=self.phase,fault=self.fault,first_fresh_sample=self.first_fresh,
            discovery_delay_s=self.first_fresh-self.started if self.first_fresh is not None else None,
            latest_receive_age_s=now-self.last['receive'] if self.last else None,
            source_gap=self.last.get('source_gap') if self.last else None,
            receive_gap=self.last.get('receive_gap') if self.last else None)
