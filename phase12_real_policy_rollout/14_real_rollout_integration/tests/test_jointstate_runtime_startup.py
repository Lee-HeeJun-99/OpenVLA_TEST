import unittest
from jointstate_runtime_startup import JointStateStartup

class StartupTests(unittest.TestCase):
    def row(self,t,source=None,age=0,valid=True):
        return dict(receive=t,source=t if source is None else source,header_age=age,valid=valid)
    def test_cached_does_not_start(self):
        s=JointStateStartup(0);s.sample(self.row(1,age=3));self.assertIsNone(s.first_fresh)
    def test_timeout(self):
        s=JointStateStartup(0);s.update(60);self.assertIsNone(s.fault)
        s.update(120);self.assertEqual(s.fault,'JOINTSTATE_READINESS_TIMEOUT')
        self.assertEqual(s.phase,'READINESS_TIMEOUT')
    def test_warmup_gap_preserved_clean_reset(self):
        s=JointStateStartup(0);s.sample(self.row(1));s.sample(self.row(4))
        self.assertEqual(s.first_fresh,1);self.assertEqual(s.phase,'WARMUP');self.assertEqual(len(s.events),1)
        self.assertEqual(s.clean_since,4);self.assertIsNone(s.fault)
        self.assertEqual(s.events[0]['event'],'STARTUP_WARMUP_EVENT')
    def clean(self,s,start):
        for i in range(1002):s.sample(self.row(start+i*.01))
        self.assertEqual(s.phase,'RUNTIME_READY')
    def test_runtime_gap_fault(self):
        s=JointStateStartup(0);self.clean(s,1)
        s.sample(self.row(11.2));self.assertEqual(s.phase,'RUNTIME_FAULT')
    def test_runtime_invalid_fault(self):
        s=JointStateStartup(0);self.clean(s,1);s.sample(self.row(11.02,valid=False))
        self.assertEqual(s.phase,'RUNTIME_FAULT')
    def test_silence_fault_without_callback(self):
        s=JointStateStartup(0);self.clean(s,1);s.update(11.2)
        self.assertEqual(s.phase,'RUNTIME_FAULT')
    def test_warmup_stale_poll_not_terminal(self):
        s=JointStateStartup(0);s.sample(self.row(1));s.update(4)
        self.assertIsNone(s.fault);self.assertIsNone(s.clean_since)
        self.assertEqual(s.phase,'WARMUP')
    def test_gap_recovery_requires_new_ten_seconds(self):
        s=JointStateStartup(0);s.sample(self.row(1));s.sample(self.row(4))
        for i in range(1,701):s.sample(self.row(4+i*.01))
        self.assertEqual(s.phase,'WARMUP')
        for i in range(701,1002):s.sample(self.row(4+i*.01))
        self.assertEqual(s.phase,'RUNTIME_READY');self.assertGreaterEqual(s.ready_at,14)
    def test_perpetual_gaps_timeout(self):
        s=JointStateStartup(0)
        for i in range(1,120,3):s.sample(self.row(i))
        s.update(120);self.assertEqual(s.fault,'JOINTSTATE_READINESS_TIMEOUT')
    def test_runtime_fault_never_recovers(self):
        s=JointStateStartup(0);self.clean(s,1);s.sample(self.row(14))
        for i in range(2000):s.sample(self.row(14.01+i*.01))
        self.assertEqual(s.phase,'RUNTIME_FAULT')
    def test_startup_events_excluded_from_task_invalid_count(self):
        import sys
        from pathlib import Path
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'15_valid_invalid_rollout_policy'))
        from rollout_trial_classifier import RolloutTrial
        s=JointStateStartup(0);s.sample(self.row(1));s.sample(self.row(4));self.clean(s,4.01)
        trial=RolloutTrial(1);trial.start({**s.status(14.02),'first_fresh_after_rearm':True,'warmup_seconds':s.status(14.02)['clean_window_s']})
        self.assertEqual(trial.summary()['jointstate_gap_count'],0)
        s.sample(self.row(14.2));trial.invalidate(s.fault)
        self.assertEqual(trial.summary()['status'],'INVALID');self.assertIsNone(trial.summary()['task_success'])
        self.assertFalse(trial.command_allowed())
