import unittest
from jointstate_runtime_startup import JointStateStartup

class StartupTests(unittest.TestCase):
    def row(self,t,source=None,age=0,valid=True):
        return dict(receive=t,source=t if source is None else source,header_age=age,valid=valid)
    def test_cached_does_not_start(self):
        s=JointStateStartup(0);s.sample(self.row(1,age=3));self.assertIsNone(s.first_fresh)
    def test_timeout(self):
        s=JointStateStartup(0);s.update(60);self.assertEqual(s.fault,'JOINTSTATE_DISCOVERY_TIMEOUT')
    def test_warmup_gap_preserved_no_reset(self):
        s=JointStateStartup(0);s.sample(self.row(1));s.sample(self.row(4))
        self.assertEqual(s.first_fresh,1);self.assertEqual(s.phase,'WARMUP');self.assertEqual(len(s.events),1)
    def test_runtime_gap_fault(self):
        s=JointStateStartup(0);s.sample(self.row(1));s.sample(self.row(10.99));s.sample(self.row(11))
        self.assertEqual(s.phase,'RUNTIME_READY');s.sample(self.row(11.2));self.assertEqual(s.phase,'RUNTIME_FAULT')
    def test_runtime_invalid_fault(self):
        s=JointStateStartup(0);s.sample(self.row(1));s.sample(self.row(10.99));s.sample(self.row(11,valid=False))
        self.assertEqual(s.phase,'RUNTIME_FAULT')
    def test_silence_fault_without_callback(self):
        s=JointStateStartup(0);s.sample(self.row(1));s.sample(self.row(10.99));s.update(11.2)
        self.assertEqual(s.phase,'RUNTIME_FAULT')
