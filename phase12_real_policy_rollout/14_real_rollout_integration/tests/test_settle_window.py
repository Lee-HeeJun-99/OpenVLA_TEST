import unittest
from settle_window import ExactZeroSettleWindow, wait_for_settle


def sample(t, v=0):
    return dict(receive=t,source=100+t,velocity=[v,0,0,0,0,0],valid=True)


class SettleTests(unittest.TestCase):
    def test_one_zero_and_repeated_cached_sample_do_not_pass(self):
        w=ExactZeroSettleWindow()
        self.assertFalse(w.sample(sample(0)))
        for _ in range(100):self.assertFalse(w.sample(sample(0)))
        self.assertEqual(w.clean_count,1)

    def test_consecutive_clean_samples_pass(self):
        w=ExactZeroSettleWindow()
        for i in range(20):self.assertFalse(w.sample(sample(i*.01)))
        self.assertTrue(w.sample(sample(.2)))

    def test_tiny_nonzero_resets_not_terminal(self):
        w=ExactZeroSettleWindow()
        for i in range(20):w.sample(sample(i*.01))
        self.assertFalse(w.sample(sample(.2,2.8649228624999523e-5)))
        for i in range(21,41):self.assertFalse(w.sample(sample(i*.01)))
        self.assertTrue(w.sample(sample(.41)))
        self.assertEqual(w.reset_count,1)

    def test_invalid_gap_and_regression_do_not_count_as_settle_retry(self):
        for row in (sample(.1),sample(-.01),dict(sample(.01),valid=False),sample(.01,float('nan'))):
            w=ExactZeroSettleWindow();w.sample(sample(0))
            with self.assertRaises(RuntimeError):w.sample(row)

    def test_whole_batch_later_nonzero_invalidates_earlier_window(self):
        w=ExactZeroSettleWindow()
        batch=[sample(i*.01) for i in range(21)]+[sample(.21,1e-10)]
        ready=False
        for row in batch:ready=w.sample(row)
        self.assertFalse(ready)
        self.assertIsNone(w.clean_since)

    def simulate(self,velocity,check=lambda:None):
        now=[0.];cursor=[0]
        def drain():
            rows=[]
            while cursor[0]*.01<=now[0]+1e-10:
                t=cursor[0]*.01;rows.append(sample(t,velocity(t)));cursor[0]+=1
            return rows
        def sleep(dt):now[0]+=dt
        return wait_for_settle(drain,check,clock=lambda:now[0],sleep=sleep)

    def test_bounded_wait_recovers_after_isolated_feedback_nonzero(self):
        row,evidence=self.simulate(lambda t:1e-10 if .1<=t<.11 else 0)
        self.assertGreaterEqual(row['receive'],.31)
        self.assertEqual(evidence['reset_count'],1)

    def test_sustained_nonzero_times_out(self):
        with self.assertRaisesRegex(TimeoutError,'settle_timeout'):self.simulate(lambda t:1e-10)

    def test_runtime_fault_is_immediate(self):
        def bad():raise RuntimeError('joint_state_runtime_invalid_or_gap')
        with self.assertRaisesRegex(RuntimeError,'runtime_invalid'):self.simulate(lambda t:0,bad)

    def test_no_new_samples_cannot_pass(self):
        now=[0.]
        def sleep(dt):now[0]+=dt
        with self.assertRaises(TimeoutError):wait_for_settle(lambda:[],lambda:None,clock=lambda:now[0],sleep=sleep)

if __name__=='__main__':unittest.main()
