import sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rollout_batch_summary import summarize_trials,performance_groups,RolloutBatch
from test_rollout_trial_classifier import READY

def row(i,status):return dict(trial_id=i,status=status,runtime_valid=status!='INVALID',task_success=None if status=='INVALID' else status=='SUCCESS',invalid_category='INVALID_JOINTSTATE_GAP' if status=='INVALID' else None)
class BatchTests(unittest.TestCase):
    def test_example_30(self):
        rows=[row(i,'SUCCESS' if i<17 else 'FAILURE' if i<24 else 'INVALID') for i in range(30)]
        s=summarize_trials(rows);self.assertEqual(s['valid_trials'],24);self.assertAlmostEqual(s['task_success_rate'],17/24);self.assertEqual(s['invalid_rate'],.2)
    def test_zero_denominators(self):
        self.assertIsNone(summarize_trials([])['task_success_rate']);self.assertIsNone(summarize_trials([row(1,'INVALID')])['task_success_rate'])
    def test_duplicate_and_contradiction_rejected(self):
        with self.assertRaises(ValueError):summarize_trials([row(1,'SUCCESS'),row(1,'FAILURE')])
        with self.assertRaises(ValueError):summarize_trials([{**row(1,'INVALID'),'task_success':False}])
    def test_valid_groups(self):
        groups=performance_groups([row(1,'SUCCESS'),row(2,'FAILURE'),row(3,'INVALID')]);self.assertEqual([len(v) for v in groups.values()],[1,1])
    def test_retry_new_ready(self):
        generations=[]
        def prepare(i):generations.append(i);return {**READY,'generation':i}
        def execute(t):
            if t.trial_id==1:t.invalidate('JOINTSTATE_RECEIVE_GAP')
            return True
        with tempfile.TemporaryDirectory() as d:
            s=RolloutBatch(2,3).run(prepare,execute,Path(d)/'batch');self.assertEqual(s['total_attempts'],3);self.assertEqual(s['valid_trials'],2);self.assertEqual(generations,[1,2,3])
    def test_old_ready_and_physical_batch_blocked(self):
        with tempfile.TemporaryDirectory() as d:
            s=RolloutBatch(1,1).run(lambda _: {**READY,'generation':0},lambda _: True,Path(d)/'a')
            self.assertEqual(s['invalid_trials'],0);self.assertEqual(s['total_attempts'],0)
            self.assertEqual(len(s['pretrial_blocks']),1)
            with self.assertRaises(PermissionError):RolloutBatch(1,1,physical=True).run(lambda _:READY,lambda _:True,Path(d)/'b')

if __name__=='__main__':unittest.main()
