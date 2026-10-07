import json,sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rollout_trial_classifier import RolloutTrial,invalid_category

READY=dict(phase='RUNTIME_READY',fault=None,first_fresh_after_rearm=True,warmup_seconds=10)
def trial():
    t=RolloutTrial(1);t.start(READY);return t
def joint(**kw):
    return dict(names=[f'joint_{i}' for i in range(1,7)],position=[0]*6,velocity=[0]*6,source=1.,sample_id=1,latest_age=.01,source_gap=.01,receive_gap=.01,**kw)

class ClassifierTests(unittest.TestCase):
    def test_joint_boundary_invalid(self):
        for key,value in [('source_gap',.1),('receive_gap',.1),('latest_age',.5),('source_gap',0),('source_gap',-.001)]:
            t=trial();j=joint();j[key]=value;t.observe({'jointstate':j})
            self.assertEqual(t.status,'INVALID');self.assertIsNone(t.task_success);self.assertFalse(t.command_allowed())
    def test_invalid_values_missing(self):
        for field,value in [('position',[float('nan')]*6),('velocity',[float('inf')]*6),('names',['joint_1'])]:
            t=trial();j=joint();j[field]=value;t.observe({'jointstate':j});self.assertEqual(t.status,'INVALID')
    def test_camera_invalid(self):
        baseline=dict(selected_age=.01,encoding='bgra8',resolution_valid=True,stream_alive=True)
        for key,value in [('selected_age',.5),('fresh_frame_timeout',True),('encoding','invalid'),('resolution_valid',False),('stream_alive',False)]:
            t=trial();c={**baseline,key:value};t.observe({'camera':c});self.assertEqual(t.summary()['invalid_category'],'INVALID_CAMERA_STALE')
    def test_model_input_invalid(self):
        t=trial();t.invalidate('MODEL_INPUT_CONSTRUCTION_FAILURE');self.assertEqual(t.summary()['invalid_category'],'INVALID_MODEL_INPUT')
    def test_outcome_valid_only(self):
        for outcome,status in [(True,'SUCCESS'),(False,'FAILURE')]:
            t=trial();self.assertEqual(t.finish(outcome)['status'],status);self.assertTrue(t.runtime_valid)
        with self.assertRaises(ValueError):trial().finish(None)
    def test_terminal_once_no_reset(self):
        t=trial();t.invalidate('JOINTSTATE_RECEIVE_GAP');before=t.summary();t.finish(True);t.invalidate('logger_failure')
        self.assertEqual(t.summary(),before)
        with self.assertRaises(RuntimeError):t.start(READY)
        with self.assertRaises(PermissionError):t.command_dispatched()
    def test_provenance_and_saved_nonfinite(self):
        t=trial();t.observe({'phase':'approach','marker':'lastvalid'});t.prediction({'action':[float('nan')]});t.command_dispatched();t.invalidate('canonical_action_nonfinite')
        self.assertEqual(t.invalid_event['command_count_before_invalid'],1)
        with tempfile.TemporaryDirectory() as d:
            t.save(d);r=json.loads((Path(d)/'invalid_event.json').read_text());self.assertEqual(r['last_valid_observation']['marker'],'lastvalid')
            self.assertEqual(r['last_prediction']['action'][0],{'nonfinite':'nan'})
    def test_logger_commit_failure_invalid_not_success(self):
        def broken(_):raise OSError('disk full')
        t=trial();t.finish(True,persist=broken);self.assertEqual(t.status,'INVALID');self.assertIsNone(t.task_success)
    def test_new_timestamp_nonincreasing(self):
        t=trial();t.observe({'jointstate':joint()});j=joint();j['sample_id']=2;t.observe({'jointstate':j});self.assertEqual(t.status,'INVALID')
    def test_cached_poll_not_new_sample(self):
        t=trial();self.assertTrue(t.observe({'jointstate':joint()}));self.assertTrue(t.observe({'jointstate':joint()}))
    def test_startup_not_trial(self):
        t=RolloutTrial(1)
        with self.assertRaises(PermissionError):t.start({**READY,'phase':'WARMUP'})
        self.assertIsNone(t.status)

if __name__=='__main__':unittest.main()
