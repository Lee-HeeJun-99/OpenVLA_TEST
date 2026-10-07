import ast, tempfile, unittest, json
from pathlib import Path
import final_end_to_end_readiness as audit

class LongAuditTests(unittest.TestCase):
    def test_no_physical_capability(self):
        tree=ast.parse(Path(audit.__file__).read_text())
        calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertFalse(calls & {'create_client','create_publisher','call_async','send_goal_async'})
    def test_single_persistent_sensor_pair(self):
        text=Path(audit.__file__).read_text()
        self.assertEqual(text.count("'/dsr01/joint_states'"),1)
        self.assertEqual(text.count("'/zed/zed_node/rgb/color/rect/image'"),1)
        self.assertNotIn('Popen',text)
    def test_runtime_gap_not_hidden(self):
        rows=[dict(receive=i/100,source_gap=.01,receive_gap=.01,valid=True) for i in range(3000)]
        state={'fault':None,'latest_receive_age_s':.01}
        self.assertEqual(audit.window_result(rows,0,30,state)['status'],'PASS')
        rows[100]['source_gap']=3.07
        self.assertEqual(audit.window_result(rows,0,30,state)['status'],'FAIL')
    def test_regression_duplicate_reject(self):
        rows=[dict(receive=i/100,source_gap=.01,receive_gap=.01,valid=True) for i in range(3000)]
        rows[10]['source_gap']=0
        self.assertEqual(audit.window_result(rows,0,30,{'fault':None,'latest_receive_age_s':.01})['duplicate'],1)
    def test_training_dedup_and_hz(self):
        with tempfile.TemporaryDirectory() as d:
            for k in ('a','b'):
                p=Path(d)/k;p.mkdir()
                (p/'episode.json').write_text(json.dumps({'record_frequency_hz':10}))
                (p/'steps.jsonl').write_text('\n'.join(json.dumps({'action':[.001,0,0,0,0,0,g]}) for g in (0,1)))
            result=audit.training_audit(Path(d))
            self.assertEqual(result['unique_episodes'],1)
            self.assertEqual(result['episodes'][0]['first_close_seconds'],.1)
    def test_empty_window_cannot_pass(self):
        self.assertEqual(audit.window_result([],0,180,{'fault':None,'latest_receive_age_s':.01})['status'],'FAIL')
    def test_camera_input_failure_not_model_failure(self):
        result=audit.failure_counts([{'valid':False,'error':'no_frame_within_10ms'}, {'valid':True,'camera_age':.49}])
        self.assertEqual(result['camera_failure'],1)
        self.assertEqual(result['model_failures'],0)
        self.assertEqual(result['selected_age_failures'],0)

if __name__=='__main__':unittest.main()
