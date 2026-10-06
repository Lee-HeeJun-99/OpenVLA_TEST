import json,sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'runtime'),str(ROOT/'readiness')]
from rollout_state_machine import RolloutStateMachine
from rollout_runner import run
from pre_rollout_check import check
import yaml

class ReadinessTests(unittest.TestCase):
    def test_state_requires_approval(self):
        state=RolloutStateMachine();self.assertEqual(state.state,'COMMAND_DISABLED')
        state.advance()
        with self.assertRaises(RuntimeError):state.advance(checks_passed=True)
        state.abort('manual_abort')
        with self.assertRaises(RuntimeError):state.advance(hardware_approval=True)

    def test_unknown_evidence_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            safety=yaml.safe_load((ROOT.parent/'01_configs/safety_limits.yaml').read_text())
            result=check({},safety,directory)
            self.assertFalse(result['MOTION_READY']);self.assertFalse(result['MODEL_READY'])

    def replay(self,model,faults=None,vector=None):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'input.jsonl';output=Path(directory)/'out.jsonl'
            rows=[]
            for f in range(5):
                r=dict(frame_id=f,episode_id='fixture',valid=True,phase='alignment',raw_ee_position=[.6,0,.5] if (faults or {}).get('workspace_violation') else [.4,0,.5],instruction='Pick up the orange cube.')
                v=vector or [0,0,0,0,0,0,0]
                if model=='openvla':r['openvla_denormalized_action']=v
                elif f==0:r['oft_denormalized_action_chunk']=[v]*5
                rows.append(r)
            path.write_text('\n'.join(json.dumps(r) for r in rows))
            result=run(model,path,output,initial_open_acknowledged=True,faults=faults)
            records=[json.loads(r) for r in output.read_text().splitlines()]
            self.assertTrue(all(not r['command_issued'] and not r['command_acknowledged'] for r in records))
            return result,records

    def test_mock_both_models(self):
        for model in ('openvla','oft'):
            result,rows=self.replay(model)
            self.assertEqual(result['records'],5)
            self.assertEqual([r['target_step'] for r in rows],list(range(5)))

    def test_failures_abort(self):
        for fault in ('camera_stale','joint_state_stale','tcp_stale','model_timeout','logger_failure','oft_underrun','manual_abort','workspace_violation'):
            with self.subTest(fault=fault):
                result,_=self.replay('oft',{fault:True});self.assertTrue(result['aborted'])
        for vector in ([float('nan'),0,0,0,0,0,0],[.1,0,0,0,0,0,0],[0,0,0,1,0,0,0],[0,0,0,0,0,0,1]):
            result,_=self.replay('openvla',vector=vector);self.assertTrue(result['aborted'])

if __name__=='__main__':unittest.main()
