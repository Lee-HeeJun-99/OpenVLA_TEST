import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from verify_live_model_server import validate_identity
from robot_state_monitor import RobotStateAdapter
from types import SimpleNamespace
from oft_action_scheduler import ActionScheduler

class FinalContracts(unittest.TestCase):
    def health(self,model):
        from verify_live_model_server import ROOT
        import yaml
        c=yaml.safe_load((ROOT/'configs'/f'real_{model}.yaml').read_text())
        import json
        processor=json.loads((ROOT.parents[2]/('models/vanilla_s1_balanced_step8130/processor/preprocessor_config.json' if model=='openvla' else 'runtime_state/oft_mixed480_step28560_merged/preprocessor_config.json')).read_text())
        return dict(server_version='phase12-health-v1',model=model,checkpoint=c['checkpoint'],variant=c['variant'],
            action_dim=7,chunk_size=c['chunk_size'],requires_proprio=False,center_crop=model=='oft',
            preprocessing=dict(color_order='RGB',source_dtype='uint8',tensor_dtype='bfloat16',processor=processor,
                instruction_handling='clean_instruction_then_build_inference_prompt' if model=='openvla' else 'lower_whitespace_strip_terminal_punctuation',crop_bottom_fraction=0.,center_crop=True,center_crop_area_scale=.9))
    def test_all_identity_rejections(self):
        for model in ('openvla','oft'):
            h=self.health(model);validate_identity(h,model)
            for key,value in (('checkpoint','wrong'),('chunk_size',99),('requires_proprio',True),('action_dim',6)):
                with self.assertRaises(ValueError):validate_identity({**h,key:value},model)
            with self.assertRaises(ValueError):validate_identity({**h,'preprocessing':{**h['preprocessing'],'color_order':'BGR'}},model)
    def test_operator_confirmation_provenance(self):
        m=RobotStateAdapter();m.update(SimpleNamespace(disconnected=False,robot_state=1),'driver',now=1,
            confirmations={'operation_mode':'AUTO','servo_enabled':True,'authority':True})
        self.assertEqual(m.snapshot(1)['hardware_state']['provenance']['servo_enabled'],'OPERATOR_CONFIRMED')
    def test_scheduler_flush(self):
        scheduler=ActionScheduler(lambda _: {})
        scheduler.abort()
        with self.assertRaises(RuntimeError):scheduler.dispatch(0,0,0,None)
        scheduler.close()
    def test_rt_mode_adapter(self):
        from live_observation import LiveObservation
        import threading,time
        obj=LiveObservation.__new__(LiveObservation);obj.lock=threading.Lock();obj.rt_state={}
        obj.hardware_monitor=RobotStateAdapter();obj.evidence={'verified_wall_time':time.time(),
            'hardware_operator_confirmations':{'servo_enabled':True,'authority':True}}
        obj.rt_robot_state(SimpleNamespace(data=[1.]),'robot_state')
        obj.rt_robot_state(SimpleNamespace(data=[1.]),'robot_mode')
        status=obj.hardware_monitor.snapshot()
        self.assertTrue(status['servo_mode_ok'])
        self.assertEqual(status['hardware_state']['provenance']['operation_mode'],'DRIVER_REPORTED')
        obj.rt_robot_state(SimpleNamespace(data=[99.]),'robot_mode')
        self.assertFalse(obj.hardware_monitor.snapshot()['servo_mode_ok'])
    def test_gap_fields_allow_null(self):
        from integration_metrics import collect
        row={'matched_pair_id':'p','condition':'baseline','canonical_action':[0]*7}
        link=collect([row],[])['observation_gap_links'][0]
        self.assertEqual(link['matched_pair_id'],'p');self.assertIsNone(link['sim_observation_id'])
    def test_operator_cannot_override_driver_authority_loss(self):
        m=RobotStateAdapter();m.update(SimpleNamespace(disconnected=False,robot_state=1,access_control=3),
            'driver',now=1,confirmations={'authority':True,'servo_enabled':True,'operation_mode':'AUTO'})
        self.assertFalse(m.snapshot(1)['authority_ok'])

if __name__=='__main__':unittest.main()
