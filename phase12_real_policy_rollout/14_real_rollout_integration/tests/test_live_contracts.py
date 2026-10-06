import sys,time,threading,tempfile,json,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'03_shadow_mode'))
from robot_state_monitor import RobotStateMonitor
from concurrent_logger import ConcurrentLogger
from integrated_logger import FsyncJsonlLogger
from model_health import validate_runtime_health
from types import SimpleNamespace

class LiveContracts(unittest.TestCase):
    def test_state_unknown_mode_servo_fails_closed(self):
        m=RobotStateMonitor();m.update(SimpleNamespace(disconnected=False,robot_state=1),'AUDITED_SOURCE',now=1)
        self.assertFalse(m.snapshot(now=1)['servo_mode_ok'])
    def test_state_stale_stop_servo(self):
        for code,servo in ((5,True),(6,True),(1,False)):
            m=RobotStateMonitor();m.update(SimpleNamespace(disconnected=False,robot_state=code),'fake',robot_mode='AUTO',servo_enabled=servo,now=1)
            status=m.snapshot(now=1)
            self.assertFalse(status['protective_stop_ok'] and status['servo_mode_ok'])
            self.assertFalse(m.snapshot(now=2)['robot_state_ok'])
    def test_concurrent_jsonl(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'log.jsonl'
            with FsyncJsonlLogger(path,minimum_free_bytes=0) as base:
                log=ConcurrentLogger(base)
                threads=[threading.Thread(target=lambda k=k:[log.append({'command_id':f'{k}-{i}','terminal':'COMPLETED'}) for i in range(20)]) for k in range(4)]
                for t in threads:t.start()
                for t in threads:t.join()
            rows=[json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len(rows),80);self.assertEqual(len({r['command_id'] for r in rows}),80)
            self.assertEqual([r['sequence_number'] for r in rows],list(range(80)))
    def test_health_full_missing_mismatch(self):
        h=dict(server_version='v1',model='oft',checkpoint='28560',variant='vision',action_dim=7,
            chunk_size=5,requires_proprio=False,center_crop=True,preprocessing=dict(color_order='RGB',source_dtype='uint8',
            tensor_dtype='bfloat16',processor={'size':224},instruction_handling='lower'))
        self.assertTrue(validate_runtime_health(h,h))
        with self.assertRaises(ValueError):validate_runtime_health({**h,'chunk_size':1},h)
        with self.assertRaises(ValueError):validate_runtime_health({k:v for k,v in h.items() if k!='preprocessing'},h)

if __name__=='__main__':unittest.main()
