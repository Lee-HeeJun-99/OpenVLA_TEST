import ast,unittest
from pathlib import Path
import tcp_hardware_precheck as audit

class PrecheckTests(unittest.TestCase):
    def test_only_audited_getters(self):
        self.assertEqual({v[1] for v in audit.SERVICES.values()},
            {'GetCurrentTcp','GetCurrentTool','GetRobotMode','GetRobotState','GetRobotSystem'})
    def test_no_motion_capability(self):
        tree=ast.parse(Path(audit.__file__).read_text())
        calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertNotIn('create_publisher',calls)
        self.assertNotIn('send_goal_async',calls)
        self.assertEqual(sum(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='call_async' for n in ast.walk(tree)),1)
    def test_single_joint_subscription(self):
        source=Path(audit.__file__).read_text()
        self.assertEqual(source.count("'/dsr01/joint_states'"),1)
        self.assertIn('post_start<10',source)
