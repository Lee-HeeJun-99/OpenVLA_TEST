import ast,unittest
from pathlib import Path
import final_hardware_tcp_precheck as audit

class FinalPrecheckTests(unittest.TestCase):
    def test_no_tcp_tool_refresh(self):
        self.assertEqual({v[0] for v in audit.REFRESH.values()},{'GetRobotMode','GetRobotState','GetRobotSystem'})
    def test_single_joint_subscriber_no_motion_publish(self):
        source=Path(audit.__file__).read_text();tree=ast.parse(source)
        calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertNotIn('create_publisher',calls);self.assertNotIn('send_goal_async',calls)
        self.assertEqual(source.count("'/dsr01/joint_states'"),1)
    def test_cache_not_misclassified_null_risk(self):
        entries={e['service_type']:e for e in audit.source_audit()}
        self.assertEqual(entries['GetCurrentToolFlangePosx']['classification'],'READ_ONLY_BUT_STABILITY_UNVERIFIED')
        self.assertEqual(entries['GetCurrentPose']['classification'],'UNSAFE_OR_NULL_RISK')
