import ast,unittest
from pathlib import Path
import complete_readiness_comparison as audit

class ComparisonTests(unittest.TestCase):
    def test_no_command_or_getter_capability(self):
        tree=ast.parse(Path(audit.__file__).read_text())
        calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertFalse(calls & {'create_client','create_publisher','call_async','send_goal_async'})
    def test_same_joint_subscriber(self):
        source=Path(audit.__file__).read_text()
        self.assertEqual(source.count("'/dsr01/joint_states'"),1)
        self.assertNotIn("ros2','launch','dsr",source)
    def test_perturbation_flags(self):
        self.assertEqual(audit.FULL,{'camera','tf','tf_static','rosout','dynamic','cli','services','hash','http'})
    def test_runtime_and_window_not_relaxed(self):
        source=Path(audit.__file__).read_text()
        self.assertIn('all(0<x<.1 for x in sg+rg)',source)
        self.assertIn('t1=t0+duration',source)
        self.assertIn('duration=60',source)
    def test_service_flag_isolates_cli(self):
        self.assertEqual(audit.cli_commands({'cli','dynamic'}),[['ros2','topic','list','-t']])
        self.assertEqual(len(audit.cli_commands({'cli','services'})),2)
