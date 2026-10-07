import ast
from pathlib import Path
import unittest
from jointstate_startup_warmup_validation import phase_at

class StartupPhaseTests(unittest.TestCase):
    def test_no_fresh_is_startup(self):
        self.assertEqual(phase_at(100,None,None),'STARTUP_DISCOVERY_PHASE')
    def test_before_fresh_is_startup(self):
        self.assertEqual(phase_at(9.99,10,20),'STARTUP_DISCOVERY_PHASE')
    def test_exact_first_is_warmup(self):
        self.assertEqual(phase_at(10,10,20),'POST_DISCOVERY_WARMUP')
    def test_exact_t0_is_runtime(self):
        self.assertEqual(phase_at(20,10,20),'RUNTIME_CLEAN_WINDOW')
    def test_command_capability_absent(self):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'jointstate_startup_warmup_validation.py').read_text())
        forbidden={'create_client','create_publisher','ActionClient','call_async'}
        for node in ast.walk(tree):
            if isinstance(node,ast.Call):
                name=getattr(node.func,'attr',getattr(node.func,'id',None))
                self.assertNotIn(name,forbidden)
