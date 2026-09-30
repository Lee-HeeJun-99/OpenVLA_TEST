from pathlib import Path
import math
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "04_planner_reference"))
from reference_command_extractor import relative_rotvec  # noqa: E402


class DoosanConversionSyntheticFixture(unittest.TestCase):
    """SYNTHETIC_FIXTURE: tests collector convention, not vendor semantics."""
    def test_identity_and_yaw(self):
        self.assertEqual(relative_rotvec([0, 0, 0], [0, 0, 0]), [0, 0, 0])
        value = relative_rotvec([0, 0, 0], [0, 0, math.pi / 2])
        self.assertAlmostEqual(value[2], math.pi / 2)

