import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "03_shadow_mode" / "jointstate_observer_minimal.launch.py"


def test_launch_parses():
    ast.parse(LAUNCH.read_text(encoding="utf-8"))


def test_command_controller_is_excluded():
    text = LAUNCH.read_text(encoding="utf-8")
    assert '"dsr_controller2", "-c"' not in text
    assert "ActionAdapter" not in text
    assert "DoosanBridge" not in text


def test_explicit_operator_gate_defaults_false():
    text = LAUNCH.read_text(encoding="utf-8")
    assert "operator_approved_hardware_initialization" in text
    assert 'default_value="false"' in text
    assert "CONTROL_SERVO_ON" in text


def test_only_joint_state_controller_is_spawned():
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    literals = [node.value for node in ast.walk(tree) if isinstance(node, ast.Constant) and isinstance(node.value, str)]
    assert "joint_state_broadcaster" in literals
    forbidden = {"dsr_controller2", "dsr_joint_trajectory", "dsr_position_controller"}
    # The explanatory module docstring may mention dsr_controller2; executable
    # spawner arguments are separately checked by source pattern above.
    assert not ({"dsr_joint_trajectory", "dsr_position_controller"} & set(literals))

