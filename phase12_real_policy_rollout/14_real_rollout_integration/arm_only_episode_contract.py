"""Physical arm-only boundary and original full-task termination limits."""
import math
from pathlib import Path
import yaml
from real_sink import RealDoosanCommandSink


class ArmOnlyEpisodeSink(RealDoosanCommandSink):
    physical_gripper_disabled = True
    def _send(self, kind, request, **kwargs):
        if kind not in ('pose', 'hold', 'stop'):
            raise PermissionError('arm_only_episode_all_gripper_IO_disabled')
        return super()._send(kind, request, **kwargs)

    def send_gripper(self, closed):
        raise PermissionError('arm_only_episode_gripper_disabled')


def episode_limits():
    config = yaml.safe_load((Path(__file__).parent/'configs/full_task.yaml').read_text())
    steps, duration = config['maximum_actions'], config['maximum_duration_s']
    if isinstance(steps, bool) or not isinstance(steps, int) or steps <= 0:
        raise ValueError('invalid_existing_episode_step_limit')
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('invalid_existing_episode_duration_limit')
    return steps, duration


def gripper_intent(value):
    if not math.isfinite(value) or not 0 <= value <= 1:
        raise ValueError('invalid_raw_gripper')
    return 'OPEN' if value < .3 else 'CLOSED' if value > .7 else 'HOLD'


def approved_episode_limits(approval, requested_duration_s=None):
    steps,duration=episode_limits()
    if requested_duration_s is None:
        return steps,duration
    # This run-specific extension is explicitly requested; never modifies the
    # production default, step cap or any action/sensor safety threshold.
    if (isinstance(requested_duration_s,bool) or requested_duration_s != 70
            or approval.get('approved_max_duration_s') != 70
            or approval.get('explicit_one_episode') is not True):
        raise PermissionError('explicit_70_second_arm_only_approval_required')
    return steps,70.
