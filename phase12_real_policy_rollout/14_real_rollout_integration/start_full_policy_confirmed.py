"""Interactive operator evidence wrapper for the existing one-episode launcher."""
import getpass
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    here = Path(__file__).resolve().parent
    print('실제 ARM 및 GRIPPER가 움직이는 full-policy episode 1회입니다.')
    import yaml
    limits = yaml.safe_load((here/'configs/full_task.yaml').read_text())
    print(f"Episode 상한: {limits['maximum_duration_s']}초 / {limits['maximum_actions']} actions. 모든 production safety gate를 유지합니다.")
    print('TCP는 formal VERIFIED가 아닌 zero-offset consistency 기반입니다.')
    print('fault 시 gripper DO1/DO2 OFF를 최대 3회 시도하며 jaw 상태는 보장하지 않습니다.')
    fields = {
        'operator_present': 'Operator present',
        'workspace_clear': 'Workspace clear',
        'estop_accessible': 'E-stop immediately accessible',
        'robot_stationary': 'Robot stationary',
        'tcp_engineering_acceptance': '위 TCP engineering execution basis 승인',
        'gripper_mapping_confirmed': '현재 장비의 OPEN/CLOSE mapping 확인 완료',
        'explicit_one_episode': '위 fault 정책을 포함한 physical episode 1회 승인',
    }
    for label in fields.values():
        if input(label + ' [yes/NO]: ').strip().lower() != 'yes':
            print('승인되지 않아 시작하지 않습니다.')
            return 1
    output = here/'real_trials'/time.strftime('%Y%m%d_%H%M%S_full_policy_episode')
    # Keep input evidence outside output; the production launcher creates output.
    evidence = output.with_name(output.name + '_operator_inputs')
    evidence.mkdir(parents=True, exist_ok=False)
    approval = evidence/'approval.json'
    confirmation = evidence/'open_confirmation.json'
    approval.write_text(json.dumps(dict.fromkeys(fields, True) | {
        'execution_mode': 'FULL_POLICY_ROLLOUT',
        'confirmed_wall_time': time.time(), 'operator': getpass.getuser(),
    }, indent=2))
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1')
    child = subprocess.Popen([
        sys.executable, '-u', str(here/'run_live_full_policy.py'),
        '--approval', str(approval), '--open-confirmation', str(confirmation),
        '--output', str(output),
    ], cwd=here, env=env)
    print('Result directory:', output, flush=True)
    asked = False
    try:
        while child.poll() is None:
            path = output/'gripper_initialization.json'
            if not asked and path.exists():
                try:
                    init = json.loads(path.read_text())
                except (ValueError, OSError):
                    init = {}
                if init.get('ack_wall_time'):
                    asked = True
                    answer = input('OPEN 초기화 ACK 후 실제 gripper가 OPEN입니까? [yes/NO]: ')
                    confirmation.write_text(json.dumps({
                        'observed_open': answer.strip().lower() == 'yes',
                        'operator': getpass.getuser(), 'confirmed_wall_time': time.time(),
                    }, indent=2))
            time.sleep(.1)
    except (KeyboardInterrupt, EOFError):
        # Request the production abort path, never create a new motion/IO path.
        if output.exists():
            (output/'operator_abort.request').touch()
        print('Production operator-abort 요청. 물리 이상 시 E-stop을 사용하세요.', flush=True)
        child.wait()
        return 1
    print('Episode 종료. 자동 재시도하지 않습니다. Result:', output)
    return child.returncode


if __name__ == '__main__':
    raise SystemExit(main())
