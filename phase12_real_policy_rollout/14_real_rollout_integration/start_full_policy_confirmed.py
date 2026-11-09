"""One-episode launcher: direct MoveLine start by default, no manual prompts.

Hardware/sensor gates remain. Manual site/jaw observations are NOT fabricated.
Optional --interactive or evidence files preserve the previous confirmation modes.
"""
import getpass
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import signal
import math


def main(controller=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', choices=('moveline','servo'), default=controller or 'moveline')
    parser.add_argument('--approval',type=Path,help='현재 현장 확인 JSON; 60초 이내, 모든 필수 항목 true 및 operator 필요')
    parser.add_argument('--open-confirmation',type=Path,help='초기화 ACK 이후 실제 OPEN을 확인하고 작성할 JSON 경로')
    parser.add_argument('--interactive',action='store_true',help='기존 yes 입력 방식; 기본 MoveLine은 확인 없이 직접 시작')
    args = parser.parse_args()
    controller = args.controller
    file_mode=args.approval is not None or args.open_confirmation is not None
    direct=controller=='moveline' and not args.interactive and not file_mode
    if args.interactive and (args.approval is not None or args.open_confirmation is not None):
        parser.error('--interactive와 확인 파일 옵션은 함께 사용할 수 없습니다.')
    if not direct and not args.interactive and (args.approval is None or args.open_confirmation is None):
        parser.error('확인 파일 방식은 --approval 및 --open-confirmation을 함께 지정하세요. ServoL은 직접 시작 정책 대상이 아닙니다.')
    here = Path(__file__).resolve().parent
    print('실제 ARM 및 GRIPPER가 움직이는 full-policy episode 1회입니다.')
    import yaml
    limits = yaml.safe_load((here/'configs/full_task.yaml').read_text())
    count = limits['maximum_actions'] if controller=='moveline' else yaml.safe_load((here/'configs/servo_episode.yaml').read_text())['maximum_predictions']
    print(f"Controller: {controller}. Episode 상한: {limits['maximum_duration_s']}초 / {count} predictions/actions. 설정된 safety gate 적용.")
    if controller=='moveline':
        profile=yaml.safe_load((here/'configs/hardware_interface.yaml').read_text())
        print(f"MoveLine vel={profile['velocity']} [mm/s, deg/s], acc={profile['acceleration']} [mm/s², deg/s²].")
        if profile.get('moveline_delta_limiter_enabled',True) is False:
            print('DELTA LIMITER OFF: 허용된 원본 arm delta 전체를 MoveLine 목표로 전달합니다. 기존 0.8/4mm 축소는 없습니다.')
        if profile.get('moveline_raw_translation_gate_enabled',True) is False:
            print('RAW TRANSLATION GATE OFF: 50mm 원본 이동량 거부 기준 해제. 전체 target workspace/NaN·Inf/runtime gate는 유지.')
        else:
            print('Raw translation gate 유지.')
        print('회전4deg/workspace/watchdog 유지. 실제 target 도달을 ACK만으로 보장하지는 않습니다.')
        print('120/120 이동 profile은 물리 미검증입니다. 도달 불가/관절 한계 문제는 미해결입니다.')
    if controller=='servo':
        print('실험용 ServoL: arm 20Hz와 gripper pulse worker는 병행합니다. 명령별 arm ACK는 없습니다.')
        print('관절 한계 SAFE_OFF2/Stop ACK 실패 및 보간 overshoot 원인은 미해결입니다. full trajectory 안전성 검증 완료가 아닙니다.')
    print('TCP는 formal VERIFIED가 아닌 zero-offset consistency 기반입니다.')
    print('fault 시 gripper DO1/DO2 OFF를 최대 3회 시도하며 jaw 상태는 보장하지 않습니다.')
    if direct:
        print('DIRECT START: yes/승인파일/실제 OPEN 수동 확인 없이 자동 preflight 후 1회 실행합니다.')
        print('현장 안전 항목은 미확인으로 기록하며 OPEN 상태는 IO ACK 기반 command knowledge입니다.')
    fields = {
        'operator_present': 'Operator present',
        'workspace_clear': 'Workspace clear',
        'estop_accessible': 'E-stop immediately accessible',
        'robot_stationary': 'Robot stationary',
        'tcp_engineering_acceptance': '위 TCP engineering execution basis 승인',
        'gripper_mapping_confirmed': '현재 장비의 OPEN/CLOSE mapping 확인 완료',
        'explicit_one_episode': '위 fault 정책을 포함한 physical episode 1회 승인',
    }
    if controller=='servo':
        fields.update(servo_risks_accepted='위 ServoL 미해결 위험 및 speed slider 비의존 승인',
                      concurrent_arm_gripper_accepted='연속 arm stream과 gripper pulse 병행 실행 승인',
                      authority_and_stops_confirmed='Authority/servo 정상 및 protective/emergency stop clear',
                      no_other_motion_executor='다른 rollout/motion/servo 실행기 없음')
    if direct:
        from direct_moveline_mode import DirectMoveLineMode
        approval_data=DirectMoveLineMode.launch_request(getpass.getuser())
        DirectMoveLineMode().validate(approval_data)
    elif args.interactive:
        for label in fields.values():
            if input(label + ' [yes/NO]: ').strip().lower() != 'yes':
                print('승인되지 않아 시작하지 않습니다.')
                return 1
        approval_data=dict.fromkeys(fields,True) | {
            'execution_mode':'FULL_POLICY_ROLLOUT','physical_controller':controller,
            'confirmed_wall_time':time.time(),'operator':getpass.getuser(),
            'evidence_source':'INTERACTIVE_CURRENT_OPERATOR_INPUT'}
    else:
        try:
            approval_data=json.loads(args.approval.read_text())
            if not isinstance(approval_data,dict):raise ValueError('approval_not_object')
            if not all(approval_data.get(k) is True for k in fields):raise ValueError('current_operator_confirmation_required')
            if approval_data.get('execution_mode')!='FULL_POLICY_ROLLOUT' or approval_data.get('physical_controller')!=controller:
                raise ValueError('approval_controller_or_scope_mismatch')
            operator=approval_data.get('operator');stamp=approval_data.get('confirmed_wall_time')
            if not isinstance(operator,str) or not operator.strip():raise ValueError('operator_required')
            if type(stamp) not in (int,float) or not math.isfinite(stamp) or not 0<=time.time()-stamp<=60:
                raise ValueError('fresh_operator_approval_expired')
            if args.approval.resolve()==args.open_confirmation.resolve():raise ValueError('separate_OPEN_evidence_file_required')
        except (OSError,ValueError,TypeError) as exc:
            print('NOT_EXECUTED: 확인 파일 오류:',str(exc))
            return 1
    suffix='full_policy_episode' if controller=='moveline' else 'full_policy_servo_episode'
    output = here/'real_trials'/time.strftime('%Y%m%d_%H%M%S_'+suffix)
    # Keep input evidence outside output; the production launcher creates output.
    evidence = output.with_name(output.name + '_operator_inputs')
    evidence.mkdir(parents=True, exist_ok=False)
    approval = evidence/('launch_request.json' if direct else 'approval.json')
    confirmation = None if direct else evidence/'open_confirmation.json' if args.interactive else args.open_confirmation.resolve()
    if file_mode:
        approval_data=dict(approval_data,approval_input_file=str(args.approval.resolve()),
            open_confirmation_input_file=str(confirmation))
    # Preserve the operator's original timestamp; copying does not renew consent.
    approval.write_text(json.dumps(approval_data,indent=2))
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1')
    command = [sys.executable, '-u', str(here/('run_live_full_policy.py' if controller=='moveline' else 'start_servo_episode_confirmed.py'))]
    if controller=='servo':command.append('--full-policy')
    command+=['--approval',str(approval),'--output',str(output)]
    command+=['--direct'] if direct else ['--open-confirmation',str(confirmation)]
    child = subprocess.Popen(command,cwd=here,env=env,start_new_session=True)
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
                    if direct:
                        print('OPEN IO ACK 완료. 수동 입력 없이 계속 진행합니다. 실제 jaw 상태는 UNKNOWN입니다.',flush=True)
                    elif args.interactive:
                        answer = input('OPEN 초기화 ACK 후 실제 gripper가 OPEN입니까? [yes/NO]: ')
                        confirmation.write_text(json.dumps({
                            'observed_open': answer.strip().lower() == 'yes',
                            'operator': getpass.getuser(), 'confirmed_wall_time': time.time(),
                        }, indent=2))
                    else:
                        print('입력 질문 없음. 실제 OPEN 확인 후 ACK 이후 timestamp의 확인 JSON을 작성하세요:',confirmation,flush=True)
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
    def terminate(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    raise SystemExit(main())
