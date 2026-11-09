"""Direct execution policy; never fabricate current human or jaw observations.

Construction and validation do not import ROS or send any command. The common
runner must still pass current hardware/sensor/workspace/clock/logger gates.
"""
import hashlib
import json
from pathlib import Path
import time
import uuid
import yaml
from full_policy_live_mode import FullPolicyLiveMode

ROOT=Path(__file__).resolve().parent
MANUAL_FIELDS=('operator_present','workspace_clear','estop_accessible','robot_stationary',
               'tcp_engineering_acceptance','gripper_mapping_confirmed')


class DirectMoveLineMode(FullPolicyLiveMode):
    allows_command_ack_initialization=True

    def __init__(self,policy_path=None,gripper_path=None):
        super().__init__('UNUSED_NO_MANUAL_CONFIRMATION')
        self.policy_path=Path(policy_path or ROOT/'configs/direct_moveline_launch.yaml')
        self.gripper_path=Path(gripper_path or ROOT/'configs/gripper_hardware.yaml')
        self.config_paths=[self.policy_path,self.gripper_path,Path(__file__)]

    @staticmethod
    def launch_request(launcher_user):
        return dict.fromkeys(MANUAL_FIELDS,None) | dict(
            execution_mode='FULL_POLICY_ROLLOUT',physical_controller='moveline',
            launch_policy='DIRECT_MOVELINE_NO_MANUAL_CONFIRMATION',request_id=uuid.uuid4().hex,
            launcher_user=launcher_user,launcher_started_wall_time=time.time(),
            manual_confirmation_performed=False,operator=None,confirmed_wall_time=None,
            explicit_one_episode=True,evidence_source='DIRECT_PROGRAM_INVOCATION_NOT_SITE_CONFIRMATION')

    def validate(self,request):
        policy=yaml.safe_load(self.policy_path.read_text())
        if (policy.get('enabled') is not True or policy.get('mode')!='DIRECT_MOVELINE_NO_MANUAL_CONFIRMATION'
                or policy.get('gripper_initialization_basis')!='COMMAND_ACK_ONLY'
                or request.get('launch_policy')!=policy['mode']
                or request.get('execution_mode')!='FULL_POLICY_ROLLOUT'
                or request.get('physical_controller')!='moveline'
                or not isinstance(request.get('request_id'),str) or not request['request_id']
                or request.get('explicit_one_episode') is not True):
            raise PermissionError('direct_moveline_policy_required')
        if request.get('manual_confirmation_performed') is not False or any(request.get(k) is not None for k in MANUAL_FIELDS):
            raise PermissionError('do_not_fabricate_current_manual_confirmation')
        if hashlib.sha256(self.gripper_path.read_bytes()).hexdigest()!=policy.get('gripper_config_sha256'):
            raise PermissionError('previous_gripper_mapping_config_changed')

    def approved_gripper_config(self,request):
        self.validate(request)
        cfg=yaml.safe_load(self.gripper_path.read_text())
        if (cfg.get('command_mode')!='operator_timed_pulse' or cfg.get('abort_behavior')!='BOTH_OUTPUTS_OFF'
                or cfg.get('abort_policy_operator_approved') is not True):
            raise PermissionError('existing_gripper_command_abort_contract_required')
        # Static wiring/sequence acceptance from prior user-confirmed tests, NOT
        # a fresh physical observation or a new wiring/channel guess.
        cfg.update(polarity_confirmed=True,mapping_confirmation_basis='PRIOR_USER_TESTS_CONFIG_PIN',
                   mapping_current_session_verified=False)
        return cfg

    def initialize(self,controller,check,save):
        check()
        receipt=controller.full_policy_gripper.initialize_open()
        check()
        controller.full_policy_gripper.accept_commanded_open(policy_source=str(self.policy_path))
        self.initialization=dict(receipt=receipt,ack_wall_time=time.time(),observed_open=None,
            measured_gripper_state=None,commanded_gripper_state='OPEN',
            initialization_basis='COMMAND_ACK_ONLY',manual_confirmation_required=False,
            note='IO ACK is not proof of physical jaw OPEN')
        save('gripper_initialization.json',self.initialization)
        save('manual_confirmation.json',dict(performed=False,status='NOT_CHECKED',
            **dict.fromkeys(MANUAL_FIELDS,None)))
        save('direct_launch_policy.json',yaml.safe_load(self.policy_path.read_text()))
        print('OPEN 초기화 IO ACK 완료. 실제 jaw OPEN은 미측정; 수동 확인 없이 다음 preflight로 진행합니다.',flush=True)
