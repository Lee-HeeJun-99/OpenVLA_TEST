"""Explicit full-policy live session wiring; construction never sends IO."""
import json
import time
from time import monotonic
from pathlib import Path
import yaml
from real_sink import RealDoosanCommandSink,Authorization


class FullPolicyLiveMode:
    scope='FULL_POLICY_ROLLOUT'
    def __init__(self,open_confirmation,*,confirmed_initialization=None):
        self.open_confirmation=Path(open_confirmation)
        self.confirmed_initialization=Path(confirmed_initialization) if confirmed_initialization else None
        self.initialization=None

    def validate(self,approval):
        required=('operator_present','workspace_clear','estop_accessible','robot_stationary',
                  'tcp_engineering_acceptance','gripper_mapping_confirmed','explicit_one_episode')
        if approval.get('execution_mode')!=self.scope or not all(approval.get(k) is True for k in required):
            raise PermissionError('explicit_full_policy_approval_required')
        if not 0<=time.time()-approval.get('confirmed_wall_time',0)<=60:
            raise PermissionError('fresh_operator_approval_expired')

    def approved_gripper_config(self,approval):
        self.validate(approval)
        cfg=yaml.safe_load((Path(__file__).parent/'configs/gripper_hardware.yaml').read_text())
        if cfg.get('command_mode')!='operator_timed_pulse':raise PermissionError('current_gripper_contract_required')
        if cfg.get('abort_behavior')!='BOTH_OUTPUTS_OFF' or cfg.get('abort_policy_operator_approved') is not True:
            raise PermissionError('operator_approved_abort_contract_required')
        cfg['polarity_confirmed']=approval['gripper_mapping_confirmed']
        return cfg

    def make_sink(self,factory,approval):
        cfg=self.approved_gripper_config(approval)
        return RealDoosanCommandSink(factory,Authorization(True,True,True,'MOTION_ENABLED'),gripper_config=cfg)

    def initialize(self,controller,check,save):
        check()
        if self.confirmed_initialization is not None:
            evidence=json.loads(self.open_confirmation.read_text())
            previous=json.loads(self.confirmed_initialization.read_text())
            parent=self.confirmed_initialization.parent
            cleanup=json.loads((parent/'gripper_abort_result.json').read_text())
            summary=json.loads((parent/'trial_summary.json').read_text())
            stamp=evidence.get('confirmed_wall_time')
            if (evidence.get('observed_open') is not True or not evidence.get('operator')
                    or not isinstance(stamp,(int,float)) or not 0<=time.time()-stamp<=60
                    or stamp<summary['episode_ended_wall_time']
                    or (previous.get('receipt') or {}).get('state')!='completed'
                    or summary.get('reason')!='actual_OPEN_confirmation_timeout'
                    or summary.get('arm_commands')!=0
                    or cleanup.get('state')!='completed'
                    or not all(r.get('success') is True and r.get('value')==0 for r in cleanup['attempts'][-1]['readback'])):
                raise PermissionError('fresh_confirmed_initialized_OPEN_required')
            # This is an explicit continuation of prior successful electrical
            # initialization, not automatic reuse of a trial's runtime gate.
            g=controller.full_policy_gripper
            g.initialization_ack=True
            controller.sink.gripper_outputs_initialized=True
            controller.sink.last_gripper=False
            g.confirm_observed_open(observed_open=True,evidence_source=str(self.open_confirmation))
            self.initialization=dict(receipt=previous['receipt'],observed_open=True,
                operator_evidence=evidence,previous_initialization_source=str(self.confirmed_initialization),
                new_OPEN_command_issued=False)
            save('gripper_initialization.json',self.initialization)
            return
        result=controller.full_policy_gripper.initialize_open()
        ack_wall=time.time()
        self.initialization=dict(receipt=result,observed_open=None,ack_wall_time=ack_wall)
        save('gripper_initialization.json',self.initialization)
        print('OPEN initialization ACK 완료. 실제 OPEN을 확인한 뒤 fresh open-confirmation 파일을 작성하세요.',flush=True)
        deadline=monotonic()+60
        while monotonic()<deadline:
            check()
            if self.open_confirmation.exists():
                try:evidence=json.loads(self.open_confirmation.read_text())
                except (ValueError,OSError):evidence={}
                stamp=evidence.get('confirmed_wall_time')
                if isinstance(stamp,(int,float)) and ack_wall<=stamp<=time.time() and time.time()-stamp<=60:
                    if evidence.get('observed_open') is not True or not evidence.get('operator'):
                        raise PermissionError('actual_OPEN_not_confirmed')
                    controller.full_policy_gripper.confirm_observed_open(observed_open=True,evidence_source=str(self.open_confirmation))
                    self.initialization.update(observed_open=True,operator_evidence=evidence)
                    save('gripper_initialization.json',self.initialization)
                    return
            time.sleep(.02)
        raise TimeoutError('actual_OPEN_confirmation_timeout')

    @staticmethod
    def finalize_outcome(summary,outcome_evidence=None):
        # Runtime faults cannot become task SUCCESS, including operator claims.
        if summary.get('reason') or summary.get('status') in ('INVALID','OPERATOR_ABORT','NOT_EXECUTED'):
            return summary
        if outcome_evidence is None:return summary
        if summary.get('runtime_valid') is not True or summary.get('arm_commands',0)<1:
            raise ValueError('completed_valid_physical_trial_required')
        if type(outcome_evidence.get('task_success')) is not bool or not outcome_evidence.get('operator'):
            raise ValueError('explicit_operator_task_outcome_required')
        stamp=outcome_evidence.get('confirmed_wall_time')
        if not isinstance(stamp,(int,float)) or not summary['episode_ended_wall_time']<=stamp<=time.time():
            raise ValueError('post_episode_task_outcome_required')
        result=dict(summary,status='SUCCESS' if outcome_evidence['task_success'] else 'FAILURE',
                    runtime_valid=True,task_success=outcome_evidence['task_success'],task_performance_denominator_included=True)
        result['task_outcome_evidence']=outcome_evidence
        return result
