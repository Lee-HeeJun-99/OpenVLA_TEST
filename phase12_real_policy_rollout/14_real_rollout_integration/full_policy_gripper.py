"""Explicit full-policy gripper lifecycle; no ROS calls at import/construction.

Command knowledge is not measured jaw feedback. Abort output requires separate
hardware evidence, never inferred from a normal pulse's trailing OFF edge.
"""
import math
import threading


class FullPolicyGripper:
    def __init__(self, sink, logger, contract):
        self.sink = sink
        self.logger = logger
        self.contract = dict(contract)
        self.state = None
        self.initialized = False
        self.initialization_ack = False
        self.aborted = False
        self.lock = threading.RLock()
        # This adapter supports the existing sink's selected-coil OFF abort only.
        # Other behaviors need their own proven transport implementation.
        if not (contract.get('abort_behavior') == 'OUTPUT_OFF'
                and contract.get('abort_behavior_confirmed') is True
                and contract.get('abort_evidence_source')
                and sink.gripper_config.get('abort_value') == 0
                and sink.gripper_config.get('abort_value_confirmed') is True):
            raise PermissionError('gripper_abort_behavior_unverified')
        if sink.gripper_config.get('polarity_confirmed') is not True:
            raise PermissionError('gripper_mapping_unverified')
        if sink.gripper_config.get('open_pulse_count') != 1:
            raise PermissionError('full_policy_requires_single_open_initialization_pulse')

    def initialize_open(self):
        with self.lock:
            if self.aborted or self.sink.aborted:
                raise PermissionError('gripper_aborted')
            if self.initialization_ack:
                raise PermissionError('open_initialization_already_requested')
            # One logical OPEN, one pulse; retain the existing channel/value/time.
            self.logger.append({'event': 'GRIPPER_INITIALIZATION_REQUEST', 'desired': 'OPEN'})
            result = self.sink.send_gripper(False)
            self.logger.append({'event': 'GRIPPER_INITIALIZATION_ACK', 'receipt': result,
                                'observed_open': None})
            if result.get('state') != 'completed':
                raise RuntimeError('gripper_initialization_ack_failure')
            self.initialization_ack = True
            return result

    def confirm_observed_open(self, *, observed_open, evidence_source):
        with self.lock:
            if self.aborted or self.sink.aborted:
                raise PermissionError('gripper_aborted')
            if not self.initialization_ack or observed_open is not True or not evidence_source:
                raise PermissionError('actual_initial_open_confirmation_required')
            self.logger.append({'event': 'GRIPPER_INITIALIZATION_OBSERVED',
                                'observed_open': True, 'evidence_source': evidence_source})
            self.state = 'OPEN'
            self.initialized = True

    def desired(self, closedness):
        with self.lock:
            if self.aborted or self.sink.aborted:
                raise PermissionError('gripper_aborted')
            if not self.initialized:
                raise PermissionError('gripper_not_initialized_open')
            value = float(closedness)
            if not math.isfinite(value) or not 0 <= value <= 1:
                raise ValueError('invalid_gripper_closedness')
            if value < .3:
                return 'OPEN'
            if value > .7:
                return 'CLOSED'
            return self.state

    def transition(self, closedness):
        with self.lock:
            desired = self.desired(closedness)
            before = self.state
            self.logger.append({'event': 'FULL_POLICY_GRIPPER_CANDIDATE',
                                'raw_closedness': closedness, 'logical_before': before,
                                'desired': desired, 'command_required': desired != before})
            if desired == before:
                return {'state': 'suppressed', 'logical_before': before,
                        'logical_after': before, 'measured_gripper_state': None}
            result = self.sink.send_gripper(desired == 'CLOSED')
            if result.get('state') != 'completed':
                raise RuntimeError('gripper_transition_ack_failure')
            if self.aborted or self.sink.aborted:
                raise PermissionError('gripper_aborted')
            self.logger.append({'event': 'FULL_POLICY_GRIPPER_RESULT',
                                'raw_closedness': closedness, 'logical_before': before,
                                'logical_after': desired, 'receipt': result,
                                'measured_gripper_state': None})
            self.state = desired
            return result

    def inhibit(self):
        # No physical OPEN/CLOSE on a fault; the existing sink cancels a pulse
        # and applies its independently verified abort output.
        self.aborted = True
        self.initialized = False
