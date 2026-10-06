"""Protocol readiness states; never grants command capability."""
STATES = ('COMMAND_DISABLED','READINESS_CHECK','MINIMUM_MOTION_READY','SHORT_HORIZON_READY','FULL_ROLLOUT_READY')

class RolloutStateMachine:
    def __init__(self):
        self.state=STATES[0]
        self.aborted=False
        self.reason=None

    def advance(self, *, checks_passed=False, hardware_approval=False, previous_protocol_passed=False):
        if self.aborted: raise RuntimeError('session_aborted')
        index=STATES.index(self.state)
        if index==len(STATES)-1: raise RuntimeError('final_state')
        if index>=1 and not (checks_passed and hardware_approval and previous_protocol_passed):
            raise RuntimeError('explicit_hardware_approval_and_validation_required')
        self.state=STATES[index+1]
        return self.state

    def abort(self, reason):
        self.aborted=True
        self.reason=reason
        self.state=STATES[0]
