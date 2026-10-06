"""Pure validation for measured state. No ROS dependency or command capability."""
from __future__ import annotations
import math

CANONICAL_JOINTS=tuple(f'joint_{i}' for i in range(1,7))
READ_ONLY_ALLOWLIST={
 '/dsr01/aux_control/get_current_posx':'dsr_msgs2/srv/GetCurrentPosx',
 '/dsr01/system/get_robot_state':'dsr_msgs2/srv/GetRobotState',
 '/dsr01/system/get_robot_mode':'dsr_msgs2/srv/GetRobotMode',
 '/dsr01/system/get_last_alarm':'dsr_msgs2/srv/GetLastAlarm',
 '/dsr01/aux_control/get_control_mode':'dsr_msgs2/srv/GetControlMode',
 '/dsr01/aux_control/get_control_space':'dsr_msgs2/srv/GetControlSpace',
}

class FeedbackReadinessGate:
    """Reject startup/transient samples until a continuous feedback window passes."""
    def __init__(self, stable_duration_ns=2_000_000_000, warmup_max_gap_ns=50_000_000,
                 runtime_stale_gap_ns=100_000_000, minimum_stable_samples=180):
        self.stable_duration_ns=int(stable_duration_ns)
        self.warmup_max_gap_ns=int(warmup_max_gap_ns)
        self.runtime_stale_gap_ns=int(runtime_stale_gap_ns)
        self.minimum_stable_samples=int(minimum_stable_samples)
        self.reset('waiting_for_feedback')

    def reset(self, reason='reset'):
        self.state='WAITING_FOR_FEEDBACK'
        self.ready=False
        self.reason=reason
        self.window_start_receive_ns=None
        self.previous_source_ns=None
        self.previous_receive_ns=None
        self.stable_samples=0

    def observe(self, *, source_ns, receive_monotonic_ns, sample_valid=True):
        source_ns=int(source_ns); receive_monotonic_ns=int(receive_monotonic_ns)
        source_gap=None if self.previous_source_ns is None else source_ns-self.previous_source_ns
        receive_gap=None if self.previous_receive_ns is None else receive_monotonic_ns-self.previous_receive_ns
        invalid=(not sample_valid or (source_gap is not None and source_gap<=0) or
                 (receive_gap is not None and receive_gap<=0))
        excessive=(source_gap is not None and source_gap>self.warmup_max_gap_ns) or \
                  (receive_gap is not None and receive_gap>self.warmup_max_gap_ns)
        stale_while_ready=self.ready and ((source_gap is not None and source_gap>self.runtime_stale_gap_ns) or
                                           (receive_gap is not None and receive_gap>self.runtime_stale_gap_ns))
        if invalid or excessive or stale_while_ready:
            self.state='WARMING_UP'
            self.ready=False
            self.reason='invalid_feedback' if invalid else ('runtime_stale_feedback' if stale_while_ready else 'initial_discovery_warmup')
            self.window_start_receive_ns=receive_monotonic_ns
            self.stable_samples=1 if sample_valid else 0
        else:
            if self.window_start_receive_ns is None:
                self.window_start_receive_ns=receive_monotonic_ns
                self.state='WARMING_UP'
                self.stable_samples=1
            else:
                self.stable_samples+=1
            stable_ns=receive_monotonic_ns-self.window_start_receive_ns
            if stable_ns>=self.stable_duration_ns and self.stable_samples>=self.minimum_stable_samples:
                self.state='FEEDBACK_READY'; self.ready=True; self.reason=None
        self.previous_source_ns=source_ns; self.previous_receive_ns=receive_monotonic_ns
        return {'state':self.state,'feedback_ready':self.ready,'exclusion_reason':self.reason,
                'stable_samples':self.stable_samples,'source_gap_ns':source_gap,'receive_gap_ns':receive_gap}

def require_allowed_service(name,type_name):
    if READ_ONLY_ALLOWLIST.get(name)!=type_name:
        raise RuntimeError(f'REJECTED_NON_ALLOWLIST_SERVICE:{name}:{type_name}')

def canonicalize_joint_state(names,position,velocity,effort,source_stamp_ns,previous_stamp_ns=None):
    if len(names)!=len(position) or len(names)!=len(velocity): raise ValueError('joint_array_length_mismatch')
    if set(CANONICAL_JOINTS)-set(names): raise ValueError('missing_required_joint')
    if len(set(names))!=len(names): raise ValueError('duplicate_joint_name')
    by_name={n:i for i,n in enumerate(names)}
    pos=[float(position[by_name[n]]) for n in CANONICAL_JOINTS]
    vel=[float(velocity[by_name[n]]) for n in CANONICAL_JOINTS]
    if not all(math.isfinite(x) for x in pos+vel): raise ValueError('nonfinite_position_or_velocity')
    stamp=int(source_stamp_ns)
    if previous_stamp_ns is not None and stamp<=int(previous_stamp_ns): raise ValueError('stale_or_duplicate_timestamp')
    effort_available=(len(effort)==len(names) and all(math.isfinite(float(x)) for x in effort))
    return {'joint_names':list(CANONICAL_JOINTS),'position':pos,'velocity':vel,
            'effort':([float(effort[by_name[n]]) for n in CANONICAL_JOINTS] if effort_available else None),
            'effort_status':'available' if effort_available else 'unsupported_or_not_available',
            'source_timestamp_ns':stamp,'source_clock_domain':'ros_message_header'}

def tcp_record(values,*,success,receive_ros_ns,receive_monotonic_ns,latency_sec):
    raw=[float(x) for x in values]
    if not success or len(raw)!=7 or not all(math.isfinite(x) for x in raw): raise ValueError('invalid_tcp_response')
    return {'xyz_mm':raw[:3],'abc_deg_raw':raw[3:6],'solution_space':int(raw[6]),
            'reference_coordinate':'DR_BASE(0)','source':'measured_controller_feedback',
            'source_timestamp':None,'receive_ros_timestamp_ns':int(receive_ros_ns),
            'receive_monotonic_timestamp_ns':int(receive_monotonic_ns),'latency_seconds':float(latency_sec),
            'orientation_convention':'UNRESOLVED_ROTATION_CONVENTION','success':True,
            'executed_action':None,'robot_delivered_command':None,'command_issued':False}
