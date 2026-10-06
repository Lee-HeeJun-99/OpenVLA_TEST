"""Evidence supplied by observation and signed operator preflight; fail closed."""
import math
import time

REQUIRED = ('ros_graph_alive','camera_ready','joint_state_ready','tcp_ready','robot_mode_confirmed',
            'servo_confirmed','estop_confirmed','protective_stop_confirmed','workspace_confirmed',
            'gripper_polarity_confirmed','gripper_initial_confirmed','clock_valid','model_ready','logger_ready',
            'disk_ready','command_path_confirmed','hold_path_confirmed')

def preflight(evidence, *, dry_run=True, now=None):
    now=time.time() if now is None else now
    verified=evidence.get('verified_wall_time')
    fresh=isinstance(verified,(int,float)) and math.isfinite(verified) and 0<=now-verified<=60
    checks={name:evidence.get(name) is True and fresh for name in REQUIRED}
    result={'checks':checks,'evidence_fresh':fresh,
        'OBSERVATION_READY':all(checks[x] for x in ('ros_graph_alive','camera_ready','joint_state_ready','tcp_ready','clock_valid')),
        'MODEL_READY':checks['model_ready'],'LOGGER_READY':checks['logger_ready'] and checks['disk_ready'],
        'COMMAND_PATH_READY':checks['command_path_confirmed'] and checks['hold_path_confirmed'],
        'ROBOT_STATE_READY':all(checks[x] for x in ('robot_mode_confirmed','servo_confirmed','estop_confirmed','protective_stop_confirmed')),
        'SAFETY_READY':all(checks[x] for x in ('workspace_confirmed','gripper_polarity_confirmed','gripper_initial_confirmed'))}
    result['MOTION_READY']=all(checks.values()) and not dry_run
    result['missing']=[k for k,v in checks.items() if not v]
    return result

def select_tcp(observation, *, fk_function=None, fk_approved=False):
    if observation.get('measured_tcp_pose') is not None:
        return {'pose':observation['measured_tcp_pose'],'tcp_source':'MEASURED'}
    if fk_approved and fk_function and observation.get('joint_positions_by_name'):
        return {'pose':fk_function(observation['joint_positions_by_name']),'tcp_source':'FK_ESTIMATED'}
    raise RuntimeError('tcp_unavailable')
