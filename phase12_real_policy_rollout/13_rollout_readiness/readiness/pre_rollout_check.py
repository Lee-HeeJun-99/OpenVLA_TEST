"""Checks supplied evidence; does not fabricate live readiness."""
import math
import shutil
from pathlib import Path

REQUIRED=('camera_ready','joint_state_ready','tcp_ready','model_server_ready','checkpoint_loaded',
          'instruction_valid','action_contract_valid','clock_valid','gripper_initial_acknowledged')

def check(evidence, safety, output_directory):
    flags={key:evidence.get(key) is True for key in REQUIRED}
    bounds=safety.get('workspace_m',{})
    flags['workspace_valid']=all(len(bounds.get(axis,[]))==2 and
        all(math.isfinite(float(v)) for v in bounds[axis]) and bounds[axis][0]<bounds[axis][1] for axis in ('x','y','z'))
    keys=('translation_step_m','rotation_step_deg','translation_velocity_mm_s','rotation_velocity_deg_s',
          'translation_acceleration_mm_s2','rotation_acceleration_deg_s2','max_action_age_sec','control_rate_hz')
    flags['safety_config_loaded']=all(isinstance(safety.get(k),(int,float)) and math.isfinite(safety[k]) and safety[k]>0 for k in keys)
    flags['command_gate_disabled']=evidence.get('command_mode')=='disabled'
    directory=Path(output_directory)
    directory.mkdir(parents=True,exist_ok=True)
    try:
        import tempfile
        with tempfile.TemporaryFile(dir=directory) as f:f.write(b'readiness');f.flush()
        flags['logger_writable']=True
    except OSError:flags['logger_writable']=False
    flags['disk_space_valid']=shutil.disk_usage(directory).free>=100*1024*1024
    return dict(checks=flags,MODEL_READY=all(flags[k] for k in ('model_server_ready','checkpoint_loaded','instruction_valid')),
        OBSERVATION_READY=all(flags[k] for k in ('camera_ready','joint_state_ready','tcp_ready','clock_valid')),
        SAFETY_READY=all(flags[k] for k in ('action_contract_valid','workspace_valid','safety_config_loaded','gripper_initial_acknowledged','command_gate_disabled')),
        LOGGER_READY=flags['logger_writable'] and flags['disk_space_valid'],MOTION_READY=False,
        evidence_mode=evidence.get('mode','UNVERIFIED'),unconfirmed=[k for k,v in flags.items() if not v])
