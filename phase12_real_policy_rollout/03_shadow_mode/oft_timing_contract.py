"""Single source of truth for OFT 1 Hz / K=5 / 5 Hz timing."""
CONTROL_RATE_HZ=5.0
CONTROL_DT_SEC=1.0/CONTROL_RATE_HZ

def target_step(inference_frame:int,chunk_index:int)->int:
    frame=int(inference_frame);index=int(chunk_index)
    if frame<0 or index<0:raise ValueError('negative_timing_index')
    return frame+index

def target_time_sec(inference_frame:int,chunk_index:int)->float:
    return target_step(inference_frame,chunk_index)*CONTROL_DT_SEC

def inference_time_sec(inference_frame:int)->float:
    frame=int(inference_frame)
    if frame<0:raise ValueError('negative_inference_frame')
    return frame*CONTROL_DT_SEC

def action_age_sec(inference_frame:int,chunk_index:int)->float:
    return target_time_sec(inference_frame,chunk_index)-inference_time_sec(inference_frame)

def legacy_recorded_time_sec(inference_frame:int,chunk_index:int)->float:
    """Historical artifact only; never use for new runtime/replay decisions."""
    return int(inference_frame)*1.0+int(chunk_index)*CONTROL_DT_SEC
