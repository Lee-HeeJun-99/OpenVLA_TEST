"""Recorded metrics; command intent and observed task outcomes stay distinct."""
import math

def metrics(rows, outcome=None):
    positions=[r['tcp_pose'][:3] for r in rows if r.get('tcp_pose')]
    length=sum(math.dist(a,b) for a,b in zip(positions,positions[1:])) if len(positions)>1 else None
    close=next((r['target_time'] for r in rows if r.get('gripper_closedness',0)>=.7),None)
    rejected=sum(not r['safety_accepted'] for r in rows)
    lat=[r['latency'] for r in rows if isinstance(r.get('latency'),(int,float))]
    deviation=[math.dist(r['tcp_pose'][:3],r['reference_tcp_pose'][:3]) for r in rows if r.get('tcp_pose') and r.get('reference_tcp_pose')]
    return dict(classification='OFFLINE_RECORDED_DRY_RUN',task_success=None if outcome is None else outcome.get('success'),
        completion_time_s=None if outcome is None else outcome.get('completion_time_s'),
        trajectory_length_m=length,tcp_path_deviation_mean_m=sum(deviation)/len(deviation) if deviation else None,
        first_close_time_s=close,first_close_type='MODEL_INTENT',safety_rejection_count=rejected,
        safety_intervention_count=sum(r.get('hold_required',False) for r in rows),
        model_latency_mean_s=sum(lat)/len(lat) if lat else None,control_latency_s=None,
        translation_action_mean_m=sum(r.get('translation_norm') or 0 for r in rows)/len(rows) if rows else None,
        rotation_action_mean_rad=sum(r.get('rotation_norm') or 0 for r in rows)/len(rows) if rows else None,
        oft_chunk_reject_rate={str(k):sum(not r['safety_accepted'] for r in rows if r.get('chunk_index')==k)/max(1,sum(r.get('chunk_index')==k for r in rows)) for k in range(5)},
        records=len(rows),command_issued_count=sum(r['command_issued'] for r in rows),
        observation_gap_join_keys=['episode_id','condition','matched_pair_id'])
