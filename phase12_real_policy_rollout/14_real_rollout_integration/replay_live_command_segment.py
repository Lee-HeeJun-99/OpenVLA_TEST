"""Replay recorded actions only. Never imports ROS or creates a transport."""
import argparse
import hashlib
import json
import time
import statistics
from pathlib import Path
import yaml
from servo_experimental.live_segment import LiveCommandSegment, stream_planning_limits


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--trial',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parent
    args.output.mkdir(parents=True,exist_ok=False)
    def rows(name):return [json.loads(line) for line in (args.trial/name).read_text().splitlines() if line.strip()]
    predictions=rows('predictions.jsonl')
    commands=[r for r in rows('servo_commands.jsonl') if 'pos' in r]
    limits=yaml.safe_load((root.parent/'01_configs/safety_limits.yaml').read_text())
    config=yaml.safe_load((root/'configs/servo_episode.yaml').read_text())
    limits=stream_planning_limits(limits,config)
    # Fake release evidence ONLY permits pure candidate computation in replay.
    fake=dict(normalized_euler_controller_verified=True,physical_stop_verified=True,
        controller_joint_protection_verified=True,following_translation_mm=10.,
        following_rotation_deg=3.,feedback_max_age_s=.5)
    tracker=LiveCommandSegment(predictions[0]['current_controller_pose'],limits,
        transport=config['servo_transport'],protection=fake)
    by_source={p['camera_timing']['inference_start_monotonic']:p for p in predictions}
    output=[];source=None;failure=None;planning_durations=[]
    for row in commands:
        try:
            now=row['requested_monotonic']
            if row['source_monotonic']!=source:
                source=row['source_monotonic'];prediction=by_source[source]
                planning_started=time.perf_counter()
                tracker.set_action(prediction['action_pipeline']['limited_canonical_action'],source,now)
                planning_durations.append(time.perf_counter()-planning_started)
            result=tracker.tick(now)
            tracker.mark_published(now,result['pos'])
            output.append(result)
        except Exception as exc:
            failure=dict(reason=str(exc),source=source,requested_monotonic=now,
                         step=by_source[source]['step'],additional_dispatch_allowed=False)
            if tracker.segment is not None:
                active_source=tracker.segment['source']
                failure.update(active_source_monotonic=active_source,
                    active_policy_step=by_source[active_source]['step'],
                    active_action_age_s=now-active_source,
                    pending_source_monotonic=None if tracker.pending is None else tracker.pending[1])
            break
    segments={r['source_monotonic']:r for r in output}
    progress=[r['planned_execution_fraction'] for r in segments.values()]
    summary=dict(physical_commands=0,transport_created=False,recorded_actions=len(predictions),
        effective_stream_limits={k:limits[k] for k in config['stream_planning']},
        servo_transport=config['servo_transport'],
        generated_candidates=len(output),first_inhibit=failure,
        activated_segments=tracker.activated_segments, replaced_pending=tracker.replaced_pending,
        braking_plans=tracker.braking_plans,
        deadline_limited_segments=int(sum(f<1-1e-9 for f in progress)),
        planned_execution_fraction_min=min(progress,default=0.),
        planned_execution_fraction_median=statistics.median(progress) if progress else None,
        abc_entirely_normalized=all(all(-180<=a<=180 for a in r['pos'][3:]) for r in output),
        input_sources_without_recorded_stream=len(set(by_source)-set(r['source_monotonic'] for r in commands)),
        max_offline_planning_wall_s=max(planning_durations,default=0.),
        max_translation_speed_mm_s=max((sum(v*v for v in r['planning_velocity'][:3])**.5 for r in output),default=0.),
        max_rotation_speed_deg_s=max((sum(v*v for v in r['planning_velocity'][3:])**.5 for r in output),default=0.),
        max_translation_acceleration_mm_s2=max((sum(v*v for v in r['planning_acceleration'][:3])**.5 for r in output),default=0.),
        max_rotation_acceleration_deg_s2=max((sum(v*v for v in r['planning_acceleration'][3:])**.5 for r in output),default=0.),
        production_release=config['segment_release'],
        replay_following_guard_used=False,
        caveat='Recorded actual poses are not counterfactual feedback; tracking not replay-validated.',
        max_arc_translation_mm=max((r['cumulative_translation_mm'] for r in output),default=0),
        max_arc_rotation_deg=max((r['cumulative_rotation_deg'] for r in output),default=0),
        inputs={str(args.trial/name):hashlib.sha256((args.trial/name).read_bytes()).hexdigest()
                for name in ('predictions.jsonl','servo_commands.jsonl')},
        code_sha256={str(root/name):hashlib.sha256((root/name).read_bytes()).hexdigest()
            for name in ('servo_experimental/live_segment.py','servo_experimental/bounded_segment.py',
                         'replay_live_command_segment.py','configs/servo_episode.yaml')})
    (args.output/'replay_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (args.output/'candidates.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in output))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
