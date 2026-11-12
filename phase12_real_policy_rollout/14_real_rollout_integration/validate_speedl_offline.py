"""Replay recorded pose/action targets, never import ROS or contact a server."""
import argparse
import json
import math
from pathlib import Path
import yaml
from scipy.spatial.transform import Rotation
from speedl_experimental.controller import SpeedLShadowController


def run(trial, output):
    if output.exists():
        raise FileExistsError(output)
    limits = yaml.safe_load((Path(__file__).parent.parent/'01_configs/safety_limits.yaml').read_text())
    observations = {r['step_id']: r for r in map(json.loads, (trial/'observations.jsonl').read_text().splitlines()) if 'current_pose' in r}
    predictions = list(map(json.loads, (trial/'predictions.jsonl').read_text().splitlines()))
    output.mkdir(parents=True, exist_ok=False)
    rows = []
    for p in predictions:
        if p['step_id'] not in observations:
            continue
        current = observations[p['step_id']]['current_pose']
        raw = p['raw_action']
        row = dict(step_id=p['step_id'], raw_action=raw, physical_commands=0)
        try:
            if len(raw) != 7 or not all(math.isfinite(v) for v in raw):
                raise ValueError('invalid_model_action')
            if sum(v*v for v in raw[:3])**0.5 > limits['translation_step_m'] + 1e-12:
                raise ValueError('existing_raw_translation_gate')
            if sum(v*v for v in raw[3:6])**0.5 > math.radians(limits['rotation_step_deg']) + 1e-12:
                raise ValueError('existing_rotation_gate')
            target = [current[i]+1000*raw[i] for i in range(3)]
            target += (Rotation.from_rotvec(raw[3:6])*Rotation.from_euler('ZYZ', current[3:], degrees=True)).as_euler('ZYZ', degrees=True).tolist()
            # Each sample is an independent first-tick calculation; not a robot
            # dynamics simulation or an asserted multi-step closed-loop replay.
            c = SpeedLShadowController(limits)
            row.update(current_pose=current, target_pose=target,
                       candidate=c.candidate(current, target, now=0.0, source_time=0.0))
        except (ValueError, RuntimeError) as exc:
            row['rejection'] = str(exc)
        rows.append(row)
    summary = dict(status='OFFLINE_CALCULATION_COMPLETE' if rows else 'NO_MATCHED_SAMPLES',
        samples=len(rows), candidate_count=sum('candidate' in r for r in rows),
        rejected_count=sum('rejection' in r for r in rows), physical_commands=0,
        physical_rollout_ready=False, server_requests=0, gripper_commands=0,
        joint_limits_changed=False, production_launchers_changed=False,
        source_trial=str(trial.resolve()), existing_limits=limits,
        limitation='Independent recorded first ticks; no IK, live frame/feedback/stop validation or physical motion simulation.')
    (output/'candidates.jsonl').write_text(''.join(json.dumps(r, allow_nan=False)+'\n' for r in rows))
    (output/'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trial', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    run(args.trial, args.output)
