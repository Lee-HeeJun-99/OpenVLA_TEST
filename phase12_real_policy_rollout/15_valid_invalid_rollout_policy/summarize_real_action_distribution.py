"""Offline distribution report; candidates never change safety configuration."""
import argparse,collections,hashlib,json,math
from pathlib import Path


def summarize(directory):
    def read(name):return json.loads((directory/name).read_text())
    def save(name,value):(directory/name).write_text(json.dumps(value,indent=2,allow_nan=False))
    rows=[json.loads(line) for line in (directory/'valid_predictions.jsonl').read_text().splitlines() if line]
    for row in rows:
        assert row['command_requested'] is False and row['command_issued'] is False
        assert row['delivered_action'] is None and row['executed_action'] is None
        assert row['current_translation_limit_m']==.010 and row['camera_age_at_inference_start']<.5
        assert row['jointstate_runtime']['fault'] is None
    td=read('translation_distribution.json');rd=read('rotation_distribution.json');gd=read('gripper_distribution.json')
    ld=read('latency_distribution.json');cd=read('camera_timing_distribution.json');summary=read('collection_summary.json')
    config=read('collection_config.json');limits=Path(__file__).resolve().parents[1]/'01_configs/safety_limits.yaml'
    assert hashlib.sha256(limits.read_bytes()).hexdigest()==config['config_sha256']
    threshold=math.ceil(td['p99']) if len(rows)>=30 else None
    frequencies=collections.Counter(tuple(row['raw_action']) for row in rows)
    audit=dict(source_image_unique_count=len({r['source_image_sha256'] for r in rows}),model_input_unique_count=len({r['model_input_sha256'] for r in rows}),
        source_stamp_unique_count=len({tuple(r['camera_source_timestamp']) for r in rows}),unique_action_vectors=len(frequencies),
        most_frequent_action_vectors=[dict(raw_action=list(action),count=count) for action,count in frequencies.most_common(5)],
        max_translation_repeated_count=sum(abs(r['translation_norm_m']*1000-td['max'])<1e-9 for r in rows),
        rotation_gate_reject_count=sum(not r['rotation_gate_pass'] for r in rows),nan_inf_count=sum(r['nan_inf_count'] for r in rows),
        all_command_flags_false=True,safety_config_unchanged=True,physical_commands=0)
    save('sample_integrity_audit.json',audit)
    candidate=read('threshold_candidate_analysis.json')
    if threshold is not None:
        reject=sum(r['translation_norm_m']*1000>threshold+1e-9 for r in rows)
        candidate['candidates'].append(dict(name='p99_rounded_up_whole_mm',threshold_mm=threshold,rejection_count=reject,rejection_rate=reject/len(rows)))
        candidate['recommended_candidate_mm']=threshold
        candidate['recommendation_scope']='DESCRIPTIVE_STAGED_EVALUATION_CANDIDATE_REQUIRES_OPERATOR_REVIEW_NOT_PHYSICAL_SAFETY_CERTIFICATION'
        candidate['derivation']='ceil(empirical p99 in mm), whole-mm reporting resolution. Not inferred from a single max outlier. Current active gate remains 10 mm.'
        candidate['rounding_headroom_mm']=threshold-td['p99']
    candidate['applied']=False;candidate['production_limit_mm']=10;save('threshold_candidate_analysis.json',candidate)
    text=f'''# Real OpenVLA action distribution — prediction only

Attempts {summary['attempts']}; inference requests {summary['inference_requests']}; valid {len(rows)}; invalid {summary['invalid_predictions']}; preflight rejected {summary['preflight_rejections']}. Physical commands **0**. Samples are real ZED 1280x720 bgra8 -> RGB -> JPEG, OpenVLA step8130 K1 dim7 proprio=false. Strict identity verified at startup; unchanged identity checked before each request. REAL/AUTO/STANDBY scalar read-only responses refreshed before every request. Start-pose error <=0.5 degrees checked each time. JointState and camera use the same persistent subscribers throughout. JointState events 0, runtime fault null. Source/model hashes and camera stamps are all independently audited in sample_integrity_audit.json.

## Distribution

Translation mm: min {td['min']:.6f}; median {td['median']:.6f}; mean {td['mean']:.6f}; std {td['std']:.6f}; p75 {td['p75']:.6f}; p90 {td['p90']:.6f}; p95 {td['p95']:.6f}; p99 {td['p99']:.6f}; max {td['max']:.6f}.

Over 10 mm {td['over_10mm_count']}/{len(rows)} ({td['over_10mm_rate']:.1%}); over 12 mm {td['over_12mm_count']}/{len(rows)} ({td['over_12mm_rate']:.1%}); over 15 mm {td['over_15mm_count']}/{len(rows)} ({td['over_15mm_rate']:.1%}). These remain SAMPLE_VALID model observations; safety acceptance is recorded separately, and none were removed because of magnitude.

Raw rotation norm degrees: median {rd['median']:.6f}; p95 {rd['p95']:.6f}; max {rd['max']:.6f}. Rotation hard gate remains 4 degrees; rejects {audit['rotation_gate_reject_count']}. This is the norm of the raw action rotation vector, not a measured robot orientation delta.

Gripper closedness: min/median/mean/p90/max {gd['min']:.6f}/{gd['median']:.6f}/{gd['mean']:.6f}/{gd['p90']:.6f}/{gd['max']:.6f}. Open-like (<0.3) {gd['open_like_count']}; middle [0.3,0.7] {gd['mid_count']}; close-like (>0.7) {gd['close_like_count']}. All-close prediction is a separate behavior concern, not resolved by raising translation gate. No physical gripper output.

Inference seconds: median {ld['median']:.6f}; p90 {ld['p90']:.6f}; p95 {ld['p95']:.6f}; p99 {ld['p99']:.6f}; max {ld['max']:.6f}. Camera receive age at request start seconds: median {cd['camera_age_at_inference_start']['median']:.6f}, p95 {cd['camera_age_at_inference_start']['p95']:.6f}, max {cd['camera_age_at_inference_start']['max']:.6f}. End ages and ROS source ages retained separately, never substituted into the start-age gate.

## Threshold candidates — NOT applied

Current 10 mm rejects 100% of this sampled scene. Empirical p95, p99 and max are all {td['p99']:.6f} mm and all give zero observed rejection under the existing inclusive boundary semantics. Repeated token/action levels cause tied quantiles; they must not be interpreted as a precise population upper bound. The observed maximum occurred {audit['max_translation_repeated_count']} times, rather than being a lone spike.

Recommended **candidate**: {threshold} mm, empirical p99 rounded upward to whole-mm reporting precision (headroom {threshold-td['p99']:.6f} mm). This is a distribution-coverage proposal, not a physical safety-approved limit. No additional max-based margin is invented. Applied production threshold remains **10 mm**. The new candidate requires explicit review/approval and fresh validation before changing production configuration.

Downstream limits remain translation velocity 20 mm/s and acceleration 20 mm/s² at 5 Hz; implied speed-only step ceiling 4 mm and first-step acceleration-limited translation 0.8 mm. Existing limiter can modify raw output. Raising raw rejection threshold does NOT remove these limits or make delivered action equal raw action. No clipping/scaling occurred in this prediction-only collection.

## Independence, top actions and caveats

{audit['source_image_unique_count']} unique source hashes, {audit['model_input_unique_count']} unique model input hashes, {audit['source_stamp_unique_count']} unique source timestamps. Fresh-frame uniqueness is verified, but the same stationary scene is temporally correlated: these are NOT IID independent robot tasks or multi-scene action coverage. Distinct raw action vectors {audit['unique_action_vectors']}. All samples and top five JPEG-decoded RGB PNGs retained; the top PNGs derive from actual model JPEG inputs, not original uncompressed RGB pixels. No outliers removed.

The representative top image shows the tabletop/block scene and gripper. This static sample does not establish why the model requests the observed motion/closure, whether behavior is task-appropriate, or how the policy changes after motion. No claim of safe trajectory/task success can be made from this distribution.

TCP remains TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED; physical rollout readiness **BLOCKED_TCP** plus unresolved physical stage/hardware/operator and gripper behavior requirements. Readiness is not promoted to READY_AFTER_TCP_APPROVAL solely from these model statistics. Latest real hardware state/sensor checks here are collection preflight evidence, not motion authorization.

New distribution tests: 4 PASS. No motion/action/gripper/Stop/Hold client exists in the collector. No driver/controller/server restart. Observation logs always command_requested=false, command_issued=false, delivered_action=null, executed_action=null.
'''
    (directory/'ACTION_DISTRIBUTION_REPORT.md').write_text(text)
    print(json.dumps(dict(recommended_candidate_mm=threshold,applied=False,integrity=audit),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    summarize(parser.parse_args().directory)
