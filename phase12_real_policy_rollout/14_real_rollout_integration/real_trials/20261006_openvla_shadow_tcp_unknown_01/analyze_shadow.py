import json,math,statistics
from pathlib import Path
ROOT=Path(__file__).resolve().parent
records=[json.loads(s) for s in (ROOT/'live_shadow.jsonl').read_text().splitlines()]
rows=[r for r in records if r['event']=='PREDICTION']
summary=json.loads((ROOT/'live_shadow_summary.json').read_text())
def stats(values):
    s=sorted(values)
    return dict(mean=statistics.mean(s),median=statistics.median(s),p95=s[math.ceil(.95*len(s))-1],max=max(s),min=min(s))
summary['distributions']={k:stats([r[k] for r in rows]) for k in ('translation_norm_m','rotation_norm_deg','gripper_closedness','inference_latency_s','end_to_end_latency_s','camera_age_s','jointstate_age_s')}
summary['model_inference_valid']=all(all(math.isfinite(v) for v in list(r['canonical_action']['translation_m'])+list(r['canonical_action']['rotation_rotvec_rad'])+[r['gripper_closedness']]) for r in rows)
summary['close_candidate_count']=sum(r['gripper_closedness']>=.7 for r in rows)
summary['physical_command_invariant_pass']=all(not r.get('command_issued',False) and r.get('executed_action') is None and r.get('robot_delivered_command') is None for r in records)
if summary['blockers'].get('camera_failure',0):
    summary['status']='LIVE_SHADOW_FAIL'
    summary['freshness_finding']='Selected model input aged beyond existing 0.5s safety threshold on 9 responses; latest camera stream watchdog remained fresh. No threshold was relaxed.'
(ROOT/'live_shadow_summary.json').write_text(json.dumps(summary,indent=2))
table='\n'.join(f"| {k} | {v['median']:.6f} | {v['p95']:.6f} | {v['max']:.6f} |" for k,v in summary['distributions'].items())
(ROOT/'LIVE_SHADOW_REPORT.md').write_text(f'''# OpenVLA prediction-only Shadow with unknown TCP

Verdict: {summary['status']}. Motion unauthorized; minimum/short/full protocols NOT_AUTHORIZED. This run is diagnostic inference, not a physical rollout or successful task episode.

Actual duration {summary['shadow_duration_s']:.3f}s; predictions {len(rows)}. Starting JointState recent20s gate PASS:2000 samples,99.999Hz,max source gap11.732ms,max receive gap19.546ms,latest age2.229ms,invalid0. Older samples were retained in jointstate_observed.json and a clean NEW20s interval was required; no recent gap was masked. Independent20ms watchdog monitored camera/Joints throughout HTTP waits; fault=None. Hardware mode/servo/stop/TCP motion readiness was not asserted.

GPU health MODEL_HEALTH_PASS:real OpenVLA step8130,K1,dim7,propriofalse; strict health identity/processor/instruction rechecked for every prediction. No fixture used. Real1280x720 bgra8→RGB via tested runtime helper, JPEGquality95→actual server processor; per-record source/RGB/JPEG hashes and timestamps preserved.

| Metric (units in field name) | Median | p95 | Max |
|---|---:|---:|---:|
{table}

Safety rejection ratio100%. Blockers: {summary['blockers']}. Safety rejected every action because real TCP and initial physical gripper state are unknown. Current flange is FK_ESTIMATED_FLANGE only; tool_offset_verified=false,tcp_contract_verified=false,not measured TCP. No zero-offset TCP approval was invented. Pipeline flange-relative numerical context is explicitly diagnostic; absolute TCP/workspace/gripper-phase conclusions cannot be drawn. phase=unknown, not a fabricated grasp phase. TCP=false in every inspection, although primary technical blocker camera_failure can hide tcp_failure on a row.

Camera age exceeded0.5s at inference response on9 predictions. This is selected-frame age, not a camera-stream watchdog disconnect; inference latency plus image conversion/health/network delay contributed to selected input age. Existing SafetyPipeline rejected these rows. No camera timeout, inference timeout or safety limit was raised. That known input freshness failure prevents LIVE_SHADOW_PASS even apart from TCP uncertainty.

Action magnitude step threshold exceedances0 (4mm/4deg); NaN/Inf0. Close candidates>=0.7: {summary['close_candidate_count']}/{len(rows)}. Closedness and phase-independent prediction behavior are descriptive only. Rejection ratio is not task success rate and unknown-state rejects do not prove dangerous model behavior. Sequence61 predictions/terminal record is saved with logger flush/fsync and command invariant validation.

Prediction-only inference is valid because OpenVLA is vision-only/proprio-disabled. However, TCP-dependent motion/safety validation remains incomplete because the active tool transform is unknown.

Physical command invariant: {summary['physical_command_invariant_pass']}. All records command_requested=false,command_issued=false,delivered_action=null,executed_action=null,robot_delivered_command=null. NullCommandSink only; no service/action/client was created in this diagnostic module. Motion/gripper/Home/trajectory/Hold/Stop/real rollout calls0.

Executed CLI: run_real_rollout.py --dry-run --live --model openvla --protocol short_horizon --prediction-only-flange --output real_trials/20261006_openvla_shadow_tcp_unknown_01/live_shadow.jsonl. This dedicated diagnostic option requires all four flags and bypasses NO motion safety: it refuses non-dry mode before ROS initialization and never enters the real runner. It changes diagnostic duration only, not the2sec physical short-horizon protocol.

Tests:14_real_rollout_integration unittest suite36PASS,0FAIL (includes4 new diagnostic gate tests). Initial test import-path error was corrected and rerun; no live command occurred. Production configs/thresholds/watchdog unchanged.

Next allowed stage: TCP_CROSSCHECK_REQUIRED_BEFORE_MOTION, plus selected-input freshness validation. Minimum motion remains blocked. No approval artifact created. Observation Gap join fields are retained as null; no experiment provenance fabricated.
''')
print(json.dumps(summary,indent=2))
