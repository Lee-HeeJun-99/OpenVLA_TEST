"""Offline evidence audit: no ROS imports, getters or robot commands."""
import argparse,csv,json,subprocess
from pathlib import Path
from validate_zero_offset_tcp_once import compare

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]


def distribution(values):
    if not values:return {k:None for k in ('min','median','mean','p90','p95','p99','max')}
    import numpy as np
    return dict(min=float(np.min(values)),median=float(np.median(values)),mean=float(np.mean(values)),
                p90=float(np.percentile(values,90)),p95=float(np.percentile(values,95)),p99=float(np.percentile(values,99)),max=float(np.max(values)))


def run(output,current):
    output.mkdir(parents=True,exist_ok=False)
    def save(name,value):(output/name).write_text(json.dumps(value,indent=2,allow_nan=False))
    pattern=r'"(controller_pose|controller_flange_pose|controller_current_posx|actual_tcp_position|current_tool_posx|current_posx|measured_tcp|feedback_pose)"\s*:'
    # Stream through ALL repository JSON/JSONL, including large logs. Do not
    # treat schema mentions or controller interface availability as observations.
    result=subprocess.run(['rg','-l',pattern,str(REPO),'--glob','*.json','--glob','*.jsonl'],capture_output=True,text=True)
    if result.returncode not in (0,1):raise RuntimeError(result.stderr)
    candidates=result.stdout.splitlines()
    exclusions=[dict(source=p,reason='NO_AUDITABLE_SYNCHRONIZED_FK_CONTROLLER_PAIR; interface declarations/null unavailable feedback are not measured samples') for p in candidates if not p.startswith(str(current)) and not p.startswith(str(output))]
    episodes=[]
    inventory=REPO/'phase10_planner_based_shadow_mode/00_audit/real_episode_provenance_audit.csv'
    with inventory.open() as stream:
        provenance=list(csv.DictReader(stream))
    data=REPO.parent/'data/real_world/raw_dataset_oft/episodes'
    for i in range(1,11):
        episode=data/f'episode_{i:06d}';metadata=json.loads((episode/'metadata.json').read_text())
        steps=[json.loads(line) for line in (episode/'steps.jsonl').read_text().splitlines() if line.strip()]
        audit=next((row for row in provenance if row['episode_id']==episode.name),{})
        episodes.append(dict(source=str(episode),step_count=len(steps),planned_pose_samples=metadata.get('planned_pose_samples'),feedback_pose_samples=metadata.get('feedback_pose_samples'),pose_source=audit.get('pose_source'),measured_feedback_available=audit.get('measured_feedback_available'),included=False,reason='PLANNED_COMMANDED_POSE_NOT_MEASURED_CONTROLLER_EE'))
        if metadata.get('feedback_pose_samples',0)>0:raise RuntimeError('new_controller_feedback_requires_explicit_pair_adapter')
    historical_fk=HERE/'real_trials/20261006_1838_openvla_02/fk_live_validation.json'
    fk_report=json.loads(historical_fk.read_text())
    exclusions.append(dict(source=str(historical_fk),jointstate_gate=fk_report.get('jointstate_gate'),reason='FK_ONLY_NO_CONTROLLER_POSE_PAIR'))
    # Only explicit dedicated pair artifacts can supply paired measurements;
    # the current target is held out, never used to set its own tolerance.
    pairfiles=list(REPO.rglob('tcp_pose_comparison.json'))
    independent=[p for p in pairfiles if not p.is_relative_to(current) and not p.is_relative_to(output)]
    if independent:raise RuntimeError('additional_pose_pairs_require_validity_audit:'+str(independent))
    valid=[]
    save('historical_pose_samples.json',dict(valid_samples=valid,valid_independent_sample_count=0,current_sample_held_out=str(current),structured_candidate_files=candidates,excluded_candidates=exclusions,real_episodes=episodes,synthetic_samples_excluded=True,scope='repository JSON/JSONL keyword scan plus actual real episodes 1-10 and historical FK validation'))
    save('pose_error_distribution.json',dict(sample_count=0,translation_error_mm=distribution([]),rotation_error_deg=distribution([]),reason='NO_INDEPENDENT_VALID_HISTORICAL_FK_CONTROLLER_PAIRS'))
    save('pose_tolerance_definition.json',dict(translation_tolerance_mm=None,rotation_tolerance_deg=None,derivation='NOT_DEFINED: no existing official FK/controller pose tolerance found and no independent historical measured pairs. Current test is held out; p99/max undefined.',sample_count=0,dataset_sources=[str(inventory),str(historical_fk)],synthetic_samples_excluded=True,current_sample_excluded_from_derivation=True,status='INSUFFICIENT_EVIDENCE',other_safety_thresholds_changed=False))
    fk=json.loads((current/'fk_flange_pose.json').read_text())
    flange=json.loads((current/'controller_flange_pose.json').read_text())
    posx=json.loads((current/'controller_current_posx.json').read_text())
    errors=compare(fk['position_mm']+fk['orientation_zyz_deg'],posx['pose'])
    between=compare(flange['pose'],posx['pose'])
    save('current_tcp_reclassification.json',dict(classification='TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED',tcp_contract_verified=False,current_error=errors,controller_flange_vs_posx=between,translation_result='INCONCLUSIVE_NO_TOLERANCE',rotation_result='INCONCLUSIVE_NO_TOLERANCE',active_tcp_name_directly_observed=False,verification_is_consistency_based=False,physical_short_horizon_readiness='BLOCKED',physical_commands=0,additional_known_blocker='latest real prediction 12.214 mm exceeds unchanged 10 mm gate'))
    text=f'''# Historical TCP tolerance validation

Independent valid historical FK/controller pose pairs: **0**.

Repository JSON/JSONL keyword scan found {len(candidates)} candidate files. Entries lacking paired measured controller poses cannot become samples. Historical FK validation contains computed flange only. Actual real episode 1-10 metadata/steps were checked: {sum(x['step_count'] for x in episodes)} steps total; these use planned/commanded/interpolated poses and expose no measured EE pose feedback. They are not calibration pairs. Simulation, fixture and unavailable/null feedback are excluded. File-by-file reasons and provenance are in historical_pose_samples.json.

Current target is deliberately held out. Its FK-vs-current-PosX translation error is {errors['translation_error_mm']:.9f} mm and geodesic rotation error {errors['orientation_error_deg']:.9f} degrees, recomputed from matrices using intrinsic ZYZ. Controller flange vs PosX is {between['translation_error_mm']:.9f} mm / {between['orientation_error_deg']:.9f} degrees. These are one stationary observation, not two independent calibration samples. The cached flange getter has no timestamp and ignores ref, so requested BASE alone is not affirmative cache frame proof.

No existing official FK/controller crosscheck tolerance was found in safety config/source audit. Existing joint-angle tolerance, workspace bounds, action limits, numerical floating-point tolerance and simulator FK tests are not TCP pose error tolerances. No threshold was invented. Translation/rotation distribution min/median/mean/p90/p95/p99/max are null because sample_count=0. Recommended p99/max-based derivation cannot be evaluated without a historical population. Current target cannot calibrate a threshold that then validates itself.

Tolerance: **NOT_DEFINED**. Current translation/rotation acceptance: **INCONCLUSIVE**, not numerical FAIL and not PASS. Classification remains TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED; tcp_contract_verified=false. Active TCP name was not directly observed. Absence of a nonzero transform is not positive binding evidence.

Physical short-horizon: BLOCKED. Independently, the latest prediction-only sample (20261014_after_home_check_01) is 12.214 mm > existing 10 mm and predicts gripper close; this analysis does not override those gates or claim old camera/model/hardware checks are current. No ROS process/getter/request/physical command was executed in this analysis.

Next evidence needed: synchronized, independently measured, valid real FK/controller pose pairs with BASE/unit conventions, freshness and clean-runtime provenance, ideally at multiple stationary configurations, and explicit current TCP binding. A candidate tolerance derived from such evidence must be reviewed before being used as a physical dispatch gate.
'''
    (output/'TCP_TOLERANCE_VALIDATION_REPORT.md').write_text(text)
    print(json.dumps(dict(historical_valid_samples=0,real_episode_steps=sum(x['step_count'] for x in episodes),current_translation_error_mm=errors['translation_error_mm'],current_rotation_error_deg=errors['orientation_error_deg'],classification='TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED',physical_commands=0),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--current',type=Path,required=True)
    args=parser.parse_args();run(args.output.resolve(),args.current.resolve())
