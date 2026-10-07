"""Offline interpretation of long live audit artifacts; no ROS import or calls."""
import collections, json, math, os, re, sys
from pathlib import Path
from final_end_to_end_readiness import distribution, failure_counts

def summarize(out):
    def read(name):return json.loads((out/(name+'.json')).read_text())
    def save(name,payload):(out/(name+'.json')).write_text(json.dumps(payload,indent=2))
    baseline=read('01_jointstate_180s');shadow=read('06_live_shadow_summary');train=read('08_action_distribution_training')
    rows=read('jointstate_all_samples');host=read('host_stats');runtime=read('22_watchdog_logger');tcp=read('13_tcp_contract')
    predictions=[json.loads(s) for s in (out/'05_live_shadow.jsonl').read_text().splitlines()]
    valid=[p for p in predictions if p.get('valid') and p['segment']=='RUNTIME_SHADOW']
    initial=out/'06_live_shadow_summary_initial_classification.json'
    if not initial.exists():initial.write_text(json.dumps(shadow,indent=2))
    shadow.update(failure_counts([p for p in predictions if p['segment']=='RUNTIME_SHADOW']))
    shadow['successful_predictions']=len(valid)
    shadow['nan_inf_count']=sum(any(not math.isfinite(x) for x in p.get('raw_action',[])) for p in predictions)
    shadow['inference_latency_s']=distribution([p['inference_latency'] for p in valid])
    shadow['safety_accepted']=sum(p.get('safety',{}).get('accepted',False) for p in valid)
    shadow['safety_rejected']=sum(p.get('safety',{}).get('rejected',False) for p in valid)
    shadow['safety_blocker_occurrences']=dict(collections.Counter(reason for p in valid for reason in p.get('safety',{}).get('reason',[])))
    shadow['safety_context_caveat']='TCP is unverified; flange position and placeholder orientation are diagnostic context only. Safety acceptance/task success/motion readiness cannot be inferred from this context.'
    save('06_live_shadow_summary',shadow);save('03_camera_120s',shadow)
    real=[p['translation_mm'] for p in valid]
    comparison=dict(training=train['translation_norm_mm'],real=distribution(real),
        real_xyz_mm={k:distribution([p['raw_action'][i]*1000 for p in valid]) for i,k in enumerate(('dx','dy','dz'))},
        real_max_within_training_observed_range=bool(real) and max(real)<=train['translation_norm_mm']['max'],
        training_hz=sorted(set(e['hz'] for e in train['episodes'])),live_prediction_hz=len(valid)/120,
        instruction=valid[0]['instruction'] if valid else None,
        interpretation='Observed live magnitude is compared descriptively to an unmatched limited training sample, not calibrated physical risk or causality. A tail within this sample does not authorize commands.',
        step_limit=dict(translation_m=.004,rotation_deg=4,category='CONFIGURED_FAIL_CLOSED_RAW_ACTION_STEP_LIMIT',
                       source='02_safety/command_disabled_runtime.py evaluate + 01_configs/safety_limits.yaml',
                       history='74bb158e Initial commit; no independently documented manufacturer or hardware-calibrated derivation found',
                       model_control='Exceeding raw limit remains rejected',deterministic_minimum_motion='Uses independent 0.5 mm delta, never model output; model-tail warning alone is not proof this minimum delta unsafe'))
    save('07_action_distribution_real',comparison)
    correlated=[]
    first_fresh=baseline.get('state',{}).get('first_fresh_sample')
    for row in rows:
        if row.get('source_gap') is None or (0<row['source_gap']<.1 and 0<row['receive_gap']<.1):continue
        nearest=min(host,key=lambda h:abs(h['receive']-row['receive'])) if host else None
        phase_bucket='STARTUP_DISCOVERY_PHASE' if first_fresh is None or row['receive']<first_fresh else ('POST_DISCOVERY_WARMUP' if row['receive']<first_fresh+10 else 'RUNTIME_WINDOW')
        correlated.append(dict(wall_time=row['wall_time'],receive=row['receive'],source_after=row['source'],phase_bucket=phase_bucket,
            source_before=row['source']-row['source_gap'],receive_before=row['receive']-row['receive_gap'],
            source_gap=row['source_gap'],receive_gap=row['receive_gap'],header_age=row['header_age'],phase=row['phase'],
            host_nearest=nearest,camera_callback=row['camera_callback'],http_activity=row['http_activity'],graph_cli_activity=False,
            note='Host snapshots 1 Hz, not sub-100 ms instrumentation; association is not causal proof'))
    save('gap_host_correlation',correlated)
    cpu=[]
    for a,b in zip(host,host[1:]):
        av=list(map(int,a['cpu'].split()[1:]));bv=list(map(int,b['cpu'].split()[1:]));d=[y-x for x,y in zip(av,bv)]
        process_cpu={};dt=b['receive']-a['receive']
        for pid,text in b['processes'].items():
            if pid not in a['processes']:continue
            x=a['processes'][pid].rsplit(')',1)[1].split();y=text.rsplit(')',1)[1].split()
            ticks=(int(y[11])+int(y[12]))-(int(x[11])+int(x[12]))
            process_cpu[pid]=100*ticks/os.sysconf('SC_CLK_TCK')/dt
        total=sum(d[:8]);idle=sum(d[3:5]);cpu.append(dict(wall_time=b['wall_time'],cpu_busy_percent=100*(total-idle)/total if total>0 else None,load=b['load'],per_process_cpu_percent=process_cpu))
    save('host_cpu_intervals',cpu)
    def net(text):
        result={}
        for line in text.splitlines():
            if ':' not in line:continue
            interface,fields=line.split(':',1);fields=list(map(int,fields.split()))
            result[interface.strip()]={k:fields[i] for k,i in [('rx_errors',2),('rx_drops',3),('tx_errors',10),('tx_drops',11)]}
        return result
    net_delta={}
    if host:
        before,after=net(host[0]['network']),net(host[-1]['network'])
        net_delta={i:{k:after[i][k]-before[i][k] for k in after[i]} for i in after if i in before}
    save('host_network_deltas',net_delta)
    driver_log=Path('/home/ubuntu/.ros/log/ros2_control_node_3061204_1791355878269.log')
    lines=driver_log.read_text().splitlines() if driver_log.exists() else []
    driver_events=[]
    for event in correlated:
        if event['phase_bucket']!='RUNTIME_WINDOW':continue
        matches=[]
        for line in lines:
            match=re.search(r'\[(\d{10}\.\d+)\]',line)
            if match and abs(float(match[1])-event['wall_time'])<=1:matches.append(line)
        driver_events.append(dict(event_wall_time=event['wall_time'],source_gap=event['source_gap'],receive_gap=event['receive_gap'],lines_within_1s=matches))
    save('driver_gap_correlation',dict(file=str(driver_log),events=driver_events,event_silence_not_safety_pass=True))
    for name in ('14_robot_mode','15_robot_state','16_connection','17_authority','18_servo','19_protective_stop','20_emergency_stop'):
        state=read(name);state.update(getter_called=False,reason='Current persistent runtime stability/source evidence insufficient for controlled refresh; no inference from STANDBY or event silence')
        if name in ('14_robot_mode','15_robot_state'):
            state['previous_not_current']={'14_robot_mode':'AUTO','15_robot_state':'STANDBY'}[name]
            state['previous_evidence']='20261007_162114_tcp_hardware_precheck, same driver session; not fresh safety PASS'
        save(name,state)
    source=read('11_tcp_sources')
    source.update(controller_config=dict(file='/home/ubuntu/robot_ws/src/doosan-robot2/dsr_controller2/config/dsr_controller2.yaml',use_rt_topic_pub=False),
                  historical_candidate=dict(file='01_configs/pre_rollout_verified_config.yaml',name='Tool_v1',offset=[0]*6,verified_current_binding=False),
                  flange_cache=dict(getter_callback_line=1045,update_callback='OnMonitoringDataExCB',update_line=2940,
                      documented_callback_interval_s=.1,measured_callback_interval='UNKNOWN',timestamp='dSyncTime internal only; no stamp/sequence in getter response',called=False),
                  hardware_note='Monitoring/access-control callbacks cache state internally but no validated current public state samples were obtained; RobotState/RobotStateRt schemas are not live evidence')
    save('11_tcp_sources',source)
    logger_status=json.loads((out/'05_live_shadow.jsonl.status.json').read_text())
    runtime.update(logger_session=logger_status,jsonl_records=len(predictions),unique_frame_ids=len({p['frame_id'] for p in predictions}),
                   watchdog_scope='Independent JointState receive/source watchdog; selected-camera-age fail-closed at each response plus passive camera callback recording. No physical abort path was invoked.')
    save('22_watchdog_logger',runtime)
    final=read('24_final_readiness')
    final['summary']=shadow
    if shadow['model_failures']==0:final['blockers']=[b for b in final['blockers'] if b!='MODEL_FAILURE']
    final['action_analysis']=comparison
    final['watchdog_logger']=runtime
    final['jointstate_classification']='JOINTSTATE_RUNTIME_UNSTABLE' if shadow['jointstate']['status']!='PASS' else ('JOINTSTATE_ROLLOUT_RUNTIME_PASS' if baseline['status']=='PASS' else 'JOINTSTATE_RUNTIME_UNSTABLE')
    final['production_thresholds_changed']=False
    if (out/'tests_result.json').exists():final['tests']=read('tests_result')
    final['software_complete_manual_only']=False
    final['remaining_blockers']=final['blockers']+['MANUAL_SAFETY_CONFIRMATIONS_REQUIRED']
    save('24_final_readiness',final)
    trans=shadow['translation_mm'];rot=shadow['rotation_deg'];temporal=read('10_action_temporal_consistency')
    report=f'''# Final end-to-end readiness — {out.name}

Latest verdict: `{final['verdict']}`. Physical robot commands **0**. Driver/controller not restarted; thresholds unchanged. Minimum-motion, model rollout, gripper, Home, trajectory and physical Hold/Stop not executed.

## JointState

Fixed baseline: {baseline['status']}; {baseline.get('duration_s')} s. Messages {baseline.get('message_count')}; rate {baseline.get('rate_hz')} Hz. Max source/receive gaps {baseline.get('max_source_gap_s')} / {baseline.get('max_receive_gap_s')} s. ≥100 ms source/receive events {baseline.get('source_events_100ms')} / {baseline.get('receive_events_100ms')}. Invalid {baseline.get('invalid')}, missing {baseline.get('missing')}, duplicate {baseline.get('duplicate')}, regression {baseline.get('regression')}.

FIRST_FRESH→10 s warm-up is retained; startup/warm-up events are preserved separately in all samples. Runtime faults latch; no failed window reset. Same subscriber continues into Shadow. Shadow JointState: {shadow['jointstate']['status']}. This is current evidence, not a reused PASS artifact.

## Camera / OpenVLA

Current strict GPU health: MODEL_HEALTH_PASS. OpenVLA step8130, K=1, dim=7, proprio=false. Actual server is RTX3090; no fixture prediction. One first-trial prediction is saved separately (server health initially request_count=0), then 10 s model warm-up, then fixed 120 s Shadow.

Shadow attempts {shadow['predictions']}, successful predictions {len(valid)}; model failures {shadow['model_failures']}. Selected-camera maximum age {shadow['max_selected_age']} s; age-threshold failures {shadow['selected_age_failures']}, input failures {shadow['input_failures']}, total camera failures {shadow['camera_failure']}; camera {'PASS' if shadow['camera_pass'] else 'FAIL'}. A missing ≤10 ms fresh snapshot is a camera input failure, not a GPU inference failure; initial automatic aggregate is preserved separately. BGRA8→RGB uses actual encoding/row stride then JPEG95→processor. No heavy health, graph/CLI, SHA or image disk writes before inference; background logger saves image/hash after response. Selected-age includes inference latency, not just snapshot age.

Translation median/p95/max: {trans.get('median')} / {trans.get('p95')} / {trans.get('max')} mm. Rotation max {rot.get('max')} deg. ≥0.7 close candidates {shadow['close_candidates']}. >4 mm {shadow['over_4mm']}; >4 deg {shadow['over_4deg']}. Outlier records retain raw vector, frame image, JointState, timestamps and latency. Model failures include any nonfinite/schema failure and are not hidden.

## Action distribution interpretation

Training: {train['unique_episodes']} unique episodes / {train['steps']} steps, 10 Hz, 10 corrective +1 nominal. Translation median/p95/p99/max: {train['translation_norm_mm']['median']} / {train['translation_norm_mm']['p95']} / {train['translation_norm_mm']['p99']} / {train['translation_norm_mm']['max']} mm. Live max inside observed training range: {comparison['real_max_within_training_observed_range']}.

4 mm is the configured fail-closed **raw action step acceptance limit**, not a manufacturer physical limit and not merely a diagnostic plot threshold. An action around 4.6 mm is not outside the observed training range, but remains rejected by current command safety. No threshold tuning performed. Training sample checkpoint membership is unverified; differing frequency, unmatched views and phases forbid causal/distribution-equivalence claims or direct implied-velocity comparison. Minimum-motion uses a separate deterministic 0.5 mm action, never model output.

Temporal adjacent translation Δaction statistics: {json.dumps(temporal['delta_translation_mm'])}. >4 mm runs: {temporal['over_4mm_runs']}; isolated runs: {temporal['isolated_runs']}. Images are retained; no visual causal claim made from action timing alone. No physical gripper output; training close timings are descriptive, not a current physical phase.

## TCP / hardware

TCP classification: TCP_FLANGE_ONLY. Active TCP/tool UNKNOWN; offset/current TCP unverified. Existing empty-name getters not repeated. FK is base_link→link_6 only, never measured TCP. Tool_v1 zero offset is historical, not current binding.

RobotState/RobotStateRt schemas contain relevant fields but current graph produced no verified live samples. Optional RT publisher configuration is disabled; no parameter mutation/RT start was performed. Flange cache getter lacks public timestamp/sequence: no freshness proof, not called. Pointer-risk current Cartesian and LastAlarm getters not called. No getter refresh without stable current JointState evidence.

Mode/state/system prior AUTO/STANDBY/REAL remain timestamped previous evidence only. Current connection/authority/servo/protective stop/E-stop UNKNOWN. STANDBY and event silence do not establish those states. Gripper physical state UNKNOWN; output and pulse capability absent in this audit.

## Runtime / research logs / manual gates

Independent JointState watchdog: {runtime['watchdog_status']}. Camera failure checked at each response; this measurement does not validate a physical abort/stop response. Logger: {runtime['logger_status']}; session {logger_status['state']}; records {len(predictions)}. Same sensors persist; failures do not authorize motion. Host sampled at 1 Hz with CPU/network/process counters; gap correlation is descriptive, no unsupported root-cause attribution. No runtime ROS graph/CLI polling.

Image IDs, source/model JPEG hashes, timestamps, instruction, action, JointState and latency retained. Observation Gap join fields null where unavailable; no invented matched pairs or phase labels.

Operator presence, workspace physically clear and physical E-stop reachable: MANUAL_CONFIRMATION_REQUIRED. They are not the only blockers.

## Final gate

Remaining blockers: {', '.join(final['remaining_blockers'])}.

Next allowed stage: BLOCKED. No minimum-motion execution; no short-horizon/full-task execution. Resolve current stream/latency and verified TCP/hardware evidence, then obtain manual confirmations and re-evaluate fresh gates. Do not bypass 100 ms JointState /0.5 s camera thresholds.
'''
    (out/'FINAL_END_TO_END_READINESS_REPORT.md').write_text(report)
    print(json.dumps({'trial':str(out),'final':final['verdict'],'baseline':baseline,'shadow':shadow},indent=2))

if __name__=='__main__':summarize(Path(sys.argv[1]))
