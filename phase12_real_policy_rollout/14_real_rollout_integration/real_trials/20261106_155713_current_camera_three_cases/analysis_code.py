import json, re, statistics, shutil
from pathlib import Path
p=Path('/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261106_155713_current_camera_three_cases')
result={'physical_commands':0,'existing_processes_restarted':False,'configuration_changed':False,'cases':[]}
for d in [p/'A_camera_server_idle',p/'B_camera_repeated_inference',p/'C_inference_action_preparation_no_dispatch']:
    rows=[json.loads(s) for s in (d/'images.jsonl').read_text().splitlines()]
    diags=[json.loads(s) for s in (d/'diagnostics.jsonl').read_text().splitlines()]
    capture=[]; drops=[]
    for r in diags:
        for s in r['statuses']:
            for kv in s['values']:
                if kv['key']=='Data Capture' and 'Mean Frequency:' in kv['value']:
                    capture.append(float(re.search(r'Mean Frequency: ([0-9.]+)',kv['value'])[1]))
                if kv['key']=='Frame Drop rate':
                    m=re.search(r'(\d+)/(\d+)',kv['value'])
                    if m:drops.append(tuple(map(int,m.groups())))
    events=[r for r in rows if 'receive_gap_s' in r]
    pred=[]; failures=[]
    if d.name.startswith('B'):
        data=json.loads((p/(d.name+'_predictions.json')).read_text())
        pred=[r for r in data if 'raw_action' in r]; failures=[r for r in data if 'error' in r]
    if d.name.startswith('C'):
        for f in sorted(p.glob(d.name+'_probe_*/summary.json')):
            s=json.loads(f.read_text());pred.extend(s['samples'])
            if s['status']!='PASS':failures.append({'probe':f.parent.name,'reason':s.get('reason'),'status':s['status']})
    latency=[r.get('timing',r.get('camera_timing',{})).get('inference_latency') for r in pred]
    latency=[v for v in latency if v is not None]
    summary=json.loads((d/'summary.json').read_text())
    result['cases'].append(dict(case=d.name,summary=summary['streams']['image'],
        receive_gap_ge_1s=sum(r['receive_gap_s']>=1 for r in events),source_gap_ge_1s=sum(r['source_gap_s']>=1 for r in events),
        diagnostic_capture_hz_min=min(capture) if capture else None,diagnostic_capture_hz_median=statistics.median(capture) if capture else None,
        frame_drop_counter_first=drops[0] if drops else None,frame_drop_counter_last=drops[-1] if drops else None,
        prediction_count=len(pred),probe_errors=failures,inference_median_s=statistics.median(latency) if latency else None,
        inference_max_s=max(latency) if latency else None))
result['limitations']=['Each stage is one sequential 120s window, not randomized causal evidence.',
 'A retains an idle loaded model server, Doosan driver and normal desktop processes; it is camera-topic-only workload, not SDK-only.',
 'C exercises inference/JPEG/PNG/fsync/read-only robot-state/settle; not complete physical Controller dispatch, motors or gripper.',
 'Data Capture diagnostics are moving aggregates, not per-grab timestamps.',
 'A subscriber sees received image/header gaps, not direct USB packet or SDK-grab traces.']
(p/'analysis.json').write_text(json.dumps(result,indent=2))
shutil.copyfile('/tmp/current_camera_three_cases.py',p/'measurement_code.py')
shutil.copyfile(__file__,p/'analysis_code.py')
print(json.dumps(result,indent=2))
