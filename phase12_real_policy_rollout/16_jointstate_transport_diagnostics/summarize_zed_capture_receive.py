"""Summarize passive ZED evidence; descriptive correlations, no causal verdict."""
import json,re,sys,statistics
from pathlib import Path
p=Path(sys.argv[1])
def rows(name):return [json.loads(x) for x in (p/name).read_text().splitlines()]
images=rows('images.jsonl');infos=rows('camera_info.jsonl');diag=rows('diagnostics.jsonl');host=rows('host.jsonl')
rates=[];processing=[];targets=[];drops=[];statuses=[]
for record in diag:
    for s in record['statuses']:
        statuses.append(dict(receive_wall=record['receive_wall'],level=s['level'],message=s['message']))
        for v in s['values']:
            text=v['value'];key=v['key']
            if key.startswith('Data Capture'):
                a=re.search(r'Mean Frequency:\s*([\d.]+) Hz',text)
                if a:rates.append(float(a[1]))
                a=re.search(r'Tot\. Processing Time:\s*([\d.]+) sec',text)
                if a:processing.append(float(a[1]))
            if key.startswith('Camera Grab rate'):
                a=re.search(r'([\d.]+) Hz',text)
                if a:targets.append(float(a[1]))
            if key=='Frame Drop rate':drops.append(dict(wall=record['receive_wall'],text=text))
def stats(a):
    return dict(count=len(a),min=min(a),median=statistics.median(a),mean=statistics.mean(a),max=max(a)) if a else None
events=[]
for a,b in sorted(zip(images,images[1:]),key=lambda ab:ab[1]['receive_monotonic']-ab[0]['receive_monotonic'],reverse=True)[:10]:
    between=[r for r in infos if a['receive_monotonic']<r['receive_monotonic']<=b['receive_monotonic']]
    nearest=min(diag,key=lambda r:abs(r['receive_monotonic']-b['receive_monotonic'])) if diag else None
    events.append(dict(receive_wall=b['receive_wall'],receive_gap_s=b['receive_monotonic']-a['receive_monotonic'],
        published_source_gap_s=b['source_timestamp']-a['source_timestamp'],source_age_at_arrival_s=b['source_age_s'],
        camera_info_callbacks_in_interval=len(between),camera_info_latest_source_in_interval=between[-1]['source_timestamp'] if between else None,
        nearest_diagnostic=nearest))
idle=[]
for a,b in zip(host,host[1:]):
    av=[int(x) for x in a['proc_stat'].split()[1:9]];bv=[int(x) for x in b['proc_stat'].split()[1:9]]
    d=[y-x for x,y in zip(av,bv)];total=sum(d)
    if total>0:idle.append(100*(d[3]+d[4])/total)
result=dict(summary=json.loads((p/'summary.json').read_text()),
    zed_reported_grab_loop_hz=stats(rates),zed_configured_grab_hz=stats(targets),zed_reported_processing_s=stats(processing),
    diagnostic_status_counts={m:sum(s['message']==m for s in statuses) for m in {s['message'] for s in statuses}},
    drop_counters_first_last=[drops[0],drops[-1]] if drops else [],aggregate_cpu_idle_or_iowait_percent=stats(idle),
    image_receive_gap_s=stats([b['receive_monotonic']-a['receive_monotonic'] for a,b in zip(images,images[1:])]),
    image_source_age_s=stats([r['source_age_s'] for r in images]),
    receive_gap_ge100ms_with_source_gap_lt50ms=sum(b['receive_monotonic']-a['receive_monotonic']>=.1 and b['source_timestamp']-a['source_timestamp']<.05 for a,b in zip(images,images[1:])),
    top10_image_receive_gaps=events,
    limitations=['No SDK/physical camera opened. No per-grab timestamp probe inserted into running driver.',
        'SDK diagnostics expose grab-loop moving averages. Image source gaps can arise from capture, processing, publishing or subscriber loss.',
        'CameraInfo is a low-bandwidth comparison on the same passive node, not a second independent image subscriber.',
        'No inference load was added; this is not full rollout runtime validation.'])
(p/'analysis.json').write_text(json.dumps(result,indent=2))
print(json.dumps({k:v for k,v in result.items() if k not in ('summary','top10_image_receive_gaps')},indent=2),flush=True)
