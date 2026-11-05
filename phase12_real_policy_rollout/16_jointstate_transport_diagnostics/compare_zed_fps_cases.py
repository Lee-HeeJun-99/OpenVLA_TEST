"""Descriptive comparison of the two passive FPS measurements; no writes to A."""
import json,re,sys
from pathlib import Path
baseline=Path(sys.argv[1]);case=Path(sys.argv[2]);a=json.loads((baseline/'analysis.json').read_text());b=json.loads((case/'measurement/analysis.json').read_text())
settings=json.loads((case/'parameter_comparison.json').read_text())
assert settings['only_fps_changed']
def extract(x):
    img=x['summary']['streams']['image'];info=x['summary']['streams']['camera_info']
    ds=x['drop_counters_first_last'];drops=None
    if len(ds)==2:
        p=[re.match(r'(\d+)/(\d+)',r['text']) for r in ds]
        if all(p):
            d=int(p[1][1])-int(p[0][1]);total=int(p[1][2])-int(p[0][2])
            drops=dict(dropped_increment=d,total_increment=total,interval_rate=d/total if total>0 else None)
    return dict(duration_s=x['summary']['observation_duration_s'],image_count=img['count'],
        configured_fps=x['zed_configured_grab_hz']['median'],reported_grab_loop_hz=x['zed_reported_grab_loop_hz'],
        receiver_hz=img['receive_hz'],max_receive_gap_s=img['max_receive_gap_s'],max_source_gap_s=img['max_source_gap_s'],
        max_source_age_s=img['max_source_age_s'],receive_gaps_ge100ms=img['receive_gap_ge_100ms'],receive_gaps_ge500ms=img['receive_gap_ge_500ms'],
        camera_info_receiver_hz=info['receive_hz'],camera_info_max_receive_gap_s=info['max_receive_gap_s'],
        processing_s=x['zed_reported_processing_s'],cpu_idle=x['aggregate_cpu_idle_or_iowait_percent'],frame_drop_interval=drops,
        long_receive_gap_with_short_source_gap_count=x['receive_gap_ge100ms_with_source_gap_lt50ms'])
result=dict(case_60fps=extract(a),case_30fps=extract(b),parameter_validation=settings,
    physical_commands=0,driver_restarted=False,zed_restart_required_for_readonly_grab_fps=True,
    limitations=['Single before/after comparison, not proof that FPS alone causally explains all differences. ZED restart/time/automatic ISP state may contribute.',
        'Camera is left running at 30 FPS with a trial-specific override; original production YAML is unchanged.',
        'No OpenVLA inference load was added. No image-corruption detector or full physical readiness validation was run.'])
(case/'fps_comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
