import json,os,time
from pathlib import Path

PREVIOUS={'minimum_motion':None,'short_horizon':'minimum_motion','full_task':'short_horizon'}
def require_stage(protocol,directory,identity):
    previous=PREVIOUS[protocol]
    if previous:
        result=json.loads((Path(directory)/f'{previous}_result.json').read_text())
        if result.get('status')!='PASS' or result.get('dry_run') is not False or result.get('identity')!=identity:
            raise PermissionError('previous_stage_hardware_pass_required')

def save_stage(protocol,directory,identity, *, passed,dry_run,details):
    path=Path(directory)/f'{protocol}_result.json'
    if path.exists():raise FileExistsError('stage_result_preserve_existing_trial')
    path.parent.mkdir(parents=True,exist_ok=True)
    events=details.get('command_events',[]);metrics=details.get('metrics',{})
    payload={'status':'PASS' if passed and not dry_run else 'DRY_RUN_PASS' if passed else 'FAIL',
        'stage':protocol,'physical_success':None,'ended_wall_time':time.time(),
        'abort_reason':details.get('abort_reason'),'watchdog_fault':details.get('watchdog_fault'),
        'commands_requested':len(events),'commands_sent':sum(e.get('sent_at') is not None for e in events),
        'commands_acknowledged':sum(e.get('ack_at') is not None for e in events),
        'commands_completed':sum(e.get('state')=='completed' for e in events),
        'safety_rejections':metrics.get('safety_rejection_count',0),
        'model_latency_s':metrics.get('model_latency_s',[]),
        'command_latency_s':metrics.get('command_latency_s',[]),
        'max_lateness_ms':max((r.get('lateness_ms',0) for r in details.get('dispatch_timeline',[])),default=0),
        'dry_run':dry_run,'identity':identity,'details':details}
    with path.open('x') as f:
        json.dump(payload,f,indent=2);f.flush();os.fsync(f.fileno())
    return path
