import json
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
    with path.open('x') as f:json.dump({'status':'PASS' if passed and not dry_run else 'DRY_RUN_PASS' if passed else 'FAIL',
        'dry_run':dry_run,'identity':identity,'details':details},f,indent=2)
    return path
