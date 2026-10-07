"""Task denominator excludes INVALID; invalid reliability is a separate metric."""
from collections import Counter
from rollout_trial_classifier import STATUSES

def summarize_trials(trials):
    ids=set();counts=Counter();reasons=Counter()
    if len({t.get('evaluation_scope','task') for t in trials})>1:raise ValueError('mixed_evaluation_scopes_forbidden')
    for trial in trials:
        if trial['trial_id'] in ids:raise ValueError('duplicate_trial_id')
        ids.add(trial['trial_id']);status=trial['status']
        if status not in STATUSES:raise ValueError('nonterminal_trial_status')
        if status=='INVALID':
            if trial.get('runtime_valid') is not False or trial.get('task_success') is not None:raise ValueError('invalid_trial_cannot_have_task_outcome')
            reasons[trial.get('invalid_category') or trial.get('invalid_reason') or 'INVALID_RUNTIME_ERROR']+=1
        elif trial.get('runtime_valid') is not True or trial.get('task_success') is not (status=='SUCCESS'):
            raise ValueError('valid_trial_outcome_inconsistent')
        counts[status]+=1
    total=len(trials);valid=counts['SUCCESS']+counts['FAILURE']
    return dict(total_attempts=total,valid_trials=valid,invalid_trials=counts['INVALID'],success_trials=counts['SUCCESS'],failure_trials=counts['FAILURE'],
                task_success_rate=counts['SUCCESS']/valid if valid else None,invalid_rate=counts['INVALID']/total if total else None,invalid_reason_counts=dict(reasons))

def performance_groups(trials):
    summarize_trials(trials)
    return {name:[t for t in trials if t['status']==name and t.get('evaluation_scope','task')=='task'] for name in ('SUCCESS','FAILURE')}

class RolloutBatch:
    def __init__(self,target_valid_rollouts,max_attempts,*,physical=False):
        if not 0<target_valid_rollouts<=max_attempts:raise ValueError('invalid_batch_bounds')
        self.target=target_valid_rollouts;self.maximum=max_attempts;self.physical=physical

    def run(self,prepare,execute,output):
        import json
        from pathlib import Path
        from rollout_trial_classifier import RolloutTrial
        output=Path(output);output.mkdir(parents=True,exist_ok=False);trials=[];stop=None;pretrial_blocks=[]
        if self.physical:raise PermissionError('physical_batch_not_authorized_requires_validated_recovery_and_operator_reset')
        for attempt in range(1,self.maximum+1):
            if sum(t['status']!='INVALID' for t in trials)>=self.target:break
            trial=RolloutTrial(attempt,evaluation_scope='simulation_task_fixture')
            directory=output/f'trial_{attempt:03d}'
            try:
                ready=prepare(attempt)
                if ready.get('generation')!=attempt:raise PermissionError('prior_readiness_cannot_be_reused')
                trial.start(ready)
                outcome=execute(trial)
                if trial.status is None:trial.finish(outcome,persist=lambda summary:trial.save(directory,summary=summary))
            except Exception as exc:
                if trial.started is None:
                    blocked=dict(attempt=attempt,reason=str(exc),task_status=None,task_success=None)
                    directory.mkdir(parents=True,exist_ok=True)
                    (directory/'pretrial_blocked.json').write_text(json.dumps(blocked,indent=2))
                    pretrial_blocks.append(blocked);stop='PRETRIAL_READINESS_BLOCKED';break
                trial.invalidate(str(exc))
            trials.append(trial.summary())
            try:
                if trial.status=='INVALID':trial.save(directory)
            except Exception as exc:
                stop='ARTIFACT_PERSISTENCE_FAILURE:'+str(exc);break
        summary=summarize_trials(trials)
        reached=summary['valid_trials']>=self.target
        summary.update(stop_reason=stop,trials=trials,pretrial_blocks=pretrial_blocks,physical_commands=0,evaluation_scope='simulation_task_fixture',
                       target_valid_rollouts=self.target,max_attempts=self.maximum,target_reached=reached,
                       termination_reason=stop or ('TARGET_REACHED' if reached else 'MAX_ATTEMPTS_REACHED'))
        (output/'batch_summary.json').write_text(json.dumps(summary,indent=2));return summary
