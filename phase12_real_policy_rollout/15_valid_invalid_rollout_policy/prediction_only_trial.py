"""Runtime-only completion: never fabricate a physical task outcome."""
import json
import os
from pathlib import Path
from rollout_trial_classifier import RolloutTrial, json_safe


class PredictionOnlyTrial(RolloutTrial):
    def __init__(self, trial_id, **kwargs):
        super().__init__(trial_id, evaluation_scope='prediction_only', **kwargs)
        self.runtime_completed = False

    def command_allowed(self):
        return False

    def start(self, readiness):
        super().start(readiness)
        self.commands_inhibited = True

    def invalidate(self, *args, **kwargs):
        with self.lock:
            if self.runtime_completed:
                return self.summary()
            return super().invalidate(*args, **kwargs)

    def observe(self, observation):
        with self.lock:
            return False if self.runtime_completed else super().observe(observation)

    def prediction(self, *args, **kwargs):
        with self.lock:
            return False if self.runtime_completed else super().prediction(*args, **kwargs)

    def finish(self, *args, **kwargs):
        raise PermissionError('prediction_only_cannot_assign_task_outcome')

    def finish_runtime(self):
        with self.lock:
            if self.status is None:
                self.ended = self.clock()
                self.runtime_completed = True
                self.commands_inhibited = True
            return self.summary()

    def summary(self):
        result = super().summary()
        result['status'] = 'INVALID' if self.status == 'INVALID' else None
        result['runtime_status'] = ('INVALID' if self.status == 'INVALID' else
                                    'VALID' if self.runtime_completed else 'IN_PROGRESS')
        result['task_success'] = None
        result['motion_authorized'] = False
        return result

    def save(self, directory, **kwargs):
        with self.lock:
            if not (self.runtime_completed or self.status == 'INVALID'):
                raise RuntimeError('runtime_trial_not_terminal')
            directory = Path(directory)
            directory.mkdir(parents=True, exist_ok=True)
            for name, value in [('trial_summary.json', self.summary()),
                                ('runtime_events.json', self.runtime_events),
                                ('invalid_event.json', self.invalid_event)]:
                temporary = directory / (name + '.tmp')
                with temporary.open('w') as file:
                    json.dump(json_safe(value), file, allow_nan=False, indent=2)
                    file.flush(); os.fsync(file.fileno())
                os.replace(temporary, directory / name)
            for name, rows in [('observations.jsonl', self.observations),
                               ('predictions.jsonl', self.predictions)]:
                with (directory / name).open('w') as file:
                    for row in rows:
                        file.write(json.dumps(json_safe(row), allow_nan=False) + '\n')
                    file.flush(); os.fsync(file.fileno())


def summarize_runtime_trials(trials):
    from collections import Counter
    if len({row['trial_id'] for row in trials}) != len(trials):
        raise ValueError('duplicate_trial_id')
    for row in trials:
        if row.get('evaluation_scope') != 'prediction_only' or row.get('task_success') is not None:
            raise ValueError('runtime_scope_or_task_outcome_mismatch')
        if row['runtime_status'] not in ('VALID', 'INVALID'):
            raise ValueError('nonterminal_runtime_trial')
        if row['runtime_valid'] is not (row['runtime_status'] == 'VALID'):
            raise ValueError('runtime_validity_mismatch')
    invalid = [row for row in trials if row['runtime_status'] == 'INVALID']
    return dict(total_attempts=len(trials), valid_trials=len(trials)-len(invalid),
                invalid_trials=len(invalid), invalid_rate=len(invalid)/len(trials) if trials else None,
                invalid_reason_counts=dict(Counter(row['invalid_reason'] for row in invalid)),
                invalid_category_counts=dict(Counter(row['invalid_category'] for row in invalid)),
                task_success_rate=None, physical_commands=0)
