import tempfile
import unittest
from pathlib import Path
from prediction_only_trial import PredictionOnlyTrial, summarize_runtime_trials


class PredictionOnlyTests(unittest.TestCase):
    def ready(self):
        return dict(phase='RUNTIME_READY',fault=None,first_fresh_after_rearm=True,warmup_seconds=10)

    def test_valid_has_no_task_outcome(self):
        trial=PredictionOnlyTrial(1);trial.start(self.ready());trial.finish_runtime()
        row=trial.summary()
        self.assertTrue(row['runtime_valid']);self.assertEqual(row['runtime_status'],'VALID')
        self.assertIsNone(row['status']);self.assertIsNone(row['task_success'])
        self.assertFalse(trial.command_allowed())
        with self.assertRaises(PermissionError):trial.finish(True)
        with tempfile.TemporaryDirectory() as directory:trial.save(Path(directory))

    def test_invalid_is_terminal(self):
        trial=PredictionOnlyTrial(1);trial.start(self.ready());trial.invalidate('CAMERA_STALE')
        trial.finish_runtime();self.assertEqual(trial.summary()['runtime_status'],'INVALID')
        self.assertFalse(trial.prediction([0]*7));self.assertIsNone(trial.summary()['task_success'])

    def test_runtime_aggregation_excludes_invalid(self):
        valid=PredictionOnlyTrial(1);valid.start(self.ready());valid.finish_runtime()
        invalid=PredictionOnlyTrial(2);invalid.invalidate('JOINTSTATE_RECEIVE_GAP')
        summary=summarize_runtime_trials([valid.summary(),invalid.summary()])
        self.assertEqual(summary['valid_trials'],1);self.assertEqual(summary['invalid_rate'],.5)
        self.assertIsNone(summary['task_success_rate'])

    def test_fresh_readiness_and_no_reopen(self):
        trial=PredictionOnlyTrial(1)
        with self.assertRaises(PermissionError):trial.start({**self.ready(),'first_fresh_after_rearm':False})
        trial.start(self.ready());trial.finish_runtime();trial.invalidate('CAMERA_STALE')
        self.assertEqual(trial.summary()['runtime_status'],'VALID')
        with self.assertRaises(RuntimeError):trial.start(self.ready())
