import csv,json,unittest
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
OUT=HERE/'results_001'
OLD=HERE.parent/'20261028_componentwise_mean_loo/results_001'

class ResultsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with (OUT/'observation_metrics.csv').open() as f:cls.obs=list(csv.DictReader(f))
        with (OUT/'metrics.csv').open() as f:cls.metrics={r['condition']:r for r in csv.DictReader(f)}
        cls.folds=json.loads((OUT/'folds.json').read_text())

    def test_225_matched_pairs_and_pixel_identity(self):
        ids={c:{r['pair_id'] for r in self.obs if r['condition']==c} for c in self.metrics}
        self.assertEqual(len(ids['A']),225)
        for c in ('B','C1','C2'):
            self.assertEqual(ids[c],ids['A'])
            self.assertEqual(sum(r['condition']==c for r in self.obs),225)
        pixel={(r['condition'],r['pair_id']):float(r['pixel_mse']) for r in self.obs}
        for pid in ids['A']:
            self.assertEqual(pixel['A',pid],pixel['C1',pid])
            self.assertEqual(pixel['A',pid],pixel['C2',pid])

    def test_folds_and_opposite_vectors(self):
        self.assertEqual(len(self.folds),5)
        with np.load(OUT/'correction_vectors.npz') as vectors, np.load(OLD/'mean_correction_vectors.npz') as prior:
            all_test=[]
            for fold in self.folds:
                ep=fold['held_out_episode'];train=set(fold['train_pair_ids']);test=set(fold['test_pair_ids'])
                self.assertFalse(train & test)
                self.assertEqual(len(train)+len(test),225)
                self.assertTrue(all(pid.startswith(ep) for pid in test))
                self.assertTrue(all(not pid.startswith(ep) for pid in train))
                np.testing.assert_array_equal(vectors[ep],-prior['A_'+ep])
                all_test+=list(test)
            self.assertEqual(len(all_test),len(set(all_test)))

    def test_reaggregation_and_change_formula(self):
        for c,row in self.metrics.items():
            subset=[r for r in self.obs if r['condition']==c]
            for metric in ('translation_l2','rotation_l2','gripper_abs'):
                measured=np.mean([float(r[metric]) for r in subset])
                self.assertAlmostEqual(measured,float(row[metric]),places=14)
                expected=100*(measured-float(self.metrics['A'][metric]))/float(self.metrics['A'][metric])
                self.assertAlmostEqual(expected,float(row[metric+'_change_percent']),places=11)

if __name__=='__main__':unittest.main()
