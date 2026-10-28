import unittest
import numpy as np
from analyze import metrics,aggregate,unit_rgb
from analyze import HERE,read,manifests,arr,FEATURE

class MetricsTests(unittest.TestCase):
    def test_all_five_chunk_steps(self):
        a=np.zeros((5,7));a[4,0]=5;a[4,3]=10;a[4,6]=1
        r=metrics(a,np.zeros_like(a))
        self.assertEqual(r['translation_l2'],1)
        self.assertEqual(r['rotation_l2'],2)
        self.assertEqual(r['gripper_abs'],.2)
        self.assertAlmostEqual(r['action7_l2'],np.sqrt(126)/5)
    def test_squared_components_add_not_l2(self):
        r=metrics(np.ones((5,7)),np.zeros((5,7)))
        self.assertEqual(r['translation_squared']+r['rotation_squared']+r['gripper_squared'],7)
        self.assertNotEqual(r['action7_l2'],r['translation_l2']+r['rotation_l2']+r['gripper_abs'])
    def test_shape_and_finite(self):
        with self.assertRaises(AssertionError):metrics(np.zeros((1,7)),np.zeros((1,7)))
        with self.assertRaises(AssertionError):metrics(np.full((5,7),np.nan),np.zeros((5,7)))
    def test_pair_weighting_not_episode_weighting(self):
        rs=[dict(c='A',translation_l2=x) for x in [0,0,6]]
        self.assertEqual(aggregate(rs,['c'])[0]['translation_l2'],2)
    def test_fixed_inverse_not_image_minmax(self):
        p={'tvf_normalize_params':[dict(mean=[.5]*3,std=[.5]*3)]*2}
        self.assertTrue(np.all(unit_rgb(np.zeros((1,6,224,224)),p)[0]==.5))

@unittest.skipUnless((HERE/'results_001/mean_correction_vectors.npz').exists(),'No completed artifact available')
class ArtifactTests(unittest.TestCase):
    def test_all_replays_exact(self):
        d=read(HERE/'results_001/head_validation.json')
        self.assertTrue(d['passed']);self.assertEqual(d['samples'],1350)
        self.assertEqual(d['max_normalized_abs_error'],0)
        self.assertEqual(d['max_decoded_abs_error'],0)
    def test_all_fold_partitions(self):
        folds=read(HERE/'results_001/folds.json');self.assertEqual(len(folds),10)
        for c in ('A','C'):
            fs=[f for f in folds if f['condition']==c]
            ids=[p for f in fs for p in f['test_pair_ids']]
            self.assertEqual(len(ids),225);self.assertEqual(len(set(ids)),225)
            for f in fs:
                self.assertFalse(set(f['train_pair_ids'])&set(f['test_pair_ids']))
                self.assertEqual(len(f['train_pair_ids'])+len(f['test_pair_ids']),225)
    def test_independently_recompute_train_only_delta(self):
        folds=read(HERE/'results_001/folds.json')
        with np.load(HERE/'results_001/mean_correction_vectors.npz') as z:
            for c in ('A','C'):
                fold=next(f for f in folds if f['condition']==c)
                data=manifests(c);total=np.zeros((35,4096),dtype=np.float64)
                for pid in fold['train_pair_ids']:
                    total+=arr(data['sim'][pid],FEATURE)[0].astype(np.float64)-arr(data['real'][pid],FEATURE)[0].astype(np.float64)
                expected=total/fold['train_count']
                np.testing.assert_allclose(z[c+'_'+fold['held_out_episode']],expected,rtol=0,atol=1e-12)

if __name__=='__main__':unittest.main()
