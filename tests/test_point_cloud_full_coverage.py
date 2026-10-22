import unittest
from types import SimpleNamespace
import numpy as np
import torch
from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as p


class FullCoverageTests(unittest.TestCase):
    def test_fixed_size_groups_cover_every_point_and_keep_full_truck_context(self):
        self.assertTrue(hasattr(p, 'iter_full_coverage_samples'), 'Missing fixed-size direct coverage sampler')
        xyz=np.random.default_rng(7).uniform((0,0,0),(3000,12000,500),(1231,3)).astype(np.float32)
        groups=list(p.iter_full_coverage_samples(xyz,256,42))
        self.assertTrue(all(g.shape==(256,) for g in groups))
        coverage=np.bincount(np.concatenate(groups),minlength=len(xyz))
        self.assertTrue(np.all(coverage>0))
        self.assertTrue(all(np.ptp(xyz[g,1])>10000 for g in groups))
        for a,b in zip(groups,p.iter_full_coverage_samples(xyz,256,42)):
            np.testing.assert_array_equal(a,b)

    def test_small_cloud_is_padded_without_omitting_any_point(self):
        self.assertTrue(hasattr(p, 'iter_full_coverage_samples'), 'Missing coverage sampler')
        xyz=np.arange(51,dtype=np.float32).reshape(-1,3)
        groups=list(p.iter_full_coverage_samples(xyz,64,42))
        self.assertEqual(len(groups),1)
        self.assertEqual(len(groups[0]),64)
        self.assertEqual(set(groups[0]),set(range(17)))

    def test_network_sees_fixed_batches_shared_normalization_and_direct_predictions(self):
        pipe=p.PointCloudPipeline.__new__(p.PointCloudPipeline)
        from dataclasses import replace
        pipe.config=replace(p.PipelineConfig(),pointnet_num_point=64)
        class Probe(torch.nn.Module):
            def __init__(self):super().__init__();self.received=[]
            def forward(self,xyz,cls):
                self.received.append(xyz.detach().clone())
                values=xyz.transpose(1,2)
                labels=(values[:,:,1]>0).long()+2
                scores=torch.full((1,xyz.shape[2],4),-10.)
                scores.scatter_(2,labels.unsqueeze(2),0.)
                return scores,None
        model=Probe()
        pipe.segmenter=SimpleNamespace(model=model,device='cpu',num_votes=1)
        xyz=np.random.default_rng(12).uniform((2000,-4000,1000),(5000,8000,1600),(181,3)).astype(np.float32)
        labels,_,stats=pipe.segment_xyz_full_coverage(xyz)
        self.assertTrue(all(t.shape==(1,3,64) for t in model.received),'Network must never see the dense point count')
        self.assertTrue(stats['all_points_directly_predicted'])
        self.assertGreaterEqual(stats['coverage_min'],1)
        expected=(xyz[:,1]>xyz[:,1].mean()).astype(int)+2
        np.testing.assert_array_equal(labels,expected)
        # Every observed model coordinate is from the same full-cloud normalization.
        order=np.lexsort((xyz[:,2],xyz[:,1],xyz[:,0]));norm=p.pc_normalize(xyz[order].copy())
        actual=torch.cat([v.transpose(1,2)[0] for v in model.received]).numpy()
        for row in actual:self.assertLess(np.linalg.norm(norm-row,axis=1).min(),1e-6)

if __name__=='__main__':unittest.main()
