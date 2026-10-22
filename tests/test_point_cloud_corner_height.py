import unittest
import numpy as np
from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as p


class LocalCornerHeightTests(unittest.TestCase):
    def estimator(self):
        self.assertTrue(hasattr(p,'estimate_corner_heights'),'Missing local height estimation')
        return p.estimate_corner_heights

    def test_long_warped_deck_uses_local_surface_not_global_plane(self):
        estimate=self.estimator()
        x,y=np.meshgrid(np.linspace(0,2400,100),np.linspace(0,12000,400))
        z=1300+.01*x+.002*y+120*((y-6000)/6000)**2
        xyz=np.column_stack([x.ravel(),y.ravel(),z.ravel()])
        corners=np.array([[0,0,1340],[2400,0,1364],[0,12000,1364],[2400,12000,1388]],float)
        corrected,reports=estimate(xyz,corners)
        expected=np.array([1420,1444,1444,1468])
        np.testing.assert_array_equal(corrected[:,:2],corners[:,:2])
        np.testing.assert_allclose(corrected[:,2],expected,atol=3,rtol=0)
        self.assertTrue(all(v['status']=='local_plane' for v in reports))

    def test_high_outliers_do_not_raise_height_or_remove_real_high_deck(self):
        estimate=self.estimator();rng=np.random.default_rng(4)
        xy=rng.uniform((0,0),(600,600),(2400,2))
        z=2100+.04*xy[:,0]-.025*xy[:,1]+rng.normal(0,2,len(xy))
        points=np.column_stack([xy,z])
        noise=np.column_stack([rng.uniform((0,0),(600,600),(700,2)),rng.uniform(2250,2600,700)])
        source=np.vstack([points,noise]);corner=np.array([[0,0,2000]],float)
        result,reports=estimate(source,corner)
        self.assertAlmostEqual(result[0,2],2100,delta=3)
        self.assertGreater(reports[0]['rejected_high_points'],0)
        perm=rng.permutation(len(source));again,_=estimate(source[perm],corner)
        np.testing.assert_array_equal(result,again)

    def test_sparse_support_is_explicit_fallback_not_claimed_local_height(self):
        estimate=self.estimator();corner=np.array([[0.,0.,1234.]])
        result,reports=estimate(np.array([[0,0,1400],[1,1,1400]],float),corner)
        np.testing.assert_array_equal(result,corner)
        self.assertEqual(reports[0]['status'],'global_plane_fallback')

    def test_collinear_points_do_not_define_a_local_surface(self):
        estimate=self.estimator();x=np.linspace(0,500,300)
        xyz=np.column_stack([x,np.zeros_like(x),1400+.1*x])
        _,reports=estimate(xyz,np.array([[0,0,1300]],float))
        self.assertEqual(reports[0]['status'],'global_plane_fallback')

    def test_dense_high_noise_cannot_claim_a_supported_surface_from_tiny_consensus(self):
        estimate=self.estimator();rng=np.random.default_rng(8)
        xy=rng.uniform((0,0),(400,400),(5000,2))
        noise=np.column_stack([xy,rng.uniform(1550,2350,5000)])
        rng=np.random.default_rng(9)
        board=np.column_stack([rng.uniform((0,0),(400,400),(100,2)),np.full(100,1400.)])
        corner=np.array([[0.,0.,1400.]])
        corrected,reports=estimate(np.vstack([board,noise]),corner)
        self.assertEqual(reports[0]['status'],'global_plane_fallback')
        np.testing.assert_array_equal(corrected,corner)

    def test_service_keeps_explicit_unreliable_height_status(self):
        from services.point_cloud_processing_service import PointCloudProcessingService
        board=p.compute_geometry_result(np.array([[0,0,1400],[2000,0,1400],[0,8000,1400],[2000,8000,1400]]))
        board['corner_height_all_supported']=False
        board['corner_height_estimation']=[{'status':'global_plane_fallback'}]*4
        normalized=PointCloudProcessingService._normalize_board('label_2',board)
        self.assertIs(normalized['corner_height_all_supported'],False)
        self.assertEqual(normalized['corner_height_estimation'],board['corner_height_estimation'])

if __name__=='__main__':unittest.main()
