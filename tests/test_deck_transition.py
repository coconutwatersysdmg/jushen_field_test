import unittest
import numpy as np
from algorithm_modules.point_cloud_segment_module.point_cloud_pipeline import PipelineConfig


class DeckTransitionTests(unittest.TestCase):
    def test_usable_front_follows_full_width_floor_after_support(self):
        from algorithm_modules.point_cloud_segment_module.deck_transition import estimate_usable_front
        u,v=np.meshgrid(np.arange(200.,2000.,15),np.arange(-1230.,1231.,20))
        keep=(v<100)|(u>=430)
        xyz=np.column_stack([u[keep],v[keep],np.full(keep.sum(),1100.)])
        location,quality=estimate_usable_front(xyz,0.,np.zeros(2),np.array([1.,0.]),np.array([0.,1.]),[-1250.,1250.],np.array([0.,0.,1.,-1100.]),PipelineConfig())
        self.assertGreaterEqual(location,420)
        self.assertLess(location,480)
        self.assertGreaterEqual(quality['clear_floor_fraction'],.95)

    def test_sparse_columns_do_not_invent_a_full_width_floor(self):
        from algorithm_modules.point_cloud_segment_module.deck_transition import estimate_usable_front
        from algorithm_modules.point_cloud_segment_module.shared_deck_geometry import UnsupportedGeometry
        u,v=np.meshgrid(np.arange(200.,2000.,15),np.arange(-1200.,-700.,20))
        xyz=np.column_stack([u.ravel(),v.ravel(),np.full(u.size,1100.)])
        with self.assertRaises(UnsupportedGeometry):
            estimate_usable_front(xyz,0.,np.zeros(2),np.array([1.,0.]),np.array([0.,1.]),[-1250.,1250.],np.array([0.,0.,1.,-1100.]),PipelineConfig())

    def test_vertical_face_support_can_confirm_near_coincident_joint(self):
        from algorithm_modules.point_cloud_segment_module.deck_transition import inspect_transition
        v,z=np.meshgrid(np.linspace(-1200,1200,100),np.linspace(1150,1550,30))
        face=np.column_stack([np.full(v.size,5.),v.ravel(),z.ravel()])
        report=inspect_transition(face,0.,20.,np.array([0.,0.,1.,-1600.]),np.array([0.,0.,1.,-1100.]),np.zeros(2),np.array([1.,0.]),np.array([0.,1.]),[-1250.,1250.],PipelineConfig())
        self.assertEqual(report['structure'],'vertical_step')
        self.assertTrue(report['vertical_face_supported'])
        self.assertAlmostEqual(report['shared_transition_u_mm'],5.,delta=1)

    def test_missing_face_does_not_confirm_vertical_step(self):
        from algorithm_modules.point_cloud_segment_module.deck_transition import inspect_transition
        report=inspect_transition(np.zeros((0,3)),0.,20.,np.array([0.,0.,1.,-1600.]),np.array([0.,0.,1.,-1100.]),np.zeros(2),np.array([1.,0.]),np.array([0.,1.]),[-1250.,1250.],PipelineConfig())
        self.assertEqual(report['structure'],'near_coincident_unconfirmed')
        self.assertIsNone(report['shared_transition_u_mm'])

    def test_gap_alone_is_not_confirmed_support_presence(self):
        from algorithm_modules.point_cloud_segment_module.deck_transition import inspect_transition
        report=inspect_transition(np.zeros((0,3)),0.,300.,np.array([0.,0.,1.,-1600.]),np.array([0.,0.,1.,-1100.]),np.zeros(2),np.array([1.,0.]),np.array([0.,1.]),[-1250.,1250.],PipelineConfig())
        self.assertEqual(report['structure'],'separated_usable_start')
        self.assertFalse(report['support_evidence_present'])
        self.assertIsNone(report['shared_transition_u_mm'])

    def test_support_evidence_in_gap_does_not_join_the_two_boundaries(self):
        from algorithm_modules.point_cloud_segment_module.deck_transition import inspect_transition
        u,v=np.meshgrid(np.linspace(80,220,40),np.linspace(-1000,1000,50))
        block=np.column_stack([u.ravel(),v.ravel(),np.full(u.size,1400.)])
        report=inspect_transition(block,0.,300.,np.array([0.,0.,1.,-1600.]),np.array([0.,0.,1.,-1100.]),np.zeros(2),np.array([1.,0.]),np.array([0.,1.]),[-1250.,1250.],PipelineConfig())
        self.assertTrue(report['support_evidence_present'])
        self.assertEqual(report['structure'],'separated_usable_start')
        self.assertIsNone(report['shared_transition_u_mm'])

    def test_near_coincident_horizontal_patch_is_not_a_vertical_riser(self):
        from algorithm_modules.point_cloud_segment_module.deck_transition import inspect_transition
        u,v=np.meshgrid(np.linspace(-30,30,40),np.linspace(-1200,1200,50))
        patch=np.column_stack([u.ravel(),v.ravel(),np.full(u.size,1400.)])
        report=inspect_transition(patch,0.,20.,np.array([0.,0.,1.,-1600.]),np.array([0.,0.,1.,-1100.]),np.zeros(2),np.array([1.,0.]),np.array([0.,1.]),[-1250.,1250.],PipelineConfig())
        self.assertFalse(report['vertical_face_supported'])
        self.assertEqual(report['structure'],'near_coincident_unconfirmed')

if __name__=='__main__':unittest.main()
