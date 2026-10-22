import unittest
from unittest.mock import patch

import numpy as np

from algorithm_modules.point_cloud_segment_module import deck_transition as transition
from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as p
from algorithm_modules.point_cloud_segment_module import shared_deck_geometry as shared
from algorithm_modules.point_cloud_segment_module import transition_support as support


def grid_face(u, z_min=1150., z_max=1550., width=2400.):
    v, z = np.meshgrid(np.linspace(-width/2, width/2, 81), np.linspace(z_min, z_max, 25))
    return np.column_stack([np.full(v.size, u), v.ravel(), z.ravel()])


def floor(start, stop, z):
    u, v = np.meshgrid(np.linspace(start, stop, 81), np.linspace(-1250, 1250, 81))
    return np.column_stack([u.ravel(), v.ravel(), np.full(u.size, z)])


class TransitionSupportTests(unittest.TestCase):
    def detect(self, candidates, low_start=20., extra=None, mode='geometry_evidence'):
        self.assertTrue(hasattr(transition, 'detect_transition_support'), 'Missing measured support classifier')
        high, low = floor(-1800., 0., 1600.), floor(max(0., low_start), 1800., 1100.)
        parts = [high, low, candidates]
        labels = [np.full(len(high), 2), np.full(len(low), 3), np.ones(len(candidates))]
        if extra is not None:
            parts.append(extra); labels.append(np.ones(len(extra)))
        xyz = np.vstack(parts); labels = np.concatenate(labels).astype(np.int32)
        before = xyz.copy()
        result = transition.detect_transition_support(
            xyz, labels, 0., low_start, np.array([0.,0.,1.,-1600.]), np.array([0.,0.,1.,-1100.]),
            np.zeros(2), np.array([1.,0.]), np.array([0.,1.]), [-1250.,1250.], p.PipelineConfig(),
            parameters=support.SupportParameters(decision_mode=mode))
        np.testing.assert_array_equal(xyz, before)
        return result

    def test_observed_vertical_step_and_clear_lower_floor_means_absent(self):
        report = self.detect(grid_face(5.))
        self.assertEqual(report['decision'], 'absent')
        self.assertTrue(report['vertical_riser_supported'])
        self.assertFalse(report['merge_recommended'])

    def test_support_top_and_two_faces_means_present(self):
        candidates = np.vstack([grid_face(0.), floor(20.,300.,1450.), grid_face(300.,1150.,1450.)])
        report = self.detect(candidates, low_start=320.)
        self.assertEqual(report['decision'], 'present')
        self.assertGreaterEqual(report['plane_count'], 2)
        self.assertGreaterEqual(report['normal_direction_count'], 2)
        self.assertTrue(report['merge_recommended'])
        self.assertGreater(report['support_end_u_mm'], 250.)

    def test_one_flat_support_top_is_not_mistaken_for_an_absent_support(self):
        report = self.detect(floor(80.,320.,1400.), low_start=350.)
        self.assertEqual(report['decision'], 'present')
        self.assertFalse(report['vertical_riser_supported'])

    def test_coplanar_high_floor_fragment_alone_does_not_prove_support(self):
        report=self.detect(floor(80.,320.,1600.),low_start=350.)
        self.assertEqual(report['decision'],'present')
        self.assertEqual(report['geometry_decision'],'unknown')

    def test_support_top_at_high_floor_height_requires_observed_rear_face(self):
        candidates=np.vstack([floor(20.,320.,1600.),grid_face(320.,1150.,1550.)])
        report=self.detect(candidates,low_start=350.)
        self.assertEqual(report['decision'],'present')

    def test_connected_local_normals_can_detect_support_when_no_global_plane_is_reliable(self):
        top=floor(80.,400.,1400.)
        top[:,2]+=70*np.sin((top[:,0]-80)/320*np.pi)
        with patch('algorithm_modules.point_cloud_segment_module.transition_support._surface_planes',return_value=[]):
            report=self.detect(top,low_start=430.)
        self.assertEqual(report['decision'],'present')

    def test_missing_label1_defaults_to_present_without_inventing_extent(self):
        report = self.detect(np.empty((0,3)), low_start=350.)
        self.assertEqual(report['decision'], 'present')
        self.assertEqual(report['geometry_decision'],'unknown')
        self.assertFalse(report['merge_recommended'])

    def test_sparse_floating_points_do_not_confirm_support(self):
        noise = np.random.default_rng(15).uniform([80,-1200,1200],[400,1200,1550],(15,3))
        report = self.detect(noise, low_start=350.)
        self.assertEqual(report['decision'], 'present')
        self.assertEqual(report['geometry_decision'],'unknown')

    def test_narrow_vertical_patch_does_not_prove_absence_across_truck(self):
        report = self.detect(grid_face(5., width=160.))
        self.assertEqual(report['decision'], 'present')
        self.assertEqual(report['geometry_decision'],'unknown')

    def test_coarse_label_front_hint_does_not_override_observed_floor_immediately_behind_step(self):
        report=self.detect(grid_face(5.),low_start=180.,extra=floor(0.,180.,1100.))
        self.assertEqual(report['decision'],'absent')

    def test_floor_only_farther_behind_does_not_prove_clear_space_at_joint(self):
        report=self.detect(grid_face(5.),low_start=180.)
        self.assertEqual(report['decision'],'present')
        self.assertEqual(report['geometry_decision'],'unknown')

    def test_front_baffle_is_excluded_despite_having_the_same_label(self):
        plain = self.detect(grid_face(5.))
        with_front = self.detect(grid_face(5.), extra=grid_face(-4000.,1100.,2500.))
        self.assertEqual(plain, with_front)

    def test_detection_is_independent_of_candidate_order(self):
        candidates = np.vstack([grid_face(0.), floor(20.,300.,1450.), grid_face(300.,1150.,1450.)])
        a = self.detect(candidates,320.)
        shuffled = candidates[np.random.default_rng(18).permutation(len(candidates))]
        self.assertEqual(a,self.detect(shuffled,320.))

    def test_support_diagnostic_survives_low_boundary_failure_and_service_normalization(self):
        high, low = floor(-1800.,0.,1600.), floor(320.,5000.,1100.)
        low = np.repeat(low, 5, axis=0)  # Keep a valid low board above 30000 points.
        candidate = np.vstack([grid_face(0.), floor(20.,300.,1450.), grid_face(300.,1150.,1450.)])
        xyz = np.vstack([high,low,candidate])
        labels = np.r_[np.full(len(high),2),np.full(len(low),3),np.ones(len(candidate))]
        from dataclasses import replace
        cfg=replace(p.PipelineConfig(),truck_head_dir=(-1.,0.,0.),edge_plane_iterations=40)
        original=shared._build_board
        def fail_low(points,label,*args,**kwargs):
            if label==3:raise shared.UnsupportedGeometry('synthetic_low_end_failure')
            return original(points,label,*args,**kwargs)
        with patch.object(shared,'_build_board',side_effect=fail_low):
            result,_,_=p.process_segmented_points_timed(xyz,labels,cfg)
        self.assertIn('transition_support',result['label_2'])
        self.assertEqual(result['label_2']['transition_support']['decision'],'present')
        from services.point_cloud_processing_service import PointCloudProcessingService
        normalized=PointCloudProcessingService.normalize_result(result)
        self.assertEqual(normalized['boards'][0]['transition_support'],result['label_2']['transition_support'])

    def test_normal_classification_does_not_require_a_localizable_support_boundary(self):
        report=self.detect(floor(65.,105.,1400.),low_start=350.,mode='normal_concentration')
        self.assertEqual(report['decision'],'absent')
        self.assertEqual(report['geometry_decision'],'unknown')
        self.assertFalse(report['support_extent_available'])
        self.assertFalse(report['merge_recommended'])

    def test_small_height_difference_does_not_override_concentrated_joint_normals(self):
        candidates=grid_face(5.,1161.,1199.)
        report=transition.detect_transition_support(candidates,np.ones(len(candidates),int),0.,100.,
            [0.,0.,1.,-1100.],[0.,0.,1.,-1100.],np.zeros(2),np.array([1.,0.]),np.array([0.,1.]),
            [-1250.,1250.],p.PipelineConfig())
        self.assertEqual(report['high_low_height_difference_mm'],0.)
        self.assertEqual(report['decision'],'absent')
        self.assertGreaterEqual(report['normal_summary']['valid'],50)

    def test_missing_height_reference_still_returns_a_binary_classification(self):
        report=transition.detect_transition_support(np.empty((0,3)),np.empty(0,int),0.,100.,
            [0.,0.,1.,-1100.],[0.,0.,1.,-1100.],np.zeros(2),np.array([1.,0.]),np.array([0.,1.]),
            [-1250.,1250.],p.PipelineConfig())
        self.assertEqual(report['decision'],'present')
        self.assertEqual(report['normal_classification']['decision'],'present')
        self.assertFalse(report['merge_recommended'])


class NormalConcentrationTests(unittest.TestCase):
    def assess(self, concentration, valid=687, invalid=1453):
        self.assertTrue(hasattr(support,'classify_normal_concentration'),'Missing normal concentration decision')
        return support.classify_normal_concentration(dict(valid=valid,invalid=invalid,
            dominant_fraction=concentration,riser_fraction=0.))

    def test_concentrated_riser_normals_mean_ordinary_step(self):
        self.assertEqual(self.assess(.974,valid=1361,invalid=88)['decision'],'absent')

    def test_dispersed_33_normals_mean_support_despite_unknown_boundary(self):
        result=self.assess(.29839883551673946)
        self.assertEqual(result['decision'],'present')
        self.assertAlmostEqual(result['deviation_fraction'],.7016011644832605)

    def test_concentrated_normals_do_not_need_to_match_the_truck_length_axis(self):
        self.assertEqual(self.assess(1.,valid=1000,invalid=20)['decision'],'absent')

    def test_too_few_valid_normals_default_to_present(self):
        self.assertEqual(self.assess(.3,valid=12,invalid=10)['decision'],'present')
        self.assertEqual(self.assess(.99,valid=12,invalid=10)['decision'],'present')

    def test_mostly_invalid_normals_default_to_present(self):
        result=self.assess(.99,valid=60,invalid=1000)
        self.assertEqual(result['decision'],'present')
        self.assertEqual(result['reason'],'absence_not_confirmed_insufficient_normals')

    def test_exact_minimum_valid_count_and_fraction_can_confirm_absence(self):
        self.assertEqual(self.assess(.99,valid=50,invalid=200)['decision'],'absent')
        self.assertEqual(self.assess(.99,valid=49,invalid=196)['decision'],'present')
        self.assertEqual(self.assess(.99,valid=50,invalid=201)['decision'],'present')


    def test_missing_or_invalid_concentration_defaults_to_present(self):
        for concentration in (None,float('nan'),-1.,1.5):
            with self.subTest(concentration=concentration):
                self.assertEqual(self.assess(concentration)['decision'],'present')

    def test_default_software_mode_uses_concentration(self):
        self.assertEqual(support.SupportParameters().decision_mode,'normal_concentration')

    def test_eighty_percent_threshold_requires_strictly_more_than_equality(self):
        self.assertEqual(self.assess(.7999)['decision'],'present')
        self.assertEqual(self.assess(.80)['decision'],'present')
        self.assertEqual(self.assess(.8001)['decision'],'absent')


class StructuralEvidenceTests(unittest.TestCase):
    detect=TransitionSupportTests.detect
    def test_sparse_same_height_top_is_not_removed_from_directional_evidence(self):
        top=floor(50.,300.,1600.)[::13]
        report=self.detect(np.vstack([grid_face(0.),top]),low_start=350.,mode='structural_evidence')
        self.assertEqual(report['decision'],'present')
        evidence=report['structural_evidence']['orientation_evidence']
        self.assertTrue(evidence['support_top_observed'])
        self.assertFalse(evidence['near_high_plane_points_removed'])

    def test_high_floor_rim_does_not_become_a_support_top(self):
        report=self.detect(np.vstack([floor(-90.,0.,1600.),grid_face(5.)]),mode='structural_evidence')
        self.assertEqual(report['decision'],'absent')
        evidence=report['structural_evidence']['orientation_evidence']
        self.assertGreater(evidence['top_normal_count'],30)
        self.assertFalse(evidence['support_top_observed'])

    def test_top_direction_uses_tilted_floor_reference(self):
        top=floor(50.,300.,1600.)[::13]
        candidates=np.vstack([grid_face(0.),top])
        high,low=floor(-1800.,0.,1600.),floor(350.,1800.,1100.)
        xyz=np.vstack([high,low,candidates]);xyz[:,2]+=.4*xyz[:,1]
        labels=np.r_[np.full(len(high),2),np.full(len(low),3),np.ones(len(candidates))]
        report=transition.detect_transition_support(xyz,labels,0.,350.,[0.,-.4,1.,-1600.],
            [0.,-.4,1.,-1100.],np.zeros(2),np.array([1.,0.]),np.array([0.,1.]),
            [-1250.,1250.],p.PipelineConfig(),support.SupportParameters(decision_mode='structural_evidence'))
        self.assertEqual(report['decision'],'present')
        evidence=report['structural_evidence']['orientation_evidence']
        self.assertTrue(evidence['support_top_observed'])
        normal=np.array([0.,-.4,1.]);normal/=np.linalg.norm(normal)
        np.testing.assert_allclose(evidence['reference_normal_uvz'],normal)

    def test_pipeline_passes_trial_mode_to_joint_detector(self):
        from dataclasses import replace
        high=floor(-1800.,0.,1600.);low=np.repeat(floor(350.,5000.,1100.),5,axis=0)
        candidates=np.vstack([grid_face(0.),floor(50.,300.,1600.)[::13]])
        xyz=np.vstack([high,low,candidates])
        labels=np.r_[np.full(len(high),2),np.full(len(low),3),np.ones(len(candidates))]
        config=replace(p.PipelineConfig(),truck_head_dir=(-1.,0.,0.),edge_plane_iterations=40,
                       support_decision_mode='structural_evidence')
        result,_,_=p.process_segmented_points_timed(xyz,labels,config)
        report=result['label_2']['transition_support']
        self.assertEqual(report['decision_basis'],'structural_evidence')
        self.assertIn('orientation_evidence',report['structural_evidence'])

    def test_step_with_high_floor_rim_is_absent_despite_mixed_normal_directions(self):
        candidates=np.vstack([floor(-90.,0.,1600.),grid_face(5.)])
        report=self.detect(candidates,mode='structural_evidence')
        self.assertEqual(report['decision'],'absent')
        self.assertLessEqual(report['structural_evidence']['middle_u_residual_p90_mm'],1.)
        self.assertGreater(report['structural_evidence']['cleaned_normal_concentration'],.99)

    def test_two_parallel_faces_cannot_become_absent_just_because_normals_are_concentrated(self):
        candidates=np.vstack([grid_face(0.),grid_face(300.)])
        report=self.detect(candidates,low_start=350.,mode='structural_evidence')
        self.assertEqual(report['decision'],'present')
        self.assertGreater(report['structural_evidence']['middle_u_residual_p90_mm'],100.)

    def test_support_level_with_high_deck_keeps_two_faces_as_presence_evidence(self):
        candidates=np.vstack([grid_face(0.),floor(20.,320.,1600.),grid_face(320.,1150.,1550.)])
        report=self.detect(candidates,low_start=350.,mode='structural_evidence')
        self.assertEqual(report['decision'],'present')
        self.assertGreaterEqual(report['plane_count'],2)

    def test_missing_joint_does_not_confirm_absence(self):
        report=self.detect(np.empty((0,3)),low_start=350.,mode='structural_evidence')
        self.assertEqual(report['decision'],'present')
        self.assertFalse(report['structural_evidence']['reliable'])

    def test_sparse_broad_rear_face_is_presence_evidence_despite_dense_front(self):
        v,z=np.meshgrid(np.linspace(-1200.,1200.,41),np.linspace(1150.,1550.,5))
        rear=np.column_stack([np.full(v.size,300.),v.ravel(),z.ravel()])
        report=self.detect(np.vstack([grid_face(0.),rear]),low_start=350.,mode='structural_evidence')
        self.assertGreaterEqual(report['plane_count'],2)
        self.assertEqual(report['decision'],'present')

    def test_inclined_high_floor_rim_stays_out_of_middle_face(self):
        candidates=np.vstack([floor(-90.,0.,1600.),grid_face(5.)])
        high,low=floor(-1800.,0.,1600.),floor(20.,1800.,1100.)
        xyz=np.vstack([high,low,candidates]);xyz[:,2]+=.05*xyz[:,1]
        labels=np.r_[np.full(len(high),2),np.full(len(low),3),np.ones(len(candidates))]
        report=transition.detect_transition_support(xyz,labels,0.,20.,[0.,-.05,1.,-1600.],
            [0.,-.05,1.,-1100.],np.zeros(2),np.array([1.,0.]),np.array([0.,1.]),
            [-1250.,1250.],p.PipelineConfig(),support.SupportParameters(decision_mode='structural_evidence'))
        self.assertEqual(report['decision'],'absent')
        self.assertLessEqual(report['structural_evidence']['middle_u_residual_p90_mm'],1.)

class DominantNormalTests(unittest.TestCase):
    def concentration(self,normals):
        self.assertTrue(hasattr(support,'_dominant_normal_concentration'),
                        'Missing data-derived axial normal concentration')
        return support._dominant_normal_concentration(np.asarray(normals,float),20.)

    def test_tilted_antipodal_normals_share_one_direction(self):
        n=np.array([np.cos(np.deg2rad(40)),np.sin(np.deg2rad(40)),0.])
        result=self.concentration(np.vstack([np.tile(n,(70,1)),np.tile(-n,(30,1))]))
        self.assertEqual(result['fraction'],1.)
        self.assertAlmostEqual(abs(np.asarray(result['direction'])@n),1.)

    def test_componentwise_absolute_values_cannot_merge_orthogonal_directions(self):
        normals=np.vstack([np.tile([1.,1.,0.],(60,1)),np.tile([1.,-1.,0.],(40,1))])
        self.assertEqual(self.concentration(normals)['fraction'],.6)

    def test_cone_can_be_centered_between_observed_directions(self):
        angle=np.deg2rad(19.)
        normals=np.vstack([np.tile([np.cos(angle),np.sin(angle),0.],(50,1)),
                           np.tile([np.cos(angle),-np.sin(angle),0.],(50,1))])
        self.assertEqual(self.concentration(normals)['fraction'],1.)

    def test_twenty_degree_cone_boundary_is_included(self):
        angle=np.deg2rad(20.)
        normals=np.vstack([np.tile([1.,0.,0.],(90,1)),[[np.cos(angle),np.sin(angle),0.]],
                           [[np.cos(angle),-np.sin(angle),0.]],np.tile([0.,0.,1.],(8,1))])
        self.assertEqual(self.concentration(normals)['fraction'],.92)

    def test_rotation_and_point_order_do_not_change_the_classification(self):
        normals=np.vstack([np.tile([1.,0.,0.],(81,1)),np.tile([0.,0.,1.],(19,1))])
        rotation=np.linalg.qr(np.random.default_rng(33).normal(size=(3,3)))[0]
        rotated=normals[np.random.default_rng(19).permutation(len(normals))]@rotation
        a,b=self.concentration(normals),self.concentration(rotated)
        self.assertEqual(a['fraction'],.81)
        self.assertEqual(a['fraction'],b['fraction'])
        report=support.classify_normal_concentration(dict(valid=100,invalid=0,dominant_fraction=b['fraction']))
        self.assertEqual(report['decision'],'absent')

    def test_tilted_plane_is_concentrated_despite_low_fixed_axis_fraction(self):
        candidates=grid_face(5.)
        a=np.deg2rad(40.)
        candidates[:,:2]=candidates[:,:2]@np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
        summary,_=support._normal_summary(candidates,support.SupportParameters())
        self.assertEqual(summary.get('dominant_fraction'),1.)
        self.assertEqual(summary['riser_fraction'],0.)
        self.assertEqual(support.classify_normal_concentration(summary)['decision'],'absent')


if __name__ == '__main__':
    unittest.main()
