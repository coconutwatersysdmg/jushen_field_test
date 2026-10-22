import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from dataclasses import replace
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as p
from algorithm_modules.point_cloud_segment_module import shared_deck_geometry as shared
from services.point_cloud_processing_service import PointCloudProcessingService
from services.board_data_service import build_board_data


class GeometrySchemeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        angle=np.deg2rad(17)
        cls.u=np.array([np.cos(angle),np.sin(angle)])
        cls.v=np.array([-np.sin(angle),np.cos(angle)])
        clouds=[];labels=[]
        for label,start,end,n in [(2,0,4000,120),(3,4100,12000,320)]:
            u,v=np.meshgrid(np.linspace(start,end,n),np.linspace(-1250,1250,100))
            curve=0 if label==2 else 40*np.sin(np.pi*(u-start)/(end-start))
            xy=np.array([3500,1000])+u[...,None]*cls.u+(v+curve)[...,None]*cls.v
            z=(1600 if label==2 else 1100)+.001*u+.002*v
            clouds.append(np.column_stack([xy.reshape(-1,2),z.ravel()]))
            labels.extend([label]*u.size)
        cls.xyz=np.vstack(clouds).astype(np.float32);cls.labels=np.array(labels)

    def config(self,scheme):
        self.assertIn('geometry_scheme',p.PipelineConfig.__dataclass_fields__)
        return replace(p.PipelineConfig(),geometry_scheme=scheme,edge_plane_iterations=40,
                       line_ransac_iterations=80,truck_head_dir=(-self.u[0],-self.u[1],0))

    def test_scheme2_shares_high_board_width_axis_and_keeps_independent_height(self):
        result,_,_=p.process_segmented_points_timed(self.xyz,self.labels,self.config(2))
        self.assertIsNotNone(result['label_2']);self.assertIsNotNone(result['label_3'])
        # Test fitted-line constraints before millimetre output quantization.
        a=np.array(result['label_2']['global_plane_radar_corners_xyz_mm']);b=np.array(result['label_3']['global_plane_radar_corners_xyz_mm'])
        axis=a[2,:2]-a[0,:2];axis/=np.linalg.norm(axis)
        self.assertLess(np.rad2deg(np.arccos(np.clip(axis@self.u,-1,1))),.1)
        width=np.linalg.norm(a[1,:2]-a[0,:2])
        self.assertAlmostEqual(width,2500,delta=4)
        self.assertAlmostEqual(np.linalg.norm(b[1,:2]-b[0,:2]),width,delta=1e-6)
        cross=lambda x,y:x[0]*y[1]-x[1]*y[0]
        for i in range(4):
            self.assertAlmostEqual(cross(b[i,:2]-a[i%2,:2],axis),0,delta=1e-6)
        for corners in [a,b]:
            x=corners[1,:2]-corners[0,:2];y=corners[2,:2]-corners[0,:2]
            rear=corners[3,:2]-corners[2,:2]
            self.assertAlmostEqual(cross(x,rear)/(np.linalg.norm(x)*np.linalg.norm(rear)),0,delta=1e-10)
        self.assertGreater(a[:,2].mean()-b[:,2].mean(),400)
        self.assertEqual(result['label_3']['geometry_scheme'],2)
        self.assertIn('shared_geometry_quality',result['label_3'])

    def test_removed_scheme1_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'geometry_scheme'):
            p.process_segmented_points_timed(self.xyz,self.labels,self.config(1))

    def test_only_high_board_does_not_invent_low_board(self):
        mask=self.labels==2
        result,_,_=p.process_segmented_points_timed(self.xyz[mask],self.labels[mask],self.config(2))
        self.assertIsNotNone(result['label_2']);self.assertIsNone(result['label_3'])

    def test_missing_high_board_reports_unsupported_without_independent_fallback(self):
        mask=self.labels==3
        result,_,_=p.process_segmented_points_timed(self.xyz[mask],self.labels[mask],self.config(2))
        self.assertIsNone(result['label_2']);self.assertIsNone(result['label_3'])
        self.assertEqual(result['geometry_status'],'unsupported')
        self.assertEqual(result['geometry_failures'][0]['reason'],'missing_high_deck_reference')
        normalized=PointCloudProcessingService.normalize_result(result)
        self.assertFalse(normalized['success'])
        self.assertEqual(normalized['geometry_failures'],result['geometry_failures'])
        self.assertIn('missing_high_deck_reference',normalized['message'])

    def test_failed_high_reference_does_not_return_old_geometry(self):
        with patch.object(shared,'_parallel_reference',side_effect=shared.UnsupportedGeometry('missing_near_edge')):
            result,_,_=p.process_segmented_points_timed(self.xyz,self.labels,self.config(2))
        self.assertIsNone(result['label_2']);self.assertIsNone(result['label_3'])
        self.assertEqual(result['geometry_status'],'unsupported')
        self.assertIn('missing_near_edge',result['geometry_failures'][0]['reason'])

    def test_failed_low_retains_valid_high_corners_but_is_not_a_successful_single_board(self):
        original=shared._build_board
        def low_failure(points,label,*args,**kwargs):
            if label==3:raise shared.UnsupportedGeometry('missing_low_end')
            return original(points,label,*args,**kwargs)
        with patch.object(shared,'_build_board',side_effect=low_failure):
            result,_,_=p.process_segmented_points_timed(self.xyz,self.labels,self.config(2))
        self.assertIsNotNone(result['label_2']);self.assertIsNone(result['label_3'])
        self.assertEqual(result['label_2']['geometry_scheme'],2)
        self.assertEqual(result['geometry_status'],'partial')
        self.assertIn('missing_low_end',result['geometry_failures'][0]['reason'])
        self.assertIsNone(result['label_2']['height_mean_mm'])
        self.assertEqual(result['label_2']['height_estimation']['status'],'geometry_incomplete')
        normalized=PointCloudProcessingService.normalize_result(result)
        self.assertFalse(normalized['success'])
        self.assertEqual(normalized['board_mode'],'incomplete')
        self.assertEqual(normalized['board_count'],1)
        self.assertIsNone(normalized['is_high_low_board'])
        self.assertEqual(normalized['geometry_failures'],result['geometry_failures'])
        displayed=build_board_data({'radar_result':normalized})[0]['radar']
        self.assertIsNone(displayed['height_mean_mm'])

    def test_file_record_reports_geometry_failure_and_saves_diagnostics(self):
        pipe=p.PointCloudPipeline.__new__(p.PointCloudPipeline)
        pipe.config=self.config(2)
        for status in ('unsupported','partial'):
            with self.subTest(status=status), TemporaryDirectory() as folder:
                raw=dict(label_2=None,label_3=None,geometry_status=status,
                         geometry_failures=[dict(board_label='label_3',reason='unsupported_end')])
                filtered=SimpleNamespace(points=self.xyz)
                with patch.object(p,'read_point_cloud',return_value=filtered), \
                     patch.object(p,'preprocess_point_cloud_timed',return_value=(filtered,{},{})), \
                     patch.object(pipe,'segment_xyz_full_coverage',return_value=(self.labels,{},dict(inference_s=0.))), \
                     patch.object(p,'process_segmented_points_timed',return_value=(raw,[],0.)):
                    record=pipe.process_file(Path(folder)/'truck.pcd',folder)
                self.assertEqual(record['status'],'failed')
                self.assertIn('unsupported_end',record['error'])
                self.assertTrue(Path(record['json']).is_file())
                self.assertEqual(record['result'],raw)

    def test_invalid_scheme_is_rejected(self):
        with self.assertRaises(ValueError):
            p.process_segmented_points_timed(self.xyz,self.labels,self.config(9))

    def test_service_switch_and_quality_are_not_dropped(self):
        self.config(2)
        service=PointCloudProcessingService(device='cpu');service.pc_cfg['geometry_scheme']=2
        with patch('algorithm_modules.point_cloud_segment_module.point_cloud_pipeline.PointCloudPipeline',side_effect=lambda **kwargs:SimpleNamespace(config=kwargs['config'])):
            self.assertEqual(service._ensure_pipeline().config.geometry_scheme,2)
        board=p.compute_geometry_result([[0,0,1500],[2000,0,1500],[0,4000,1500],[2000,4000,1500]])
        board.update(geometry_scheme=2,shared_geometry_quality={'side_alignment_supported':False},
                     transition_geometry={'structure':'separated_usable_start'},geometry_revision='near_high_and_transition_v2')
        normalized=service._normalize_board('label_2',board)
        self.assertEqual(normalized['geometry_scheme'],2)
        self.assertFalse(normalized['shared_geometry_quality']['side_alignment_supported'])
        self.assertEqual(normalized['transition_geometry']['structure'],'separated_usable_start')
        self.assertEqual(normalized['geometry_revision'],'near_high_and_transition_v2')

    def test_service_rejects_old_scheme_before_loading_the_model(self):
        service=PointCloudProcessingService(device='cpu');service.pc_cfg['geometry_scheme']=1
        with patch('algorithm_modules.point_cloud_segment_module.point_cloud_pipeline.PointCloudPipeline') as factory:
            with self.assertRaisesRegex(RuntimeError,'geometry_scheme'):
                service._ensure_pipeline()
        factory.assert_not_called()

    def test_scheme2_is_invariant_to_source_row_order(self):
        cfg=self.config(2)
        original,_,_=p.process_segmented_points_timed(self.xyz,self.labels,cfg)
        perm=np.random.default_rng(29).permutation(len(self.xyz))
        shuffled,_,_=p.process_segmented_points_timed(self.xyz[perm],self.labels[perm],cfg)
        for key in ['label_2','label_3']:
            np.testing.assert_array_equal(original[key]['radar_corners_xyz_mm'],shuffled[key]['radar_corners_xyz_mm'])

    def test_short_high_deck_keeps_length_axis_even_when_width_is_larger(self):
        relative=(self.xyz[:,:2]-[3500,1000])@self.u
        keep=(self.labels==3)|(relative<2000)
        result,_,_=p.process_segmented_points_timed(self.xyz[keep],self.labels[keep],self.config(2))
        self.assertEqual(result['label_2']['geometry_scheme'],2)
        points=np.array(result['label_2']['global_plane_radar_corners_xyz_mm'])
        axis=points[2,:2]-points[0,:2];axis/=np.linalg.norm(axis)
        self.assertGreater(axis@self.u,.999)

    def test_short_high_only_uses_heading_to_distinguish_length_from_width(self):
        relative=(self.xyz[:,:2]-[3500,1000])@self.u
        keep=(self.labels==2)&(relative<2000)
        result,_,_=p.process_segmented_points_timed(self.xyz[keep],self.labels[keep],self.config(2))
        points=np.array(result['label_2']['global_plane_radar_corners_xyz_mm'])
        axis=points[2,:2]-points[0,:2];axis/=np.linalg.norm(axis)
        self.assertGreater(axis@self.u,.999)

    def test_narrow_high_only_does_not_treat_elongated_width_as_truck_length(self):
        relative=(self.xyz[:,:2]-[3500,1000])@self.u
        keep=(self.labels==2)&(relative<1000)
        result,_,_=p.process_segmented_points_timed(self.xyz[keep],self.labels[keep],self.config(2))
        points=np.array(result['label_2']['global_plane_radar_corners_xyz_mm'])
        axis=points[2,:2]-points[0,:2];axis/=np.linalg.norm(axis)
        self.assertGreater(axis@self.u,.999)

    def test_near_high_edge_controls_direction_despite_far_edge_bias(self):
        u,t=np.meshgrid(np.linspace(0,4000,150),np.linspace(0,1,100))
        v=-1250+t*(2500+.025*u)
        xyz=np.column_stack([u.ravel(),v.ravel(),np.full(u.size,1600.)])
        _,axis,_,_,quality=shared._parallel_reference(xyz,np.array([1.,0.]),self.config(2))
        self.assertLess(abs(axis[1]),.001)
        self.assertEqual(quality['direction_reference'],'high_near_v_min_P1_P3')

    def test_high_far_edge_cannot_replace_missing_near_direction_silently(self):
        u,t=np.meshgrid(np.linspace(0,4000,150),np.linspace(0,1,100))
        xyz=np.column_stack([u.ravel(),(-1250+2500*t).ravel(),np.full(u.size,1600.)])
        original=shared.__dict__['_parallel_reference']
        extract=p.extract_edge_candidates_2d
        def missing_near(uv,config):
            candidates=extract(uv,config);candidates['v_min']=candidates['v_min'][:10]
            return candidates
        with patch.object(p,'extract_edge_candidates_2d',side_effect=missing_near):
            with self.assertRaises(shared.UnsupportedGeometry):original(xyz,np.array([1.,0.]),self.config(2))

    def test_failed_low_front_recovers_usable_floor_without_losing_shared_sides(self):
        original=shared._fit_edge_line
        def front_failure(candidates,edge,config,required_span):
            median=np.median(candidates[:,0])
            if edge=='u_min' and 2020<median<3000:raise shared.UnsupportedGeometry('synthetic_bad_low_front')
            return original(candidates,edge,config,required_span)
        u,v=np.meshgrid(np.arange(4100.,12001.,15.),np.arange(-1250.,1251.,20.))
        xy=np.array([3500,1000])+u[...,None]*self.u+v[...,None]*self.v
        low=np.column_stack([xy.reshape(-1,2),(1100+.001*u+.002*v).ravel()])
        high=self.xyz[self.labels==2]
        xyz=np.vstack([high,low]);labels=np.r_[np.full(len(high),2),np.full(len(low),3)]
        with patch.object(shared,'_fit_edge_line',side_effect=front_failure):
            result,_,_=p.process_segmented_points_timed(xyz,labels,self.config(2))
        board=result['label_3']
        self.assertEqual(board['geometry_scheme'],2)
        self.assertEqual(board['shared_geometry_quality']['end_edge_support'][0]['method'],'persistent_3d_low_floor_onset')
        self.assertEqual(result['label_2']['shared_xy_reference'],board['shared_xy_reference'])

    def test_supported_vertical_joint_shares_xy_but_keeps_each_deck_z(self):
        u,v=np.meshgrid(np.linspace(4020,12000,320),np.linspace(-1250,1250,100))
        low_xy=np.array([3500,1000])+u[...,None]*self.u+v[...,None]*self.v
        low=np.column_stack([low_xy.reshape(-1,2),np.full(u.size,1100.)])
        v,z=np.meshgrid(np.linspace(-1200,1200,100),np.linspace(1150,1550,30))
        face_xy=np.array([3500,1000])+4005*self.u+v[...,None]*self.v
        face=np.column_stack([face_xy.reshape(-1,2),z.ravel()])
        high=self.xyz[self.labels==2]
        xyz=np.vstack([high,low,face]);labels=np.r_[np.full(len(high),2),np.full(len(low),3),np.zeros(len(face))]
        result,_,_=p.process_segmented_points_timed(xyz,labels,self.config(2))
        high_result=result['label_2'];low_result=result['label_3']
        self.assertEqual(low_result['transition_geometry']['structure'],'vertical_step')
        a=np.array(high_result['radar_corners_xyz_mm']);b=np.array(low_result['radar_corners_xyz_mm'])
        np.testing.assert_allclose(a[2:,:2],b[:2,:2],atol=1e-7,rtol=0)
        self.assertGreater(a[2:,2].mean()-b[:2,2].mean(),400)

if __name__=='__main__':unittest.main()
