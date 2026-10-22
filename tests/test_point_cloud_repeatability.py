"""Repeatability regressions use synthetic geometry, never field HMI coordinates."""
from dataclasses import replace
import unittest

import numpy as np
import torch

from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as pipeline


class PlaneRepeatabilityTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(1803)
        xy = rng.uniform(-4000, 4000, (2400, 2))
        z = 1200 + .025 * xy[:, 0] - .01 * xy[:, 1] + rng.normal(0, 15, len(xy))
        clutter = rng.uniform((-4000, -4000, 0), (4000, 4000, 2300), (500, 3))
        self.xyz = np.vstack((np.column_stack((xy, z)), clutter))
        self.config = replace(pipeline.PipelineConfig(), edge_plane_distance=35,
                              edge_plane_iterations=120)

    def test_plane_and_support_repeat_after_unrelated_random_calls(self):
        original = self.xyz.copy()
        model, indices = pipeline.fit_plane_once(self.xyz, self.config)
        for seed in (7, 91, 1803):
            pipeline.o3d.utility.random.seed(seed)
            again, again_indices = pipeline.fit_plane_once(self.xyz, self.config)
            np.testing.assert_array_equal(again_indices, indices)
            np.testing.assert_array_equal(again, model)
        np.testing.assert_array_equal(self.xyz, original)
        expected_normal = np.array([-.025, .01, 1.0])
        expected_normal /= np.linalg.norm(expected_normal)
        actual = np.array(model[:3]) * np.sign(model[2])
        np.testing.assert_allclose(actual, expected_normal, atol=.002, rtol=0)

    def test_input_permutation_preserves_plane_and_source_index_mapping(self):
        model, indices = pipeline.fit_plane_once(self.xyz, self.config)
        perm = np.random.default_rng(9).permutation(len(self.xyz))
        shuffled_model, shuffled_indices = pipeline.fit_plane_once(self.xyz[perm], self.config)
        np.testing.assert_array_equal(shuffled_model, model)
        np.testing.assert_array_equal(np.sort(perm[shuffled_indices]), np.sort(indices))

    def test_equally_supported_planes_have_a_stable_winner(self):
        x, y = np.meshgrid(np.arange(10)*10., np.arange(10)*10.)
        xy = np.column_stack((x.ravel(), y.ravel()))
        points = np.vstack((np.column_stack((xy, np.zeros(len(xy)))),
                            np.column_stack((xy, np.full(len(xy), 1000.)))))
        config = replace(self.config, edge_plane_distance=.1, edge_plane_iterations=800)
        model, indices = pipeline.fit_plane_once(points, config)
        for _ in range(40):
            again, again_indices = pipeline.fit_plane_once(points, config)
            np.testing.assert_array_equal(again_indices, indices)
            np.testing.assert_array_equal(again, model)


class NetworkRepeatabilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old_threads = torch.get_num_threads()
        torch.set_num_threads(4)
        cls.pipe = pipeline.PointCloudPipeline(device="cpu")
        rng = np.random.default_rng(20261015)
        cls.xyz = rng.uniform((2200, -9000, 900), (5800, 8500, 1500), (2048, 3)).astype(np.float32)

    @classmethod
    def tearDownClass(cls):
        torch.set_num_threads(cls.old_threads)

    def test_same_input_repeats_scores_and_labels_despite_other_random_work(self):
        scores = []
        hook = self.pipe.segmenter.model.register_forward_hook(
            lambda module, inputs, output: scores.append(output[0].detach().cpu().clone()))
        try:
            first, _, _ = self.pipe.segment_xyz_full_coverage(self.xyz)
            torch.rand(113)
            second, _, _ = self.pipe.segment_xyz_full_coverage(self.xyz)
        finally:
            hook.remove()
        np.testing.assert_array_equal(second, first)
        self.assertTrue(torch.equal(scores[0], scores[1]), "Same points must yield the same class scores")

    def test_inference_does_not_advance_callers_torch_random_state(self):
        before = torch.random.get_rng_state().clone()
        self.pipe.segment_xyz_full_coverage(self.xyz)
        self.assertTrue(torch.equal(torch.random.get_rng_state(), before))

    def test_permuted_input_returns_labels_in_original_source_row_order(self):
        labels, _, _ = self.pipe.segment_xyz_full_coverage(self.xyz)
        perm = np.random.default_rng(71).permutation(len(self.xyz))
        shuffled_labels, _, _ = self.pipe.segment_xyz_full_coverage(self.xyz[perm])
        np.testing.assert_array_equal(shuffled_labels[np.argsort(perm)], labels)


if __name__ == "__main__":
    unittest.main()
