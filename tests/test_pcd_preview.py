from __future__ import annotations

import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np
import open3d as o3d

from utils.pcd_preview import load_xyz, render_birdseye


class PcdPreviewTests(unittest.TestCase):
    def test_compressed_pcd_in_chinese_folder_loads_and_renders(self):
        with tempfile.TemporaryDirectory(prefix="pcd_test_") as folder:
            root = Path(folder)
            ascii_path = root / "compressed.pcd"
            cloud = o3d.geometry.PointCloud()
            expected = np.array([[39, 0, 5830], [40, 10, 5800], [-15, 20, 1000]], dtype=float)
            cloud.points = o3d.utility.Vector3dVector(expected)
            self.assertTrue(o3d.io.write_point_cloud(str(ascii_path), cloud, compressed=True))
            target = root / "中文路径 空格" / "PCD" / "雷达 [1].pcd"
            target.parent.mkdir(parents=True)
            shutil.copyfile(ascii_path, target)

            np.testing.assert_allclose(load_xyz(target), expected)
            image, metadata = render_birdseye(target, size=240)
            self.assertIsNotNone(image, metadata)
            self.assertEqual(metadata["point_count"], 3)
            self.assertEqual(metadata["error"], "")
