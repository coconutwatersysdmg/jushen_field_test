"""Point-cloud segmentation and truck-board geometry pipeline.

Current production flow:
    PCD -> ROI -> voxel downsample -> main-plane removal -> DBSCAN
    -> statistical filter -> PointNet++ fixed-size direct coverage inference
    -> board-plane RANSAC -> projected head/near anchors / shared deck boundary
    -> per-deck H assigned to four corner Z -> WORLD geometry -> loading plan
    -> output/json/<name>.json

Only per-file JSON results are written to disk.
"""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
from utils.measurement_rounding import round_measurement

from .full_coverage_sampling import iter_full_coverage_samples
from .corner_height import estimate_corner_heights

try:
    import open3d as o3d
except ImportError:
    o3d = None

try:
    from . import PointNet2Segmenter
    from .inference import DEFAULT_NUM_CLASSES, DEFAULT_NUM_PART, pc_normalize
except ImportError as exc:
    PointNet2Segmenter = None
    POINTNET_IMPORT_ERROR = exc
else:
    POINTNET_IMPORT_ERROR = None


# ============================================================================
# 用户可调参数区
# 现场调试时，优先只修改这里。
# ============================================================================
@dataclass(frozen=True)
class PipelineConfig:
    # ------------------------------------------------------------------------
    # 1. ROI 裁剪范围（mm）
    # ------------------------------------------------------------------------
    roi_min: tuple[float, float, float] = (300.0, -11000.0, -200.0)
    roi_max: tuple[float, float, float] = (9000.0, 14000.0, 3500.0)
    roi_redundancy: float = 0.025

    # ------------------------------------------------------------------------
    # 2. 点云预处理
    # ------------------------------------------------------------------------
    voxel_size: float = 10.0

    # 地面/主平面 RANSAC
    plane_threshold: float = 140.0
    plane_ransac_n: int = 3
    plane_iterations: int = 500

    # DBSCAN
    dbscan_eps: float = 100.0
    dbscan_min_points: int = 50

    # 统计离群点滤波
    stat_neighbors: int = 6
    stat_std_ratio: float = 3.0

    # ------------------------------------------------------------------------
    # 3. PointNet++
    # 每次固定点数、整车上下文、多次覆盖全部原始点；不做KNN标签恢复。
    # ------------------------------------------------------------------------
    pointnet_num_point: int = 49152
    pointnet_num_votes: int = 1
    pointnet_context_fraction: float = 0.5
    target_labels: tuple[int, int] = (2, 3)
    seed: int = 42

    # ------------------------------------------------------------------------
    # 4. 底板平面 RANSAC
    # ------------------------------------------------------------------------
    edge_plane_distance: float = 100.0
    edge_plane_ransac_n: int = 5
    edge_plane_iterations: int = 800
    edge_min_inliers: int = 3000
    # 仅保留2：高板参考，共方向/共宽/共线；1已移除，拒绝旧配置。
    geometry_scheme: int = 2
    corner_height_radii: tuple[float, ...] = (250., 400., 600., 900.)
    corner_height_min_points: int = 40
    corner_height_min_tolerance: float = 15.0
    corner_height_max_tolerance: float = 40.0
    corner_height_iterations: int = 160
    corner_height_max_slope: float = 0.5

    # 高度选区沿本板长度后移5%，宽度两侧各舍弃5%；五条带中心高度等权平均。
    # H独立于原角点Z估计，最终统一赋给本板四角。A/B/C/D均保留。
    height_scheme: str = 'A'
    height_length_start_fraction: float = 0.05
    height_width_start_fraction: float = 0.05
    single_height_length_fraction: float = 0.6
    single_height_width_fraction: float = 0.90
    high_height_length_fraction: float = 0.8
    low_height_length_fraction: float = 0.4
    high_low_height_width_fraction: float = 0.90
    single_height_min_points: int = 40
    single_height_min_tolerance: float = 15.0
    single_height_max_tolerance: float = 40.0
    single_height_iterations: int = 160
    single_height_max_slope: float = 0.5

    # ------------------------------------------------------------------------
    # 5. PCA + 分段切片边缘提取
    # U = 车长方向；V = 车宽方向
    # ------------------------------------------------------------------------
    slice_width_u: float = 50.0
    slice_width_v: float = 30.0
    edge_u_bins_min: int = 80
    edge_u_bins_max: int = 350
    edge_v_bins_min: int = 50
    edge_v_bins_max: int = 220
    slice_min_points: int = 20

    # ------------------------------------------------------------------------
    # 6. 四边 RANSAC
    # ------------------------------------------------------------------------
    line_ransac_distance: float = 30.0
    line_ransac_iterations: int = 300
    line_min_inliers: int = 25

    # ------------------------------------------------------------------------
    # 7. 车辆方向定义
    # 指向车头的大致雷达坐标方向，用于固定 Umin = 车头端（角点转 WORLD 前）。
    # ------------------------------------------------------------------------
    truck_head_dir: tuple[float, float, float] = (-1.0, 1.0, 0.0)

    # ------------------------------------------------------------------------
    # 8. 第一作业面装载规划
    # 仅当 label_2 与 label_3 都成功识别时，对 label_2 计算。
    # ------------------------------------------------------------------------
    pallet_length_mm: float = 1200.0
    pad_trigger_ratio: float = 0.50

    # Structural evidence remains an explicit trial option until same-height
    # support-object misses are resolved; the established mode stays available.
    support_decision_mode: str = 'normal_concentration'

    def __post_init__(self):
        if self.support_decision_mode not in ('normal_concentration','geometry_evidence','structural_evidence'):
            raise ValueError('Unsupported support_decision_mode')
        if self.geometry_scheme != 2:
            raise ValueError('geometry_scheme only supports 2; scheme 1 has been removed')


DEFAULT_CONFIG = PipelineConfig()

# Temporary field correction, applied to the complete post-inference point set.
LOW_DECK_LABEL_MERGE_THRESHOLD = 30000

# Torch exposes process-global random generators. Serialize this module's
# seed/use regions so separate pipeline instances cannot interleave them.
_INFERENCE_RANDOM_LOCK = threading.RLock()


def calculate_roi_bounds(
    roi_min,
    roi_max,
    redundancy_ratio: float = 0.025,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    low = np.asarray(roi_min, dtype=np.float64)
    high = np.asarray(roi_max, dtype=np.float64)
    delta = high - low
    redundancy = np.where(delta > 1e-6, delta * float(redundancy_ratio), 0.0)
    return tuple(low - redundancy), tuple(high + redundancy)


def collect_pcd_files(input_path: str | Path) -> list[Path]:
    path = Path(input_path).expanduser().resolve()
    if path.is_file():
        if path.suffix.lower() != ".pcd":
            raise ValueError(f"输入文件不是 PCD：{path}")
        return [path]
    if path.is_dir():
        files = sorted(
            candidate
            for candidate in path.iterdir()
            if candidate.is_file() and candidate.suffix.lower() == ".pcd"
        )
        if files:
            return files
        raise FileNotFoundError(f"目录中未找到 PCD 文件：{path}")
    raise FileNotFoundError(f"输入路径不存在：{path}")


def require_open3d():
    if o3d is None:
        raise RuntimeError("缺少 open3d，请先在运行环境中安装 open3d。")
    return o3d


def _least_squares_plane(points):
    """Orthogonal least-squares plane, with a canonical coefficient sign."""
    center = np.mean(points, axis=0)
    centered = points - center
    values, vectors = np.linalg.eigh(centered.T @ centered)
    if values[-1] <= 0 or values[1] <= np.finfo(np.float64).eps * values[-1] * 8:
        return None  # Coincident or collinear points do not define a plane.
    normal = vectors[:, 0]
    if normal[np.argmax(np.abs(normal))] < 0:
        normal = -normal
    return np.append(normal, -np.dot(normal, center))


def _segment_plane_reproducibly(pcd, *, distance_threshold, ransac_n,
                                num_iterations, seed):
    """Fixed RANSAC candidates, sequential scoring and deterministic tie handling.

    Open3D's parallel best-model updates can race even with a fixed seed and
    full iteration budget when models have equal support and error. Keep our
    random generator local; score every candidate by support, then mean squared
    residual, retaining the earlier candidate on exact ties. No physical target
    dimensions or external reference coordinates participate in this selection.
    """
    xyz = np.asarray(pcd.points, dtype=np.float64)
    order = np.lexsort((xyz[:, 2], xyz[:, 1], xyz[:, 0]))
    canonical = xyz[order]
    rng = np.random.default_rng(seed)
    best_support = None
    best_score = (0, -np.inf)
    for _ in range(num_iterations):
        sample = canonical[rng.choice(len(canonical), ransac_n, replace=False)]
        candidate = _least_squares_plane(sample)
        if candidate is None:
            continue
        residual = np.abs(canonical[:, 0] * candidate[0]
                          + canonical[:, 1] * candidate[1]
                          + canonical[:, 2] * candidate[2] + candidate[3])
        support = residual < distance_threshold
        count = int(np.count_nonzero(support))
        if count < max(3, best_score[0]):
            continue
        mean_squared_error = float(np.mean(np.square(residual[support])))
        score = (count, -mean_squared_error)
        if score > best_score:
            best_score = score
            best_support = support
    if best_support is None:
        return np.zeros(4), np.empty(0, dtype=np.int32)
    model = _least_squares_plane(canonical[best_support])
    if model is None:
        return np.zeros(4), np.empty(0, dtype=np.int32)
    source_inliers = np.sort(order[np.flatnonzero(best_support)])
    return model, source_inliers.astype(np.int32)


def read_point_cloud(path: str | Path):
    """Read a PCD even when the Windows path contains non-ASCII characters.

    Open3D's ``io.read_point_cloud`` can raise ``UnicodeDecodeError`` for paths
    that include Chinese folder names.  Copy to an ASCII-only temp file first.
    """
    import shutil
    import tempfile

    backend = require_open3d()
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"点云文件不存在：{source}")

    def _load(candidate: Path):
        cloud = backend.io.read_point_cloud(str(candidate))
        if len(cloud.points) == 0:
            raise ValueError(f"点云为空或读取失败：{source}")
        return cloud

    try:
        source_text = str(source)
    except Exception:
        source_text = ""
    needs_ascii_copy = any(ord(ch) > 127 for ch in source_text)
    if not needs_ascii_copy:
        try:
            return _load(source)
        except UnicodeDecodeError:
            needs_ascii_copy = True

    suffix = source.suffix if source.suffix else ".pcd"
    with tempfile.TemporaryDirectory(prefix="jushen_pcd_") as tmp_dir:
        temp_path = Path(tmp_dir) / f"input{suffix}"
        shutil.copy2(source, temp_path)
        return _load(temp_path)


def write_json(path: str | Path, payload) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def preprocess_point_cloud_timed(pcd, config: PipelineConfig):
    backend = require_open3d()
    original_count = len(pcd.points)
    if original_count == 0:
        raise ValueError("输入点云为空")

    timings = {}
    total_start = time.perf_counter()

    step_start = time.perf_counter()
    low, high = calculate_roi_bounds(
        config.roi_min,
        config.roi_max,
        config.roi_redundancy,
    )
    bounds = backend.geometry.AxisAlignedBoundingBox(min_bound=low, max_bound=high)
    roi = pcd.crop(bounds)
    timings["roi_crop_s"] = time.perf_counter() - step_start
    if len(roi.points) == 0:
        raise ValueError("ROI 裁剪后点云为空")

    step_start = time.perf_counter()
    downsampled = roi.voxel_down_sample(config.voxel_size)
    timings["voxel_downsample_s"] = time.perf_counter() - step_start
    if len(downsampled.points) < config.plane_ransac_n:
        raise ValueError("降采样后点数不足以进行平面拟合")

    step_start = time.perf_counter()
    _, plane_indices = _segment_plane_reproducibly(
        downsampled,
        distance_threshold=config.plane_threshold,
        ransac_n=config.plane_ransac_n,
        num_iterations=config.plane_iterations,
        seed=config.seed,
    )
    timings["ground_plane_ransac_s"] = time.perf_counter() - step_start

    step_start = time.perf_counter()
    non_plane = downsampled.select_by_index(plane_indices, invert=True)
    timings["ground_plane_remove_s"] = time.perf_counter() - step_start
    if len(non_plane.points) == 0:
        raise ValueError("平面过滤后点云为空")

    step_start = time.perf_counter()
    cluster_labels = np.asarray(
        non_plane.cluster_dbscan(
            eps=config.dbscan_eps,
            min_points=config.dbscan_min_points,
            print_progress=False,
        )
    )
    timings["dbscan_s"] = time.perf_counter() - step_start
    valid_labels = cluster_labels[cluster_labels >= 0]
    if valid_labels.size == 0:
        raise ValueError("DBSCAN 没有找到有效聚类")

    step_start = time.perf_counter()
    unique_labels, cluster_sizes = np.unique(valid_labels, return_counts=True)
    largest_label = unique_labels[np.argmax(cluster_sizes)]
    largest_cluster = non_plane.select_by_index(
        np.flatnonzero(cluster_labels == largest_label)
    )
    timings["largest_cluster_select_s"] = time.perf_counter() - step_start

    step_start = time.perf_counter()
    statistical, _ = largest_cluster.remove_statistical_outlier(
        nb_neighbors=config.stat_neighbors,
        std_ratio=config.stat_std_ratio,
    )
    timings["statistical_filter_s"] = time.perf_counter() - step_start
    if len(statistical.points) == 0:
        raise ValueError("统计过滤后点云为空")

    timings["preprocess_total_s"] = time.perf_counter() - total_start
    counts = {
        "original": original_count,
        "roi": len(roi.points),
        "downsampled": len(downsampled.points),
        "plane_filtered": len(non_plane.points),
        "cluster_filtered": len(largest_cluster.points),
        "stat_filtered": len(statistical.points),
    }
    return statistical, counts, timings


def normalize_vector(vector) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float64)
    norm = np.linalg.norm(vector)
    if norm < 1e-12:
        return vector
    return vector / norm


def fit_plane_once(points, config: PipelineConfig):
    if len(points) < config.edge_plane_ransac_n:
        return None, None
    backend = require_open3d()
    pcd = backend.geometry.PointCloud()
    pcd.points = backend.utility.Vector3dVector(np.asarray(points, dtype=np.float64))
    plane_model, inliers = _segment_plane_reproducibly(
        pcd,
        distance_threshold=config.edge_plane_distance,
        ransac_n=config.edge_plane_ransac_n,
        num_iterations=config.edge_plane_iterations,
        seed=config.seed + 1,
    )
    return plane_model, np.asarray(inliers, dtype=np.int32)


def get_ransac_normal(plane_model) -> np.ndarray:
    normal = normalize_vector(np.asarray(plane_model[:3], dtype=np.float64))
    if normal[2] < 0:
        normal = -normal
    return normal


def compute_pca_info(points, plane_model, truck_head_dir):
    """建立真实朝向的底板局部 UVN 坐标系。

    U：车长方向；Umin 固定为车头端。
    V：车宽方向。
    N：拟合平面法向，整体朝 world +Z。
    """

    points = np.asarray(points, dtype=np.float64)
    raw_center = np.mean(points, axis=0)

    plane = np.asarray(plane_model, dtype=np.float64)
    plane_normal_raw = plane[:3]
    plane_normal_sq = float(np.dot(plane_normal_raw, plane_normal_raw))
    if plane_normal_sq < 1e-12:
        raise ValueError("RANSAC 平面法向量无效")

    signed_scale = (
        float(np.dot(plane_normal_raw, raw_center)) + float(plane[3])
    ) / plane_normal_sq
    center = raw_center - signed_scale * plane_normal_raw

    centered = points - raw_center
    covariance = np.cov(centered, rowvar=False)
    _, eigenvectors = np.linalg.eigh(covariance)
    eigenvectors = eigenvectors[:, ::-1]

    long_dir = normalize_vector(eigenvectors[:, 0])
    normal_dir = get_ransac_normal(plane_model)
    if normal_dir[2] < 0:
        normal_dir = -normal_dir

    long_dir = normalize_vector(
        long_dir - np.dot(long_dir, normal_dir) * normal_dir
    )
    short_dir = normalize_vector(np.cross(normal_dir, long_dir))

    head_dir = np.asarray(truck_head_dir, dtype=np.float64)
    head_dir = head_dir - np.dot(head_dir, normal_dir) * normal_dir
    if np.linalg.norm(head_dir) < 1e-12:
        raise ValueError("truck_head_dir 投影后与拟合平面法向平行")
    head_dir = normalize_vector(head_dir)

    # Umin 位于 -long_dir 一侧，要求它朝向车头。
    if np.dot(-long_dir, head_dir) < 0:
        long_dir = -long_dir
        short_dir = -short_dir

    return {
        "center": center,
        "u_dir": long_dir,
        "v_dir": short_dir,
        "normal_dir": normal_dir,
    }


def project_points_to_uvn(points, pca_info):
    points = np.asarray(points, dtype=np.float64)
    centered = points - pca_info["center"]
    return (
        centered @ pca_info["u_dir"],
        centered @ pca_info["v_dir"],
        centered @ pca_info["normal_dir"],
    )


def uv_to_3d(u, v, pca_info):
    return (
        pca_info["center"]
        + float(u) * pca_info["u_dir"]
        + float(v) * pca_info["v_dir"]
    )


def compute_dynamic_bins(uv, config: PipelineConfig):
    u_range = float(np.ptp(uv[:, 0]))
    v_range = float(np.ptp(uv[:, 1]))
    u_bins = int(
        np.clip(
            np.ceil(u_range / config.slice_width_u),
            config.edge_u_bins_min,
            config.edge_u_bins_max,
        )
    )
    v_bins = int(
        np.clip(
            np.ceil(v_range / config.slice_width_v),
            config.edge_v_bins_min,
            config.edge_v_bins_max,
        )
    )
    return u_bins, v_bins


def line_from_two_points(point_1, point_2):
    point_1 = np.asarray(point_1, dtype=np.float64)
    point_2 = np.asarray(point_2, dtype=np.float64)
    direction = point_2 - point_1
    if np.linalg.norm(direction) < 1e-12:
        return None
    a = direction[1]
    b = -direction[0]
    norm = np.hypot(a, b)
    if norm < 1e-12:
        return None
    a /= norm
    b /= norm
    c = -(a * point_1[0] + b * point_1[1])
    return np.asarray([a, b, c], dtype=np.float64)


def line_distances(points_2d, line):
    a, b, c = line
    return np.abs(a * points_2d[:, 0] + b * points_2d[:, 1] + c)


def refit_line_svd(points_2d):
    points_2d = np.asarray(points_2d, dtype=np.float64)
    if len(points_2d) < 2:
        return None
    center = np.mean(points_2d, axis=0)
    _, _, right_vectors = np.linalg.svd(points_2d - center, full_matrices=False)
    direction = normalize_vector(right_vectors[0])
    normal = normalize_vector(
        np.asarray([direction[1], -direction[0]], dtype=np.float64)
    )
    a, b = normal
    c = -(a * center[0] + b * center[1])
    return np.asarray([a, b, c], dtype=np.float64)


def ransac_fit_line_2d(points_2d, config: PipelineConfig, rng):
    points_2d = np.asarray(points_2d, dtype=np.float64)
    if len(points_2d) < max(2, config.line_min_inliers):
        return None

    best_inliers = None
    best_count = 0
    for _ in range(config.line_ransac_iterations):
        first, second = rng.choice(len(points_2d), size=2, replace=False)
        line = line_from_two_points(points_2d[first], points_2d[second])
        if line is None:
            continue
        inliers = np.flatnonzero(
            line_distances(points_2d, line) <= config.line_ransac_distance
        )
        if len(inliers) > best_count:
            best_count = len(inliers)
            best_inliers = inliers

    if best_inliers is None or len(best_inliers) < config.line_min_inliers:
        return None

    refined_line = refit_line_svd(points_2d[best_inliers])
    if refined_line is None:
        return None

    refined_inliers = np.flatnonzero(
        line_distances(points_2d, refined_line) <= config.line_ransac_distance
    )
    if len(refined_inliers) < config.line_min_inliers:
        return None

    return refit_line_svd(points_2d[refined_inliers])


def line_intersection(line_1, line_2):
    if line_1 is None or line_2 is None:
        return None
    a_1, b_1, c_1 = line_1
    a_2, b_2, c_2 = line_2
    coefficients = np.asarray([[a_1, b_1], [a_2, b_2]], dtype=np.float64)
    values = np.asarray([-c_1, -c_2], dtype=np.float64)
    if abs(np.linalg.det(coefficients)) < 1e-9:
        return None
    return np.linalg.solve(coefficients, values)


def pick_edge_point(points_bin, edge_type):
    points_bin = np.asarray(points_bin, dtype=np.float64)
    if points_bin.ndim != 2 or points_bin.shape[1] != 2 or len(points_bin) == 0:
        raise ValueError(f"边缘候选输入必须为非空 [N,2]，当前为 {points_bin.shape}")

    if edge_type == "v_min":
        return points_bin[np.argmin(points_bin[:, 1])]
    if edge_type == "v_max":
        return points_bin[np.argmax(points_bin[:, 1])]
    if edge_type == "u_min":
        return points_bin[np.argmin(points_bin[:, 0])]
    if edge_type == "u_max":
        return points_bin[np.argmax(points_bin[:, 0])]
    raise ValueError(f"不支持的边缘类型：{edge_type}")


def extract_edge_candidates_2d(uv, config: PipelineConfig):
    uv = np.asarray(uv, dtype=np.float64)
    if uv.ndim != 2 or uv.shape[1] != 2 or len(uv) == 0:
        raise ValueError(f"UV 点必须为非空 [N,2]，当前为 {uv.shape}")

    u = uv[:, 0]
    v = uv[:, 1]
    u_bin_count, v_bin_count = compute_dynamic_bins(uv, config)
    candidates = {"v_min": [], "v_max": [], "u_min": [], "u_max": []}

    # 沿 U（长度方向）切片，找 Vmin / Vmax 两条纵向侧边。
    u_edges = np.linspace(np.min(u), np.max(u), u_bin_count + 1)
    for index in range(u_bin_count):
        upper_test = (
            u <= u_edges[index + 1]
            if index == u_bin_count - 1
            else u < u_edges[index + 1]
        )
        points_bin = uv[(u >= u_edges[index]) & upper_test]
        if len(points_bin) < config.slice_min_points:
            continue
        candidates["v_min"].append(pick_edge_point(points_bin, "v_min"))
        candidates["v_max"].append(pick_edge_point(points_bin, "v_max"))

    # 沿 V（宽度方向）切片，找 Umin / Umax 车头端与车尾端。
    v_edges = np.linspace(np.min(v), np.max(v), v_bin_count + 1)
    for index in range(v_bin_count):
        upper_test = (
            v <= v_edges[index + 1]
            if index == v_bin_count - 1
            else v < v_edges[index + 1]
        )
        points_bin = uv[(v >= v_edges[index]) & upper_test]
        if len(points_bin) < config.slice_min_points:
            continue
        candidates["u_min"].append(pick_edge_point(points_bin, "u_min"))
        candidates["u_max"].append(pick_edge_point(points_bin, "u_max"))

    return {
        name: np.asarray(points, dtype=np.float64)
        for name, points in candidates.items()
    }


def radar_to_world(points):
    """提取角点后将雷达 XYZ(mm) 转到固定 WORLD；不改变点序和高度。"""
    points = np.asarray(points, dtype=np.float64)
    angle = np.deg2rad(-90.7)
    c = np.cos(angle)
    s = np.sin(angle)
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    world_x = x * c - y * s + 13939
    world_y = x * s + y * c + 7715
    world_z = z
    return np.column_stack((world_x, world_y, world_z))


def compute_geometry_result(corners_3d, corner_start: int = 1):
    """将提取的雷达角点转到 WORLD，再计算尺寸、高度及姿态。"""

    corners = np.asarray(corners_3d, dtype=np.float64)
    if corners.shape != (4, 3):
        raise ValueError(f"corners_3d 必须为 [4,3]，当前为 {corners.shape}")
    radar_corners = corners
    corners = radar_to_world(radar_corners)

    p1, p2, p3, p4 = corners

    # U = length
    length_mm = (
        np.linalg.norm(p3 - p1) + np.linalg.norm(p4 - p2)
    ) / 2.0

    # V = width
    width_mm = (
        np.linalg.norm(p2 - p1) + np.linalg.norm(p4 - p3)
    ) / 2.0

    # Umin = 车头端；Umax = 车尾端。
    u_min_center = np.mean((p1, p2), axis=0)
    u_max_center = np.mean((p3, p4), axis=0)
    u_vector = u_max_center - u_min_center

    # 纵向倾斜角：车头 -> 车尾，相对于水平面。
    delta_height = float(u_vector[2])
    horizontal_run = float(np.linalg.norm(u_vector[:2]))
    tilt_angle_deg = (
        0.0
        if horizontal_run < 1e-9
        else float(np.degrees(np.arctan2(delta_height, horizontal_run)))
    )

    # 水平偏移角：Umin -> Umax 相对于理论 world -Y。
    u_xy = np.asarray([u_vector[0], u_vector[1]], dtype=np.float64)
    offset_angle_deg = (
        0.0
        if np.linalg.norm(u_xy) < 1e-9
        else float(np.degrees(np.arctan2(u_xy[0], -u_xy[1])))
    )

    corner_ids = [f"P{corner_start + i}" for i in range(4)]
    corners_rounded = [
        [round_measurement(float(value)) for value in point]
        for point in corners
    ]

    return {
        "coordinate_frame": "world",
        "coordinate_unit": "mm",
        "corner_ids": corner_ids,
        "corners_xyz_mm": corners_rounded,
        "radar_corners_xyz_mm": [[round_measurement(float(value)) for value in point] for point in radar_corners],
        "length_mm": round_measurement(float(length_mm)),
        "width_mm": round_measurement(float(width_mm)),
        "height_mean_mm": round_measurement(float(np.mean(corners[:, 2]))),
        "tilt_angle_deg": round_measurement(float(tilt_angle_deg)),
        "offset_angle_deg": round_measurement(float(offset_angle_deg)),
    }


def compute_first_workface_loading_plan(
    board_length_mm: float,
    pallet_length_mm: float,
    pad_trigger_ratio: float,
):
    """计算两块底板场景下 label_2 第一作业面的剩余空间规划。"""

    board_length_mm = float(board_length_mm)
    pallet_length_mm = float(pallet_length_mm)
    pad_trigger_ratio = float(pad_trigger_ratio)

    if board_length_mm < 0:
        raise ValueError("board_length_mm 不能为负数")
    if pallet_length_mm <= 0:
        raise ValueError("pallet_length_mm 必须大于 0")
    if not 0.0 <= pad_trigger_ratio <= 1.0:
        raise ValueError("pad_trigger_ratio 必须位于 [0, 1]")

    normal_pallet_count = int(np.floor(board_length_mm / pallet_length_mm))
    remaining_length_mm = board_length_mm - normal_pallet_count * pallet_length_mm

    if abs(remaining_length_mm) < 1e-6:
        remaining_length_mm = 0.0

    remaining_ratio = remaining_length_mm / pallet_length_mm

    if remaining_length_mm <= 1e-6:
        pad_required = False
        compensation_mm = 0.0
        final_pallet_count = normal_pallet_count
    elif remaining_ratio >= pad_trigger_ratio:
        pad_required = True
        compensation_mm = pallet_length_mm - remaining_length_mm
        final_pallet_count = normal_pallet_count + 1
    else:
        pad_required = False
        compensation_mm = 0.0
        final_pallet_count = normal_pallet_count

    return {
        "pallet_length_mm": round_measurement(pallet_length_mm),
        "normal_pallet_count": normal_pallet_count,
        "remaining_length_mm": round_measurement(float(remaining_length_mm)),
        "remaining_ratio": round(float(remaining_ratio), 6),
        "pad_trigger_ratio": round(float(pad_trigger_ratio), 6),
        "pad_required": pad_required,
        "remaining_workface_compensation_mm": round_measurement(float(compensation_mm)),
        "final_pallet_count": final_pallet_count,
    }



def _withhold_corner_z(board, report):
    """Keep XY diagnostics without publishing an invented corner height."""
    radar = np.asarray(board.get('global_plane_radar_corners_xyz_mm') or
                       board.get('radar_corners_xyz_mm') or [], float)
    if radar.shape == (4, 3):
        xy_geometry = radar.copy()
        xy_geometry[:, 2] = 0.
        metrics = compute_geometry_result(xy_geometry, 1)
        for key in ('length_mm', 'width_mm', 'tilt_angle_deg', 'offset_angle_deg'):
            board[key] = metrics[key]
    board['available_xy_corners_radar_mm'] = radar[:, :2].tolist() if radar.shape == (4, 3) else []
    board['corners_xyz_mm'] = []
    board['radar_corners_xyz_mm'] = []
    board['height_mean_mm'] = None
    board['corner_height_mean_mm'] = None
    board['corner_height_all_supported'] = False
    board['corner_height_estimation'] = [dict(status='height_unavailable', reason=report.get('reason')) for _ in range(4)]
    board.pop('loading_plan', None)


def merge_sparse_low_deck_labels(labels):
    """Merge fewer than 30000 label-3 points into label 2 without changing the source."""
    corrected = np.asarray(labels).copy()
    low_mask = corrected == 3
    low_count = int(np.count_nonzero(low_mask))
    high_count = int(np.count_nonzero(corrected == 2))
    applied = 0 < low_count < LOW_DECK_LABEL_MERGE_THRESHOLD
    if applied:
        corrected[low_mask] = 2
    return corrected, dict(rule='sparse_label_3_to_label_2',
        threshold_points=LOW_DECK_LABEL_MERGE_THRESHOLD, applied=applied,
        merged_points=low_count if applied else 0,
        label_2_points_before=high_count, label_2_points_after=high_count+(low_count if applied else 0),
        label_3_points_before=low_count, label_3_points_after=0 if applied else low_count)


def process_segmented_points_timed(xyz, labels, config: PipelineConfig):
    if config.height_scheme not in ('A', 'B', 'C', 'D'):
        raise ValueError('height_scheme must be A, B, C, or D')
    if config.geometry_scheme != 2:
        raise ValueError('geometry_scheme only supports 2; scheme 1 has been removed')
    labels, label_correction = merge_sparse_low_deck_labels(labels)
    from .shared_deck_geometry import process_shared_decks_timed
    result, rows, seconds = process_shared_decks_timed(xyz, labels, config)
    result['label_correction'] = label_correction
    result, height_rows, height_seconds = apply_deck_reference_heights_timed(xyz, labels, result, config)
    return result, rows + height_rows, seconds + height_seconds


def apply_deck_reference_heights_timed(xyz, labels, result, config: PipelineConfig):
    """Apply H and corner Z to existing geometry without refitting XY or labels."""
    if config.height_scheme not in ('A', 'B', 'C', 'D'):
        raise ValueError('height_scheme must be A, B, C, or D')
    rows, seconds = [], 0.
    # An unsuccessful paired-deck fit does not establish a single-board truck.
    # Preserve available corners, but defer H until the board type is supported.
    if result.get('geometry_status') == 'partial':
        for label_name in ('label_2', 'label_3'):
            board = result.get(label_name)
            if isinstance(board, dict):
                board['height_estimation'] = dict(
                    height_scheme=config.height_scheme, method='geometry_incomplete',
                    status='geometry_incomplete', height_mm=None,
                    reason='board_type_not_confirmed_after_geometry_failure')
                _withhold_corner_z(board, board['height_estimation'])
        return result, rows, seconds
    has_high = isinstance(result.get('label_2'), dict)
    has_low = isinstance(result.get('label_3'), dict)
    if has_high and has_low:
        height_targets = [('label_2', 2, 'high', config.high_height_length_fraction, config.high_low_height_width_fraction),
                          ('label_3', 3, 'low', config.low_height_length_fraction, config.high_low_height_width_fraction)]
    elif has_high:
        height_targets = [('label_2', 2, 'single', config.single_height_length_fraction, config.single_height_width_fraction)]
    else:
        height_targets = []
    for label_name, label_id, role, fraction, width in height_targets:
        from .deck_height_methods import estimate_deck_height
        board = result[label_name]
        start = time.perf_counter()
        source_corners = board.get('height_source_radar_corners_xyz_mm', board['radar_corners_xyz_mm'])
        if source_corners is None:
            report = dict(height_scheme=config.height_scheme, method='front_five_strip_planes',
                          status='unsupported', height_mm=None, reason='final_height_crop_unavailable',
                          length_fraction=fraction, width_fraction=width, strips=[])
        else:
            report = estimate_deck_height(
                np.asarray(xyz)[np.asarray(labels) == label_id], source_corners,
                scheme=config.height_scheme, length_fraction=fraction, width_fraction=width,
                length_start_fraction=config.height_length_start_fraction,
                width_start_fraction=config.height_width_start_fraction,
                min_points=config.single_height_min_points,
                min_tolerance=config.single_height_min_tolerance,
                max_tolerance=config.single_height_max_tolerance,
                iterations=config.single_height_iterations,
                max_slope=config.single_height_max_slope, seed=config.seed)
        # World conversion is yaw + XY translation: Z is unchanged.
        plane_corners = board.get('global_plane_radar_corners_xyz_mm')
        board['geometry_plane_height_mean_mm'] = (float(np.mean(np.asarray(plane_corners, float)[:, 2]))
            if plane_corners is not None else board['height_mean_mm'])
        report.update(board_role=role, crop_start='P5_P6_low_usable_start' if role == 'low' else 'this_deck_head',
                      height_reference='deck_region_reference', selection_basis='existing_successful_geometry_board_count',
                      length_fraction_of='this_deck', crop_start_corner_ids=board['corner_ids'][:2],
                      width_start='P5_P7_near_scanner_side' if role == 'low' else 'P1_P3_near_scanner_side')
        board['height_estimation'] = report
        report['height_crop_basis'] = 'final_this_deck_geometry'
        if report['status'] == 'supported' and report['height_mm'] is not None:
            height = round_measurement(float(report['height_mm']))
            # Never feed rounded published XY back into fitting or WORLD conversion.
            flat_corners = np.asarray(board.get('global_plane_radar_corners_xyz_mm',
                                                board['radar_corners_xyz_mm']), float).copy()
            flat_corners[:, 2] = height
            board.update(compute_geometry_result(flat_corners, 5 if label_id == 3 else 1))
            board['corner_height_mean_mm'] = height
            board['corner_height_all_supported'] = True
            board['corner_height_estimation'] = [dict(status='deck_reference_height', height_mm=height,
                height_scheme=config.height_scheme, used_local_corner_fit=False) for _ in range(4)]
            report['height_mm'] = height
            for strip in report.get('strips', []):
                if strip.get('height_mm') is not None:
                    strip['height_mm'] = round_measurement(strip['height_mm'])
        else:
            _withhold_corner_z(board, report)
            result['geometry_status'] = 'partial'
            result.setdefault('geometry_failures', []).append(dict(board_label=label_name,
                reason='height_reference_unavailable:'+str(report.get('reason', 'unsupported'))))
        elapsed = time.perf_counter() - start
        rows.append((label_name, f'底板参考高度{config.height_scheme}', elapsed))
        seconds += elapsed
    if has_high and has_low and result.get('geometry_status') != 'partial':
        high_board = result['label_2']
        plan_length = high_board['length_mm']
        raw_corners = high_board.get('global_plane_radar_corners_xyz_mm')
        if raw_corners is not None:
            xy = np.asarray(raw_corners, float)[:, :2]
            plan_length = float((np.linalg.norm(xy[2]-xy[0]) + np.linalg.norm(xy[3]-xy[1]))/2)
        result['label_2']['loading_plan'] = compute_first_workface_loading_plan(
            plan_length, config.pallet_length_mm, config.pad_trigger_ratio)
    return result, rows, seconds


class PointCloudPipeline:
    """持有一次加载的 PointNet++ 模型，并处理单个或多个 PCD。"""

    def __init__(
        self,
        config: PipelineConfig | None = None,
        checkpoint_path: str | None = None,
        device: str | None = None,
        segmenter_factory=None,
    ) -> None:
        self.config = config or PipelineConfig()
        factory = segmenter_factory or PointNet2Segmenter
        if factory is None:
            raise RuntimeError(
                f"无法导入 point_cloud_segment_module：{POINTNET_IMPORT_ERROR}"
            )
        self.segmenter = factory(
            checkpoint_path=checkpoint_path,
            device=device,
            num_point=self.config.pointnet_num_point,
            num_votes=self.config.pointnet_num_votes,
            seed=self.config.seed,
        )

    def segment_xyz_full_coverage(self, xyz):
        """Fixed-size full-truck groups; every retained point is directly predicted."""

        dense_xyz = np.asarray(xyz, dtype=np.float32)
        if dense_xyz.ndim != 2 or dense_xyz.shape[1] != 3 or len(dense_xyz) == 0 or not np.isfinite(dense_xyz).all():
            raise ValueError(f"整云推理输入必须为非空 [N,3]，当前为 {dense_xyz.shape}")

        dense_count = int(len(dense_xyz))

        prepare_start = time.perf_counter()
        # Canonicalize once for repeatability; each sampled group is shuffled.
        # Training normalizes the complete cloud before selecting points.
        canonical_order = np.lexsort((dense_xyz[:, 2], dense_xyz[:, 1], dense_xyz[:, 0]))
        canonical_xyz = dense_xyz[canonical_order].copy()
        normalized_xyz = pc_normalize(canonical_xyz.copy())
        cls_label = torch.zeros((1, 1), dtype=torch.long, device=self.segmenter.device)
        cls_one_hot = torch.eye(
            DEFAULT_NUM_CLASSES,
            device=self.segmenter.device,
        )[cls_label.cpu().numpy()]
        prepare_s = time.perf_counter() - prepare_start

        inference_start = time.perf_counter()
        vote_count = max(int(self.segmenter.num_votes), 1)
        probability_sum = np.zeros((dense_count, DEFAULT_NUM_PART), dtype=np.float64)
        coverage = np.zeros(dense_count, dtype=np.int64)
        group_count = 0
        group_points = int(self.config.pointnet_num_point)

        device = torch.device(self.segmenter.device)
        cuda_devices = (
            [device.index if device.index is not None else torch.cuda.current_device()]
            if device.type == "cuda" else []
        )
        backend_flags = (
            torch.backends.cudnn.flags(benchmark=False, deterministic=True, allow_tf32=False)
            if cuda_devices else nullcontext()
        )
        try:
            with _INFERENCE_RANDOM_LOCK, torch.random.fork_rng(devices=cuda_devices), backend_flags, torch.inference_mode():
                torch.random.default_generator.manual_seed(self.config.seed)
                if cuda_devices:
                    with torch.cuda.device(cuda_devices[0]):
                        torch.cuda.manual_seed(self.config.seed)
                self.segmenter.model.eval()
                for indices in iter_full_coverage_samples(
                    canonical_xyz, group_points, self.config.seed,
                    self.config.pointnet_context_fraction,
                ):
                    points = torch.from_numpy(normalized_xyz[indices]).float().unsqueeze(0).to(self.segmenter.device).transpose(2, 1)
                    for _ in range(vote_count):
                        seg_pred, _ = self.segmenter.model(points, cls_one_hot)
                        if tuple(seg_pred.shape) != (1, group_points, DEFAULT_NUM_PART):
                            raise ValueError(f'PointNet++ fixed group output mismatch: {tuple(seg_pred.shape)}')
                        probabilities = torch.exp(seg_pred[0]).cpu().numpy()
                        if not np.isfinite(probabilities).all():
                            raise ValueError('PointNet++ produced nonfinite probabilities')
                        # add.at includes every occurrence of repeated source indices.
                        # These are each point's own network predictions, not KNN labels.
                        np.add.at(probability_sum, indices, probabilities)
                        np.add.at(coverage, indices, 1)
                    group_count += 1
        except torch.cuda.OutOfMemoryError as exc:
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            raise RuntimeError(
                f"固定点数组推理显存不足：每次 {group_points} 点。"
                "未输出不完整标签。"
            ) from exc

        if not np.all(coverage > 0):
            raise RuntimeError('Direct inference coverage incomplete; refusing missing-point labels')
        averaged_scores = probability_sum / coverage[:, None]
        inference_s = time.perf_counter() - inference_start

        restore_start = time.perf_counter()
        dense_labels = np.argmax(averaged_scores, axis=1).astype(np.int32)
        source_labels = np.empty_like(dense_labels)
        source_labels[canonical_order] = dense_labels
        dense_labels = source_labels
        self.last_direct_prediction_counts = np.empty_like(coverage)
        self.last_direct_prediction_counts[canonical_order] = coverage
        restore_s = time.perf_counter() - restore_start

        label_counts = {
            label: int(np.count_nonzero(dense_labels == label))
            for label in range(DEFAULT_NUM_PART)
        }

        print(
            f"    PointNet++ 全点覆盖: {inference_s:.3f} s | "
            f"每组 {group_points} 点 × {group_count} 组 | 原点 {dense_count} | 无KNN"
        )

        return dense_labels, label_counts, {
            "device": str(self.segmenter.device),
            "dense_count": dense_count,
            "group_points": group_points,
            "group_count": group_count,
            "forward_count": group_count * vote_count,
            "model_point_occurrences": int(coverage.sum()),
            "coverage_min": int(coverage.min()),
            "coverage_max": int(coverage.max()),
            "all_points_directly_predicted": True,
            "normalization": "full_cloud_before_sampling",
            "label_source": "mean_of_own_network_probabilities",
            "context_fraction": self.config.pointnet_context_fraction,
            "prepare_s": round(float(prepare_s), 6),
            "inference_s": round(float(inference_s), 6),
            "restore_s": round(float(restore_s), 6),
        }

    def process_file(self, input_path: str | Path, output_root: str | Path):
        input_path = Path(input_path)
        output_root = Path(output_root)
        file_start = time.perf_counter()

        point_cloud = read_point_cloud(input_path)
        filtered, stage_counts, preprocess_timings = preprocess_point_cloud_timed(
            point_cloud,
            self.config,
        )

        dense_xyz = np.asarray(filtered.points, dtype=np.float32)
        dense_labels, dense_label_counts, pointnet_stats = self.segment_xyz_full_coverage(
            dense_xyz
        )

        result, geometry_steps, geometry_s = process_segmented_points_timed(
            dense_xyz,
            dense_labels,
            self.config,
        )

        json_path = output_root / "json" / f"{input_path.stem}.json"
        write_json(json_path, result)

        geometry_failed = result.get('geometry_status') in ('partial', 'unsupported')
        geometry_error = '; '.join(str(item.get('reason', '')) for item in result.get('geometry_failures', []))
        completion_label = '几何未通过' if geometry_failed else '完成'
        total_s = time.perf_counter() - file_start
        print(
            f"[{completion_label}] {input_path.name}: 总 {total_s:.3f} s | "
            f"PointNet++ {pointnet_stats['inference_s']:.3f} s | "
            f"几何后处理 {geometry_s:.3f} s"
        )

        return {
            "input": str(input_path),
            "status": "failed" if geometry_failed else "success",
            **({'error': geometry_error} if geometry_failed else {}),
            "json": str(json_path),
            "stage_counts": stage_counts,
            "dense_label_counts": dense_label_counts,
            "preprocess_timing": {
                key: round(float(value), 6)
                for key, value in preprocess_timings.items()
            },
            "pointnet_timing": pointnet_stats,
            "geometry_steps": [
                {
                    "label": label,
                    "step": step,
                    "seconds": round(float(seconds), 6),
                }
                for label, step, seconds in geometry_steps
            ],
            "geometry_s": round(float(geometry_s), 6),
            "total_s": round(float(total_s), 6),
            "result": result,
        }

    def process_path(self, input_path: str | Path, output_root: str | Path):
        files = collect_pcd_files(input_path)
        resolved_output = Path(output_root).expanduser().resolve()
        resolved_output.mkdir(parents=True, exist_ok=True)

        records = []
        batch_start = time.perf_counter()

        for index, path in enumerate(files, start=1):
            print("=" * 78)
            print(f"[{index}/{len(files)}] 开始处理：{path.name}")
            one_start = time.perf_counter()
            try:
                records.append(self.process_file(path, resolved_output))
            except Exception as exc:
                elapsed = time.perf_counter() - one_start
                print(
                    f"[失败] {path.name}: {elapsed:.3f} s | "
                    f"{type(exc).__name__}: {exc}"
                )
                records.append(
                    {
                        "input": str(path),
                        "status": "failed",
                        "error": f"{type(exc).__name__}: {exc}",
                        "total_s": round(float(elapsed), 6),
                    }
                )

        batch_total_s = time.perf_counter() - batch_start
        summary = {
            "input": str(Path(input_path).expanduser().resolve()),
            "output": str(resolved_output),
            "total": len(records),
            "succeeded": sum(row["status"] == "success" for row in records),
            "failed": sum(row["status"] == "failed" for row in records),
            "batch_total_s": round(float(batch_total_s), 6),
            "files": records,
        }

        print("=" * 78)
        print(
            f"处理完成：成功 {summary['succeeded']} / {summary['total']}，"
            f"总耗时 {batch_total_s:.3f} s"
        )
        print(f"JSON目录：{resolved_output / 'json'}")
        return summary


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="PCD -> PointNet++ -> 底板角点/姿态/装载规划 -> JSON"
    )
    parser.add_argument("input", help="单个 PCD 文件或包含 PCD 的目录")
    parser.add_argument(
        "-o",
        "--output",
        default="point_cloud_output",
        help="输出根目录；最终只写 output/json/*.json",
    )
    parser.add_argument("--device", choices=("cpu", "cuda"), default=None)
    parser.add_argument("--checkpoint", default=None, help="模型权重路径")
    parser.add_argument('--geometry-scheme', type=int, choices=(2,), default=DEFAULT_CONFIG.geometry_scheme,
                        help='仅支持2：高板参考共同方向和宽度')

    # 保留常用 CLI 覆盖项；其余参数直接在 PipelineConfig 集中修改。
    parser.add_argument(
        "--num-point",
        type=int,
        default=DEFAULT_CONFIG.pointnet_num_point,
    )
    parser.add_argument(
        "--num-votes",
        type=int,
        default=DEFAULT_CONFIG.pointnet_num_votes,
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=DEFAULT_CONFIG.voxel_size,
    )
    parser.add_argument(
        "--plane-threshold",
        type=float,
        default=DEFAULT_CONFIG.plane_threshold,
    )
    parser.add_argument(
        "--dbscan-eps",
        type=float,
        default=DEFAULT_CONFIG.dbscan_eps,
    )
    parser.add_argument(
        "--dbscan-min-points",
        type=int,
        default=DEFAULT_CONFIG.dbscan_min_points,
    )
    return parser


def main(argv=None) -> int:
    args = build_argument_parser().parse_args(argv)

    config = PipelineConfig(
        geometry_scheme=args.geometry_scheme,
        pointnet_num_point=args.num_point,
        pointnet_num_votes=args.num_votes,
        voxel_size=args.voxel_size,
        plane_threshold=args.plane_threshold,
        dbscan_eps=args.dbscan_eps,
        dbscan_min_points=args.dbscan_min_points,
    )

    pipeline = PointCloudPipeline(
        config=config,
        checkpoint_path=args.checkpoint,
        device=args.device,
    )
    summary = pipeline.process_path(args.input, args.output)
    return 0 if summary["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
