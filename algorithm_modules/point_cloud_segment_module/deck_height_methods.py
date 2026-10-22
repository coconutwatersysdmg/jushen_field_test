"""Switchable deck-reference heights; no changes to corner XYZ or board type.

A: five strip planes (existing default), B: area-balanced strip planes,
C: five center-neighborhood medians, D: one whole-region plane.
All regions start at this deck's head/usable-start and scanner-side edge.
"""
from __future__ import annotations

import numpy as np

from .single_board_height import _quad_frame, _strip_plane, estimate_single_board_height


METHODS = {
    'A': 'front_five_strip_planes',
    'B': 'grid_balanced_five_strip_planes',
    'C': 'five_center_neighborhood_medians',
    'D': 'whole_region_plane',
}


def _weighted_median(values, weights):
    order = np.argsort(values, kind='stable')
    cumulative = np.cumsum(weights[order])
    return float(values[order[np.searchsorted(cumulative, cumulative[-1] * .5)]])


def _balanced_plane(points, uv, center, bounds, initial, fit):
    if initial['status'] != 'supported':
        return dict(status='unsupported', reason='strip_plane_seed_unsupported')
    lower, upper = np.asarray(bounds[0]), np.asarray(bounds[1])
    pos = (uv - lower) / (upper - lower)
    grid = np.clip((pos * [5, 10]).astype(int), 0, [4, 9])
    cell = grid[:, 0] * 10 + grid[:, 1]
    scale = max(float(np.linalg.norm(np.ptp(points[:, :2], axis=0))), 1.)
    design = np.column_stack([(points[:, :2] - center) / scale, np.ones(len(points))])
    original = np.asarray(initial['plane_radar_z_equals_ax_by_c'])
    coeff = np.r_[original[:2] * scale, center @ original[:2] + original[2]]
    threshold = initial['threshold_mm']
    for _ in range(7):
        residual = points[:, 2] - design @ coeff
        inside = np.abs(residual) <= threshold
        if inside.sum() < fit['min_points']:
            return dict(status='unsupported', reason='insufficient_balanced_support')
        counts = np.bincount(cell[inside], minlength=50)
        weights = 1. / counts[cell[inside]]
        a = design[inside] * np.sqrt(weights[:, None])
        z = points[inside, 2] * np.sqrt(weights)
        coeff, _, rank, _ = np.linalg.lstsq(a, z, rcond=None)
        if rank < 3 or np.linalg.norm(coeff[:2] / scale) > fit['max_slope']:
            return dict(status='unsupported', reason='degenerate_balanced_plane')
        r = points[inside, 2] - design[inside] @ coeff
        sigma = 1.4826 * _weighted_median(np.abs(r - _weighted_median(r, weights)), weights)
        threshold = float(np.clip(3.5 * sigma, fit['min_tolerance'], fit['max_tolerance']))
    residual = points[:, 2] - design @ coeff
    inside = np.abs(residual) <= threshold
    if inside.sum() < fit['min_points'] or inside.mean() < .5:
        return dict(status='unsupported', reason='insufficient_balanced_consensus')
    spans = np.percentile(pos[inside], 95, axis=0) - np.percentile(pos[inside], 5, axis=0)
    cells = np.clip((pos[inside] * [3, 5]).astype(int), 0, [2, 4])
    coverage = float(np.mean(np.bincount(cells[:, 0] * 5 + cells[:, 1], minlength=15) >= 3))
    rms = float(np.sqrt(np.mean(residual[inside] ** 2)))
    if spans[0] < .55 or spans[1] < .65 or coverage < .6 or rms > fit['max_tolerance'] * .6:
        return dict(status='unsupported', reason='balanced_plane_quality')
    return dict(status='supported', height_mm=float(coeff[2]), points=int(len(points)),
                inlier_points=int(inside.sum()), residual_rms_mm=rms,
                occupied_grid_fraction=coverage, threshold_mm=threshold,
                plane_radar_z_equals_ax_by_c=[float(coeff[0] / scale), float(coeff[1] / scale),
                    float(coeff[2] - center @ (coeff[:2] / scale))],
                weighting='all accepted points; equal total weight per occupied 5x10 cell')


def _center_median(points, center, fit):
    distance = np.linalg.norm(points[:, :2] - center, axis=1)
    for radius in (250., 400., 600.):
        local = points[distance <= radius]
        if len(local) < fit['min_points']:
            continue
        z = local[:, 2]
        mask = np.ones(len(z), bool)
        for _ in range(4):
            if mask.sum() < fit['min_points']:
                break
            median = float(np.median(z[mask]))
            sigma = float(1.4826 * np.median(np.abs(z[mask] - median)))
            tolerance = float(np.clip(3.5 * sigma, fit['min_tolerance'], fit['max_tolerance']))
            mask = np.abs(z - median) <= tolerance
        if mask.sum() < fit['min_points'] or mask.mean() < .5:
            continue
        xy = (local[mask, :2] - center) / radius
        quadrants = (xy[:, 0] >= 0).astype(int) * 2 + (xy[:, 1] >= 0).astype(int)
        if np.any(np.bincount(quadrants, minlength=4) < 3):
            continue
        return dict(status='supported', height_mm=float(np.median(z[mask])),
                    points=int(len(local)), inlier_points=int(mask.sum()), radius_mm=radius,
                    robust_z_sigma_mm=float(1.4826 * np.median(np.abs(z[mask] - np.median(z[mask])))))
    return dict(status='unsupported', reason='insufficient_center_neighborhood')


def estimate_deck_height(points, corners, *, scheme='A', length_fraction=.6,
                         width_fraction=.95, min_points=40, min_tolerance=15.,
                         max_tolerance=40., iterations=160, max_slope=.5, seed=42):
    """Estimate this deck's reference height; never extrapolate it to corners."""
    if scheme not in METHODS:
        raise ValueError('height_scheme must be A, B, C, or D')
    fit = dict(min_points=min_points, min_tolerance=min_tolerance,
               max_tolerance=max_tolerance, iterations=iterations, max_slope=max_slope)
    # Preserve the validated A path exactly, including its quality checks.
    report = estimate_single_board_height(points, corners, length_fraction=length_fraction,
                                         width_fraction=width_fraction, seed=seed, **fit)
    report['height_scheme'] = scheme
    if scheme == 'A':
        return report
    report['method'] = METHODS[scheme]
    report['height_mm'] = None
    report['status'] = 'unsupported'
    forward, inverse = _quad_frame(np.asarray(corners, float))
    points = np.asarray(points, float)
    points = points[np.isfinite(points).all(axis=1)]
    uv = inverse(points[:, :2])
    keep = (uv[:, 0] >= -1e-8) & (uv[:, 0] <= length_fraction + 1e-8) & (uv[:, 1] >= -1e-8) & (uv[:, 1] <= width_fraction + 1e-8)
    points, uv = points[keep], uv[keep]
    order = np.lexsort((points[:, 2], uv[:, 1], uv[:, 0]))
    points, uv = points[order], uv[order]
    results = []
    if scheme in ('B', 'C'):
        edges = np.linspace(0, length_fraction, 6)
        for i in range(5):
            lo, hi = edges[i:i+2]
            selected = (uv[:, 0] >= lo - (1e-8 if i == 0 else 0)) & (uv[:, 0] <= hi + 1e-8 if i == 4 else uv[:, 0] < hi)
            center = forward([(lo + hi) / 2, width_fraction / 2])
            if scheme == 'B':
                result = _balanced_plane(points[selected], uv[selected], center,
                                         [[lo, 0], [hi, width_fraction]], report['strips'][i], fit)
            else:
                result = _center_median(points[selected], center, fit)
            result.update(index=i+1, length_range_fraction=[float(lo), float(hi)],
                          center_radar_xyz_mm=[float(center[0]), float(center[1]), result.get('height_mm')])
            results.append(result)
        report['strips'] = results
    else:
        center = forward([length_fraction / 2, width_fraction / 2])
        result = _strip_plane(points, uv, center, [[0, 0], [length_fraction, width_fraction]], seed=seed, **fit)
        result['center_radar_xyz_mm'] = [float(center[0]), float(center[1]), result.get('height_mm')]
        results = [result]
        report['strips'] = []
        report['strip_count'] = 0
        report['region_plane'] = result
    if all(value['status'] == 'supported' for value in results):
        report.update(status='supported', height_mm=float(np.mean([r['height_mm'] for r in results])))
        report.pop('reason', None)
    else:
        report['reason'] = 'one_or_more_regions_unsupported'
    return report
