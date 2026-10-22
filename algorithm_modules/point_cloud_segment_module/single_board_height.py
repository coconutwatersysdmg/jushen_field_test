"""Single-deck reference H from five supported front-strip planes (mm).

P1/P3 is the scanner-side edge. XY geometry only defines the crop; corner Z
never seeds or constrains the planes. All retained same-label points in each
strip are considered, including points rejected by the global deck plane.
"""
from __future__ import annotations

import numpy as np


METHOD = 'front_five_strip_planes'


def _quad_frame(corners):
    origin = corners[0, :2]
    along = corners[2, :2] - origin
    across = corners[1, :2] - origin
    warp = corners[3, :2] - origin - along - across
    ring = corners[[0, 1, 3, 2], :2]
    edges = np.roll(ring, -1, axis=0) - ring
    cross = edges[:, 0] * np.roll(edges, -1, axis=0)[:, 1] - edges[:, 1] * np.roll(edges, -1, axis=0)[:, 0]
    basis = np.column_stack([along, across])
    if not (np.all(cross > 1e-6) or np.all(cross < -1e-6)) or np.linalg.cond(basis) > 1e4:
        raise ValueError('Height crop requires a non-degenerate convex deck quadrilateral')

    def forward(uv):
        uv = np.asarray(uv, float)
        return origin + uv[..., 0, None] * along + uv[..., 1, None] * across + np.prod(uv, axis=-1)[..., None] * warp

    def inverse(xy):
        uv = (xy - origin) @ np.linalg.inv(basis).T
        for _ in range(8):
            residual = forward(uv) - xy
            du = along + uv[:, 1, None] * warp
            dv = across + uv[:, 0, None] * warp
            det = du[:, 0] * dv[:, 1] - du[:, 1] * dv[:, 0]
            if np.any(np.abs(det) < 1e-9):
                raise ValueError('Height crop inverse is degenerate')
            uv -= np.column_stack([(residual[:, 0] * dv[:, 1] - residual[:, 1] * dv[:, 0]) / det,
                                   (du[:, 0] * residual[:, 1] - du[:, 1] * residual[:, 0]) / det])
        return uv

    return forward, inverse


def _refine(design, z, coeff, *, min_points, min_tolerance, max_tolerance, scale, max_slope):
    threshold = max_tolerance
    for _ in range(5):
        residual = z - design @ coeff
        inside = np.abs(residual) <= threshold
        if np.count_nonzero(inside) < min_points:
            return None
        coeff, _, rank, _ = np.linalg.lstsq(design[inside], z[inside], rcond=None)
        if rank < 3 or np.linalg.norm(coeff[:2] / scale) > max_slope:
            return None
        residual = z - design @ coeff
        centered = residual[inside] - np.median(residual[inside])
        sigma = 1.4826 * np.median(np.abs(centered))
        threshold = float(np.clip(3.5 * sigma, min_tolerance, max_tolerance))
    # Refit every final accepted point, not just the RANSAC hypothesis subset.
    inside = np.abs(z - design @ coeff) <= threshold
    if inside.sum() < min_points:
        return None
    coeff, _, rank, _ = np.linalg.lstsq(design[inside], z[inside], rcond=None)
    if rank < 3 or np.linalg.norm(coeff[:2] / scale) > max_slope or np.linalg.cond(design[inside]) > 1e4:
        return None
    return coeff, threshold


def _strip_plane(points, uv, center_xy, bounds, *, min_points, min_tolerance,
                 max_tolerance, iterations, max_slope, seed):
    lower, upper = np.asarray(bounds[0]), np.asarray(bounds[1])
    relative = (uv - (lower + upper) / 2) / ((upper - lower) / 2)
    scale = max(float(np.linalg.norm(np.ptp(points[:, :2], axis=0))), 1.) if len(points) else 1.
    design = np.column_stack([(points[:, :2] - center_xy) / scale, np.ones(len(points))])
    z = points[:, 2]
    report = dict(status='unsupported', reason='insufficient_center_support', points=int(len(points)))
    if len(points) < min_points:
        return report
    rng = np.random.default_rng(seed)
    coeff = None
    growth = []
    seed_points = 0
    seed_fraction = None
    # Seed at the strip center; grow a rectangle to the entire strip. The
    # geometry and all Z values remain in the original horizontal XY frame.
    for fraction in (.3, .5, .75, 1.):
        active = np.max(np.abs(relative), axis=1) <= fraction + 1e-8
        indexes = np.flatnonzero(active)
        if len(indexes) < min_points:
            continue
        if coeff is None:
            probe = indexes[np.linspace(0, len(indexes) - 1, min(len(indexes), 3000), dtype=int)]
            best = None
            best_score = (0, -np.inf)
            for _ in range(iterations):
                chosen = rng.choice(indexes, 3, replace=False)
                a = design[chosen]
                if abs(np.linalg.det(a)) < 1e-9:
                    continue
                candidate = np.linalg.solve(a, z[chosen])
                if np.linalg.norm(candidate[:2] / scale) > max_slope:
                    continue
                residual = np.abs(z[probe] - design[probe] @ candidate)
                inside = residual <= min_tolerance
                score = (int(inside.sum()), -float(np.mean(residual[inside] ** 2)) if inside.any() else -np.inf)
                if score > best_score:
                    best, best_score = candidate, score
            if best is None:
                continue
            candidate = best
        else:
            candidate = coeff
        fitted = _refine(design[active], z[active], candidate, min_points=min_points,
                         min_tolerance=min_tolerance, max_tolerance=max_tolerance,
                         scale=scale, max_slope=max_slope)
        if fitted is None:
            continue
        coeff, threshold = fitted
        if seed_fraction is None:
            seed_fraction, seed_points = fraction, int(len(indexes))
        growth.append(dict(fraction=fraction, points=int(len(indexes))))
    if coeff is None or not growth or growth[-1]['fraction'] != 1.:
        return report
    residual = z - design @ coeff
    inside = np.abs(residual) <= threshold
    count = int(inside.sum())
    report.update(seed_points=seed_points, seed_fraction=seed_fraction, growth=growth,
                  inlier_points=count, consensus_fraction=float(np.mean(inside)), threshold_mm=threshold,
                  rejected_high_points=int(np.count_nonzero(residual > threshold)),
                  rejected_low_points=int(np.count_nonzero(residual < -threshold)))
    if count < min_points or count < len(points) * .5:
        report['reason'] = 'insufficient_plane_consensus'
        return report
    supported_uv = (uv[inside] - lower) / (upper - lower)
    spans = np.percentile(supported_uv, 95, axis=0) - np.percentile(supported_uv, 5, axis=0)
    cells = np.clip((supported_uv * [3, 5]).astype(int), 0, [2, 4])
    occupancy = np.bincount(cells[:, 0] * 5 + cells[:, 1], minlength=15)
    coverage = float(np.mean(occupancy >= 3))
    nearest_center = float(np.min(np.max(np.abs(relative[inside]), axis=1)))
    rms = float(np.sqrt(np.mean(residual[inside] ** 2)))
    report.update(support_span_fraction=spans.tolist(), occupied_grid_fraction=coverage,
                  grid_inlier_counts=occupancy.tolist(), nearest_center_fraction=nearest_center,
                  residual_rms_mm=rms)
    if spans[0] < .55 or spans[1] < .65 or coverage < .6 or nearest_center > .5:
        report['reason'] = 'insufficient_spatial_coverage'
        return report
    if rms > max_tolerance * .6:
        report['reason'] = 'excessive_plane_residual'
        return report
    report.update(status='supported', reason=None, height_mm=float(coeff[2]),
                  plane_radar_z_equals_ax_by_c=[float(coeff[0] / scale), float(coeff[1] / scale),
                      float(coeff[2] - center_xy @ (coeff[:2] / scale))])
    return report


def estimate_single_board_height(points, corners, *, length_fraction=.6, width_fraction=.95,
                                 min_points=40, min_tolerance=15., max_tolerance=40.,
                                 iterations=160, max_slope=.5, seed=42):
    """Return H only if all five strips have supported center-plane heights.

    Crop by bilinear XY coordinates inside P1/P2/P3/P4 (front near/far, rear
    near/far). This also handles the mildly oblique quadrilateral of scheme 1.
    There is no compensation, corner snapping, or missing-strip fallback.
    """
    points, corners = np.asarray(points, float), np.asarray(corners, float)
    if points.ndim != 2 or points.shape[1] != 3 or corners.shape != (4, 3) or not np.isfinite(corners).all():
        raise ValueError('Expected [N,3] points and four finite radar XYZ corners')
    if not (0 < length_fraction <= 1 and 0 < width_fraction <= 1):
        raise ValueError('Height crop fractions must be in (0,1]')
    if min_points < 3 or iterations < 1 or not (0 < min_tolerance <= max_tolerance) or max_slope <= 0:
        raise ValueError('Invalid height plane support/fitting parameters')
    forward, inverse = _quad_frame(corners)
    finite = points[np.isfinite(points).all(axis=1)]
    uv = inverse(finite[:, :2])
    crop = (uv[:, 0] >= -1e-8) & (uv[:, 0] <= length_fraction + 1e-8) & (uv[:, 1] >= -1e-8) & (uv[:, 1] <= width_fraction + 1e-8)
    points, uv = finite[crop], uv[crop]
    order = np.lexsort((points[:, 2], uv[:, 1], uv[:, 0]))
    points, uv = points[order], uv[order]
    report = dict(method=METHOD, status='unsupported', height_mm=None,
                  coordinate_frame='radar', coordinate_unit='mm',
                  length_fraction=length_fraction, width_fraction=width_fraction,
                  width_start='P1_P3_near_scanner_side', strip_count=5,
                  source_points=int(len(finite)), cropped_points=int(len(points)),
                  crop_radar_xy_mm=forward([[0, 0], [0, width_fraction],
                                           [length_fraction, 0], [length_fraction, width_fraction]]).tolist(), strips=[])
    edges = np.linspace(0, length_fraction, 6)
    for i in range(5):
        lower, upper = edges[i], edges[i + 1]
        selected = (uv[:, 0] >= lower - (1e-8 if i == 0 else 0)) & (
            uv[:, 0] <= upper + 1e-8 if i == 4 else uv[:, 0] < upper)
        center = forward([(lower + upper) / 2, width_fraction / 2])
        strip = _strip_plane(points[selected], uv[selected], center, [[lower, 0], [upper, width_fraction]],
                             min_points=min_points, min_tolerance=min_tolerance,
                             max_tolerance=max_tolerance, iterations=iterations,
                             max_slope=max_slope, seed=seed + i)
        strip.update(index=i + 1, length_range_fraction=[float(lower), float(upper)],
                     center_radar_xyz_mm=[float(center[0]), float(center[1]), strip.get('height_mm')])
        report['strips'].append(strip)
    if all(strip['status'] == 'supported' for strip in report['strips']):
        report.update(status='supported', height_mm=float(np.mean([s['height_mm'] for s in report['strips']])))
    else:
        report['reason'] = 'one_or_more_strips_unsupported'
    return report
