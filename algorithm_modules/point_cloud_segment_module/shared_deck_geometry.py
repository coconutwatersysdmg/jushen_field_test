"""Projected floor geometry: head/near directions, fixed intersection anchors.

Only label-2/3 floor points establish width. Joint points classify support,
but never become floor or height-source points.
"""
import time
import numpy as np


class UnsupportedGeometry(ValueError):
    """Insufficient measured support for constrained geometry."""


def _uv(points, origin, u, v):
    return (np.asarray(points)[:, :2]-origin) @ np.column_stack([u, v])


def _prepare_plane(points, config):
    from . import point_cloud_pipeline as p
    plane, indices = p.fit_plane_once(points, config)
    if plane is None or len(indices) < config.edge_min_inliers:
        raise UnsupportedGeometry('insufficient_floor_plane')
    plane = np.asarray(plane, float)
    if abs(plane[2])/np.linalg.norm(plane[:3]) < .5:
        raise UnsupportedGeometry('dominant_plane_is_not_a_floor')
    return plane, np.asarray(points)[indices]


def _fit_edge_line(candidates, edge, config, required_span):
    from . import point_cloud_pipeline as p
    points = np.asarray(candidates, float).reshape(-1, 2)
    seed = config.seed+{'v_min': 202, 'v_max': 203, 'u_min': 204, 'u_max': 205}[edge]
    line = p.ransac_fit_line_2d(points, config, np.random.default_rng(seed))
    if line is None:
        raise UnsupportedGeometry('edge_not_supported:'+edge)
    inside = p.line_distances(points, line) <= config.line_ransac_distance
    axis = 0 if edge.startswith('v_') else 1
    span = float(np.ptp(points[inside, axis])) if inside.any() else 0.
    if inside.sum() < config.line_min_inliers or inside.mean() < .5 or span < required_span:
        raise UnsupportedGeometry('edge_support_degenerate:'+edge)
    if abs(np.array([line[1], -line[0]])[axis]) < .5:
        raise UnsupportedGeometry('edge_direction_degenerate:'+edge)
    return line, dict(candidates=len(points), inliers=int(inside.sum()), consensus=float(inside.mean()),
        span_mm=span, residual_p95_mm=float(np.percentile(p.line_distances(points[inside], line), 95)),
        raw_line_uv=line.tolist())


def _intersection(first, second):
    from . import point_cloud_pipeline as p
    point = p.line_intersection(first, second)
    if point is None or not np.isfinite(point).all():
        raise UnsupportedGeometry('reference_lines_do_not_have_a_finite_intersection')
    return point


def _parallel_reference(high_points, rough_axis, config):
    from . import point_cloud_pipeline as p
    origin = high_points[:, :2].mean(0)
    rough = np.asarray(rough_axis, float); rough /= np.linalg.norm(rough)
    across = np.array([-rough[1], rough[0]])
    initial = _uv(high_points, origin, rough, across)
    candidates = p.extract_edge_candidates_2d(initial, config)
    near, near_quality = _fit_edge_line(candidates['v_min'], 'v_min', config,
                                      max(100., .5*np.ptp(initial[:, 0])))
    direction = np.array([near[1], -near[0]])
    if direction[0] < 0: direction = -direction
    u = np.column_stack([rough, across]) @ direction; u /= np.linalg.norm(u)
    v = np.array([-u[1], u[0]])
    accepted = candidates['v_min'][p.line_distances(candidates['v_min'], near) <= config.line_ransac_distance]
    anchor = accepted.mean(0)
    anchor -= (anchor @ near[:2]+near[2])*near[:2]/(near[:2] @ near[:2])
    left_xy = origin+anchor[0]*rough+anchor[1]*across
    left = float((left_xy-origin) @ v)
    projected = _uv(high_points, origin, u, v)
    candidates = p.extract_edge_candidates_2d(projected, config)
    head, head_quality = _fit_edge_line(candidates['u_min'], 'u_min', config,
                                      max(100., .5*np.ptp(projected[:, 1])))
    if head[0] < 0: head = -head
    far, far_quality = _fit_edge_line(candidates['v_max'], 'v_max', config,
                                    max(100., .5*np.ptp(projected[:, 0])))
    right_front = _intersection(head, far)
    right = float(right_front[1])
    if right-left <= 1: raise UnsupportedGeometry('high_width_degenerate')
    quality = dict(direction_reference='high_near_v_min_P1_P3', direction_uses_far_edge=False,
        transverse_direction_reference='projected_label_2_head_edge', near_direction_support=near_quality,
        head_support=head_quality, far_direction_support=far_quality, head_line_uv=head.tolist(),
        right_front_anchor_uv_mm=right_front.tolist(),
        right_front_anchor_radar_xy_mm=(origin+right_front[0]*u+right_front[1]*v).tolist(),
        far_correction=dict(method='rotate_about_raw_head_far_intersection', raw_line_uv=far.tolist(),
                            corrected_line_uv=[0., 1., -right]),
        head_direction_radar_xy=(np.column_stack([u, v]) @ np.array([-head[1], head[0]])).tolist(),
        width_source='original_label_2_floor_only', force_right_angle=False)
    return origin, u, v, [left, right], quality


def _end_position(candidates, config):
    """Historical scalar helper retained for diagnostics, not angle correction."""
    values = np.asarray(candidates)[:, 0]
    if len(values) < config.line_min_inliers: raise UnsupportedGeometry('too_few_end_candidates')
    value = float(np.median(values)); inside = np.abs(values-value) <= config.line_ransac_distance
    if inside.sum() < config.line_min_inliers or inside.mean() < .5:
        raise UnsupportedGeometry('transverse_position_not_supported')
    return float(np.median(values[inside])), dict(candidates=len(values), inliers=int(inside.sum()))


def _measure_end(points, plane_points, plane, origin, u, v, sides, config, edge, head_line,
                 *, low_front=False, high_end=None):
    from . import point_cloud_pipeline as p
    candidates = np.asarray(p.extract_edge_candidates_2d(_uv(plane_points, origin, u, v), config)[edge], float).reshape(-1, 2)
    inside = (candidates[:, 1] >= sides[0]-config.line_ransac_distance) & (candidates[:, 1] <= sides[1]+config.line_ransac_distance)
    try:
        raw, quality = _fit_edge_line(candidates[inside], edge, config, max(100., .5*(sides[1]-sides[0])))
        anchor = _intersection(raw, [0., 1., -sides[0]])
        original = np.degrees(np.arctan2(raw[0], -raw[1]))
        target = np.degrees(np.arctan2(head_line[0], -head_line[1]))
        quality.update(method='rotate_measured_edge_about_near_intersection',
            rotation_delta_deg=float((target-original+90) % 180-90), rotation_anchor_uv_mm=anchor.tolist())
        return float(anchor[0]), quality
    except UnsupportedGeometry as exc:
        if not low_front or high_end is None: raise
        from .deck_transition import estimate_usable_front
        value, quality = estimate_usable_front(points, high_end, origin, u, v, sides, plane, config)
        quality.update(original_line_failure=str(exc), rotation_anchor_uv_mm=[value, sides[0]],
                       direction_reference='projected_label_2_head_edge')
        return value, quality


def _build_board(points, label, plane, plane_points, origin, u, v, sides, config,
                 corner_start, reference_quality, high_end=None, end_overrides=None):
    from . import point_cloud_pipeline as p
    head = np.asarray(reference_quality['head_line_uv'], float)
    candidates = p.extract_edge_candidates_2d(_uv(plane_points, origin, u, v), config)
    ends, end_quality = [], []
    for edge in ('u_min', 'u_max'):
        if end_overrides and edge in end_overrides:
            value, quality = end_overrides[edge]
        elif label == 2 and edge == 'u_min':
            value = float(_intersection(head, [0., 1., -sides[0]])[0])
            quality = dict(reference_quality['head_support'], method='unrotated_projected_high_head')
        else:
            value, quality = _measure_end(points, plane_points, plane, origin, u, v, sides, config,
                edge, head, low_front=label == 3 and edge == 'u_min', high_end=high_end)
        ends.append(value); end_quality.append(quality)
    if ends[1]-ends[0] <= 1: raise UnsupportedGeometry('deck_length_degenerate')
    lines = dict(v_min=[0., 1., -sides[0]], v_max=[0., 1., -sides[1]])
    for edge, value in zip(('u_min', 'u_max'), ends):
        lines[edge] = [float(head[0]), float(head[1]), float(-head[0]*value-head[1]*sides[0])]
    uv_corners = np.array([_intersection(lines['u_min'], lines['v_min']), _intersection(lines['u_min'], lines['v_max']),
                           _intersection(lines['u_max'], lines['v_min']), _intersection(lines['u_max'], lines['v_max'])])
    xy = origin+uv_corners[:, 0, None]*u+uv_corners[:, 1, None]*v
    a, b, c, d = plane
    corners = np.column_stack([xy, -(a*xy[:, 0]+b*xy[:, 1]+d)/c])
    result = p.compute_geometry_result(corners, corner_start)
    side_quality = []
    for edge, position in zip(('v_min', 'v_max'), sides):
        residual = np.asarray(candidates[edge])[:, 1]-position
        side_quality.append(dict(edge=edge, median_offset_mm=float(np.median(residual)) if len(residual) else None,
            absolute_p95_mm=float(np.percentile(np.abs(residual), 95)) if len(residual) else None,
            inliers=int(np.count_nonzero(np.abs(residual) <= config.line_ransac_distance)), candidates=len(residual)))
    frame = dict(center=[*origin, float(-(a*origin[0]+b*origin[1]+d)/c)],
        u_dir=[*u, float(-(a*u[0]+b*u[1])/c)], v_dir=[*v, float(-(a*v[0]+b*v[1])/c)],
        normal_dir=p.get_ransac_normal(plane).tolist())
    result.update(geometry_scheme=2, geometry_scheme_requested=2, geometry_method='head_and_near_anchored_shared_xy',
        geometry_revision='head_near_anchors_v3',
        shared_xy_reference=dict(origin_radar_xy_mm=origin.tolist(), length_direction_radar_xy=u.tolist(),
            width_direction_radar_xy=v.tolist(), head_transverse_direction_radar_xy=reference_quality['head_direction_radar_xy'],
            width_bounds_mm=list(sides), width_xy_mm=float((sides[1]-sides[0])*np.linalg.norm(head[:2])/abs(head[0])),
            side_separation_xy_mm=float(sides[1]-sides[0]), source_label=2,
            direction_reference='high_near_v_min_P1_P3', direction_uses_far_edge=False,
            transverse_direction_reference='projected_label_2_head_edge', force_right_angle=False),
        shared_geometry_quality=dict(side_alignment_supported=all(q['absolute_p95_mm'] is not None and
            q['absolute_p95_mm'] <= 2*config.line_ransac_distance for q in side_quality),
            side_alignment_limit_mm=2*config.line_ransac_distance, side_residuals=side_quality,
            end_edge_support=end_quality, high_reference_support=reference_quality),
        global_plane_radar_corners_xyz_mm=corners.tolist(), height_source_radar_corners_xyz_mm=corners.tolist(),
        corner_height_estimation=[dict(status='awaiting_deck_reference_height') for _ in range(4)],
        corner_height_all_supported=False,
        geometry_fit=dict(frame_type='shared_horizontal_UV_parameterized_on_own_plane', frame=frame,
            plane=plane.tolist(), plane_points=len(plane_points), candidates={k:q.tolist() for k,q in candidates.items()},
            lines=lines, intersections=uv_corners.tolist()))
    return result


def _unsupported(reason, start):
    elapsed = time.perf_counter()-start
    result = dict(label_2=None, label_3=None, geometry_status='unsupported',
                  geometry_failures=[dict(board_label='label_2', reason=reason)])
    return result, [('shared', '车头/左侧参考不足：'+reason, elapsed)], elapsed


def process_shared_decks_timed(xyz, labels, config):
    start = time.perf_counter()
    xyz, labels = np.asarray(xyz, float), np.asarray(labels, np.int32)
    if xyz.shape != (len(labels), 3) or not np.isfinite(xyz).all():
        raise ValueError('Geometry requires finite matching XYZ and labels')
    order = np.lexsort((xyz[:, 2], xyz[:, 1], xyz[:, 0])); xyz, labels = xyz[order], labels[order]
    high, low = xyz[labels == 2], xyz[labels == 3]
    if not len(high): return _unsupported('missing_high_deck_reference', start)
    floor_xy = xyz[np.isin(labels, [2, 3]), :2]
    centered = floor_xy-floor_xy.mean(0)
    eigenvalues, vectors = np.linalg.eigh(centered.T @ centered)
    heading = np.asarray(config.truck_head_dir[:2], float)
    if np.linalg.norm(heading) < 1e-9: raise ValueError('Truck heading must have a horizontal direction')
    index = 1 if len(low) and eigenvalues[1] > 2*eigenvalues[0] else int(np.argmax(np.abs(vectors.T @ heading)))
    rough = vectors[:, index]
    if rough @ heading > 0: rough = -rough
    try:
        high_plane, high_points = _prepare_plane(high, config)
        origin, u, v, sides, quality = _parallel_reference(high_points, rough, config)
    except UnsupportedGeometry as exc:
        return _unsupported('high_reference_unavailable:'+str(exc), start)
    original_high, high_failure = None, None
    try:
        original_high = _build_board(high, 2, high_plane, high_points, origin, u, v, sides, config, 1, quality)
    except UnsupportedGeometry as exc:
        high_failure = str(exc)
    if not len(low):
        if original_high is None: return _unsupported('high_reference_unavailable:'+high_failure, start)
        result = dict(label_2=original_high, label_3=None)
    else:
        result = dict(label_2=original_high, label_3=None)
        support_report = None
        try:
            low_plane, low_points = _prepare_plane(low, config)
            high_end = float(original_high['geometry_fit']['intersections'][2][0]) if original_high else float(np.percentile(_uv(high_points, origin, u, v)[:, 0], 99.5))
            low_hint = float(np.percentile(_uv(low_points, origin, u, v)[:, 0], .5))
            from .deck_transition import detect_transition_support, inspect_transition
            from .transition_support import SupportParameters
            support_report = detect_transition_support(xyz, labels, high_end, low_hint, high_plane, low_plane, origin, u, v, sides, config,
                parameters=SupportParameters(decision_mode=config.support_decision_mode))
            present = support_report['decision'] == 'present'
            if present:
                shared_u, joint_quality = _measure_end(low, low_points, low_plane, origin, u, v, sides,
                    config, 'u_min', np.array(quality['head_line_uv']), low_front=True, high_end=high_end)
                joint_source = 'corrected_low_front_near_anchor'
            else:
                if original_high is None: raise UnsupportedGeometry('absent_support_requires_measured_high_rear:'+high_failure)
                shared_u = high_end
                joint_quality = dict(original_high['shared_geometry_quality']['end_edge_support'][1])
                joint_source = 'corrected_original_high_rear_near_anchor'
            joint_quality = dict(joint_quality, shared_boundary_source=joint_source)
            high_board = _build_board(high, 2, high_plane, high_points, origin, u, v, sides, config, 1, quality,
                                      end_overrides={'u_max': (shared_u, joint_quality)})
            # H crops the final corrected region; an unreliable original rear
            # edge is diagnostic only once the low-front joint defines its end.
            if original_high is None:
                high_board['shared_geometry_quality']['original_high_tail_fit_failure'] = high_failure
            low_board = _build_board(low, 3, low_plane, low_points, origin, u, v, sides, config, 5, quality,
                high_end=high_end, end_overrides={'u_min': (shared_u, joint_quality)})
            transition = inspect_transition(xyz, high_end, low_hint, high_plane, low_plane, origin, u, v, sides, config)
            transition.update(joined_xy=True, final_gap_along_length_mm=0., shared_transition_u_mm=shared_u,
                              boundary_source=joint_source, width_source='original_label_2_floor_only')
            support_report.update(diagnostic_only=False, merge_applied=present,
                merge_basis='low_floor_front_near_anchor' if present else 'not_required', region_end_u_mm=shared_u,
                support_points_added_to_floor=False, width_source='original_label_2_floor_only')
            for board in (high_board, low_board):
                board['transition_geometry'] = transition; board['transition_support'] = support_report
            result.update(label_2=high_board, label_3=low_board)
        except UnsupportedGeometry as exc:
            result.update(geometry_status='partial' if original_high else 'unsupported',
                geometry_failures=[dict(board_label='label_3', reason='low_constraint_unavailable:'+str(exc))])
            if original_high is not None and support_report is not None: original_high['transition_support'] = support_report
    elapsed = time.perf_counter()-start
    return result, [('shared', '底板投影/车头横线与左侧基准/固定交点修正/共同分界', elapsed)], elapsed
