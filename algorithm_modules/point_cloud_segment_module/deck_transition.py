"""Measured high/low joint evidence and persistent usable-floor onset.

No field-log coordinates, declared support labels, or artificial Z points enter
these estimates. A positive XY gap alone does not confirm a support object.
"""
import numpy as np
from .transition_support import detect_transition_support


def _coordinates(points,origin,u,v):
    xy=np.asarray(points,float)[:,:2]-origin
    return np.column_stack([xy@u,xy@v])


def _plane_z(plane,points):
    a,b,c,d=plane
    return -(a*points[:,0]+b*points[:,1]+d)/c


def estimate_usable_front(points,high_end,origin,u,v,sides,plane,config):
    """Full-width persistent low-floor onset, not the nearest isolated point.

    Used only when a low-front line is unsupported. Each width column must
    contain a run of lower-floor returns. The latest spatially corroborated
    onset defines a conservative rectangular workface. Missing width coverage
    remains unsupported; it is never interpreted as an absent obstacle.
    """
    from .shared_deck_geometry import UnsupportedGeometry
    points=np.asarray(points,float);uv=_coordinates(points,origin,u,v)
    width=sides[1]-sides[0];tol=config.line_ransac_distance
    margin=max(2*tol,.025*width)
    if width<=2*margin:raise UnsupportedGeometry('usable_floor_width_degenerate')
    search=max(max(config.corner_height_radii),width*.5)
    step=config.slice_width_u
    u0=high_end-2*tol;upper=high_end+search
    floor=np.abs(points[:,2]-_plane_z(plane,points))<=min(config.edge_plane_distance,config.corner_height_max_tolerance)
    uv=uv[floor&(uv[:,0]>=u0)&(uv[:,0]<=upper+4*step)&(uv[:,1]>=sides[0]+margin)&(uv[:,1]<=sides[1]-margin)]
    if len(uv)<config.edge_min_inliers//10:raise UnsupportedGeometry('too_few_usable_floor_returns')
    v_edges=np.linspace(sides[0]+margin,sides[1]-margin,max(12,int(np.ceil((width-2*margin)/config.slice_width_v)))+1)
    u_edges=np.arange(u0,upper+5*step,step)
    columns=[]
    v_index=np.minimum(np.searchsorted(v_edges,uv[:,1],side='right')-1,len(v_edges)-2)
    for index in range(len(v_edges)-1):
        values=uv[v_index==index,0]
        if len(values)<config.slice_min_points:continue
        counts,_=np.histogram(values,u_edges);occupied=counts>=2
        start=None
        for k in range(max(0,len(occupied)-3)):
            if occupied[k] and np.count_nonzero(occupied[k:k+4])>=3:
                start=k;break
        if start is None:continue
        values_start=values[(values>=u_edges[start])&(values<u_edges[start+1])]
        if not len(values_start):continue
        onset=float(np.quantile(values_start,.1))
        if onset>upper-3*step:continue
        columns.append(dict(index=index,v_mm=float((v_edges[index]+v_edges[index+1])/2),onset_u_mm=onset,points=len(values)))
    coverage=len(columns)/(len(v_edges)-1)
    if len(columns)<config.line_min_inliers or coverage<.7:
        raise UnsupportedGeometry('usable_floor_width_coverage_insufficient')
    # A single late column can be a hole or side return. Require neighboring
    # columns to support a late onset, then verify clear floor across columns.
    onsets=np.array([c['onset_u_mm'] for c in columns]);indices=np.array([c['index'] for c in columns])
    stable=[]
    for i,index in enumerate(indices):
        adjacent=np.flatnonzero(np.abs(indices-index)<=1)
        if len(adjacent)>=3:stable.append(float(np.median(onsets[adjacent])))
    if not stable:raise UnsupportedGeometry('usable_floor_onsets_disconnected')
    position=max(stable)
    clear=float(np.mean(onsets<=position+tol))
    if clear<.95:raise UnsupportedGeometry('usable_floor_front_not_clear_across_width')
    # Verify actual returns immediately behind this inferred workface line.
    supported=0
    for column in columns:
        values=uv[v_index==column['index'],0]
        if np.count_nonzero((values>=position+tol)&(values<=position+4*step))>=config.slice_min_points//2:supported+=1
    behind=supported/len(columns)
    if behind<.95:raise UnsupportedGeometry('usable_floor_behind_front_not_supported')
    return float(position),dict(method='persistent_3d_low_floor_onset',candidates=len(columns),inliers=supported,
        consensus=behind,width_coverage=float(coverage),clear_floor_fraction=clear,floor_distance_mm=min(config.edge_plane_distance,config.corner_height_max_tolerance),
        search_from_high_end_mm=search,onset_p05_p50_p95_mm=np.percentile(onsets,[5,50,95]).tolist(),
        residual_p95_mm=float(np.percentile(np.abs(onsets-position),95)),columns=columns,
        note='Usable rectangular floor onset; not a claim that every side has a physical edge here.')


def inspect_transition(scene,high_end,low_start,high_plane,low_plane,origin,u,v,sides,config):
    """Separate gap measurement, vertical-face evidence, and support evidence."""
    scene=np.asarray(scene,float).reshape(-1,3);uv=_coordinates(scene,origin,u,v)
    gap=float(low_start-high_end);tol=config.line_ransac_distance;coincident=2*tol
    inside_v=(uv[:,1]>=sides[0])&(uv[:,1]<=sides[1])
    low_z=_plane_z(low_plane,scene);high_z=_plane_z(high_plane,scene)
    between=(scene[:,2]>low_z+tol)&(scene[:,2]<high_z-tol)
    face_mask=inside_v&between&(np.abs(uv[:,0]-(high_end+low_start)/2)<=coincident)
    face=scene[face_mask];face_uv=uv[face_mask]
    vertical=False;face_info=dict(points=len(face));shared=None
    if len(face)>=config.line_min_inliers:
        # Use a deterministic spread of points for candidate search. Every
        # candidate is scored on the same set; no global random state changes.
        probe=np.linspace(0,len(face)-1,min(len(face),5000),dtype=int);q=face[probe]
        rng=np.random.default_rng(config.seed+906);best=None;best_count=0
        for _ in range(min(160,config.edge_plane_iterations)):
            chosen=q[rng.choice(len(q),3,replace=False)]
            normal=np.cross(chosen[1]-chosen[0],chosen[2]-chosen[0]);norm=np.linalg.norm(normal)
            if norm<1e-6:continue
            normal/=norm
            if abs(normal@np.r_[u,0.])<np.cos(np.deg2rad(15)):continue
            distance=np.abs((q-chosen[0])@normal);mask=distance<=tol
            if mask.sum()>best_count:best=(normal,chosen[0]);best_count=int(mask.sum())
        if best is not None:
            normal,anchor=best;mask=np.abs((face-anchor)@normal)<=tol
            accepted=face[mask];accepted_uv=face_uv[mask]
            span_v=float(np.ptp(accepted_uv[:,1]));span_z=float(np.ptp(accepted[:,2]));fraction=float(mask.mean())
            height=float(np.median(high_z[face_mask]-low_z[face_mask]))
            vertical=bool(mask.sum()>=config.line_min_inliers and fraction>=.7 and span_v>=.5*(sides[1]-sides[0]) and span_z>=.4*height)
            face_info.update(inliers=int(mask.sum()),fraction=fraction,width_span_mm=span_v,height_span_mm=span_z,
                normal=normal.tolist(),normal_deviation_from_length_deg=float(np.degrees(np.arccos(np.clip(abs(normal@np.r_[u,0.]),0,1)))))
            if vertical and abs(gap)<=coincident:shared=float(np.median(accepted_uv[:,0]))
    support_mask=inside_v&(uv[:,0]>high_end+tol)&(uv[:,0]<low_start-tol)&(scene[:,2]>low_z+2*tol)&(scene[:,2]<=high_z+2*tol)
    support=uv[support_mask]
    support_evidence=bool(len(support)>=config.line_min_inliers and np.ptp(support[:,0])>=tol and np.ptp(support[:,1])>=100) if len(support) else False
    if abs(gap)<=coincident:structure='vertical_step' if vertical else 'near_coincident_unconfirmed'
    elif gap>coincident:structure='separated_usable_start'
    else:structure='overlapping_boundaries_unconfirmed'
    return dict(structure=structure,gap_along_length_mm=gap,near_coincident_tolerance_mm=coincident,
        vertical_face_supported=vertical,vertical_face=face_info,support_evidence_present=support_evidence,
        above_floor_gap_points=int(len(support)),shared_transition_u_mm=shared,
        support_presence='likely' if support_evidence else 'unconfirmed',
        note='Gap and normals are evidence; missing/occluded face is not proof of support presence.')
