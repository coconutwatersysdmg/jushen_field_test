"""Trial classification of the observed joint, without changing deck geometry.

Label 1/4 seeds are cropped by measured deck position. Normal concentration
classifies the joint; spatial evidence separately describes the support extent.
"""
from dataclasses import asdict, dataclass
from itertools import product

import numpy as np


@dataclass(frozen=True)
class SupportParameters:
    decision_mode: str = 'normal_concentration'
    normal_concentration_threshold: float = .80
    normal_min_valid_count: int = 50
    normal_min_valid_fraction: float = .20
    voxel_mm: float = 20.
    normal_radius_mm: float = 100.
    normal_neighbors: int = 32
    normal_min_neighbors: int = 8
    plane_distance_mm: float = 30.
    angle_deg: float = 20.
    max_planes: int = 5
    plane_iterations: int = 160
    min_surface_points: int = 30
    min_plane_fraction: float = .05
    max_probe_points: int = 6000
    support_min_length_mm: float = 80.
    support_min_width_mm: float = 150.
    structural_residual_mm: float = 40.
    structural_concentration: float = .90
    structural_angle_deg: float = 40.
    structural_min_width_fraction: float = .50

    def __post_init__(self):
        if self.decision_mode not in ('normal_concentration','geometry_evidence','structural_evidence'):
            raise ValueError('Unsupported transition support decision mode')


def classify_normal_concentration(summary,parameters=None):
    """Use the joint normals' own dominant axial direction, independent of tilt."""
    params=parameters or SupportParameters()
    valid=int(summary.get('valid',0));invalid=int(summary.get('invalid',0))
    fraction=valid/max(valid+invalid,1);concentration=summary.get('dominant_fraction')
    result=dict(decision='present',reason='absence_not_confirmed_insufficient_normals',valid_normals=valid,
                decision_policy='confirmed_absence_otherwise_presence',
                valid_fraction=float(fraction),dominant_concentration=concentration,
                dominant_normal_uvz=summary.get('dominant_normal_uvz'),
                riser_concentration=summary.get('riser_fraction'),
                deviation_fraction=None if concentration is None else float(1-concentration),
                threshold=params.normal_concentration_threshold,angle_deg=params.angle_deg,
                threshold_comparison='strictly_greater_than',
                concentration_definition='fraction_within_angle_of_data_derived_dominant_axial_normal')
    if concentration is None or not np.isfinite(concentration) or not 0<=concentration<=1:
        result['reason']='absence_not_confirmed_invalid_concentration'
        return result
    if valid<params.normal_min_valid_count or fraction<params.normal_min_valid_fraction:
        return result
    concentrated=concentration>params.normal_concentration_threshold
    result.update(decision='absent' if concentrated else 'present',
                  reason='normals_concentrated_about_dominant_direction' if concentrated else 'joint_normals_dispersed')
    return result


def _finish_report(report,params):
    report['geometry_decision']=report['decision']
    report['geometry_reason']=report['reason']
    report['support_extent_available']=report.get('support_end_u_mm') is not None
    normal=classify_normal_concentration(report['normal_summary'],params)
    report['normal_classification']=normal
    report['decision_basis']=params.decision_mode
    if params.decision_mode=='normal_concentration':
        report.update(method='transition_dominant_normal_concentration_v3',decision=normal['decision'],reason=normal['reason'])
    elif params.decision_mode=='structural_evidence':
        structural=report['structural_evidence']
        report.update(method='transition_structural_evidence_trial_v6',
                      decision=structural['decision'],reason=structural['reason'])
    elif report['decision']!='absent':
        report.update(decision='present',reason=(report['reason'] if report['geometry_decision']=='present'
                                               else 'absence_not_confirmed_geometry'))
    report['decision_policy']='confirmed_absence_otherwise_presence'
    report['merge_recommended']=bool(report['decision']=='present' and report['support_extent_available'])
    return report


def _separated_riser_faces(risers,params):
    """Keep spatially separated observed faces even when one is less dense."""
    pairs=[]
    for i,a in enumerate(risers):
        for b in risers[i+1:]:
            na,nb=np.asarray(a['normal_uvz']),np.asarray(b['normal_uvz'])
            if abs(na@nb)<np.cos(np.deg2rad(params.angle_deg)):
                continue
            ba,bb=np.asarray(a['bounds_p05_p95_uvz_mm']),np.asarray(b['bounds_p05_p95_uvz_mm'])
            lower=np.maximum(ba[0,1:],bb[0,1:]);upper=np.minimum(ba[1,1:],bb[1,1:])
            if upper[0]-lower[0]<params.support_min_width_mm or upper[1]-lower[1]<2*params.plane_distance_mm:
                continue
            aa,ab=np.asarray(a['anchor_uvz_mm']),np.asarray(b['anchor_uvz_mm'])
            separations=[]
            for lateral,height in product((lower[0],upper[0]),(lower[1],upper[1])):
                ua=aa[0]-(na[1]*(lateral-aa[1])+na[2]*(height-aa[2]))/na[0]
                ub=ab[0]-(nb[1]*(lateral-ab[1])+nb[2]*(height-ab[2]))/nb[0]
                separations.append(ub-ua)
            # Compare fitted faces over shared observations, not their anchors;
            # disjoint pieces of one tilted face must not count as two faces.
            if min(separations)>=params.support_min_length_mm or max(separations)<=-params.support_min_length_mm:
                pairs.append(dict(plane_indices=[a['index'],b['index']],
                                  min_separation_mm=float(min(abs(x) for x in separations))))
    return pairs


def _orientation_evidence(probe,point_normals,report,params):
    """Keep direction: a top normal aligns with the adjacent floor normal.

    All joint points, including points level with the high floor, participate.
    A measured top behind the high-floor boundary is a positive observation;
    missing top normals alone are not proof that a support object is absent.
    """
    frame=report['frame'];u=np.asarray(frame['length_direction_radar_xy']);v=np.asarray(frame['width_direction_radar_xy'])
    plane=np.asarray(report['high_plane_radar'])
    floor_normal=np.array([plane[:2]@u,plane[:2]@v,plane[2]])
    floor_normal/=np.linalg.norm(floor_normal)
    valid=np.isfinite(point_normals).all(1)
    dot=np.abs(point_normals[valid]@floor_normal)
    top=dot>=np.cos(np.deg2rad(params.structural_angle_deg))
    wall=dot<=np.sin(np.deg2rad(params.structural_angle_deg))
    top_points=probe[valid][top]
    # A high-floor rim before the boundary also has upward normals. Require
    # an observed connected footprint beyond that boundary, with finite area.
    after=top_points[top_points[:,0]>report['high_end_u_mm']+params.voxel_mm]
    patch=_largest_patch(after,params.normal_radius_mm)
    bounds=np.percentile(patch,[5,95],axis=0) if len(patch) else np.zeros((2,3))
    span=bounds[1]-bounds[0]
    fraction=float(valid.mean()) if len(valid) else 0.
    reliable=int(valid.sum())>=params.normal_min_valid_count and fraction>=params.normal_min_valid_fraction
    observed=bool(reliable and len(patch)>=params.min_surface_points and
                  span[0]>=params.support_min_length_mm and span[1]>=params.support_min_width_mm)
    return dict(reference='adjacent_high_floor_normal',reference_normal_uvz=floor_normal.tolist(),
        angle_deg=params.structural_angle_deg,valid_normals=int(valid.sum()),valid_fraction=fraction,
        top_normal_count=int(top.sum()),wall_normal_count=int(wall.sum()),
        top_normal_fraction=float(top.mean()) if len(top) else None,
        wall_normal_fraction=float(wall.mean()) if len(wall) else None,
        top_patch_after_high_points=len(patch),top_patch_after_high_span_uvz_mm=span.tolist(),
        support_top_observed=observed,near_high_plane_points_removed=False,
        missing_top_alone_confirms_absence=False)


def _structural_summary(points,high_z,report,params,probe=None,point_normals=None):
    """Trial: a broad, thin connecting face is step-like.

    High-plane-height points are retained for all plane/extent diagnostics.
    Only the auxiliary concentration estimate omits this band. A separate
    directional estimate keeps same-height tops as positive evidence.
    Two separated parallel faces still fail the thin-face test.
    A same-height object with just one observed face remains ambiguous and
    is an acknowledged limitation of this experimental classifier.
    """
    cleaned=points[np.abs(points[:,2]-high_z)>params.plane_distance_mm]
    if probe is None:
        probe=_sample(points,params)
    if point_normals is None:
        _,point_normals=_normal_summary(probe,params)
    orientation=_orientation_evidence(probe,point_normals,report,params)
    normal_summary,_=_normal_summary(_sample(cleaned,params),
        SupportParameters(**{**asdict(params),'angle_deg':params.structural_angle_deg}))
    concentration=normal_summary['dominant_fraction']
    middle=points[points[:,2]<high_z-params.plane_distance_mm]
    residual=None
    if len(middle)>=30 and np.ptp(middle[:,1])>1.:
        line=np.polyfit(middle[:,1],middle[:,0],1)
        deviation=middle[:,0]-np.polyval(line,middle[:,1]);deviation-=np.median(deviation)
        residual=float(np.percentile(np.abs(deviation),90))
    risers=[q for q in report['planes'] if q['surface_type']=='riser']
    separated=_separated_riser_faces(risers,params)
    best=max(risers,key=lambda q:q['fraction']) if risers else None
    width=report['roi_v_mm'][1]-report['roi_v_mm'][0]
    fraction=best['span_uvz_mm'][1]/width if best else 0.
    reliable=normal_summary['valid']>=params.normal_min_valid_count
    thin=residual is not None and residual<=params.structural_residual_mm
    aligned=concentration is not None and concentration>=params.structural_concentration
    broad=fraction>=params.structural_min_width_fraction
    top_observed=orientation['support_top_observed']
    absent=bool(reliable and thin and aligned and broad and not separated and not top_observed)
    return dict(decision='absent' if absent else 'present',
        reason=('observed_horizontal_support_top' if top_observed else
                'observed_separated_vertical_faces' if separated else 'broad_thin_step_face' if absent else
                'absence_not_confirmed_insufficient_structure' if not reliable else 'joint_has_extent_or_dispersed_structure'),
        reliable=bool(reliable),middle_points=len(middle),middle_u_residual_p90_mm=residual,
        separated_face_pairs=separated,orientation_evidence=orientation,
        cleaned_normal_concentration=concentration,cleaned_valid_normals=normal_summary['valid'],
        cleaned_normal_angle_deg=params.structural_angle_deg,riser_width_fraction=float(fraction),
        required_residual_mm=params.structural_residual_mm,required_concentration=params.structural_concentration,
        required_width_fraction=params.structural_min_width_fraction,
        same_height_single_face_limitation=True,trial_only=True)


class _Neighborhood:
    """Fixed-radius spatial hash; no extra scipy/sklearn dependency."""
    def __init__(self, points, radius):
        self.points=points;self.radius=radius;self.buckets={}
        for i,cell in enumerate(np.floor(points/radius).astype(np.int64)):
            self.buckets.setdefault(tuple(cell),[]).append(i)

    def query(self, point, limit=None):
        cell=np.floor(point/self.radius).astype(np.int64)
        ids=[]
        for offset in product((-1,0,1),repeat=3):
            ids.extend(self.buckets.get(tuple(cell+offset),()))
        ids=np.asarray(ids,dtype=np.int64)
        delta=self.points[ids]-point
        distances=np.einsum('ij,ij->i',delta,delta)
        valid=distances<=self.radius**2
        ids,distances=ids[valid],distances[valid]
        if limit is not None and len(ids)>limit:
            ids=ids[np.lexsort((ids,distances))[:limit]]
        return ids


def _sample(points, params):
    if not len(points):
        return points.copy()
    points = points[np.lexsort((points[:,2],points[:,1],points[:,0]))]
    _, indices = np.unique(np.floor(points/params.voxel_mm).astype(np.int64), axis=0, return_index=True)
    points = points[np.sort(indices)]
    if len(points)>params.max_probe_points:
        points=points[np.linspace(0,len(points)-1,params.max_probe_points,dtype=int)]
    return points


def _dominant_normal_concentration(normals, angle_deg):
    """Estimate the densest axial mode, then count its angular neighborhood.

    Search observed axes and the scatter matrix axes; refine the best distinct
    seeds using their inlier scatter matrices. This is a deterministic mode
    estimate, not an exact global spherical-cap optimizer. Opposite normals
    share an axis, but signs of individual coordinates must be preserved.
    """
    normals=np.asarray(normals,float).reshape(-1,3)
    lengths=np.linalg.norm(normals,axis=1)
    keep=np.isfinite(normals).all(1)&(lengths>1e-12)
    normals=normals[keep]/lengths[keep,None]
    if not len(normals):
        return dict(fraction=None,direction=None,inliers=0)
    signs=np.sign(normals[np.arange(len(normals)),np.abs(normals).argmax(1)])
    normals*=signs[:,None]
    normals=normals[np.lexsort((normals[:,2],normals[:,1],normals[:,0]))]
    _,axes=np.linalg.eigh(normals.T@normals)
    candidates=np.unique(np.vstack([normals,axes.T]),axis=0)
    limit=np.cos(np.deg2rad(angle_deg))-1e-12
    counts=np.empty(len(candidates),int)
    # Bound memory: a full 6000 by 6000 similarity matrix is unnecessary.
    for start in range(0,len(candidates),128):
        counts[start:start+128]=np.count_nonzero(np.abs(normals@candidates[start:start+128].T)>=limit,axis=0)
    order=np.argsort(-counts,kind='stable')
    best_axis=candidates[order[0]].copy();best_count=int(counts[order[0]])
    seeds=[]
    for index in order:
        axis=candidates[index]
        if all(abs(axis@seed)<np.cos(np.deg2rad(5.)) for seed in seeds):
            seeds.append(axis)
        if len(seeds)==8:break
    for seed in seeds:
        axis=seed.copy();previous=None
        for _ in range(12):
            inside=np.abs(normals@axis)>=limit
            count=int(inside.sum())
            if count>best_count:
                best_axis,best_count=axis.copy(),count
            if not count or (previous is not None and np.array_equal(previous,inside)):
                break
            previous=inside
            _,vectors=np.linalg.eigh(normals[inside].T@normals[inside])
            axis=vectors[:,-1]
    if best_axis[np.abs(best_axis).argmax()]<0:best_axis=-best_axis
    return dict(fraction=float(best_count/len(normals)),direction=best_axis.tolist(),inliers=best_count)


def _normal_summary(points, params):
    summary=dict(valid=0,invalid=len(points),riser_fraction=None,horizontal_fraction=None,
                 side_fraction=None,other_fraction=None,dominant_fraction=None,dominant_normal_uvz=None,
                 dominant_inliers=0)
    vectors_out=np.full((len(points),3),np.nan)
    if len(points)<params.normal_min_neighbors:
        return summary,vectors_out
    tree=_Neighborhood(points,params.normal_radius_mm)
    normals=[]
    for i,point in enumerate(points):
        local=points[tree.query(point,params.normal_neighbors)]
        if len(local)<params.normal_min_neighbors:
            continue
        centered=local-local.mean(0)
        values,vectors=np.linalg.eigh(centered.T@centered/len(local))
        if values[1]<=1. or values[1]<.02*values[2] or values[0]/max(values.sum(),1e-12)>.08:
            continue
        normal=vectors[:,0]
        normals.append(normal)
        # Per-point axial normals are used only inside this calculation.
        # Do not serialize thousands of vectors into every board result.
        vectors_out[i]=normal
    if normals:
        normals=np.asarray(normals);limit=np.cos(np.deg2rad(params.angle_deg))
        fractions=(np.abs(normals)>=limit).mean(0)
        dominant=_dominant_normal_concentration(normals,params.angle_deg)
        summary.update(valid=len(normals),invalid=len(points)-len(normals),
                       riser_fraction=float(fractions[0]),side_fraction=float(fractions[1]),
                       horizontal_fraction=float(fractions[2]),other_fraction=float(1-fractions.sum()),
                       dominant_fraction=dominant['fraction'],dominant_normal_uvz=dominant['direction'],
                       dominant_inliers=dominant['inliers'])
    return summary,vectors_out


def _largest_patch(points, radius):
    if not len(points):
        return points
    tree=_Neighborhood(points,radius);visited=np.zeros(len(points),bool);largest=[]
    for start in range(len(points)):
        if visited[start]:continue
        pending=[start];visited[start]=True;component=[]
        while pending:
            index=pending.pop();component.append(index)
            ids=tree.query(points[index]);fresh=ids[~visited[ids]]
            visited[fresh]=True;pending.extend(fresh.tolist())
        if len(component)>len(largest):largest=component
    return points[np.sort(largest)]


def _surface_planes(points, params, seed):
    remaining=points.copy();planes=[];rng=np.random.default_rng(seed)
    minimum=max(params.min_surface_points,int(np.ceil(params.min_plane_fraction*len(points))))
    for _ in range(params.max_planes):
        if len(remaining)<minimum:
            break
        best=None;best_count=0
        for _ in range(params.plane_iterations):
            triple=remaining[rng.choice(len(remaining),3,replace=False)]
            normal=np.cross(triple[1]-triple[0],triple[2]-triple[0]);norm=np.linalg.norm(normal)
            if norm<1e-6:
                continue
            normal/=norm
            mask=np.abs((remaining-triple[0])@normal)<=params.plane_distance_mm
            count=int(mask.sum())
            if count>best_count:
                best=mask;best_count=count
        if best is None or best_count<minimum:
            break
        anchor=remaining[best].mean(0)
        _,values,vectors=np.linalg.svd(remaining[best]-anchor,full_matrices=False)
        if len(values)<3 or values[1]<1.:
            break
        normal=vectors[-1]
        if normal[np.argmax(np.abs(normal))]<0:
            normal=-normal
        distance=np.abs((remaining-anchor)@normal)
        inside=distance<=params.plane_distance_mm
        patch=_largest_patch(remaining[inside],params.normal_radius_mm)
        if len(patch)>=minimum:
            extent=np.percentile(patch,[5,95],axis=0)
            direction=np.abs(normal)
            kind=('riser' if direction[0]>=np.cos(np.deg2rad(params.angle_deg)) else
                  'horizontal' if direction[2]>=np.cos(np.deg2rad(params.angle_deg)) else
                  'side' if direction[1]>=np.cos(np.deg2rad(params.angle_deg)) else 'oblique')
            planes.append(dict(index=len(planes)+1,points=len(patch),fraction=float(len(patch)/len(points)),
                normal_uvz=normal.tolist(),anchor_uvz_mm=anchor.tolist(),surface_type=kind,
                normal_to_vertical_deg=float(np.degrees(np.arccos(np.clip(direction[2],0,1)))),
                normal_to_length_deg=float(np.degrees(np.arccos(np.clip(direction[0],0,1)))),
                bounds_p05_p95_uvz_mm=extent.tolist(),span_uvz_mm=(extent[1]-extent[0]).tolist(),
                residual_p95_mm=float(np.percentile(np.abs((patch-anchor)@normal),95))))
        remaining=remaining[~inside]
    return planes


def detect_transition_support(scene,labels,high_end,low_start,high_plane,low_plane,origin,u,v,sides,config,
                              parameters=None):
    params=parameters or SupportParameters()
    scene=np.asarray(scene,float);labels=np.asarray(labels)
    if scene.shape!=(len(labels),3) or not np.isfinite(scene).all():
        raise ValueError('Transition XYZ/labels must match and be finite')
    origin=np.asarray(origin,float);u=np.asarray(u,float);v=np.asarray(v,float)
    if not np.allclose([u@u,v@v,u@v],[1.,1.,0.],atol=1e-6):
        raise ValueError('Transition axes must be horizontal orthonormal unit vectors')
    width=float(sides[1]-sides[0]);tol=params.plane_distance_mm
    search=max(max(config.corner_height_radii),width*.5)
    lower=high_end-max(100.,3*tol);upper=high_end+search
    uv=(scene[:,:2]-origin)@np.column_stack([u,v])
    def height(plane):
        a,b,c,d=plane
        return -(a*scene[:,0]+b*scene[:,1]+d)/c
    low_z,high_z=height(low_plane),height(high_plane)
    height_difference=float(np.median(high_z-low_z)) if len(scene) else 0.
    region=(uv[:,0]>=lower)&(uv[:,0]<=upper)&(uv[:,1]>=sides[0])&(uv[:,1]<=sides[1])
    above=(scene[:,2]>low_z+2*tol)&(scene[:,2]<=high_z+max(100.,2*tol))
    seeds=region&above&np.isin(labels,[1,4])
    points=np.column_stack([uv[seeds],scene[seeds,2]])
    probe=_sample(points,params)
    report=dict(method='transition_normals_multiplane_v1',decision='unknown',reason='insufficient_joint_observations',
        diagnostic_only=True,merge_recommended=False,merge_applied=False,support_end_u_mm=None,
        high_end_u_mm=float(high_end),low_start_hint_u_mm=float(low_start),
        gap_hint_mm=float(low_start-high_end),candidate_points=len(points),sampled_points=len(probe),
        candidate_labels=[1,4],roi_u_mm=[float(lower),float(upper)],roi_v_mm=list(map(float,sides)),
        high_low_height_difference_mm=height_difference,parameters=asdict(params),
        frame=dict(origin_radar_xy_mm=origin.tolist(),length_direction_radar_xy=u.tolist(),
                   width_direction_radar_xy=v.tolist()),high_plane_radar=list(map(float,high_plane)),
        low_plane_radar=list(map(float,low_plane)),planes=[],plane_count=0,normal_direction_count=0,
        vertical_riser_supported=False,clear_low_floor_width_fraction=0.)
    if len(probe)<params.min_surface_points or height_difference<=2*tol:
        report['normal_summary'],point_normals=_normal_summary(probe,params)
        if params.decision_mode=='structural_evidence':
            report['structural_evidence']=_structural_summary(points,high_z[seeds],report,params,probe,point_normals)
        return _finish_report(report,params)
    normals,point_normals=_normal_summary(probe,params)
    planes=_surface_planes(probe,params,config.seed+1906)
    directions=[]
    for plane in planes:
        normal=np.asarray(plane['normal_uvz'])
        if all(abs(normal@direction)<np.cos(np.deg2rad(params.angle_deg)) for direction in directions):
            directions.append(normal)
    risers=[q for q in planes if q['surface_type']=='riser' and q['fraction']>=.7 and
            q['span_uvz_mm'][1]>=.5*width and q['span_uvz_mm'][2]>=.4*height_difference]
    # Use actual early floor observations, not a density-dependent quantile
    # of semantic label_3 as a mandatory physical gap measurement.
    floor_mask=region&(uv[:,0]>high_end+tol)&(uv[:,0]<high_end+4*tol)&(
        np.abs(scene[:,2]-low_z)<=tol)
    floor_bins=np.clip(((uv[floor_mask,1]-sides[0])/width*12).astype(int),0,11)
    counts=np.bincount(floor_bins,minlength=12)
    floor_coverage=float(np.mean(counts>=3))
    support_planes=[]
    rear_faces=[q for q in planes if q['surface_type']=='riser' and
                q['anchor_uvz_mm'][0]>high_end+params.support_min_length_mm and
                q['span_uvz_mm'][1]>=params.support_min_width_mm and
                q['span_uvz_mm'][2]>=max(2*tol,.3*height_difference)]
    def occupied_patch(matched):
        xy=origin+matched[:,0,None]*u+matched[:,1,None]*v
        a,b,c,d=high_plane
        high_here=-(a*xy[:,0]+b*xy[:,1]+d)/c
        distinct=np.abs(matched[:,2]-high_here)>2*tol
        # A support top can be level with the high deck. A separated observed
        # rear face must corroborate that case; a high-floor fragment alone
        # is insufficient evidence of a support object.
        if rear_faces:
            rear=max(q['anchor_uvz_mm'][0] for q in rear_faces)
            distinct=distinct|(matched[:,0]<=rear+2*tol)
        after=matched[(matched[:,0]>high_end+2*tol)&distinct]
        return _largest_patch(after,params.normal_radius_mm)
    for plane in planes:
        normal=np.asarray(plane['normal_uvz']);anchor=np.asarray(plane['anchor_uvz_mm'])
        matched=probe[np.abs((probe-anchor)@normal)<=tol]
        after=occupied_patch(matched)
        if len(after)<params.min_surface_points:
            continue
        bounds=np.percentile(after,[5,95],axis=0);span=bounds[1]-bounds[0]
        if span[0]>=params.support_min_length_mm and span[1]>=params.support_min_width_mm:
            # A top/oblique face extends in U; a broad vertical riser alone
            # has no measured longitudinal occupied footprint.
            if plane['surface_type']!='riser':
                support_planes.append(dict(plane_index=plane['index'],points=len(after),
                    evidence='global_plane',bounds_p05_p95_uvz_mm=bounds.tolist(),span_uvz_mm=span.tolist()))
    # Curved/slanted supports need not admit one global plane. Require a
    # connected, measured patch of reliable local non-riser normals instead.
    reliable=np.isfinite(point_normals).all(1)&(np.abs(point_normals[:,0])<np.cos(np.deg2rad(params.angle_deg)))
    local_patch=occupied_patch(probe[reliable])
    if len(local_patch)>=params.min_surface_points:
        bounds=np.percentile(local_patch,[5,95],axis=0);span=bounds[1]-bounds[0]
        if span[0]>=params.support_min_length_mm and span[1]>=params.support_min_width_mm:
            support_planes.append(dict(plane_index=None,points=len(local_patch),evidence='local_normal_patch',
                bounds_p05_p95_uvz_mm=bounds.tolist(),span_uvz_mm=span.tolist()))
    report.update(planes=planes,plane_count=len(planes),normal_direction_count=len(directions),
                  normal_summary=normals,vertical_riser_supported=bool(risers),
                  clear_low_floor_width_fraction=floor_coverage,clear_low_floor_u_interval_mm=[float(high_end+tol),float(high_end+4*tol)],
                  support_surfaces=support_planes,rear_face_plane_indices=[q['index'] for q in rear_faces],
                  support_end_is_final_boundary=False,normal_direction_basis='extracted_plane_normals_axial')
    if support_planes:
        end=max(q['bounds_p05_p95_uvz_mm'][1][0] for q in support_planes)
        report.update(decision='present',reason='connected_above_low_floor_surface_has_longitudinal_extent',
                      merge_recommended=True,support_end_u_mm=float(end))
    elif risers and floor_coverage>=.7:
        report.update(decision='absent',reason='dominant_vertical_riser_and_observed_low_floor_behind')
    else:
        report['reason']='joint_evidence_does_not_separate_step_from_support'
    if params.decision_mode=='structural_evidence':
        report['structural_evidence']=_structural_summary(points,high_z[seeds],report,params,probe,point_normals)
    return _finish_report(report,params)
