"""Local robust Z estimates at fixed XY; no snapping or global height cap."""
import numpy as np


def _local_plane(points, corner_xy, radius, *, min_points, min_tolerance,
                 max_tolerance, iterations, max_slope, min_consensus, seed):
    xy=(points[:,:2]-corner_xy)/radius
    design=np.column_stack([xy,np.ones(len(points))])
    # Planes need two-dimensional support, not one line of edge points.
    singular=np.linalg.svd(design,compute_uv=False)
    if singular[-1]<singular[0]*1e-4:return None
    rng=np.random.default_rng(seed)
    probe=np.linspace(0,len(points)-1,min(len(points),3000),dtype=int)
    a=design[probe];z=points[probe,2]
    best=None;best_score=(0,-np.inf)
    for _ in range(iterations):
        chosen=rng.choice(len(probe),3,replace=False)
        if abs(np.linalg.det(a[chosen]))<1e-7:continue
        coeff=np.linalg.solve(a[chosen],z[chosen])
        if np.linalg.norm(coeff[:2])/radius>max_slope:continue
        residual=np.abs(z-a@coeff);inside=residual<=min_tolerance
        score=(int(inside.sum()),-float(np.mean(residual[inside]**2)) if inside.any() else -np.inf)
        if score>best_score:best_score=score;best=coeff
    if best is None:return None
    coeff=best;threshold=min_tolerance
    weights=1/(1+np.sum(xy**2,axis=1))
    for _ in range(4):
        residual=points[:,2]-design@coeff
        inside=np.abs(residual)<=threshold
        if inside.sum()<min_points:return None
        a=design[inside]*np.sqrt(weights[inside,None]);z=points[inside,2]*np.sqrt(weights[inside])
        coeff,_,rank,_=np.linalg.lstsq(a,z,rcond=None)
        if rank<3 or np.linalg.norm(coeff[:2])/radius>max_slope:return None
        residual=points[:,2]-design@coeff
        centered=residual[inside]-np.median(residual[inside])
        sigma=1.4826*np.median(np.abs(centered))
        threshold=float(np.clip(3.5*sigma,min_tolerance,max_tolerance))
    residual=points[:,2]-design@coeff
    inside=np.abs(residual)<=threshold
    if inside.sum()<min_points or np.mean(inside)<min_consensus:return None
    a=design[inside]
    if np.linalg.matrix_rank(a)<3 or np.linalg.cond(a)>1e4:return None
    # Residuals, not absolute elevation, define high/low noise.
    rms=float(np.sqrt(np.mean(residual[inside]**2)))
    covariance=np.linalg.pinv(a.T@a)*rms**2
    nearest=float(np.min(np.linalg.norm(points[inside,:2]-corner_xy,axis=1)))
    return dict(z_mm=float(coeff[2]),slope_x=float(coeff[0]/radius),slope_y=float(coeff[1]/radius),
                local_points=len(points),inlier_points=int(inside.sum()),
                consensus_fraction=float(np.mean(inside)),minimum_consensus_fraction=min_consensus,
                rejected_high_points=int(np.count_nonzero(residual>threshold)),
                rejected_low_points=int(np.count_nonzero(residual < -threshold)),
                threshold_mm=threshold,residual_rms_mm=rms,
                fit_standard_error_mm=float(np.sqrt(max(0,covariance[2,2]))),
                nearest_support_xy_mm=nearest,support_extrapolated=bool(nearest>radius*.5),
                inlier_xyz_min=points[inside].min(0).tolist(),inlier_xyz_max=points[inside].max(0).tolist())


def estimate_corner_heights(points, corners, *, radii=(250.,400.,600.,900.),
                            min_points=40, min_tolerance=15., max_tolerance=40.,
                            iterations=160, max_slope=.5, min_consensus=.5, seed=42):
    """Use original same-label 3D points, including global-plane outliers.

    Keep radar XY fixed; yaw-only world conversion also keeps world XY fixed.
    Missing/degenerate support explicitly returns the previous global-plane Z,
    never claims a local estimate. No other label's points are borrowed.
    """
    points=np.asarray(points,dtype=np.float64)
    corners=np.asarray(corners,dtype=np.float64)
    if points.ndim!=2 or points.shape[1]!=3 or corners.ndim!=2 or corners.shape[1]!=3:
        raise ValueError('Expected [N,3] points and [K,3] corners')
    if not np.isfinite(corners).all():raise ValueError('Corners must be finite')
    points=points[np.isfinite(points).all(axis=1)]
    points=points[np.lexsort((points[:,2],points[:,1],points[:,0]))]
    corrected=corners.copy();reports=[]
    for i,corner in enumerate(corners):
        distances=np.linalg.norm(points[:,:2]-corner[:2],axis=1)
        info=None
        for radius in radii:
            local=points[distances<=radius]
            if len(local)<max(min_points,3):continue
            fit=_local_plane(local,corner[:2],radius,min_points=min_points,
                             min_tolerance=min_tolerance,max_tolerance=max_tolerance,
                             iterations=iterations,max_slope=max_slope,min_consensus=min_consensus,seed=seed+i)
            if fit is not None:
                info=dict(status='local_plane',radius_mm=float(radius),**fit)
                corrected[i,2]=fit['z_mm'];break
        if info is None:
            info=dict(status='global_plane_fallback',reason='insufficient_or_degenerate_local_support',z_mm=float(corner[2]))
        info.update(global_plane_z_mm=float(corner[2]),delta_z_mm=float(corrected[i,2]-corner[2]))
        reports.append(info)
    return corrected,reports
