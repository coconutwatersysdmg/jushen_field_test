"""Training-style full-truck sampling plus explicit direct point coverage."""
import numpy as np


def voxel_uniform_choice(xyz, npoints, rng):
    """Numeric sampling used by the supplied training loader (local RNG)."""
    xyz=np.asarray(xyz,dtype=np.float32)
    n=len(xyz)
    if n==0 or npoints<1:raise ValueError('Sampling requires points and a positive size')
    if n==1:return np.zeros(npoints,dtype=np.int64)
    low=xyz.min(0);span=np.max(xyz.max(0)-low)
    size=span/np.sqrt(float(npoints)) if span>1e-6 else 1.0
    vox=np.floor((xyz-low)/size).astype(np.int64)
    perm=rng.permutation(n)
    _,positions=np.unique(vox[perm],axis=0,return_index=True)
    representatives=perm[positions].astype(np.int64)
    if len(representatives)>=npoints:
        choice=rng.choice(representatives,npoints,replace=False)
    else:
        choice=np.concatenate([representatives,rng.choice(n,npoints-len(representatives),replace=True)])
    rng.shuffle(choice)
    return choice.astype(np.int64)


def iter_full_coverage_samples(xyz, num_points=49152, seed=42, context_fraction=.5):
    """Yield full-truck index groups, ensuring every input row is forwarded.

    The caller canonicalizes source rows first for permutation repeatability.
    Mandatory coverage slots visit each row once. Other slots use the package
    loader's voxel sampling from the entire truck, so later groups still see
    sparse structures already covered earlier. No spatial block cropping or
    interpolation of labels is involved. Coverage is an extension of training
    sampling, not an assertion of an identical input distribution.
    """
    xyz=np.asarray(xyz,dtype=np.float32)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not len(xyz) or not np.isfinite(xyz).all():
        raise ValueError('Coverage input must be nonempty finite [N,3]')
    if num_points<1 or not 0<=context_fraction<1:
        raise ValueError('Invalid group size or context fraction')
    rng=np.random.RandomState(seed)
    mandatory=rng.permutation(len(xyz))
    capacity=num_points if len(xyz)<=num_points else max(1,int(num_points*(1-context_fraction)))
    for start in range(0,len(xyz),capacity):
        cover=mandatory[start:start+capacity]
        extra_count=num_points-len(cover)
        # Keep the same voxel size as training's full 49152-point selection.
        extra=voxel_uniform_choice(xyz,num_points,rng)[:extra_count] if extra_count else np.empty(0,np.int64)
        group=np.concatenate([cover,extra])
        rng.shuffle(group)
        yield group
