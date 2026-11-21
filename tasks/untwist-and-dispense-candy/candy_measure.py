"""Estimate wrapper coverage from a rigid rectangular candy and a native triangle mesh.

Sample each of six faces with an equal-area grid. Cast orthogonal rays from
0.5 mm inside the surface to 20 mm outside it. This estimates local coverage,
including underneath the candy. A separate geometric separation check guards
against missing a small piece between the sampled rays.
"""
from itertools import product
from functools import lru_cache
import numpy as np
from scipy.spatial.transform import Rotation


@lru_cache(maxsize=24)
def emission(size, grid, axis, sign, hull):
    from scipy.spatial import ConvexHull
    half=np.array(size)/2
    a,b=[k for k in range(3) if k!=axis]
    u,v=np.meshgrid((2*np.arange(grid)+1)/grid-1,(2*np.arange(grid)+1)/grid-1,indexing='ij')
    u,v=u.ravel(),v.ravel()
    directions=np.zeros((len(u),3))
    directions[:,axis]=sign
    directions[:,a]=u
    directions[:,b]=v
    if hull:
        planes=ConvexHull(hull).equations
        dot=directions@planes[:,:3].T
        depth=np.full_like(dot,np.inf)
        np.divide(-planes[:,3],dot,out=depth,where=dot>1e-12)
        face=np.argmin(depth,axis=1)
        idx=np.arange(len(u))
        along=depth[idx,face]
        weights=along**2/dot[idx,face]
    else:
        norm=np.sqrt(1+u*u+v*v)
        along=half[0]/norm
        weights=half[0]**2/norm**3
    return along*u,along*v,along-.0005,weights


def indicators(points, faces, pose, size=(.016,.024,.010), grid=9, sphere=False, hull=None):
    half = np.asarray(size)/2
    local = Rotation.from_quat(pose[3:]).inv().apply(np.asarray(points)-pose[:3])
    triangles = local[faces]
    lo, hi = triangles.min(axis=1), triangles.max(axis=1)
    box_gaps = np.maximum(np.maximum(lo-half, -half-hi),0.)
    # Nearest distance between triangle bounding boxes and candy. This lower
    # bound can certify separation, including between ray samples.
    separation = np.linalg.norm(box_gaps,axis=1).min()
    if sphere:
        nearest_box=np.maximum(np.maximum(lo,-hi),0.)
        radius=np.linalg.norm(hull,axis=1).max() if hull is not None else half[0]
        separation=max(0.,np.linalg.norm(nearest_box,axis=1).min()-radius)
        # A disjoint envelope certifies the whole sphere is outside even if a
        # distant part of the wrapper projects onto it in one camera view.
        gap=np.maximum(local.min(0)-half,-half-local.max(0))
        if np.max(gap)>=.002:
            return dict(exposed_fraction=1.,separation_lower_bound_m=float(separation))
    hull_key=tuple(map(tuple,hull)) if hull is not None else ()
    covered_area = total_area = 0.
    for axis,sign in product(range(3),(-1,1)):
        a,b = [index for index in range(3) if index != axis]
        min_depth = (0. if hull_key else half[axis]/np.sqrt(3.) if sphere else half[axis])-.0005
        max_depth = np.max(sign*local[:,axis])+.0001 if sphere else half[axis]+.020
        near = (np.maximum(sign*lo[:,axis],sign*hi[:,axis]) >= min_depth)
        near &= (np.minimum(sign*lo[:,axis],sign*hi[:,axis]) <= max_depth)
        near &= (hi[:,a] >= -half[a]) & (lo[:,a] <= half[a])
        near &= (hi[:,b] >= -half[b]) & (lo[:,b] <= half[b])
        area = size[a]*size[b]
        if sphere:
            xa,yb,ray_start,weights=emission(tuple(size),grid,axis,sign,hull_key)
            area=float(4*np.mean(weights))
        total_area += area
        tri = triangles[near]
        if not len(tri):
            continue
        ab, ac = tri[:,1]-tri[:,0], tri[:,2]-tri[:,0]
        denom = ab[:,a]*ac[:,b]-ab[:,b]*ac[:,a]
        valid = np.abs(denom)>1e-14
        tri,ab,ac,denom = tri[valid],ab[valid],ac[valid],denom[valid]
        # Use each triangle's projected bounds to reject most candidate rays
        # before calculating barycentric coordinates. This permits scoring
        # every 20 Hz control sample during a full-limit waiting trial.
        if sphere:
            ray_end=np.full_like(ray_start,max_depth)
        else:
            fractions=(np.arange(grid)+.5)/grid-.5
            xa,yb=np.meshgrid(size[a]*fractions,size[b]*fractions,indexing='ij')
            xa,yb=xa.ravel(),yb.ravel()
        hit=np.zeros(grid*grid,dtype=bool)
        lower=tri.min(axis=1)
        upper=tri.max(axis=1)
        for first in range(0,len(tri),256):
            block=slice(first,first+256)
            possible=((xa[:,None]>=lower[block,a])&(xa[:,None]<=upper[block,a])
                      &(yb[:,None]>=lower[block,b])&(yb[:,None]<=upper[block,b]))
            samples,offsets=np.nonzero(possible)
            t=first+offsets
            x=xa[samples]-tri[t,0,a]
            y=yb[samples]-tri[t,0,b]
            u=(x*ac[t,b]-y*ac[t,a])/denom[t]
            v=(ab[t,a]*y-ab[t,b]*x)/denom[t]
            depth=sign*(tri[t,0,axis]+u*ab[t,axis]+v*ac[t,axis])
            depth_ok=((depth>=ray_start[samples])&(depth<=ray_end[samples]) if sphere else (depth>=min_depth)&(depth<=max_depth))
            valid_hit=(u>=0)&(v>=0)&(u+v<=1)&depth_ok
            hit[samples[valid_hit]]=True
        covered_area += area*(np.average(hit,weights=weights) if sphere else hit.mean())
    return dict(exposed_fraction=float(np.clip(1-covered_area/total_area,0,1)),
                separation_lower_bound_m=float(separation))


def twist_degrees(points, shape):
    """Material winding on each half, with bent centerlines parallel transported."""
    rings=np.asarray(points,dtype=float).reshape(*shape,3)
    centers=rings.mean(axis=1)
    tangents=np.gradient(centers,axis=0)
    lengths=np.linalg.norm(tangents,axis=1,keepdims=True)
    if lengths.min()<1e-8:
        return None
    axes=tangents/lengths
    radial=rings-centers[:,None,:]
    radial-=np.sum(radial*axes[:,None,:],axis=2,keepdims=True)*axes[:,None,:]
    cross=np.cross(axes[:-1],axes[1:])[:,None,:]
    dot=np.sum(axes[:-1]*axes[1:],axis=1)[:,None,None]
    if np.min(dot)<-.98:
        return None
    v=radial[:-1]
    transported=v+np.cross(cross,v)+np.cross(cross,np.cross(cross,v))/(1+dot)
    sin=np.sum(np.cross(transported,radial[1:])*axes[1:,None,:],axis=(1,2))
    cos=np.sum(transported*radial[1:],axis=(1,2))
    angles=np.rad2deg(np.arctan2(sin,cos))
    middle=len(angles)//2
    return [float(angles[:middle].sum()),float(angles[middle:].sum())]
