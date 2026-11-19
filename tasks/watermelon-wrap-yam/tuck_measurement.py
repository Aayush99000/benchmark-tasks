"""Side-view union area on a fixed 2 mm grid; no mesh-node-count credit.

All lengths are metres. Grid cells are counted by their centre, exactly as the
physical image overlay. Overlapping folds count once; only the largest connected
4-neighbour patch on each side counts. This is a scoring change (version 2).
"""
import numpy as np
from scipy.ndimage import label

PITCH = .002
MIN_AREA = .000200
MIN_WIDTH = .015

def mesh_faces(nodes):
    if np.asarray(nodes).ndim != 3:
        return np.empty((0,3),int)
    nx,ny=np.asarray(nodes).shape[:2]
    return np.array([(i*ny+j,(i+1)*ny+j,(i+1)*ny+j+1)
        for i in range(nx-1) for j in range(ny-1)]+[
        (i*ny+j,(i+1)*ny+j+1,i*ny+j+1)
        for i in range(nx-1) for j in range(ny-1)],int)

def measure_tucks(nodes,pos,rot,faces=None):
    p=(np.asarray(nodes).reshape(-1,3)-pos)@rot
    faces=mesh_faces(nodes) if faces is None else np.asarray(faces,int)
    xs=np.arange(-.070+PITCH/2,.070,PITCH)
    zs=np.arange(-.098+PITCH/2,-.045,PITCH)
    xx,zz=np.meshgrid(xs,zs)
    samples=np.c_[xx.ravel(),zz.ravel()]
    masks=np.zeros((2,len(samples)),bool)
    if not np.isfinite(p).all():
        return [dict(passed=False,area_mm2=0.,width_mm=0.) for _ in range(2)]
    for tri in p[faces]:
        if tri[:,2].min()>=-.045 or tri[:,2].max()<=-.098:continue
        a,b,c=tri[:,[0,2]];v0=b-a;v1=c-a
        det=v0[0]*v1[1]-v0[1]*v1[0]
        if abs(det)<1e-12:continue
        v=samples-a
        u=(v[:,0]*v1[1]-v[:,1]*v1[0])/det
        w=(v0[0]*v[:,1]-v0[1]*v[:,0])/det
        hit=(u>=0)&(w>=0)&(u+w<=1)
        inds=np.flatnonzero(hit)
        if not len(inds):continue
        q=tri[0]+u[inds,None]*(tri[1]-tri[0])+w[inds,None]*(tri[2]-tri[0])
        r=np.linalg.norm(q,axis=1);rho=np.linalg.norm(q[:,:2],axis=1)
        near=(abs(r-.090)<.007)&(q[:,2]<-.045)&(rho<.070)
        for k,sign in enumerate((1,-1)):
            masks[k,inds]|=near&(sign*q[:,1]>.018)
    result=[]
    for mask in masks:
        lab,n=label(mask.reshape(xx.shape))
        best_area=best_width=0.
        for k in range(1,n+1):
            rows,cols=np.nonzero(lab==k)
            area=len(cols)*PITCH**2
            width=(cols.max()-cols.min()+1)*PITCH
            if area>best_area:best_area,best_width=area,width
        result.append(dict(passed=bool(best_area>=MIN_AREA and best_width>=MIN_WIDTH),
            area_mm2=round(best_area*1e6,6),width_mm=round(best_width*1000,6)))
    return result
