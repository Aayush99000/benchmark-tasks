"""Geometry-only coverage and underside scoring; controller labels are ignored."""
import numpy as np
from film import RADIUS,MELON,TABLE,NY

def coverage(nodes,faces,pos,rot):
    p=(nodes.reshape(-1,3)-pos)@rot
    # Equal-area Cartesian samples on the cut face, 5 mm spacing.
    xy=np.array([(x,y) for x in np.arange(-RADIUS,RADIUS+.001,.005) for y in np.arange(-RADIUS,RADIUS+.001,.005) if x*x+y*y<(RADIUS-.003)**2])
    covered=np.zeros(len(xy),bool)
    for tri in p[faces]:
        if tri[:,2].max()<-.002 or tri[:,2].min()>.012:continue
        a,b,c=tri[:,:2];v0=b-a;v1=c-a;det=v0[0]*v1[1]-v0[1]*v1[0]
        if abs(det)<1e-10:continue
        v=xy-a;u=(v[:,0]*v1[1]-v[:,1]*v1[0])/det;w=(v0[0]*v[:,1]-v0[1]*v[:,0])/det
        hit=(u>=0)&(w>=0)&(u+w<=1)
        z=tri[0,2]+u*(tri[1,2]-tri[0,2])+w*(tri[2,2]-tri[0,2])
        covered|=hit&(z>=-.002)&(z<=.012)
    return float(covered.mean())

def tucks(nodes,pos,rot,faces=None):
    from tuck_measurement import measure_tucks
    return [p['passed'] for p in measure_tucks(nodes,pos,rot,faces)]

class Evaluator:
    def __init__(self):self.stable=0.;self.failed=False;self.result={'score':0.}
    def update(self,film,pos,rot,tcp,clearance,dt=.05):
        nodes=film.x
        valid=np.isfinite(nodes).all() and np.isfinite(pos).all() and np.isfinite(rot).all()
        self.failed|=not valid or film.torn or pos[2]<TABLE+.035 or np.linalg.norm(pos[:2]-MELON[:2])>.15 or rot[2,2]<.7 or film.cut_before_tuck
        cov=coverage(nodes,film.faces,pos,rot) if valid else 0.
        from tuck_measurement import measure_tucks
        patches=measure_tucks(nodes,pos,rot,film.faces) if valid else []
        tucked=[p['passed'] for p in patches] if valid else [False,False]
        drawn=film.both_pinch_seen and .30<=film.paid<=.65
        released=len(film.pins)==0 and clearance>=.020
        full=drawn and cov>=.95 and all(tucked) and film.cut and released and not self.failed
        self.stable=self.stable+dt if full else 0.
        score=0.
        if not self.failed:
            if drawn:score=.2
            if drawn and cov>=.95:score=.5
            if drawn and cov>=.95 and all(tucked):score=.75
            if full and self.stable>=1.:score=1.
        self.result={'scoring_version':'2.0','tuck_patches':patches,'score':score,'coverage':cov,'underside_tucks':tucked,'dispensed_m':round(film.paid,5),'both_edges_grasped':film.both_pinch_seen,'cut':film.cut,'cut_fraction':float(film.cut_lanes.mean()),'active_grasps':len(film.pins),'released':released,'robot_clearance_m':float(clearance),'stable_seconds':self.stable,'peak_edge_strain':film.peak_strain,'invalid':self.failed,'full_task_success':score==1.}
        return self.result
