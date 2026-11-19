"""Reduced position-based film solver. No phase or goal inputs.

This is an uncalibrated hybrid prototype, not native PhysX thin-film FEM.
The rigid simulator supplies measured TCPs, jaw openings and melon pose.
Only a closed, spatially aligned edge pinch creates a compliant attachment.
Supply rows enter at the fixed dispenser lip under tensile pull. The cutter
must sweep every width interval with film present before the supply detaches.
"""
from dataclasses import dataclass
import numpy as np
from contact_fast import kernel,project

WIDTH=.40
DX=.0125
NY=24
EXIT=np.array([.180,0.,.850])
TABLE=.750
RADIUS=.09
MELON=np.array([.350,0.,.840])
MAX_LENGTH=.70

@dataclass
class Pinch:
    arm:int
    node:int
    offset:np.ndarray

class Film:
    def __init__(self):
        self.x=np.array([[EXIT+[DX*i,y,0] for y in np.linspace(-WIDTH/2,WIDTH/2,NY+1)] for i in range(6,-1,-1)])
        self.v=np.zeros_like(self.x)
        self.paid=.075
        self.cut=False
        self.cut_lanes=np.zeros(NY,dtype=bool)
        self.pins={}
        self.adhesive={}
        self.both_pinch_seen=False
        self.torn=False
        self.peak_strain=1.
        self.last_slider=None
        self.slide_started=False
        self.cut_before_tuck=False
        self.payout_count=0
        self.overstrain_time=0.
        self._rebuild()

    def _rebuild(self):
        nr,nc=self.x.shape[:2];ids=np.arange(nr*nc).reshape(nr,nc)
        self.groups=[]
        # Nonoverlapping colour batches permit in-place constraint updates.
        for axis in [0,1]:
            for parity in [0,1]:
                if axis==0:a=ids[parity:-1:2,:];b=ids[parity+1::2,:];length=DX
                else:a=ids[:,parity:-1:2];b=ids[:,parity+1::2];length=WIDTH/NY
                self.groups.append((a.ravel(),b.ravel(),length,1.))
        for parity in [0,1]:
            for col in [0,1]:
                for flip in [False,True]:
                    a=ids[parity:-1:2,col:-1:2];b=ids[parity+1::2,col+1::2]
                    if flip:a=ids[parity:-1:2,col+1::2];b=ids[parity+1::2,col:-1:2]
                    self.groups.append((a.ravel(),b.ravel(),np.hypot(DX,WIDTH/NY),.65))
        f=[]
        for i in range(nr-1):
            for j in range(nc-1):
                a,b,c,d=ids[i,j],ids[i+1,j],ids[i+1,j+1],ids[i,j+1]
                f.extend([(a,b,c),(a,c,d)])
        self.faces=np.asarray(f,np.int32)
        self.rest_edges=np.array([[a,b] for a,b,_,_ in self.groups for a,b in zip(a,b)],int)
        self.rest_lengths=np.concatenate([np.full(len(a),r) for a,b,r,s in self.groups])

    def _feed(self):
        if self.cut or self.paid+DX>MAX_LENGTH:return False
        edge=np.array([EXIT+[0,y,0] for y in np.linspace(-WIDTH/2,WIDTH/2,NY+1)])
        self.x=np.concatenate([self.x,edge[None]],axis=0)
        self.v=np.concatenate([self.v,np.zeros_like(edge)[None]],axis=0)
        self.paid+=DX;self.payout_count+=1;self._rebuild();return True

    def update_pinch(self,tcp,rot,aperture):
        flat=self.x.reshape(-1,3)
        for arm in range(2):
            if arm in self.pins:
                p=self.pins[arm];target=tcp[arm]+rot[arm]@p.offset
                if aperture[arm]>.023 or np.linalg.norm(flat[p.node]-target)>.035:
                    del self.pins[arm]
                continue
            if aperture[arm]>.012:continue
            # Grasp a material boundary, never any point in empty space.
            nr,nc=self.x.shape[:2]
            candidates=np.unique(np.r_[np.arange(nc),np.arange(nr)*nc,np.arange(nr)*nc+nc-1])
            candidates=candidates[candidates<(nr-1)*nc]
            local=(flat[candidates]-tcp[arm])@rot[arm]
            good=(np.abs(local[:,0])<.010)&(np.abs(local[:,1])<.009)&(np.abs(local[:,2])<.013)
            if good.any():
                candidate=candidates[good][np.argmin(np.linalg.norm(local[good],axis=1))]
                if any(p.node==candidate for p in self.pins.values()):continue
                self.pins[arm]=Pinch(arm,int(candidate),(flat[candidate]-tcp[arm])@rot[arm])
        # The draw witness requires opposite material side edges, not merely
        # two different nearby nodes on the same boundary.
        edge_columns={p.node % self.x.shape[1] for p in self.pins.values()}
        self.both_pinch_seen |= len(self.pins)==2 and edge_columns=={0,self.x.shape[1]-1}

    def cutter(self,slider,tucked):
        """Native carriage centre. Cutter line is 3 mm below its centre."""
        s=np.asarray(slider,float)
        aligned=abs(s[0]-EXIT[0])<.008 and abs(s[2]-(EXIT[2]+.023))<.010
        if self.last_slider is not None and aligned and not self.cut:
            if self.last_slider[1]>=WIDTH/2-.005:self.slide_started=True
            if self.slide_started and s[1]<self.last_slider[1]:
                mid=(np.linspace(-WIDTH/2,WIDTH/2,NY+1)[:-1]+WIDTH/(2*NY))
                present=np.linalg.norm(self.x[-1]-np.array([EXIT+[0,y,0] for y in np.linspace(-WIDTH/2,WIDTH/2,NY+1)]),axis=1)<.006
                self.cut_lanes|=(mid>=s[1]-.005)&(mid<=self.last_slider[1]+.005)&present[:-1]&present[1:]
            if self.cut_lanes.all():
                self.cut=True;self.cut_before_tuck=not bool(tucked)
        self.last_slider=s.copy()

    def step(self,tcp,rot,aperture,melon_pos=MELON,melon_rot=np.eye(3),dt=.05,substeps=10,gripper_boxes=()):
        self.update_pinch(tcp,rot,aperture)
        h=dt/substeps
        fed=False
        # Broad-phase bounds retain the separate collision pieces and their
        # empty spaces; a whole-gripper solid box would create false contacts.
        world_bounds=[]
        for world,bounds in gripper_boxes:
            centre=(bounds[0]+bounds[1])/2;half=(bounds[1]-bounds[0])/2+.0004
            centre=world[:3,:3]@centre+world[:3,3];half=np.abs(world[:3,:3])@half
            world_bounds.append([centre-half,centre+half])
        world_bounds=np.asarray(world_bounds)
        transforms=np.ascontiguousarray([w for w,b in gripper_boxes],dtype=np.float64)
        box_bounds=np.ascontiguousarray([b for w,b in gripper_boxes],dtype=np.float64)
        for _ in range(substeps):
            old=self.x.copy()
            self.v*=.988
            self.v[:,:,2]-=9.81*h
            self.x+=self.v*h
            flat=self.x.reshape(-1,3)
            # Tiny pinch patches retain local grasp offsets without goal snapping.
            pins={p.node:tcp[k]+rot[k]@p.offset for k,p in self.pins.items()}
            for node,target in pins.items():flat[node]=target
            # Dispenser slip brake: feed only from tensile extension; gravity alone
            # cannot release the roll. This surrogate must be calibrated in hardware.
            path=np.linalg.norm(np.diff(self.x[sorted(set(list(range(0,len(self.x),4))+[len(self.x)-1]))],axis=0),axis=2).sum(axis=0)
            if self.pins and not self.cut and not fed and np.percentile(path,90)>self.paid*1.08:
                if self._feed():
                    fed=True;old=np.concatenate([old,self.x[-1:]],axis=0);flat=self.x.reshape(-1,3)
            weights=np.ones(len(flat));nc=NY+1
            if not self.cut:weights[-nc:]=0.
            for n in pins:weights[n]=0.
            pin_mask=np.zeros(len(flat),dtype=np.uint8)
            for n in pins:pin_mask[n]=1
            for iteration in range(20):
                for aa,bb,rest,stiff in self.groups:
                    d=flat[bb]-flat[aa];ln=np.linalg.norm(d,axis=1)
                    w=weights[aa]+weights[bb]
                    corr=d*((ln-rest)/np.maximum(ln,1e-10)*stiff/np.maximum(w,1e-10))[:,None]
                    flat[aa]+=corr*weights[aa,None];flat[bb]-=corr*weights[bb,None]
                if not self.cut:flat[-nc:]=np.array([EXIT+[0,y,0] for y in np.linspace(-WIDTH/2,WIDTH/2,nc)])
                for node,target in pins.items():flat[node]=target
                # Hemispherical fruit with its cut plane z=0 in the fruit frame.
                p=(flat-melon_pos)@melon_rot
                rad=np.linalg.norm(p,axis=1);rho=np.linalg.norm(p[:,:2],axis=1)
                inside=(rad<RADIUS+.0008)&(p[:,2]<.0008)
                plane_dist=.0008-p[:,2]
                rind_dist=RADIUS+.0008-rad
                use_top=inside&(plane_dist<rind_dist)
                p[use_top,2]=.0008
                use_rind=inside&~use_top
                p[use_rind]*=((RADIUS+.0008)/np.maximum(rad[use_rind],1e-9))[:,None]
                flat[:]=p@melon_rot.T+melon_pos
                # The underside press requires film/finger contact, not just a
                # corner attachment. Use native measured URDF collision bounds.
                # Pinned material points remain compliant jaw attachments.
                if kernel is not None and len(gripper_boxes):project(flat,transforms,box_bounds,world_bounds,pin_mask)
                active=np.flatnonzero(np.all(world_bounds[:,1]>=flat.min(axis=0),axis=1)&np.all(world_bounds[:,0]<=flat.max(axis=0),axis=1)) if len(gripper_boxes) and kernel is None else []
                for box_id in active:
                    world,bounds=gripper_boxes[box_id]
                    local=(flat-world[:3,3])@world[:3,:3]
                    lo=bounds[0]-.0004;hi=bounds[1]+.0004
                    inside=np.all((local>lo)&(local<hi),axis=1)
                    for node in pins:inside[node]=False
                    ids=np.flatnonzero(inside)
                    if len(ids):
                        low=local[ids]-lo;high=hi-local[ids]
                        axis=np.argmin(np.minimum(low,high),axis=1)
                        local[ids,axis]=np.where(low[np.arange(len(ids)),axis]<high[np.arange(len(ids)),axis],lo[axis],hi[axis])
                        flat[ids]=local[ids]@world[:3,:3].T+world[:3,3]
                flat[:,2]=np.maximum(flat[:,2],TABLE+.0005)
                # Match native four rigid fruit rests with point/box exclusion.
                for dx in [-.065,.065]:
                    for dy in [-.020,.020]:
                        centre=MELON+[dx,dy,-.071];half=np.array([.006,.006,.019])+.0005
                        rel=flat-centre;inside=np.all(np.abs(rel)<half,axis=1)
                        ids=np.flatnonzero(inside)
                        if len(ids):
                            axis=np.argmin(half-np.abs(rel[ids]),axis=1)
                            flat[ids,axis]=centre[axis]+np.where(rel[ids,axis]>=0,1,-1)*half[axis]
                # Supported reset lip prevents the free edge vanishing before pickup.
                lip=(flat[:,0]>=EXIT[0]-.005)&(flat[:,0]<=EXIT[0]+.078)&(np.abs(flat[:,1])<=WIDTH/2+.005)
                flat[lip,2]=np.maximum(flat[lip,2],EXIT[2])
                for node,local in list(self.adhesive.items()):
                    target=melon_pos+melon_rot@local
                    if np.linalg.norm(flat[node]-target)>.015 or node in pins:
                        del self.adhesive[node]
                    else:flat[node]=flat[node]*.2+target*.8
            # Weak, local contact tack; does not attract distant film to the fruit.
            p=(flat-melon_pos)@melon_rot;rad=np.linalg.norm(p,axis=1)
            contact=(np.abs(rad-RADIUS)<.0018)&(p[:,2]<-.015)
            for node in np.flatnonzero(contact):
                if node not in pins:self.adhesive.setdefault(int(node),p[node].copy())
            self.v=(self.x-old)/h
            speed=np.linalg.norm(self.v,axis=2);self.v*=np.minimum(1.,2./np.maximum(speed,1e-8))[:,:,None]
            lengths=np.linalg.norm(flat[self.rest_edges[:,1]]-flat[self.rest_edges[:,0]],axis=1)
            strain=float(np.max(lengths/self.rest_lengths))
            self.peak_strain=max(self.peak_strain,strain)
            # Failure latch: gross stretch or nonfinite state invalidates all scores.
            self.overstrain_time=self.overstrain_time+h if strain>1.6 else max(0.,self.overstrain_time-h*.1)
            self.torn|=strain>3.0 or self.overstrain_time>.20 or not np.isfinite(flat).all()
        return self.x
