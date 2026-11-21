"""Fixed-station CPU reach screen. Passing IK is not contact/load feasibility."""
import json
from pathlib import Path
import numpy as np
from yam_arena import YAM_URDF,ik,station
from yam_arena.constants import READY,ARM_JOINTS,TCP_OFFSET
from yam_arena.urdf import parse_joints,rpy_matrix,axis_angle_matrix

JOINTS=[parse_joints(YAM_URDF)[n] for n in ARM_JOINTS]
ORIGINS=[rpy_matrix(j.rpy) for j in JOINTS]
RADIUS=sum(float(np.linalg.norm(j.xyz)) for j in JOINTS)+float(np.linalg.norm(TCP_OFFSET))


def fk(q,base):
    r=np.eye(3);p=np.array(base,dtype=float);origins=[];axes=[]
    for joint,origin,angle in zip(JOINTS,ORIGINS,q):
        p=p+r@joint.xyz;r=r@origin;origins.append(p.copy());axes.append(r@joint.axis)
        r=r@axis_angle_matrix(joint.axis,angle)
    tcp=p+r@np.array(TCP_OFFSET)
    return tcp,r,np.cross(np.asarray(axes),tcp-np.asarray(origins)).T,np.asarray(axes).T


def solve(pos,rot,seed,base):
    q=np.array(seed,dtype=float)
    for _ in range(180):
        p,r,jp,jr=fk(q,base);ep=np.array(pos)-p;er=ik._rot_error(rot,r)
        if np.linalg.norm(ep)<1e-4 and np.linalg.norm(er)<1e-3:break
        j=np.vstack([jp,.3*jr]);e=np.r_[ep,.3*er]
        dq=j.T@np.linalg.solve(j@j.T+.015**2*np.eye(6),e)
        q=np.clip(q+np.clip(dq,-.2,.2),ik.LOWER,ik.UPPER)
    p,r,*_=fk(q,base)
    return q,float(np.linalg.norm(p-pos)),float(np.linalg.norm(ik._rot_error(rot,r)))


POINTS={
 'duvet':{'near_insert_corner':(.25,.07,.70),'near_cover_mouth':(.25,-.10,.70),
          'far_bed_corner':(2.25,.675,.70),'opposite_far_corner':(2.25,-.675,.70)},
 'pallet':{'front_film_mid':(.37,0.,.65),'front_left_corner':(.37,.385,.65),
           'rear_film_mid':(1.53,0.,.65),'lower_front_edge':(.37,0.,.15)},
 'stocking':{'initial_cuff':(.35,-.13,.666),'working_toe':(.37,-.071,.895),
             'working_heel':(.37,-.064,.718),'knee_cuff':(.37,.26,.769)},
 'candy':{'left_tail':(.38,.033,.76),'right_tail':(.38,-.033,.76),
          'serving_tile':(.38,-.22,.78)},
}


def main():
    rng=np.random.default_rng(18)
    for _ in range(5):
        q=rng.uniform(ik.LOWER+.01,ik.UPPER-.01)
        p,r,jp,jr=fk(q,station.BASES['left']);ref,rr=ik.tcp_pose(q,station.BASES['left'])
        np.testing.assert_allclose(p,ref,atol=1e-9);np.testing.assert_allclose(r,rr,atol=1e-9)
        for i in range(6):
            q2=q.copy();q2[i]+=1e-6;p2,r2,*_=fk(q2,station.BASES['left'])
            np.testing.assert_allclose((p2-p)/1e-6,jp[:,i],atol=1e-6)
    out=dict(evidence='kinematic point screen only',generous_tcp_radius_bound_m=RADIUS,
             base_positions_m=station.BASES,tasks={})
    down=np.diag([1.,-1.,-1.])
    for task,points in POINTS.items():
        findings={}
        for label,pos in points.items():
            candidates=[]
            for side,base in station.BASES.items():
                distance=float(np.linalg.norm(np.array(pos)-base))
                if distance>RADIUS:
                    candidates.append(dict(arm=side,status='outside_generous_radius',distance_m=distance));continue
                for yaw in [0.,np.pi/2]:
                    rot=axis_angle_matrix(np.array([0.,0.,1.]),yaw)@down
                    trials=[solve(pos,rot,seed,base) for seed in [READY,[0.,2.,1.,-1.,0.,0.],rng.uniform(ik.LOWER,ik.UPPER)]]
                    q,pe,re=min(trials,key=lambda t:t[1]+.3*t[2])
                    candidates.append(dict(arm=side,yaw_rad=yaw,status='ik_candidate' if pe<.002 and re<.03 else 'ik_not_found',
                        distance_m=distance,q=q.tolist(),position_error_m=pe,rotation_error_rad=re))
            findings[label]=dict(position_m=pos,candidates=candidates,
                directly_reachable_candidate=any(c['status']=='ik_candidate' for c in candidates))
        out['tasks'][task]=findings
    path=Path(__file__).resolve().parent/'evidence/reach-audit.json'
    path.write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({task:{key:v['directly_reachable_candidate'] for key,v in points.items()} for task,points in out['tasks'].items()},indent=2))


if __name__=='__main__':main()
