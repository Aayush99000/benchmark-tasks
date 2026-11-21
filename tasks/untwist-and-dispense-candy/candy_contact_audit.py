"""Food contact bounds at every native physics step, independent of the controller."""
import json
import numpy as np
from scipy.spatial.transform import Rotation

from yam_arena import station
from yam_arena.analysis import link_hulls


def evidence(path, *, loose_from_step=None):
    from outcome_geometry import geometry_from_run
    summary=json.loads((path/'summary.json').read_text())
    bodies,_,_=geometry_from_run(path,summary)
    core=next(b for b in bodies if b['name']=='candy')
    with np.load(path/'contact-history.npz',allow_pickle=False) as data:
        poses=data['poses']
        names=list(data['left'])+list(data['right'])
    if not np.isfinite(poses).all() or len(poses)!=summary['actions']*(summary['physics_hz']//summary['control_hz'])+1:
        raise ValueError('Expected the entire native physics history')
    center=poses[:,0,:3]
    shape=np.asarray(core['vertices'])
    radius=float(np.linalg.norm(shape,axis=1).max())
    bottom=(shape@Rotation.from_quat(poses[:,0,3:]).as_matrix().swapaxes(1,2))[:,:,2].min(axis=1)+center[:,2]
    gap=float(bottom.min()-station.FLOOR_Z)
    # Tips are the designated clean food-handling surfaces. Treat every other
    # robot link and the fixed camera apparatus as unclean. A conservative
    # positive clearance certifies separation without estimating contact force.
    bounds={}
    loose_tips = {}
    steps_per_action = summary['physics_hz'] // summary['control_hz']
    hulls = link_hulls()
    for index,name in enumerate(names,1):
        is_tip = name in ('tip_left', 'tip_right')
        if is_tip and loose_from_step is None:
            continue
        parts=hulls[name]
        state=poses[:,index]
        local=Rotation.from_quat(state[:,3:]).inv().apply(center-state[:,:3])
        distances=np.full(len(local),np.inf)
        for hull in parts:
            lo,hi=hull.vertices.min(axis=0),hull.vertices.max(axis=0)
            component=np.maximum(np.maximum(lo-local,local-hi),0.)
            distances=np.minimum(distances,np.linalg.norm(component,axis=1)-radius)
        if is_tip:
            loose_tips[f'{index}-{name}'] = float(distances[loose_from_step*steps_per_action:].min())
        else:
            bounds[f'{index}-{name}']=float(distances.min())
    for name,(kind,size,pos,quat) in station.camera_stand(station.rig().top_cam).items():
        local=Rotation.from_quat(quat).inv().apply(center-pos)
        if kind=='box':
            distances=np.linalg.norm(np.maximum(np.abs(local)-np.array(size)/2,0.),axis=1)
        else:
            local[:,2]-=np.clip(local[:,2],-size[1]/2,size[1]/2)
            distances=np.linalg.norm(local,axis=1)-size[0]
        bounds[name]=float(distances.min()-radius)
    result = dict(physics_samples=len(poses),minimum_floor_clearance_m=gap,
                minimum_unclean_surface_clearance_m=min(bounds.values()),clearance_by_body_m=bounds,
                integrity_model='Single rigid sugar surrogate throughout. Real fracture force is unmeasured.')
    if loose_tips:
        result.update(loose_from_step=loose_from_step,
                      minimum_loose_candy_tip_clearance_m=min(loose_tips.values()))
    return result
