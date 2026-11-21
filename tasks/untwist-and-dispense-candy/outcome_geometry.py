"""Geometric checks for the four native task outcomes, independent of the controller.

Incomplete or fabricated damage/contamination records cannot be inferred from
native geometry. These diagnostics preserve the separate reviewed scorer.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np


def capped_mesh(points, faces):
    """Cap the only opening with an oriented fan for numerical inside tests."""
    edges=np.concatenate((faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]))
    _,ind,count=np.unique(np.sort(edges,axis=1),axis=0,return_index=True,return_counts=True)
    boundary=edges[ind[count==1]]
    if len(boundary)<3:
        raise ValueError('A garment or cover needs one open boundary')
    nodes=np.unique(boundary)
    if len(nodes)!=len(boundary):
        raise ValueError('Open boundary must be a simple loop')
    next_node=dict(boundary.tolist())
    start=int(nodes[0]); current=start
    for step in range(len(nodes)):
        current=next_node[current]
        if current==start and step<len(nodes)-1:
            raise ValueError('More than one open boundary')
    if current!=start:
        raise ValueError('Invalid directed boundary')
    center=points[nodes].mean(axis=0)
    cap=np.column_stack((boundary[:,1],boundary[:,0],np.full(len(nodes),len(points))))
    return np.vstack((points,center)),np.vstack((faces,cap))


def winding_number(points,faces,queries):
    """Generalized winding of the actual triangles, in batches of eight queries."""
    triangles=points[faces]
    values=np.zeros(len(queries))
    active=np.flatnonzero(((queries>=points.min(axis=0))&(queries<=points.max(axis=0))).all(axis=1))
    for block in np.array_split(active,max(1,(len(active)+7)//8)):
        if not len(block):
            continue
        a,b,c=(triangles[None,:,i,:]-queries[block,None,:] for i in range(3))
        la,lb,lc=(np.linalg.norm(v,axis=-1) for v in (a,b,c))
        determinant=np.sum(a*np.cross(b,c),axis=-1)
        denominator=la*lb*lc+np.sum(a*b,axis=-1)*lc+np.sum(b*c,axis=-1)*la+np.sum(c*a,axis=-1)*lb
        values[block]=np.abs(np.arctan2(determinant,denominator).sum(axis=1)/(2*np.pi))
    return values


def geometry_from_run(run, summary):
    """Regenerate geometry using the modules archived with this trial."""
    import sys
    names = ['geometry']
    variant = summary.get('variant')
    if variant:
        names.extend(['pleated_wrapper', 'layout_variants'])
    if variant in ('dropped-sheet','bedside-paddle','loose-film','low-turntable'):
        names.insert(1,'handling_setups')
    if variant=='bedside-paddle':
        names.insert(1,'bed_paddle')
    if variant in ('wire-donner','open-stocking-frame'):
        names.insert(1,'donning_frame')
    if variant in ('faceted-drop','printed-drop'):
        names.insert(1,'faceted_sweet')
    previous = {name: sys.modules.get(name) for name in names}
    try:
        for name in names:
            spec = importlib.util.spec_from_file_location(name, run/'source'/(name+'.py'))
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            spec.loader.exec_module(module)
        return module.BUILDERS[variant or summary['task']]()
    finally:
        for name, old in previous.items():
            if old is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old


def largest_corner_excursion(poses, size):
    """Maximum travel from reset of any corresponding material corner."""
    from itertools import product
    from scipy.spatial.transform import Rotation
    poses = np.asarray(poses)
    corners = np.array(list(product((-1, 1), repeat=3))) * np.asarray(size) / 2
    rotations = Rotation.from_quat(poses[:, 3:]).as_matrix()
    world = (corners @ rotations.swapaxes(1, 2)) + poses[:, None, :3]
    return float(np.linalg.norm(world - world[0], axis=-1).max())


def winding_history(rows):
    # Both arms stay in ready pose during the five-second settling interval.
    # The first exposure above 90% closes the opportunity to earn twist credit.
    baseline=[r['tail_winding_deg'] for r in rows if 80<=r['step']<=100]
    if len(baseline)!=21 or any(w is None for w in baseline):
        raise ValueError('Missing pre-grasp winding')
    rest=np.median(baseline,axis=0)
    eligible=np.abs(rest)>=180.
    window=[]
    strongest=0.
    held=0.
    for row in rows[101:]:
        if row['wrapper_coverage']['exposed_fraction']>=.90:break
        if row['tail_winding_deg'] is None:
            window.clear()
            continue
        gain=np.where(eligible,np.abs(rest)-np.abs(row['tail_winding_deg']),0.)
        # Grasp must be measured near the corresponding end of the real sheet.
        tips=np.asarray(row['paper_landmarks_m'])[[0,2]]
        grasp=np.zeros(2,dtype=bool)
        for jaw in row.get('grippers',{}).values():
            if -sum(jaw['finger_joints'])<=.019:
                grasp |= np.linalg.norm(tips-jaw['tcp_m'],axis=1)<=.035
        window=(window+[(gain,np.where(grasp,gain,0.))])[-6:]
        if len(window)==6:
            floor=np.min(window,axis=0)
            strongest=max(strongest,float(floor[0].max()))
            held=max(held,float(floor[1].max()))
    return dict(pregrasp_tail_winding_deg=rest.tolist(),sustained_unwind_deg=strongest,
                grasped_sustained_unwind_deg=held)


def evaluate(run):
    summary=json.loads((run/'summary.json').read_text())
    if not summary['status'].startswith('completed_native_'):
        raise ValueError(str(run)+' is incomplete')
    rows=[json.loads(l) for l in (run/'trace.jsonl').read_text().splitlines()]
    if [r['step'] for r in rows]!=list(range(summary['actions']+1)):
        raise ValueError('Incomplete trace')
    task=summary['task']
    bodies,surfaces,details=geometry_from_run(run,summary)
    vertices={s.name:np.load(run/f'{s.name}-{summary["actions"]:04d}.npy') for s in surfaces}
    result=dict(task=task,motion=summary.get('motion'),actions=summary['actions'],task_score=None)
    if task=='candy':
        samples=[r for r in rows if 'wrapper_coverage'in r]
        result.update(final_exposed_fraction=samples[-1]['wrapper_coverage']['exposed_fraction'],
                      peak_exposed_fraction=max(r['wrapper_coverage']['exposed_fraction'] for r in samples),
                      final_wrapper_clearance_lower_bound_m=samples[-1]['wrapper_coverage']['separation_lower_bound_m'])
        tile=next((b for b in bodies if b['name']=='serving_tile'),None)
        surface_name='tile' if tile else 'table'
        if tile is None:
            from yam_arena import station
            tile={'center':station.TABLE_CENTER,'size':station.TABLE_SIZE}
        candy=next(b for b in bodies if b['name']=='candy')
        from scipy.spatial.transform import Rotation
        from itertools import product
        from candy_measure import indicators
        last_pose=rows[-1]['rigid_objects']['candy']
        wrapper=vertices['wrapper']
        wrapper_faces=next(s.faces for s in surfaces if s.name=='wrapper')
        result['final_exposed_fraction_by_face_grid']={
            str(n):indicators(wrapper,wrapper_faces,last_pose,candy['size'],grid=n,sphere=candy['kind'] in ('sphere','faceted'),hull=candy.get('vertices'))['exposed_fraction']
            for n in (9,17,33,65)}
        local=Rotation.from_quat(last_pose[3:]).inv().apply(wrapper-last_pose[:3])
        depth=(np.array(candy['size'])/2-np.abs(local)).min(axis=1)
        if candy['kind']=='sphere':depth=candy['radius']-np.linalg.norm(local,axis=1)
        if candy['kind']=='faceted':
            from scipy.spatial import ConvexHull
            planes=ConvexHull(candy['vertices']).equations
            depth=-np.max(local@planes[:,:3].T+planes[:,3],axis=1)
        result['max_final_wrapper_vertex_inside_core_depth_m']=max(0.,float(depth.max()))
        result['fraction_of_final_wrapper_vertices_inside_core_over_half_mm']=float((depth>.0005).mean())
        def supported(row):
            pose=row['rigid_objects']['candy']
            if candy['kind']=='sphere':
                center=np.asarray(pose[:3])-tile['center']
                return bool(np.all(np.abs(center[:2])+candy['radius']<=np.array(tile['size'][:2])/2)
                            and abs(center[2]-candy['radius']-tile['size'][2]/2)<=.002)
            corners=np.array(list(product([-1,1], repeat=3)))*np.array(candy['size'])/2
            if candy['kind']=='faceted':corners=np.array(candy['vertices'])
            vertices=Rotation.from_quat(pose[3:]).apply(corners)+pose[:3]-tile['center']
            return (np.abs(vertices[:,:2])<=np.array(tile['size'][:2])/2).all() and abs(vertices[:,2].min()-tile['size'][2]/2)<=.002
        result['core_on_'+surface_name+'_through_last_two_seconds']=len(rows)>=41 and all(supported(r) for r in rows[-41:])
        if 'tail_winding_deg' in rows[0]:
            result.update(winding_history(rows))
            def disjoint(row):
                lower,upper=np.asarray(row['surfaces']['wrapper']['bounds_m'])
                pose=row['rigid_objects']['candy']
                if candy['kind']=='sphere':half=np.full(3,candy['radius'])
                elif candy['kind']=='faceted':
                    half=np.abs(Rotation.from_quat(pose[3:]).apply(candy['vertices'])).max(axis=0)
                else:
                    half=np.abs(Rotation.from_quat(pose[3:]).as_matrix())@(np.asarray(candy['size'])/2)
                center=np.asarray(pose[:3])
                return max((lower-center-half).max(),(center-half-upper).max())>=.002
            result['full_exit_final_hold']=len(rows)>=41 and all(
                r['wrapper_coverage']['exposed_fraction']>=1.-1e-9 and disjoint(r) and
                r['wrapper_coverage']['separation_lower_bound_m']>=.002 for r in rows[-41:])
    elif task=='pallet':
        cartons=[b for b in bodies if b.get('mass')]
        from scipy.spatial.transform import Rotation
        def in_load_frame(row,pose):
            if 'pallet_pose' not in row:return pose
            pp=row['pallet_pose']
            inv=Rotation.from_quat(pp[3:]).inv()
            pos=inv.apply(np.asarray(pose[:3])-pp[:3])+[.95,0.,details.get('deck_top_m',.5)]
            return np.r_[pos,(inv*Rotation.from_quat(pose[3:])).as_quat()]
        max_shift=max(np.linalg.norm(np.array(in_load_frame(r,r['rigid_objects'][b['name']])[:3])-b['center'])
                      for r in rows for b in cartons)
        result['whole_trial_max_carton_center_displacement_m']=float(max_shift)
        result['whole_trial_max_carton_corner_displacement_m']=max(
            largest_corner_excursion([in_load_frame(r,r['rigid_objects'][b['name']]) for r in rows], b['size'])
            for b in cartons)
        # Sum original mass by triangle rest area. Count only triangles whose
        # entire bounding boxes clear the load's expanded envelope. This is a
        # conservative removal indicator, not mass of all film resting in a bin.
        film=surfaces[0]
        initial=np.array([b['center'] for b in cartons]); half=np.array(cartons[0]['size'])/2
        lo=initial.min(0)-half-.025;hi=initial.max(0)+half+.025
        film_positions=vertices['film']
        if 'pallet_pose' in rows[-1]:
            pp=rows[-1]['pallet_pose']
            film_positions=Rotation.from_quat(pp[3:]).inv().apply(film_positions-pp[:3])+[.95,0.,details.get('deck_top_m',.5)]
            result['deck_rotation_deg']=float(np.rad2deg(np.unwrap([r['deck_angle_rad'] for r in rows])[-1]-rows[0]['deck_angle_rad']))
            result['displacement_frame']='wooden pallet'
        # The manufactured horizontal lines retain material identity. If they
        # still go all the way around the pallet, a deck turn did not unwrap it.
        if 'pallet_pose' in rows[-1]:
            horizontal=film_positions.reshape(-1,25,3)
            phase=np.unwrap(np.arctan2(horizontal[:,:,1],horizontal[:,:,0]-.95),axis=0)
            net=np.abs(np.rad2deg(phase[-1]-phase[0]))
            result['remaining_film_winding_deg_range']=[float(net.min()),float(net.max())]
        tri=film_positions[film.faces]
        clear=np.any((tri.min(1)>hi)|(tri.max(1)<lo),axis=1)
        reference=film.rest_points if getattr(film,'rest_points',None) is not None else film.points
        a,b,c=reference[film.faces].transpose(1,0,2)
        areas=np.linalg.norm(np.cross(b-a,c-a),axis=1)
        result['fraction_of_film_wholly_outside_load_envelope']=float(np.average(clear,weights=areas))
    elif task=='stocking':
        stocking=surfaces[0]; p,f=capped_mesh(vertices['stocking'],stocking.faces)
        foot=next(b for b in bodies if b['name']=='working_foot')
        heel=next(b for b in bodies if b['name']=='working_heel')
        centers=np.array([foot['center'],heel['center']])
        centers[0]+=np.asarray(details.get('oriented_toe_direction',[0,0,1]))*(foot['height']/2+.4*foot['radius'])
        values=winding_number(p,f,centers)
        result.update(toe_landmark_winding=float(values[0]),heel_landmark_winding=float(values[1]),
                      toe_inside=bool(.75<=values[0]<=1.25),heel_inside=bool(.75<=values[1]<=1.25))
        if summary['variant'] in ('wire-donner','open-stocking-frame'):
            flags=[r['frame_loading']['loaded'] for r in rows]
            run_length=maximum=0
            for flag in flags:
                run_length=run_length+1 if flag else 0
                maximum=max(maximum,run_length)
            result['frame_loading']=dict(loaded_at_reset=flags[0],
                longest_loaded_hold_s=max(0,maximum-1)/20.,loaded_samples=sum(flags))
    elif task=='hospital-corner':
        if summary.get('variant')=='bedside-paddle':
            from bed_paddle import measure
            result['paddle']=measure(rows)
        final=vertices['flat_sheet'].reshape(*details['mesh_shape'],3)
        near=np.asarray(details['mattress_near_xy_m'])
        mattress=next(body for body in bodies if body['name']=='mattress')
        bottom=mattress['center'][2]-mattress['size'][2]/2
        foam_path=run/f'mattress-{summary["actions"]:04d}.npy'
        shape=details.get('foam_grid_shape')
        edges=[('foot',final[0],0),('side',final[:,0],1)]
        if shape:
            from hem_measure import retained_length
            nx,ny,nz=shape
            underside=np.load(foam_path).reshape(nx,ny,nz,3)[:,:,0]
            lx,ly=mattress['size'][:2]
            local=np.stack(np.meshgrid(np.linspace(0,lx,nx),np.linspace(0,ly,ny),indexing='ij'),axis=-1)
            depths={name:retained_length(edge,underside,local,axis) for name,edge,axis in edges}
        else:
            # The old autogenerated tetrahedron mesh has no structured
            # surface indices. Preserve the original study's diagnostic.
            from scipy.interpolate import LinearNDInterpolator
            underside=None
            if foam_path.exists():
                foam=np.load(foam_path)
                foam=foam[foam[:,2]<np.median(foam[:,2])]
                underside=LinearNDInterpolator(foam[:,:2],foam[:,2],fill_value=np.nan)
            original=surfaces[0].rest_points.reshape(*details['mesh_shape'],3)+[.085,-.255,0.]
            depths={}
            for name,edge,axis in edges:
                along=1-axis
                ref=original[0] if axis==0 else original[:,0]
                band=(ref[:,along]>=near[along]+.015)&(ref[:,along]<=near[along]+.16)
                ceiling=np.full(len(edge),bottom) if underside is None else underside(edge[:,:2])
                inside=(edge[:,axis]>=near[axis]+.05)&(edge[:,2]>=.630)&(edge[:,2]<=ceiling+.002)&band
                length=longest=0.
                for good,width in zip(inside[:-1]&inside[1:],np.abs(np.diff(ref[:,along]))):
                    length=length+width if good else 0.
                    longest=max(longest,length)
                depths[name]=float(longest)
        result['continuous_tucked_hem_length_m']=depths
        result['first_tuck_threshold_reached']=max(depths.values())>=.08
    elif task=='duvet':
        insert,cover=surfaces
        p,f=capped_mesh(vertices['cover'],cover.faces)
        triangles=vertices['duvet'][insert.faces]
        sample=triangles.mean(1)
        values=winding_number(p,f,sample)
        rest=insert.rest_points if getattr(insert,'rest_points',None) is not None else insert.points
        a,b,c=rest[insert.faces].transpose(1,0,2)
        mass=np.linalg.norm(np.cross(b-a,c-a),axis=1)
        result.update(insert_triangle_centers_inside_fraction=float(np.average((values>=.75)&(values<=1.25),weights=mass)),
                      ambiguous_winding_fraction=float(np.average(((values>.25)&(values<.75))|(values>1.25),weights=mass)))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+')
    args=parser.parse_args()
    for folder in args.runs:
        result=evaluate(folder)
        (folder/'outcome-geometry.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result))
