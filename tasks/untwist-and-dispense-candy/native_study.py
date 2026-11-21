"""Native YAM material trials with scripted, Dream-1 and idle control actions.

The original cameras supply the model's observations. The judging camera and
geometric score measurements are recorded separately. Only robot joint targets
are written after reset. PhysX drives the candy and wrapper.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import shutil
import sys
import time

HERE=Path(__file__).resolve().parent
GPU='GPU-6c438b6a-5915-e8df-e338-43d7a7d9aee3'


def spawn_surface(prim_path,cfg,translation=None,orientation=None,**kwargs):
    """Apply contact offsets AFTER UsdFileCfg creates its simulation mesh.

    The installed file spawner applies collision configuration before creating
    sim_mesh, leaving that new mesh with a 2 cm default rest offset. A local
    spawner fixes authoring order without changing the shared Isaac runtime.
    """
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    from pxr import PhysxSchema
    import omni.usd
    prim=spawn_from_usd(prim_path,cfg,translation=translation,orientation=orientation,**kwargs)
    stage=omni.usd.get_context().get_stage()
    # This study has exactly one environment; do not silently apply to only one
    # instance in a future vectorized runner.
    concrete=prim.GetPath().pathString
    mesh=stage.GetPrimAtPath(concrete+'/sim_mesh')
    if not mesh:raise RuntimeError('Missing surface simulation mesh: '+concrete)
    collision=PhysxSchema.PhysxCollisionAPI.Apply(mesh)
    props=cfg.collision_props[0]
    collision.CreateRestOffsetAttr(float(props.rest_offset))
    collision.CreateContactOffsetAttr(float(props.contact_offset))
    rest_file = Path(cfg.usd_path).with_suffix('.rest.npy')
    if rest_file.is_file():
        import numpy as np
        from pxr import Vt
        points = np.load(rest_file, allow_pickle=False)
        if len(points) != len(mesh.GetAttribute('points').Get()):
            raise ValueError(f'Rest shape vertex count mismatch for {concrete}')
        mesh.GetAttribute('omniphysics:restShapePoints').Set(Vt.Vec3fArray(points.tolist()))
    return prim


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task',choices=['duvet','hospital-corner','pallet','stocking','candy'])
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--steps',type=int)
    parser.add_argument('--motion', choices=['hold','pinch','open-control','unwrap','turn-and-peel','axial-roll','lift-unwind','extract-candy','clear-wrapper','dress','hook-cuff','fold-corner','lift-and-tuck','turn-load','unroll','retain-film','staged-unroll','paddle-tuck','dream'],default='hold')
    parser.add_argument('--state-only', action='store_true', help='Collect physical telemetry without camera rendering.')
    parser.add_argument('--variant',choices=['pleated','single-fold','front-edge','staged-cuff','tabletop-twist','flat-strip','turntable','relaxed-seam','round-sweet','prepared-stocking','foam-corner','loose-film','dropped-sheet','faceted-drop','printed-drop','wire-donner','open-stocking-frame','low-turntable','bedside-paddle'])
    parser.add_argument('--gpu-index', type=int, default=1, help='Physical GPU index. The local default is GPU1.')
    args=parser.parse_args()
    from scoring import LIMITS
    is_dream = args.motion == 'dream'
    if is_dream:
        if args.task != 'candy' or args.variant != 'printed-drop' or args.state_only:
            parser.error('Dream needs the printed candy task and live cameras')
        args.steps = LIMITS['candy'] if args.steps is None else args.steps
    elif args.steps is None:
        args.steps = 40
    if args.variant:
        from layout_variants import TASKS
        if TASKS[args.variant]!=args.task:
            parser.error('Variant is for '+TASKS[args.variant])
    scripted = args.motion not in ('hold', 'dream')
    if scripted:
        from grasp_probe import joint_plan, TARGETS, STEPS, BASELINE
        args.steps = STEPS
        if args.motion=='staged-unroll':
            if args.variant!='low-turntable':parser.error('Staged unroll requires the low manual turntable')
            from staged_pallet import build
            targets=build()
            args.steps=len(targets)-1
        elif args.motion in ('turn-load','unroll','retain-film'):
            if args.variant not in ('turntable','loose-film','low-turntable'):parser.error('turn-load needs the turntable')
            from pallet_plan import build,unroll,retain_film
            targets={'turn-load':build,'unroll':unroll,'retain-film':lambda:retain_film(upper=args.variant=='low-turntable')}[args.motion]()
            args.steps=len(targets)-1
        elif args.motion=='paddle-tuck':
            if args.variant!='bedside-paddle':parser.error('Paddle tucking needs the loose bedside paddle')
            from paddle_plan import build
            targets=build()
            args.steps=len(targets)-1
        elif args.motion in ('fold-corner','lift-and-tuck'):
            if args.task!='hospital-corner':parser.error('fold-corner needs the flat sheet')
            from corner_plan import build,lift_and_tuck
            targets=lift_and_tuck() if args.motion=='lift-and-tuck' else build()
            args.steps=len(targets)-1
        elif args.motion=='hook-cuff':
            if args.variant!='wire-donner':parser.error('Cuff hooking requires the empty wire stirrup and gathered garment')
            from hook_cuff import build
            targets=build()
            args.steps=len(targets)-1
        elif args.motion == 'dress':
            if args.task != 'stocking':
                parser.error('The dressing controller is for the stocking')
            from dressing_plan import build, staged, gathered, with_frame, extended_frame
            targets={'staged-cuff': staged,'prepared-stocking':gathered,'wire-donner':with_frame,'open-stocking-frame':extended_frame}.get(args.variant,build)()
            args.steps=len(targets)-1
        elif args.motion in ('unwrap', 'turn-and-peel','axial-roll','lift-unwind','extract-candy','clear-wrapper'):
            if args.task != 'candy':
                parser.error('The unwrap controller is for candy')
            from unwrap_plan import build, turn_and_peel, axial_roll, lift_unwind
            targets = {'turn-and-peel':turn_and_peel,'axial-roll':axial_roll,'lift-unwind':lift_unwind,'extract-candy':lambda:lift_unwind(extract=True),'clear-wrapper':lambda:lift_unwind(clear=True),'unwrap':build}[args.motion]()
            args.steps = len(targets)-1
        else:
            targets = joint_plan(args.task, args.motion == 'pinch', args.variant)
        from grasp_probe import target_for
        target = target_for(args.task,args.variant)
    if not 1 <= args.steps <= LIMITS[args.task]:
        parser.error(f'{args.task} accepts 1 to {LIMITS[args.task]} actions')
    out=args.output.resolve()
    if out.exists():raise RuntimeError('Output exists; preserve previous evidence')
    gpu=subprocess.run(['nvidia-smi','-i',str(args.gpu_index),'--query-gpu=uuid,memory.used,memory.total,utilization.gpu',
                        '--format=csv,noheader,nounits'],check=True,capture_output=True,text=True).stdout.strip().split(', ')
    selected_gpu = gpu[0]
    if args.gpu_index == 1 and Path('/media/nvme/xavier/robocurve').is_dir() and selected_gpu != GPU:
        raise RuntimeError('Physical GPU1 UUID changed')
    # Utilization may still include the previous trial immediately after exit.
    for _ in range(5):
        if float(gpu[2])-float(gpu[1])>=10000 and float(gpu[3])<=20:
            break
        time.sleep(2.)
        gpu=subprocess.check_output(['nvidia-smi','-i',selected_gpu,'--query-gpu=uuid,memory.used,memory.total,utilization.gpu',
                                     '--format=csv,noheader,nounits'],text=True).strip().split(', ')
    if float(gpu[2])-float(gpu[1])<10000 or float(gpu[3])>20:
        raise RuntimeError(f'GPU{args.gpu_index} is busy; no workload was started')
    os.environ['CUDA_VISIBLE_DEVICES']=selected_gpu
    sys.argv=[sys.argv[0],'--/renderer/multiGpu/activeCudaGpus=0,']
    out.mkdir(parents=True)
    runtime_names = ['native_study.py', 'geometry.py', 'review_views.py','scoring.py']
    runtime_names.append('reach_audit.py')
    if scripted:
        runtime_names.append('grasp_probe.py')
    if is_dream:
        runtime_names.extend(('candy_dream.py', 'candy_submission.py'))
    if args.variant in ('turntable','loose-film','low-turntable'):runtime_names.append('turntable.py')
    if args.motion=='staged-unroll':runtime_names.append('staged_pallet.py')
    if args.motion in ('turn-load','unroll','retain-film','staged-unroll'):runtime_names.append('pallet_plan.py')
    if args.motion in ('fold-corner','lift-and-tuck'):runtime_names.append('corner_plan.py')
    if args.motion=='hook-cuff':runtime_names.append('hook_cuff.py')
    if args.motion in ('dress','hook-cuff'):
        runtime_names.append('dressing_plan.py')
    if args.variant in ('wire-donner','open-stocking-frame'):runtime_names.append('donning_frame.py')
    if args.variant in ('faceted-drop','printed-drop'):runtime_names.append('faceted_sweet.py')
    if args.task == 'candy':
        runtime_names.append('candy_measure.py')
    if args.motion in ('unwrap','turn-and-peel','axial-roll','lift-unwind','extract-candy','clear-wrapper'):
        runtime_names.append('unwrap_plan.py')
    if args.motion=='paddle-tuck':runtime_names.append('paddle_plan.py')
    if args.variant=='bedside-paddle':runtime_names.append('bed_paddle.py')
    if args.variant in ('dropped-sheet','bedside-paddle'):runtime_names.append('draped_sheet.npy')
    if args.variant in ('dropped-sheet','bedside-paddle','loose-film','low-turntable'):runtime_names.append('handling_setups.py')
    if args.variant:
        runtime_names += ['layout_variants.py','pleated_wrapper.py']
    if args.variant=='printed-drop':runtime_names.append('candy_finish.py')
    sources={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in runtime_names}
    (out/'source').mkdir()
    for name in sources:shutil.copy2(HERE/name,out/'source'/name)
    shutil.copy2(HERE/'LICENSE.pram-reference',out/'source/LICENSE.pram-reference')
    report=dict(task=args.task,status='starting',evidence='native_material_study_only',
                full_task_demonstrated=False,task_score=None,dream_attempted=False,gpu_uuid=selected_gpu,
                source_sha256=sources,variant=args.variant,motion=args.motion,recording=not args.state_only,control_hz=20,physics_hz=240,actions=0)
    def save():(out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    save();app=env=None;writers={};dream=None;start=time.monotonic()
    try:
        from isaaclab.app import AppLauncher
        app=AppLauncher(headless=True,device='cuda:0',enable_cameras=not args.state_only,multi_gpu=False).app
        import numpy as np
        import torch
        import carb
        import omni.usd
        import imageio.v2 as imageio
        from pxr import Usd,UsdGeom,UsdPhysics,Sdf,Vt
        import isaaclab.sim as sim
        from isaaclab.assets import DeformableObjectCfg
        from isaaclab_physx.sim.schemas import PhysxDeformableBodyPropertiesCfg,PhysxCollisionCfg
        from isaaclab_physx.sim.spawners.materials import PhysxSurfaceDeformableBodyMaterialCfg
        from isaaclab_arena.assets.object import Object,ObjectType
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.utils.pose import Pose
        from yam_arena import station
        from yam_arena.constants import READY
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import env_cfg_callback,task_room,_box
        import geometry
        import review_views

        if args.variant:
            from layout_variants import BUILDERS
            bodies,surfaces,details=BUILDERS[args.variant]()
        else:
            bodies,surfaces,details=geometry.BUILDERS[args.task]()
        report['geometry']={s.name:geometry.audit(s) for s in surfaces}
        for name,audit in report['geometry'].items():
            if audit['nonmanifold_edges'] or audit['min_triangle_area_m2']<=1e-10:
                raise RuntimeError('Invalid material topology: '+name)
        report['fidelity']=details
        assets=out/'assets';assets.mkdir()
        configs={}
        for s in surfaces:
            path=assets/(s.name+'.usda');stage=Usd.Stage.CreateNew(str(path))
            root=UsdGeom.Xform.Define(stage,'/Asset').GetPrim();stage.SetDefaultPrim(root)
            UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
            mesh=UsdGeom.Mesh.Define(stage,'/Asset/Mesh')
            mesh.CreatePointsAttr(Vt.Vec3fArray(s.points.astype(np.float32).tolist()))
            mesh.CreateFaceVertexCountsAttr([3]*len(s.faces))
            mesh.CreateFaceVertexIndicesAttr(s.faces.ravel().tolist())
            mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
            if args.variant=='printed-drop' and s.name=='wrapper':
                from candy_finish import decorate_wrapper
                decorate_wrapper(mesh,path)
            stage.GetRootLayer().Save()
            if s.rest_points is not None:
                np.save(path.with_suffix('.rest.npy'),s.rest_points.astype(np.float32))
            configs[s.name]=DeformableObjectCfg(prim_path='{ENV_REGEX_NS}/'+s.name,
                spawn=sim.UsdFileCfg(usd_path=str(path),
                    func=spawn_surface,
                    deformable_props=PhysxDeformableBodyPropertiesCfg(
                        self_collision=True,solver_position_iteration_count=32),
                    collision_props=[PhysxCollisionCfg(rest_offset=s.contact/2,contact_offset=s.contact)],
                    physics_material=PhysxSurfaceDeformableBodyMaterialCfg(
                        density=s.density,surface_thickness=s.thickness,youngs_modulus=s.young,
                        poissons_ratio=.25,surface_bend_stiffness=s.bend,
                        elasticity_damping=.05,bend_damping=.05,
                        static_friction=s.friction,dynamic_friction=s.friction*.8),
                    visual_material=None if args.variant=='printed-drop' else sim.PreviewSurfaceCfg(diffuse_color=s.color,roughness=.7)))
        if args.task in ('pallet','duvet','hospital-corner'):
            station.TABLE_SIZE=(.14,.90,.06)
            station.TABLE_CENTER=(0.,0.,station.TABLE_TOP-.03)
        rig=station.rig()
        room=task_room('station',rig.top_cam)
        if args.task in ('pallet','duvet','hospital-corner'):room.append(_box('arm_mount_beam',(.14,.90,.06),(0.,0.,.690),(.20,.22,.25)))
        apparatus=[]
        if args.variant in ('turntable','loose-film','low-turntable'):
            from turntable import assets as make_turntable
            planks=[b for b in bodies if b['name'].startswith('pallet_')]
            deck_top=details.get('deck_top_m',.5)
            deck_path,pallet_path=make_turntable(assets,planks,deck_top)
            deck_cfg=sim.UsdFileCfg(usd_path=deck_path)
            apparatus.extend([
                Object(name='turntable',object_type=ObjectType.ARTICULATION,spawner_cfg=deck_cfg,
                       initial_pose=Pose(position_xyz=(.95,0.,deck_top-.012))),
                Object(name='wooden_pallet',object_type=ObjectType.RIGID,usd_path=pallet_path,
                       initial_pose=Pose(position_xyz=(.95,0.,deck_top)))])
        for b in bodies:
            if args.variant in ('turntable','loose-film','low-turntable') and b['name'].startswith('pallet_'):
                continue
            if args.variant in ('foam-corner','dropped-sheet','bedside-paddle') and b['name']=='mattress':
                from isaaclab_physx.sim.spawners.materials import PhysxDeformableBodyMaterialCfg
                material=PhysxDeformableBodyMaterialCfg(density=35.,youngs_modulus=35000.,
                    poissons_ratio=.27,static_friction=.6,dynamic_friction=.5,elasticity_damping=.08)
                options=dict(deformable_props=PhysxDeformableBodyPropertiesCfg(solver_position_iteration_count=32),
                    physics_material=material,visual_material=sim.PreviewSurfaceCfg(diffuse_color=b['color'],roughness=.85))
                if args.variant in ('dropped-sheet','bedside-paddle'):
                    from handling_setups import foam_asset
                    foam=sim.UsdFileCfg(usd_path=foam_asset(assets,b['size']),func=spawn_surface,
                        collision_props=[PhysxCollisionCfg(rest_offset=.0005,contact_offset=.0015)],**options)
                else:
                    foam=sim.MeshCuboidCfg(size=tuple(b['size']),**options)
                configs['mattress']=DeformableObjectCfg(prim_path='{ENV_REGEX_NS}/mattress',spawn=foam,
                    init_state=DeformableObjectCfg.InitialStateCfg(pos=tuple(b['center'])))
                continue
            if b['kind'] in ('faceted','wire_frame','paddle'):
                if b['kind']=='faceted':
                    from faceted_sweet import usd_asset
                elif b['kind']=='paddle':
                    from bed_paddle import usd_asset
                else:
                    from donning_frame import usd_asset
                asset_path=usd_asset(assets,b)
                if args.variant=='printed-drop' and b['kind']=='faceted':
                    from candy_finish import glaze_sweet
                    glaze_sweet(asset_path)
                apparatus.append(Object(name=b['name'],object_type=ObjectType.RIGID,usd_path=asset_path,
                    initial_pose=Pose(position_xyz=tuple(float(v) for v in b['center']))))
                continue
            common=dict(collision_props=sim.CollisionPropertiesCfg(contact_offset=.001,rest_offset=0.),
                        visual_material=sim.PreviewSurfaceCfg(diffuse_color=b['color'],roughness=.65))
            dynamic=b.get('mass') is not None
            if dynamic:
                common.update(rigid_props=sim.RigidBodyPropertiesCfg(),
                              mass_props=sim.MassPropertiesCfg(mass=b['mass']),
                              physics_material=sim.RigidBodyMaterialCfg(static_friction=.5,dynamic_friction=.4))
            if b['kind']=='box':spawn=sim.CuboidCfg(size=tuple(b['size']),**common)
            elif b['kind']=='capsule':spawn=sim.CapsuleCfg(radius=b['radius'],height=b['height'],axis=b['axis'],**common)
            else:spawn=sim.SphereCfg(radius=b['radius'],**common)
            apparatus.append(Object(name=b['name'],object_type=ObjectType.RIGID if dynamic else ObjectType.BASE,
                spawner_cfg=spawn,initial_pose=Pose(position_xyz=tuple(float(v) for v in b['center']))))
        robot=BimanualYamEmbodiment(enable_cameras=not args.state_only,rig=rig,contact_sensors=True,robot_contact_offset=.001)
        review_views.CONFIGS=[('judging_front',(-.9,-1.7,1.65),
                              (.6,0.,.65) if args.task=='candy' else (.95,.15,.55),
                              (0.,0.,1.),18.,(640,480))]
        if args.variant=='printed-drop':
            review_views.CONFIGS=[('judging_front',(.74,-.46,1.17),(.366,.067,.797),
                                  (0.,0.,1.),34.,(640,480))]
        if not args.state_only:review_views.configure(robot)
        def configure(cfg):
            cfg=env_cfg_callback(20)(cfg)
            cfg.scene.replicate_physics=False
            for key,value in configs.items():setattr(cfg.scene,key,value)
            return cfg
        arena=IsaacLabArenaEnvironment(name='material-study-'+args.task,
            scene=Scene([*room,*apparatus]),embodiment=robot,env_cfg_callback=configure)
        env=ArenaEnvBuilder(arena,ArenaEnvBuilderCfg(num_envs=1,solve_relations=False,device='cuda:0')).make_registered()
        base=env.unwrapped
        settings=carb.settings.get_settings()
        for key,value in {'/rtx/rendermode':'PathTracing','/rtx/pathtracing/spp':32,
                          '/rtx/pathtracing/totalSpp':32,'/rtx/pathtracing/optixDenoiser/enabled':True,
                          '/rtx-transient/post/aa/limitedOps':False,'/rtx/post/aa/op':2}.items():settings.set(key,value)
        stage=omni.usd.get_context().get_stage()
        report['authored_surface_contacts']={}
        for s in surfaces:
            prim=stage.GetPrimAtPath('/World/envs/env_0/'+s.name+'/sim_mesh')
            report['authored_surface_contacts'][s.name]={key:prim.GetAttribute(key).Get()
                for key in ('physxCollision:restOffset','physxCollision:contactOffset')}
        if args.variant in ('faceted-drop','printed-drop'):
            collider=stage.GetPrimAtPath('/World/envs/env_0/candy/hard_drop')
            if not collider or not collider.IsA(UsdGeom.Mesh) or not collider.HasAPI(UsdPhysics.CollisionAPI):
                raise RuntimeError('Missing physical hull for the hard sweet')
        names=[] if args.state_only else ['top_cam','left_cam','right_cam','judging_front']
        for name in names:
            prim=stage.GetPrimAtPath(base.scene[name]._render_data.render_product.path)
            for key,value,kind in [('omni:rtx:rendermode','PathTracing',Sdf.ValueTypeNames.Token),
                                   ('omni:rtx:pt:samplesPerIteration',32,Sdf.ValueTypeNames.Int),
                                   ('omni:rtx:pt:samplesPerPixel',32,Sdf.ValueTypeNames.Int)]:
                attr=prim.GetAttribute(key)
                if not attr:attr=prim.CreateAttribute(key,kind,custom=False)
                attr.Set(value)
        ready=torch.tensor([[*READY,1.,*READY,1.]],device=base.device,dtype=torch.float32)
        env.reset()
        def array(v):return getattr(v,'torch',v).detach().cpu().numpy()
        if args.task == 'candy':
            from candy_measure import indicators, twist_degrees
            candy_body = next(b for b in bodies if b['name']=='candy')
            candy_size=candy_body['size']
            wrapper_faces = next(s.faces for s in surfaces if s.name=='wrapper')
        if args.variant in ('faceted-drop','printed-drop') and args.motion!='hold':
            # Record every physics substep for offline food-contact auditing.
            # Neither the policy nor the actuators receive these observations.
            links={side:base.scene[side+'_robot'] for side in ('left','right')}
            link_names={side:list(obj.body_names) for side,obj in links.items()}
            history=[]
            def sample_contacts():
                return np.vstack([array(base.scene['candy'].data.root_pose_w)[0],
                    *(array(obj.data.body_link_pose_w)[0] for obj in links.values())]).copy()
            history.append(sample_contacts())
            advance_scene=base.scene.update
            def update_and_observe(dt):
                advance_scene(dt)
                history.append(sample_contacts())
            base.scene.update=update_and_observe
        initial={s.name:array(base.scene[s.name].data.nodal_pos_w)[0].copy() for s in surfaces}
        previous = {name: p.copy() for name, p in initial.items()}
        from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
        from reach_audit import fk
        grippers = {}
        for side in ('left','right'):
            arm = base.scene[side+'_robot']
            grippers[side] = (arm,arm.find_joints(ARM_JOINTS,preserve_order=True)[0],
                              arm.find_joints(FINGER_JOINTS,preserve_order=True)[0])
        if scripted:
            arm,joint_ids,finger_ids=grippers[target.arm]
            commands = torch.as_tensor(targets, device=base.device,dtype=torch.float32)
            patch = None
        if is_dream:
            from candy_dream import CandyDream

            dream = CandyDream(base, grippers)
            report.update(dream_attempted=True, status='connecting_dream')
            save()
            dream.start()
        report['status']='running_' + args.motion
        save()
        video_stats={name:dict(frames=0,low_contrast_frames=0) for name in names}
        with (out/'trace.jsonl').open('w') as trace:
            for step in range(args.steps+1):
                if step:
                    if dream is not None:
                        action = torch.as_tensor(dream.action(step - 1), device=base.device)[None, :]
                    else:
                        action = commands[step:step+1] if scripted else ready
                    env.step(action)
                else:
                    if names:
                        for _ in range(4):base.sim.render()
                frame_state=dict(step=step,time_s=step/20.,surfaces={},rigid_objects={})
                frame_state['grippers']={}
                for side,(robot_arm,arm_ids,jaw_ids) in grippers.items():
                    encoders=array(robot_arm.data.joint_pos)[0]
                    frame_state['grippers'][side]=dict(tcp_m=fk(encoders[arm_ids],station.BASES[side])[0].tolist(),
                                                      finger_joints=encoders[jaw_ids].tolist())
                for s in surfaces:
                    p=array(base.scene[s.name].data.nodal_pos_w)[0]
                    if not np.isfinite(p).all():raise RuntimeError('Nonfinite material: '+s.name)
                    frame_state['surfaces'][s.name]=dict(bounds_m=[p.min(0).tolist(),p.max(0).tolist()],
                        max_vertex_displacement_m=float(np.linalg.norm(p-initial[s.name],axis=1).max()),
                        max_vertex_speed_m_s=float(np.linalg.norm(array(base.scene[s.name].data.nodal_vel_w)[0],axis=1).max()))
                    measured_speed = 20*np.linalg.norm(p-previous[s.name], axis=1)
                    frame_state['surfaces'][s.name]['finite_difference_speed_m_s'] = {
                        'max':float(measured_speed.max()), 'p99':float(np.percentile(measured_speed,99))}
                    previous[s.name] = p.copy()
                    if scripted and s.name == target.surface and step >= BASELINE:
                        if patch is None:
                            # Read-only instrumentation. The controller follows fixed joint commands.
                            patch = np.argsort(np.linalg.norm(p-target.point,axis=1))[:12]
                            report['tracked_vertex_ids'] = patch.tolist()
                        frame_state['tracked_patch_m'] = np.median(p[patch],axis=0).tolist()
                        q = array(arm.data.joint_pos)[0]
                        tcp = fk(q[joint_ids], station.BASES[target.arm])[0]
                        frame_state['measured_tcp_m'] = tcp.tolist()
                        frame_state['measured_finger_joints'] = q[finger_ids].tolist()
                    if step in (0,100,args.steps) or (args.task in ('hospital-corner','pallet','stocking') and step%25==0):np.save(out/f'{s.name}-{step:04d}.npy',p)
                if args.motion in ('dress','hook-cuff'):
                    points = previous['stocking']
                    # Mesh rings retain garment vertex identity throughout the trial.
                    ring_size=details.get('mesh_shape',[51,32])[1]
                    heel_ring=details.get('heel_ring',26)
                    frame_state['stocking_landmarks_m'] = {
                        'toe':points[-1].tolist(), 'heel_patch':points[heel_ring*ring_size:(heel_ring+1)*ring_size].mean(axis=0).tolist(),
                        'cuff_center':points[-ring_size-1:-1].mean(axis=0).tolist()}
                for b in bodies:
                    if b.get('mass') is not None:
                        frame_state['rigid_objects'][b['name']]=array(base.scene[b['name']].data.root_pose_w)[0].tolist()
                if args.variant in ('wire-donner','open-stocking-frame'):
                    from donning_frame import loading_observations
                    frame_state['frame_loading']=loading_observations(previous['stocking'],
                        frame_state['rigid_objects']['donning_frame'],details['mesh_shape'][1])
                if args.variant in ('foam-corner','dropped-sheet','bedside-paddle'):
                    foam_nodes=array(base.scene['mattress'].data.nodal_pos_w)[0]
                    frame_state['mattress_bounds_m']=[foam_nodes.min(0).tolist(),foam_nodes.max(0).tolist()]
                    if step in (0,100,args.steps):np.save(out/f'mattress-{step:04d}.npy',foam_nodes)
                if args.variant in ('turntable','loose-film','low-turntable'):
                    frame_state['deck_angle_rad']=float(array(base.scene['turntable'].data.joint_pos)[0,0])
                    frame_state['pallet_pose']=array(base.scene['wooden_pallet'].data.root_pose_w)[0].tolist()
                if args.task == 'candy':
                    candy_pose = frame_state['rigid_objects']['candy']
                    if candy_pose[2]<station.FLOOR_Z-.015:
                        raise RuntimeError('Candy crossed through the physical floor; invalid collision setup')
                    frame_state['wrapper_coverage'] = indicators(
                        previous['wrapper'], wrapper_faces, candy_pose, size=candy_size,sphere=candy_body['kind'] in ('sphere','faceted'),hull=candy_body.get('vertices'))
                    shape=(65,len(previous['wrapper'])//65)
                    frame_state['tail_winding_deg']=twist_degrees(previous['wrapper'],shape)
                    frame_state['paper_landmarks_m']=previous['wrapper'].reshape(*shape,3).mean(axis=1)[[5,32,59]].tolist()
                trace.write(json.dumps(frame_state)+'\n');trace.flush()
                for name in names:
                    camera=base.scene[name];camera.update(0.,force_recompute=True)
                    pixels=np.ascontiguousarray(array(camera.data.output['rgb'])[0,:,:,:3],dtype=np.uint8)
                    video_stats[name]['frames']+=1
                    video_stats[name]['low_contrast_frames']+=int(pixels.std()<3.)
                    if name not in writers:
                        writers[name]=imageio.get_writer(out/(name+'.mp4'),fps=20,codec='libx264',
                            pixelformat='yuv420p',macro_block_size=1,ffmpeg_params=['-movflags','+faststart'])
                        imageio.imwrite(out/(name+'-initial.png'),pixels)
                    writers[name].append_data(pixels)
                    if step==args.steps:imageio.imwrite(out/(name+'-final.png'),pixels)
                report['actions']=step
                if step%20==0:
                    if dream is not None:
                        report['dream_requests'] = len(dream.transcript()['requests'])
                    save()
                    print(json.dumps(dict(task=args.task,step=step)),flush=True)
        if args.variant in ('faceted-drop','printed-drop') and args.motion!='hold':
            base.scene.update=advance_scene
            np.savez_compressed(out/'contact-history.npz',poses=np.stack(history),
                                left=link_names['left'],right=link_names['right'])
            report['contact_history_samples']=len(history)
            if len(history)!=1+args.steps*12:raise RuntimeError('Incomplete 240 Hz contact evidence')
        report.update(status='completed_native_' + args.motion,final_state=frame_state,
                      frames_per_camera=args.steps+1 if names else 0,simulation_seconds=args.steps/20.,video_stats=video_stats)
    except BaseException as error:
        import traceback
        report.update(status='failed',error_type=type(error).__name__,error=str(error),traceback=traceback.format_exc())
        raise
    finally:
        for writer in writers.values():writer.close()
        if dream is not None:
            transcript = dream.transcript()
            report['dream_requests'] = len(transcript['requests'])
            (out/'dream-transcript.json').write_text(json.dumps(transcript, indent=2)+'\n')
        report['elapsed_wall_s']=time.monotonic()-start
        report['source_unchanged']=all((HERE/name).exists() and hashlib.sha256((HERE/name).read_bytes()).hexdigest()==value
                                       for name,value in sources.items())
        save()
        if dream is not None:dream.close()
        if env is not None:env.close()
        if app is not None:app.close()


if __name__=='__main__':main()
