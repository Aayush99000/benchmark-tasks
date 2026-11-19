"""Native rigid fruit, fixed tabletop dispenser and passive guarded slide cutter."""
import numpy as np
from film import EXIT,WIDTH,MELON,RADIUS

def build_assets():
    import trimesh
    import isaaclab.sim as sim
    from isaaclab.sim.utils import clone
    from isaaclab.sim.spawners.meshes.meshes import _spawn_mesh_geom_from_mesh
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg,PhysxCollisionPropertiesCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from pxr import UsdGeom,UsdPhysics,Gf
    from station_setup import scene_assets
    from yam_arena.scene import _box
    import omni.usd
    assets=scene_assets()
    def box(n,size,pos,color):assets.append(_box(n,size,tuple(map(float,pos)),color))
    box('dispenser_body',(.13,.46,.075),(.115,0,.7875),(.85,.87,.87))
    box('feed_lip',(.084,.412,.004),EXIT+[.038,0,-.003],(.65,.69,.72))
    box('guarded_cutter_rail',(.025,.45,.008),EXIT+[0,0,.005],(.19,.23,.27))
    for i,y in enumerate([-.218,.218]):box('table_clamp_'+str(i),(.04,.024,.04),(.13,y,.77),(.12,.14,.16))
    # Four small washable rests stabilize the round rind. Side lanes remain
    # open for the two grippers to tuck film beneath it; nothing fixes the fruit.
    for i,dx in enumerate([-.065,.065]):
        for j,dy in enumerate([-.020,.020]):
            box(f'fruit_rest_{i}_{j}',(.012,.012,.038),MELON+[dx,dy,-.071],(.25,.29,.25))
    @clone
    def roll_spawn(path,cfg,translation=None,orientation=None,**kwargs):
        prim=sim.spawn_cylinder(path,cfg,translation=translation,orientation=orientation,**kwargs)
        return prim
    assets.append(Object(name='film_roll',object_type=ObjectType.BASE,spawner_cfg=sim.CylinderCfg(radius=.035,height=WIDTH,axis='Y',visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.82,.91,.93),roughness=.2)),initial_pose=Pose(position_xyz=(.115,0,.810))))
    props=PhysxRigidBodyPropertiesCfg(sleep_threshold=0.,linear_damping=.4,angular_damping=.4,solver_position_iteration_count=32)
    contact=PhysxCollisionPropertiesCfg(contact_offset=.0005,rest_offset=0.)
    friction=sim.RigidBodyMaterialCfg(static_friction=.65,dynamic_friction=.5)
    # Lower hemisphere capped by the cut face; original procedural geometry.
    v=[[0,0,0]];faces=[];nr,nt=12,48
    for i in range(nr+1):
        a=(np.pi/2)*i/nr
        for j in range(nt):
            t=2*np.pi*j/nt;v.append([RADIUS*np.cos(a)*np.cos(t),RADIUS*np.cos(a)*np.sin(t),-RADIUS*np.sin(a)])
    for j in range(nt):faces.append([0,1+j,1+(j+1)%nt])
    for i in range(nr):
        for j in range(nt):
            a=1+i*nt+j;b=1+i*nt+(j+1)%nt;c=b+nt;d=a+nt;faces.extend([[a,d,c],[a,c,b]])
    mesh=trimesh.Trimesh(v,faces,process=True)
    @clone
    def fruit_spawn(path,cfg,translation=None,orientation=None,**kwargs):
        root=_spawn_mesh_geom_from_mesh(path,cfg,mesh,translation,orientation)
        # MeshCuboidCfg defaults to a bounding CUBE even for a custom mesh.
        # Override before PhysX initializes so the underside remains accessible.
        stage=omni.usd.get_context().get_stage()
        shape=stage.GetPrimAtPath(path+'/geometry/mesh')
        UsdPhysics.MeshCollisionAPI.Apply(shape).GetApproximationAttr().Set('convexHull')
        mass=UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(path))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(0,0,-3*RADIUS/8))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*[float(.8*RADIUS**2*k) for k in [.259375,.259375,.4]]))
        red=sim.CylinderCfg(radius=RADIUS-.006,height=.001,visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.82,.055,.08),roughness=.7))
        red.func(path+'/cut_face',red,translation=(0,0,.0006))
        for i,(x,y) in enumerate([(-.04,.025),(-.05,-.025),(.035,.045),(.05,-.03),(0,.045),(.01,-.04)]):
            seed=sim.SphereCfg(radius=.0023,visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.06,.025,.025),roughness=.7))
            seed.func(path+'/seed_'+str(i),seed,translation=(x,y,.0018))
        return root
    cfg=sim.MeshCuboidCfg(size=(.18,.18,.09),edge_refinement=1.,func=fruit_spawn,rigid_props=props,mass_props=sim.MassPropertiesCfg(mass=.8),collision_props=contact,physics_material=friction,visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.10,.32,.075),roughness=.6))
    assets.append(Object(name='watermelon',object_type=ObjectType.RIGID,spawner_cfg=cfg,initial_pose=Pose(position_xyz=tuple(map(float,MELON)))))
    @clone
    def slider_spawn(path,cfg,translation=None,orientation=None,**kwargs):
        root=sim.spawn_cuboid(path,cfg,translation=translation,orientation=orientation,**kwargs)
        stage=omni.usd.get_context().get_stage();joint=UsdPhysics.PrismaticJoint.Define(stage,path.rsplit('/',1)[0]+'/cutter_slide')
        joint.CreateBody1Rel().SetTargets([path]);joint.CreateAxisAttr('Y');joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,EXIT+[0,WIDTH/2+.015,.023])))
        joint.CreateLowerLimitAttr(-WIDTH-.030);joint.CreateUpperLimitAttr(0.);joint.CreateExcludeFromArticulationAttr(True)
        d=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear');d.CreateTypeAttr('force');d.CreateDampingAttr(.08);d.CreateStiffnessAttr(0.)
        return root
    cfg=sim.CuboidCfg(size=(.024,.024,.028),func=slider_spawn,rigid_props=props,mass_props=sim.MassPropertiesCfg(mass=.025),collision_props=contact,physics_material=sim.RigidBodyMaterialCfg(static_friction=1.,dynamic_friction=.8),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.96,.40,.055),roughness=.65))
    assets.append(Object(name='cutter',object_type=ObjectType.RIGID,spawner_cfg=cfg,initial_pose=Pose(position_xyz=tuple(map(float,EXIT+[0,WIDTH/2+.015,.023])))))
    return assets
