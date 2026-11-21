"""Physical bracket plus keyed snap-latch approximation. Never writes camera body poses.
Latch models a reusable clip, not arbitrary attachment: tight seating and low speed required.
"""
import numpy as np
BRACKET_AUDIT=[]
def install():
 import yam_arena.embodiment as emb
 import isaaclab.sim as sim
 from isaaclab.sim.utils import clone
 from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
 from yam_arena import station
 from yam_arena.demo import grip_rotation
 from yam_arena.robot import asset_link_paths
 import scene as spec
 original=emb.yam_arm_cfg;R=np.array(spec.LAYOUT['mount_rotation_in_gripper']).T;center=np.array(spec.LAYOUT['mount_position_in_gripper'])
 @clone
 def spawn(path,cfg,*args,**kwargs):
  from yam_arena.robot import spawn_in_dream_look
  prim=spawn_in_dream_look.__wrapped__(path,cfg,*args,**kwargs)
  parent=path+asset_link_paths()['gripper']
  from latch_geometry import MOUNT,TOOTH_RETRACTED,TOOTH_SIZE
  from yam_arena import ik
  from scipy.spatial.transform import Rotation
  for name,delta,size in MOUNT:
   if name=='channel_right':continue
   c=sim.CuboidCfg(size=size,collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0002,rest_offset=0),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.18,.32,.48),metallic=.6))
   c.func(parent+'/recovery_'+name,c,translation=tuple(center+R.T@np.array(delta)),orientation=tuple(ik.matrix_to_quat_xyzw(R.T)))
  # Split the right rail around the travel slot; no decorative switched tooth.
  for name,delta,size in [('right_upper',(.0122,-.025,-.002),(.004,.008,.008)),('right_lower',(.0122,-.025,-.019),(.004,.008,.014))]:
   c=sim.CuboidCfg(size=size,collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0001,rest_offset=0),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.18,.32,.48),metallic=.6))
   c.func(parent+'/recovery_'+name,c,translation=tuple(center+R.T@np.array(delta)),orientation=tuple(ik.matrix_to_quat_xyzw(R.T)))
  from quick_release import add_receiver_details
  add_receiver_details(prim.GetStage(),parent,center,R.T)
  add_spring_plunger(prim.GetStage(),path,parent,center,R.T)
  for side in (-1,1):
   start=np.array([side*.017,-.020,.025]);end=center+R.T@np.array([side*.015,-.030,-.025])
   delta=end-start;length=np.linalg.norm(delta);z=delta/length;x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
   c=sim.CuboidCfg(size=(.007,.007,float(length+.007)),collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0002,rest_offset=0),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.32,.36,.40),metallic=.6))
   c.func(parent+'/recovery_strut_'+str(side).replace('-','m'),c,translation=tuple((start+end)/2),orientation=tuple(Rotation.from_matrix(np.column_stack([x,y,z])).as_quat()))
  from camera_tether import maybe_build
  maybe_build(prim.GetStage())
  return prim
 def configured(*args,**kwargs):
  cfg=original(*args,**kwargs)
  if 'LeftArm' in cfg.prim_path:cfg.spawn.func=spawn;cfg.spawn.make_uninstanceable=True
  return cfg
 emb.yam_arm_cfg=configured
 # Spawn compound camera collision geometry before physics initialization.
 import yam_arena.scene as arena_scene
 original_object=arena_scene.task_object
 def camera_object(spec):
  obj=original_object(spec)
  if spec.name=='loose_camera':
   original_spawn=obj.spawner_cfg.func
   @clone
   def compound(path,cfg,*args,**kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    prim=spawn_cuboid.__wrapped__(path,cfg,*args,**kwargs)
    add_camera_shoe(path)
    from quick_release import add_camera_fasteners
    add_camera_fasteners(prim.GetStage(),path)
    from camera_tether import maybe_build
    maybe_build(prim.GetStage(),path)
    return prim
   obj.spawner_cfg.func=compound
   obj.object_cfg.spawn.func=compound
   obj.object_cfg.spawn.collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0002,rest_offset=0)
  return obj
 arena_scene.task_object=camera_object

def add_camera_shoe(camera_path):
 import isaaclab.sim as sim
 from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
 from latch_geometry import SHOE
 for name,pos,size in SHOE:
  if name=='shoe_leading':
   add_tapered_leading(camera_path,pos,size)
   continue
  c=sim.CuboidCfg(size=size,collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0002,rest_offset=0),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.12,.12,.14)))
  c.func(camera_path+'/'+name,c,translation=pos)

def add_spring_plunger(stage,arm_path,parent,center,mountR):
 from pxr import UsdGeom,UsdPhysics,UsdShade,Gf,Sdf,PhysxSchema
 from scipy.spatial.transform import Rotation
 from yam_arena import ik,station
 from yam_arena.constants import READY,TCP_OFFSET
 from latch_geometry import TOOTH_ENGAGED
 tcp,gR=ik.tcp_pose(np.array(READY),station.rig().bases['left'])
 gp=tcp-gR@np.array(TCP_OFFSET);local=center+mountR@np.array(TOOTH_ENGAGED)
 pos=gp+gR@local;worldR=gR@mountR
 path=arm_path.rsplit('/',1)[0]+'/SpringPlunger'
 xf=UsdGeom.Xform.Define(stage,path);xf.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)))
 quat=Rotation.from_matrix(worldR).as_quat();xf.AddOrientOp().Set(Gf.Quatf(float(quat[3]),Gf.Vec3f(*map(float,quat[:3]))))
 body=UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim());body.CreateRigidBodyEnabledAttr(True)
 mass=UsdPhysics.MassAPI.Apply(xf.GetPrim());mass.CreateMassAttr(.004);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(2e-7,5e-7,5e-7))
 api=PhysxSchema.PhysxRigidBodyAPI.Apply(xf.GetPrim());api.CreateSolverPositionIterationCountAttr(32);api.CreateSolverVelocityIterationCountAttr(8)
 mesh=UsdGeom.Mesh.Define(stage,path+'/tooth')
 pts=[(-.0015,y,-.002) for y in [-.0015,.0015]]
 pts=[(-.0015,-.0015,-.002),(.0015,-.0015,-.002),(.0015,-.0015,.002),(-.0015,.0015,-.002),(.0015,.0015,-.002),(.0015,.0015,.002)]
 mesh.CreatePointsAttr(pts);mesh.CreateFaceVertexCountsAttr([3,3,4,4,4]);mesh.CreateFaceVertexIndicesAttr([0,2,1,3,4,5,0,1,4,3,1,2,5,4,2,0,3,5]);mesh.CreateSubdivisionSchemeAttr('none')
 mesh.CreateDisplayColorAttr([(0.9,.55,.12)])
 UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
 col=PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim());col.CreateContactOffsetAttr(.0001);col.CreateRestOffsetAttr(0.)
 mat=UsdShade.Material.Define(stage,path+'/Material');shader=UsdShade.Shader.Define(stage,path+'/Material/Shader');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set((.9,.55,.12));mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
 # Contact with the left-side button moves the rear return yoke and tooth
 # together in +X. Button and tooth are one rigid body, no release trigger.
 from quick_release import BUTTON_PARTS
 for name,center,size in BUTTON_PARTS:
  cube=UsdGeom.Cube.Define(stage,path+'/'+name);cube.CreateSizeAttr(1.);cube.AddTranslateOp().Set(Gf.Vec3d(*center));cube.AddScaleOp().Set(Gf.Vec3f(*size));cube.CreateDisplayColorAttr([(.9,.55,.12)])
  UsdPhysics.CollisionAPI.Apply(cube.GetPrim());cc=PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim());cc.CreateContactOffsetAttr(.0001);cc.CreateRestOffsetAttr(0.);UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat)
 joint=UsdPhysics.PrismaticJoint.Define(stage,path+'/SpringGuide');joint.CreateBody0Rel().SetTargets([Sdf.Path(parent)]);joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
 joint.CreateAxisAttr('X');joint.CreateLowerLimitAttr(0.);joint.CreateUpperLimitAttr(.005)
 joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float,local)));q=Rotation.from_matrix(mountR).as_quat();joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))));joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0));joint.CreateLocalRot1Attr().Set(Gf.Quatf(1))
 joint.CreateExcludeFromArticulationAttr(True);joint.CreateCollisionEnabledAttr(False)
 drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(150.);drive.CreateDampingAttr(.15);drive.CreateTargetPositionAttr(-.0002);drive.CreateMaxForceAttr(2.)
 from quick_release import add_press_probe
 add_press_probe(stage,pos,worldR)


def add_tapered_leading(camera_path,pos,size):
 from pxr import UsdGeom,UsdPhysics,PhysxSchema
 import omni.usd
 stage=omni.usd.get_context().get_stage()
 # Four chamfered faces: full-width upper shoulder to a smaller leading tip.
 x,y,z=pos;hx,hy,hz=[v/2 for v in size]
 pts=[(x+sx*(hx if upper else hx-.002), y+sy*(hy if upper else hy-.001),z+(hz if upper else -hz)) for upper in (False,True) for sx,sy in [(-1,-1),(1,-1),(1,1),(-1,1)]]
 mesh=UsdGeom.Mesh.Define(stage,camera_path+'/shoe_leading');mesh.CreatePointsAttr(pts);mesh.CreateFaceVertexCountsAttr([4]*6);mesh.CreateFaceVertexIndicesAttr([0,3,2,1,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7]);mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDisplayColorAttr([(.24,.24,.26)])
 UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
 col=PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim());col.CreateContactOffsetAttr(.0001);col.CreateRestOffsetAttr(0.)
