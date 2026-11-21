"""Custom screwed plate/receiver and contact-driven push release.

Reference: Falcam F22 public push-to-attach / press-to-detach principle.
The keyed plate and rear return yoke are this task's own geometry, not F22 CAD.
Coordinates are in the unchanged camera housing frame unless stated otherwise.
"""
import numpy as np
# All four boxes belong to the SAME rigid slider as its tooth. A left-side
# button pushed in +X retracts the right-side tooth in +X. The return yoke
# passes behind the stationary backplate; there is no software release.
BUTTON_PARTS=[
 ('tooth_stem',(.0055,0,0),(.008,.002,.002)),
 ('rear_return',(.008,-.008,0),(.002,.018,.002)),
 ('rear_crossbar',(-.0126,-.016,0),(.0432,.002,.002)),
 ('release_button',(-.0332,-.016,0),(.004,.010,.010)),
]
BUTTON_PRESS_AXIS=[1.,0.,0.]
BUTTON_TRAVEL_M=.005

def add_receiver_details(stage,parent,center,mountR):
 from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
 from scipy.spatial.transform import Rotation
 def vec(p):return Gf.Vec3f(*map(float,p))
 def q(R):
  a=Rotation.from_matrix(R).as_quat();return Gf.Quatf(float(a[3]),vec(a[:3]))
 # Tapered mouths lead into the existing plate shoulders and retaining lips.
 # Bottom faces coincide with the original guide entrance at housing Z=2mm.
 for side in (-1,1):
  points=[]
  for z,inner,outer in [(.002,.0102,.0142),(.006,.0122,.0162)]:
   for x,y in [(inner,-.0292),(outer,-.0292),(outer,-.0218),(inner,-.0218)]:points.append(center+mountR@np.array([side*x,y,z]))
  mesh=UsdGeom.Mesh.Define(stage,parent+'/receiver_mouth_'+('left' if side<0 else 'right'))
  mesh.CreatePointsAttr([vec(p) for p in points]);mesh.CreateFaceVertexCountsAttr([4]*6);mesh.CreateFaceVertexIndicesAttr([0,3,2,1,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7]);mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDisplayColorAttr([(.18,.32,.48)])
  UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull');col=PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim());col.CreateContactOffsetAttr(.0001);col.CreateRestOffsetAttr(0.)
 # Rear cassette encloses the return yoke; its left aperture exposes the button.
 for name,delta,size in [('back',(.0005,-.048,-.009),(.049,.002,.020)),('top',(.0005,-.039,.001),(.049,.020,.002)),('bottom',(.0005,-.039,-.019),(.049,.020,.002)),('right',(.026,-.039,-.009),(.002,.020,.020))]:
  o=UsdGeom.Cube.Define(stage,parent+'/release_cassette_'+name);o.CreateSizeAttr(1);o.AddTranslateOp().Set(Gf.Vec3d(*map(float,center+mountR@np.array(delta))));o.AddOrientOp().Set(q(mountR));o.AddScaleOp().Set(vec(size));o.CreateDisplayColorAttr([(.18,.32,.48)]);UsdPhysics.CollisionAPI.Apply(o.GetPrim());co=PhysxSchema.PhysxCollisionAPI.Apply(o.GetPrim());co.CreateContactOffsetAttr(.0001);co.CreateRestOffsetAttr(0.)
 # Bolt feet sit at the existing strut attachment locations on the wrist.
 # Screw heads/stems are embedded visual details of an already rigid assembly.
 for side in (-1,1):
  tag='left' if side<0 else 'right';p=np.array([side*.017,-.023,.025])
  foot=UsdGeom.Cube.Define(stage,parent+'/receiver_bolt_foot_'+tag);foot.CreateSizeAttr(1);foot.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)));foot.AddScaleOp().Set(vec([.011,.004,.012]));foot.CreateDisplayColorAttr([(.30,.34,.39)]);UsdPhysics.CollisionAPI.Apply(foot.GetPrim())
  screw(stage,parent+'/receiver_screw_'+tag,p+np.array([0,-.0025,0]),axis='Y',radius=.0022,height=.0012)

def screw(stage,path,pos,axis='Y',radius=.0016,height=.00012):
 from pxr import UsdGeom,Gf
 head=UsdGeom.Cylinder.Define(stage,path);head.CreateAxisAttr(axis);head.CreateRadiusAttr(radius);head.CreateHeightAttr(height);head.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));head.CreateDisplayColorAttr([(.65,.68,.71)])
 # Dark hex recess is represented on the exposed face, within head envelope.
 face=np.array(pos);face['XYZ'.index(axis)]-=height/2+.00001
 recess=UsdGeom.Cylinder.Define(stage,path+'_hex_socket');recess.CreateAxisAttr(axis);recess.CreateRadiusAttr(radius*.48);recess.CreateHeightAttr(.000025);recess.AddTranslateOp().Set(Gf.Vec3d(*map(float,face)));recess.CreateDisplayColorAttr([(.035,.04,.045)])

def add_camera_fasteners(stage,camera_path):
 # Flush heads stay within the existing shoe/backplate running clearance.
 for i,z in enumerate([-.003,-.009]):screw(stage,camera_path+f'/plate_screw_{i}',[-.001,-.02694,z])

PRESS_PROBE=False
PROBE_INFO={}
def add_press_probe(stage,plunger_world,worldR):
 """Optional diagnostic actuator only; never present in task/demo/model scenes."""
 if not PRESS_PROBE:return
 from pxr import UsdGeom,UsdPhysics,Gf
 from scipy.spatial.transform import Rotation
 path='/World/envs/env_0/DiagnosticButtonProbe';start=np.array([-.0462,-.016,0.]);pos=plunger_world+worldR@start;q=Rotation.from_matrix(worldR).as_quat()
 xf=UsdGeom.Xform.Define(stage,path);xf.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));xf.AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))));rb=UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim());rb.CreateKinematicEnabledAttr(True)
 cube=UsdGeom.Cube.Define(stage,path+'/tip');cube.CreateSizeAttr(1);cube.AddScaleOp().Set(Gf.Vec3f(.004,.006,.006));cube.CreateDisplayColorAttr([(.85,.22,.18)]);UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
 PROBE_INFO.update(path=path,plunger_origin_world=plunger_world.tolist(),rotation_world=worldR.tolist(),start_local=start.tolist(),quaternion_xyzw=q.tolist())
