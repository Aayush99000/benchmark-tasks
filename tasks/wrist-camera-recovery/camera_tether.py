"""Camera USB tether adapted from the cable-recovery task, constructed before physics.

Same capsule chain and elastic joints. Only endpoints/rest curve are changed:
left link2 strain relief -> detached camera side boot. No runtime pose writes.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
CREATED={}
ROOT='/World/envs/env_0/camera_usb_tether'

def segments(vertices):
 d=np.diff(vertices,axis=0);lengths=np.linalg.norm(d,axis=1)
 if np.any(lengths<=.006):raise ValueError('Cable segments shorter than diameter')
 Rs=[]
 for u in d/lengths[:,None]:
  v=np.eye(3)[np.argmin(np.abs(u))];v-=u*np.dot(v,u);v/=np.linalg.norm(v);Rs.append(np.column_stack([u,v,np.cross(u,v)]))
 return (vertices[:-1]+vertices[1:])/2,lengths,np.array(Rs)

def maybe_build(stage,camera_path=None):
 from pxr import UsdGeom,UsdPhysics,Sdf,Gf,PhysxSchema
 from yam_arena.robot import asset_link_paths
 from yam_arena.urdf import forward_kinematics
 from yam_arena import YAM_URDF,station
 from yam_arena.constants import READY,ARM_JOINTS
 from cable_usd import build
 if CREATED:return
 link='/World/envs/env_0/LeftArm'+asset_link_paths()['link2']
 if not camera_path:
  cameras=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetName()=='loose_camera']
  if len(cameras)!=1:return
  camera_path=cameras[0]
 if not stage.GetPrimAtPath(link).IsValid() or not stage.GetPrimAtPath(camera_path).HasAPI(UsdPhysics.RigidBodyAPI):return
 def vec(x):return Gf.Vec3f(*map(float,x))
 def quat(R):
  q=Rotation.from_matrix(R).as_quat();return Gf.Quatf(float(q[3]),vec(q[:3]))
 def box(path,pos,size,color):
  o=UsdGeom.Cube.Define(stage,path);o.CreateSizeAttr(1);o.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));o.AddScaleOp().Set(vec(size));o.CreateDisplayColorAttr([color]);UsdPhysics.CollisionAPI.Apply(o.GetPrim());return o
 frames=forward_kinematics(YAM_URDF,dict(zip(ARM_JOINTS,map(float,READY))));F=frames['link2'].copy();F[:3,3]+=station.rig().bases['left']
 anchor=np.array([0,-.092,0.]);box(link+'/usb_anchor_block',[0,-.073,0],(.024,.014,.024),(.22,.24,.26))
 boot=UsdGeom.Cylinder.Define(stage,link+'/usb_exit_boot');boot.CreateAxisAttr('Y');boot.CreateRadiusAttr(.0045);boot.CreateHeightAttr(.014);boot.AddTranslateOp().Set(Gf.Vec3d(0,-.085,0));boot.CreateDisplayColorAttr([(.08,.10,.12)]);UsdPhysics.CollisionAPI.Apply(boot.GetPrim())
 # Connector overlaps the camera wall; strain-relief tip is the actual chain endpoint.
 port=np.array([.024,0,0]);guide=np.array([.042,0,0]);box(camera_path+'/usb_plug',port,(.009,.008,.008),(.15,.15,.17))
 tail=UsdGeom.Cylinder.Define(stage,camera_path+'/usb_camera_boot');tail.CreateAxisAttr('X');tail.CreateRadiusAttr(.003);tail.CreateHeightAttr(.018);tail.AddTranslateOp().Set(Gf.Vec3d(.033,0,0));tail.CreateDisplayColorAttr([(.12,.18,.22)]);UsdPhysics.CollisionAPI.Apply(tail.GetPrim())
 # Read the authored initial camera transform, not an assumed restored mount transform.
 C=np.array(UsdGeom.Xformable(stage.GetPrimAtPath(camera_path)).ComputeLocalToWorldTransform(0)).T
 start=(F@np.r_[anchor,1])[:3];end=(C@np.r_[guide,1])[:3]
 out0=F[:3,:3]@np.array([0.,-1.,0.]);out1=C[:3,:3]@np.array([1.,0.,0.]);tt=np.linspace(0,1,1500)[:,None]
 def curve(depth):return (1-tt)**3*start+3*(1-tt)**2*tt*(start+depth*out0)+3*(1-tt)*tt**2*(end+depth*out1)+tt**3*end
 chain_length=.74-.018
 if np.linalg.norm(end-start)>=chain_length:raise ValueError('Tether too short for initial placement')
 lo,hi=0.,1.
 for _ in range(45):
  mid=(lo+hi)/2;pts=curve(mid);length=np.linalg.norm(np.diff(pts,axis=0),axis=1).sum()
  if length<chain_length:lo=mid
  else:hi=mid
 pts=curve((lo+hi)/2);ss=np.r_[0,np.cumsum(np.linalg.norm(np.diff(pts,axis=0),axis=1))];samp=np.linspace(0,ss[-1],41);verts=np.column_stack([np.interp(samp,ss,pts[:,k]) for k in range(3)])
 centers,lengths,Rs=segments(verts)
 g={'mass_kg':.12,'radius_m':.003,'centres_world':centers.tolist(),'vertices_world':verts.tolist(),'lengths_m':lengths.tolist(),'rotations_xyzw':Rotation.from_matrix(Rs).as_quat().tolist(),'attachments':[{'local_xyz':anchor.tolist()},{'local_xyz':guide.tolist()}]}
 paths=build(stage,ROOT,[link,camera_path],g)
 # Original rest-curve elastic D6 model; terminal flex retained for camera manipulation.
 joints=[]
 for i in range(len(paths)-1):joints.append((f'bend_{i:03d}',paths[i],paths[i+1],[lengths[i]/2,0,0],[-lengths[i+1]/2,0,0],np.eye(3),Rs[i+1].T@Rs[i]))
 joints.extend([('anchor_proximal',link,paths[0],anchor,[-lengths[0]/2,0,0],F[:3,:3].T@Rs[0],np.eye(3)),('anchor_distal',paths[-1],camera_path,[lengths[-1]/2,0,0],guide,np.eye(3),C[:3,:3].T@Rs[-1])])
 for name,a,b,pa,pb,Ra,Rb in joints:
  jp=ROOT+'/'+name;stage.RemovePrim(jp);j=UsdPhysics.Joint.Define(stage,jp);j.CreateBody0Rel().SetTargets([Sdf.Path(a)]);j.CreateBody1Rel().SetTargets([Sdf.Path(b)]);j.CreateLocalPos0Attr(vec(pa));j.CreateLocalPos1Attr(vec(pb));j.CreateLocalRot0Attr(quat(Ra));j.CreateLocalRot1Attr(quat(Rb));j.CreateCollisionEnabledAttr(False);j.CreateExcludeFromArticulationAttr(True)
  for axis in ['transX','transY','transZ']:
   lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis);lim.CreateLowAttr(1.);lim.CreateHighAttr(-1.)
  for axis in ['rotX','rotY','rotZ']:
   bound=30. if axis=='rotX' or name.startswith('anchor') else 15.
   lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis);lim.CreateLowAttr(-bound);lim.CreateHighAttr(bound)
   d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),axis);d.CreateTypeAttr('force');d.CreateTargetPositionAttr(0.);d.CreateTargetVelocityAttr(0.);d.CreateStiffnessAttr(.001 if axis=='rotX' else .008);d.CreateDampingAttr(.00006 if axis=='rotX' else .0002);d.CreateMaxForceAttr(.2)
 for path in paths:
  ph=PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path));ph.CreateSolverPositionIterationCountAttr(128);ph.CreateSolverVelocityIterationCountAttr(16);ph.CreateAngularDampingAttr(.15);ph.CreateLinearDampingAttr(.15)
 for body,near in [(link,paths[:2]),(camera_path,paths[-1:])]:UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(body)).CreateFilteredPairsRel().SetTargets([Sdf.Path(p) for p in near])
 CREATED.update(source='Stable cable-recovery capsule and elastic D6 model',root=ROOT,camera_path=camera_path,robot_anchor_path=link,geometry=g,chain_length_m=float(lengths.sum()),total_length_m=float(lengths.sum()+.018),port_local=port.tolist(),camera_strain_relief_local=guide.tolist(),proximal_anchor_local=anchor.tolist(),initial_endpoint_span_m=float(np.linalg.norm(end-start)),physical_cable=True,runtime_pose_writes=False)
 Path(__file__).with_name('tether_geometry.json').write_text(json.dumps(CREATED,indent=2))
