"""Read-only native USD collidervertices andactualrigidrootposes."""
from pathlib import Path
import json,itertools,numpy as np
from surface_paper import _numpy
CAPTURED=False
def capture(base,device,output,control):
 global CAPTURED
 if control%20 and control!=1:return
 names=('cover','latch_bolt','shutter','fixed_case_walls','enclosure_frame')
 roots={}
 for name in names:
  if name not in base.scene.rigid_objects:continue
  asset=base.scene.rigid_objects[name];data=asset.data
  for field in('root_link_pose_w','root_pose_w'):
   if hasattr(data,field):roots[name]=dict(field=field,pose=_numpy(getattr(data,field))[0].tolist());break
  else:roots[name]=dict(position=_numpy(data.root_pos_w)[0].tolist(),quaternion=_numpy(data.root_quat_w)[0].tolist())
 output.mkdir(exist_ok=True,parents=True)
 with(output/'raw-rigid-root-poses.jsonl').open('a')as f:f.write(json.dumps(dict(control=control,roots=roots))+'\n')
 if CAPTURED:return
 from pxr import Usd,UsdGeom,UsdPhysics,Gf
 stage=device.mk._SIM['stage'];records=[]
 for name in names:
  path=device.mk._SIM['paths'][name];root=stage.GetPrimAtPath(path)
  for prim in Usd.PrimRange(root):
   if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
   typ=prim.GetTypeName();points=[]
   if typ=='Mesh':points=[list(p)for p in UsdGeom.Mesh(prim).GetPointsAttr().Get()]
   elif typ=='Cube':
    size=float(UsdGeom.Cube(prim).GetSizeAttr().Get());points=[[size*s/2for s in signs]for signs in itertools.product((-1,1),repeat=3)]
   if not points:continue
   world=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default());rworld=UsdGeom.Xformable(root).ComputeLocalToWorldTransform(Usd.TimeCode.Default());to_root=world*rworld.GetInverse()
   local_in_root=[list(to_root.Transform(Gf.Vec3d(*p)))for p in points];worldpoints=[list(world.Transform(Gf.Vec3d(*p)))for p in points]
   records.append(dict(part=name,path=str(prim.GetPath()),type=typ,authored_points=points,points_in_rigid_root_frame=local_in_root,USD_worldpoints_at_capture=worldpoints))
 expected=[]
 for name in names:
  for i,b in enumerate(device.kit.parts[name].boxes):
   v=np.asarray(b.vertices)if hasattr(b,'vertices')and len(b.vertices)else np.array([[s*z/2for s,z in zip(sign,b.size)]for sign in itertools.product((-1,1),repeat=3)])
   v=v@device.mk.quat_to_mat(b.quat).T+np.asarray(b.at);expected.append(dict(part=name,index=i,expected_vertices_in_rigid_root_frame=v.tolist()))
 (output/'raw-usd-collision-geometry.json').write_text(json.dumps(dict(control=control,USD_colliders=records,source_expected=expected,boundary='Read-only rawUSDmesh/cubecollisionvertices plus rootframe conversion, independent ofObjectStatecentroid. Runtimeactualposes savedseparately. USDworldtransforms may beauthoredreset transforms ifFabric doesnotwriteback; rootframevertices remain theauthoredphysicalgeometry.'),indent=2)+'\n');CAPTURED=True
