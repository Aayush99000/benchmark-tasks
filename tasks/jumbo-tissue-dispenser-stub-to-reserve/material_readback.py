"""EnforceactualfirstframeUSDboltmaterialandmass; read-only."""
import json
DONE=False
def capture(device,output,control):
 global DONE
 if DONE:return
 from pxr import Usd,UsdShade,UsdPhysics,PhysxSchema
 stage=device.mk._SIM['stage'];boltpath=device.mk._SIM['paths']['latch_bolt'];root=stage.GetPrimAtPath(boltpath);mass=float(UsdPhysics.MassAPI(root).GetMassAttr().Get());records=[]
 for prim in Usd.PrimRange(root):
  if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
  material,rel=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose='physics');api=UsdPhysics.MaterialAPI(material.GetPrim());physx=PhysxSchema.PhysxMaterialAPI(material.GetPrim());s=float(api.GetStaticFrictionAttr().Get());dy=float(api.GetDynamicFrictionAttr().Get());combine=str(physx.GetFrictionCombineModeAttr().Get());records.append(dict(collider=str(prim.GetPath()),bound_material=str(material.GetPath()),static_friction=s,dynamic_friction=dy,friction_combine=combine));
  if abs(s-.35)>1e-6 or abs(dy-.25)>1e-6 or combine!='min':raise RuntimeError('ActualUSDboltmaterialdoesnotmatchfrozenengineeringdeclaration')
 if abs(mass-.00225036)>1e-8 or len(records)!=2:raise RuntimeError('ActualUSDboltMassOrColliderCountMismatch')
 # GuardagainstaccidentallybindingthePOMmaterialtootherpartcolliders.
 unexpected=[]
 for prim in stage.Traverse():
  if not prim.HasAPI(UsdPhysics.CollisionAPI)or str(prim.GetPath()).startswith(boltpath+'/'):continue
  mat,rel=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose='physics')
  if mat and 'POM_bolt_engineering_contact'in str(mat.GetPath()):unexpected.append(str(prim.GetPath()))
 if unexpected:raise RuntimeError('UnexpectednonboltPOMmaterialbinding')
 output.mkdir(exist_ok=True,parents=True);(output/'actual-usd-bolt-material-mass.json').write_text(json.dumps(dict(status='ACTUAL_FIRSTFRAME_USD_LOCAL_BOLT_MATERIAL_MASS_PASS',control=control,bolt_mass_kg=mass,colliders=records,unexpected_nonbolt_bindings=unexpected,material_boundary='POM-C/ABS engineeringcontactestimate. Manufacturerreference.21dynamicunderASTMD3702doesnotmeasurethispair. minmodeappliestoallboltcontacts,includingrobot/receiverifcontacted; noothermaterialmodified.'),indent=2)+'\n');DONE=True
