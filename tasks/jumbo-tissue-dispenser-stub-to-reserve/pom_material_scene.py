"""DeclaredPOM-Cbolt andABScover engineeringequivalent; localboltmaterialonly."""
import latch_cam_scene as base
d=base.d
POM_DENSITY_KG_M3=1410.
# The6mm squareloadtip is fullyinside the19x14mmtriangle footprint; do not
# double-count overlappingcollisionpieces when deriving thephysicalsolidmass.
BOLT_UNION_VOLUME_M3=.019*.014/2*.012
BOLT_MASS_KG=POM_DENSITY_KG_M3*BOLT_UNION_VOLUME_M3
STATIC_FRICTION=.35;DYNAMIC_FRICTION=.25;FRICTION_COMBINE='min'
part=d.kit.parts['latch_bolt'];part.mass=BOLT_MASS_KG
assert abs(BOLT_MASS_KG-.00225036)<1e-12
d.OBJECTS=[*d.kit.objects(),d.paper.marker,*d.FIXTURES]
OBJECTS=[*d.OBJECTS,*base.base.base.closure_tool.toolkit.objects()]
original_author=d.mk._author_all
def author_local_material(stage,env_path):
 original_author(stage,env_path)
 from pxr import Usd,UsdShade,UsdPhysics,PhysxSchema
 material_path=env_path+'/Materials/POM_bolt_engineering_contact'
 material=UsdShade.Material.Define(stage,material_path);physics=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
 physics.CreateStaticFrictionAttr().Set(STATIC_FRICTION);physics.CreateDynamicFrictionAttr().Set(DYNAMIC_FRICTION);physics.CreateRestitutionAttr().Set(0.)
 PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr().Set(FRICTION_COMBINE)
 root=stage.GetPrimAtPath(env_path+'/latch_bolt');bound=[]
 for prim in Usd.PrimRange(root):
  if prim.HasAPI(UsdPhysics.CollisionAPI):
   UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,materialPurpose='physics');bound.append(str(prim.GetPath()))
 if len(bound)!=2:raise RuntimeError('Expectedexactly2originalboltcollisionpieces')
 # Onlythebolt materialisbound; gripper/door/paper/otherpartsarenotmodified.
 print('POM_BOLT_MATERIAL_AUTHORED '+str(dict(mass_kg=BOLT_MASS_KG,static=STATIC_FRICTION,dynamic=DYNAMIC_FRICTION,combine=FRICTION_COMBINE,colliders=bound)),flush=True)
d.mk._author_all=author_local_material
