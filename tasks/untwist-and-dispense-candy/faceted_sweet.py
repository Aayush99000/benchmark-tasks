"""A small boiled sweet with two flat faces and a rounded polygonal rim."""
import numpy as np


def points(radius=.0085):
    angles=np.arange(12)*np.pi/6
    rings=[]
    for scale,z in [(.63,-.82),(.90,-.51),(1.,-.13),(1.,.13),(.90,.51),(.63,.82)]:
        rings.append(radius*np.column_stack((scale*np.cos(angles),scale*np.sin(angles),np.full(12,z))))
    return np.vstack(rings)


def setup():
    from layout_variants import round_sweet
    bodies,surfaces,details=round_sweet()
    bodies[0].update(kind='faceted',vertices=points().tolist())
    details.update(candy_shape='12-sided molded hard drop with two flat faces',candy_height_m=.01394)
    return bodies,surfaces,details


def usd_asset(directory,body):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema, Vt
    path=directory/'candy.usda'
    stage=Usd.Stage.CreateNew(str(path))
    root=UsdGeom.Xform.Define(stage,'/Asset').GetPrim()
    stage.SetDefaultPrim(root)
    mesh=UsdGeom.Mesh.Define(stage,'/Asset/hard_drop')
    UsdGeom.SetStageUpAxis(stage,'Z')
    UsdGeom.SetStageMetersPerUnit(stage,1.)
    mesh.CreatePointsAttr(Vt.Vec3fArray(body['vertices']))
    faces=[list(reversed(range(12))),list(range(60,72))]
    for k in range(5):
        for i in range(12):
            a=k*12+i;b=k*12+(i+1)%12
            faces.append([a,b,b+12,a+12])
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*body['color'])])
    prim=mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
    collision=PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collision.CreateContactOffsetAttr(.001)
    collision.CreateRestOffsetAttr(0.)
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(body['mass'])
    stage.GetRootLayer().Save()
    return str(path)
