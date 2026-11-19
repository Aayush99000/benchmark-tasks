"""Conservative point-to-URDF collision-bounds clearance for all robot links."""
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

@lru_cache(maxsize=1)
def collision_bounds(urdf_path):
    import trimesh
    from yam_arena.urdf import rpy_matrix
    urdf_path=Path(urdf_path);out=[]
    for link in ET.parse(urdf_path).getroot().findall('link'):
        for collision in link.findall('collision'):
            origin=collision.find('origin');T=np.eye(4)
            if origin is not None:
                T[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                T[:3,:3]=rpy_matrix(np.fromstring(origin.get('rpy','0 0 0'),sep=' '))
            geom=collision.find('geometry');mesh=geom.find('mesh')
            if mesh is not None:
                filename=mesh.get('filename');path=(urdf_path.parent/filename).resolve()
                if filename.startswith('package://'):raise RuntimeError('Package URI needs explicit resolution: '+filename)
                bounds=trimesh.load(path,force='mesh').bounds*np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
            elif geom.find('box') is not None:
                size=np.fromstring(geom.find('box').get('size'),sep=' ');bounds=np.array([-size/2,size/2])
            elif geom.find('cylinder') is not None:
                c=geom.find('cylinder');r=float(c.get('radius'));h=float(c.get('length'))/2;bounds=np.array([[-r,-r,-h],[r,r,h]])
            elif geom.find('sphere') is not None:
                r=float(geom.find('sphere').get('radius'));bounds=np.array([[-r]*3,[r]*3])
            else:raise RuntimeError('Unsupported URDF collision primitive')
            out.append((link.get('name'),T,bounds))
    if not out:raise RuntimeError('No robot collision geometry')
    return out

def robot_clearance(points,robots):
    from yam_arena import YAM_URDF
    from yam_arena.runtime import link_poses
    best=float('inf')
    for robot in robots:
        poses=link_poses(robot)
        for name,local,bounds in collision_bounds(str(YAM_URDF)):
            world=poses[name]@local;p=(np.asarray(points)-world[:3,3])@world[:3,:3]
            distance=np.linalg.norm(np.maximum(np.maximum(bounds[0]-p,p-bounds[1]),0),axis=1).min()
            best=min(best,float(distance))
    return best

def gripper_contact_boxes(robots):
    """Native measured poses; conservative per-piece URDF mesh bounds."""
    from yam_arena import YAM_URDF
    from yam_arena.runtime import link_poses
    boxes=[]
    for robot in robots:
        poses=link_poses(robot)
        for name,local,bounds in collision_bounds(str(YAM_URDF)):
            if name in ['gripper','tip_left','tip_right']:
                boxes.append((poses[name]@local,bounds))
    return boxes
