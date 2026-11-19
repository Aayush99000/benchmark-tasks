"""Two YAMs + native rigid fruit/cutter + measured-pinch flexible-film surrogate."""
from pathlib import Path
import sys,os,json,importlib.abc,importlib.util
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from yam_arena.task_file import Box
SCENE='station'
INSTRUCTION='Use both arms to pull cling film from the dispenser, cover the watermelon half and tuck the film underneath, then slide the cutter across and release the wrapped fruit.'
SUCCESS_HOLD_S=0.
SCORE_OVER='end'
OBJECTS=[Box(name=n,pos=(.4,0,.84),size=(.03,.03,.01),mass=.02) for n in ['watermelon','cutter']]
def array(x):return (x.torch if hasattr(x,'torch') else x).detach().cpu().numpy()
class Tracker:
    def __init__(self):self.reset(None)
    def reset(self,env):
        from film import Film
        from scoring import Evaluator
        self.film=Film();self.evaluator=Evaluator();self.result={'score':0.};self.count=0;self.mesh=None;self.trace=[]
    def step(self,env):
        from yam_arena.runtime import tcp_world,link_poses
        from scipy.spatial.transform import Rotation
        from clearance import robot_clearance,gripper_contact_boxes
        from scoring import tucks
        base=env.unwrapped;self.count+=1
        robots=[base.scene.articulations[s+'_robot'] for s in ['left','right']]
        tcp=np.array([tcp_world(r) for r in robots]);rot=np.array([link_poses(r)['gripper'][:3,:3] for r in robots])
        aperture=np.array([abs(float(array(r.data.joint_pos)[0,list(r.joint_names).index('joint7')]))*2 for r in robots])
        fruit=base.scene.rigid_objects['watermelon'];pos=array(fruit.data.root_pos_w)[0];rotation=Rotation.from_quat(array(fruit.data.root_quat_w)[0]).as_matrix()
        self.film.step(tcp,rot,aperture,pos,rotation,gripper_boxes=gripper_contact_boxes(robots))
        slider=array(base.scene.rigid_objects['cutter'].data.root_pos_w)[0]
        self.film.cutter(slider,all(tucks(self.film.x,pos,rotation,self.film.faces)))
        clearance=robot_clearance(self.film.x.reshape(-1,3),robots)
        self.result=self.evaluator.update(self.film,pos,rotation,tcp,clearance);self.result['step']=self.count
        self.render()
        out=os.environ.get('MELON_EVIDENCE_DIR')
        if out and self.count%5==0:
            p=Path(out);p.mkdir(parents=True,exist_ok=True);(p/'live_result.json').write_text(json.dumps(self.result,indent=2))
            self.trace.append(dict(self.result));(p/'score_trace.json').write_text(json.dumps(self.trace))
            np.savez_compressed(p/f'state_{self.count:04d}.npz',film=self.film.x,faces=self.film.faces,melon_pos=pos,melon_rot=rotation,tcp=tcp,tcp_rot=rot,aperture=aperture,cutter=slider)
    def render(self):
        from pxr import UsdGeom,UsdShade,Sdf,Gf,Vt
        import omni.usd
        if self.mesh is None:
            stage=omni.usd.get_context().get_stage();self.mesh=UsdGeom.Mesh.Define(stage,'/World/ClingFilm')
            self.mesh.CreateDoubleSidedAttr(True);self.mesh.CreateSubdivisionSchemeAttr('none')
            mat=UsdShade.Material.Define(stage,'/World/ClingFilmMaterial');s=UsdShade.Shader.Define(stage,'/World/ClingFilmMaterial/Shader');s.CreateIdAttr('UsdPreviewSurface')
            s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.60,.85,.92));s.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(.34);s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.23)
            mat.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(self.mesh.GetPrim()).Bind(mat)
        self.mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(self.film.x.reshape(-1,3).astype(np.float32)))
        self.mesh.GetFaceVertexCountsAttr().Set([3]*len(self.film.faces));self.mesh.GetFaceVertexIndicesAttr().Set(self.film.faces.ravel().tolist())
TRACKER=Tracker()
def score(objects):return float(TRACKER.result['score'])
def success(objects):return score(objects)==1.

def patch_scene(module):
    original=module.build_task_environment
    def build(embodiment,task,control_hz=None):
        if task.module.INSTRUCTION!=INSTRUCTION:return original(embodiment,task,control_hz)
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.scene.scene import Scene
        from yam_arena.embodiment import BimanualYamEmbodiment
        from scipy.spatial.transform import Rotation
        from assets import build_assets
        embodiment.camera_config=BimanualYamEmbodiment(rig=embodiment.rig,enable_cameras=True,overview_cam=True,cam_width=160,cam_height=96).camera_config
        eye=np.array([.99,.76,1.50]);f=np.array([.32,0,.96])-eye;f/=np.linalg.norm(f);l=np.cross([0,0,1],f);l/=np.linalg.norm(l);u=np.cross(f,l)
        cam=embodiment.camera_config.overview_cam;cam.width=640;cam.height=400;cam.offset.pos=tuple(eye);cam.offset.rot=tuple(Rotation.from_matrix(np.stack([f,l,u],axis=1)).as_quat())
        def configure(cfg):
            cfg=module.env_cfg_callback(control_hz)(cfg);cfg.sim.dt=1/240;cfg.decimation=12;cfg.sim.render_interval=48
            p=cfg.sim.physics;p.gpu_max_rigid_contact_count=2**17;p.gpu_max_rigid_patch_count=2**15;p.gpu_found_lost_pairs_capacity=2**16;p.gpu_found_lost_aggregate_pairs_capacity=2**18;p.gpu_total_aggregate_pairs_capacity=2**16;p.gpu_max_soft_body_contacts=2**16;p.gpu_max_particle_contacts=2**16
            return cfg
        return IsaacLabArenaEnvironment(name='watermelon_film_yam',scene=Scene(build_assets()),embodiment=embodiment,env_cfg_callback=configure)
    module.build_task_environment=build

def patch_embodiment(module):
    cls=module.YamArenaEmbodiment
    if getattr(cls,'_melon_hook',False):return
    original_reset,original_step=cls.reset,cls._task_step
    def reset(self,*a,**kw):
        result=original_reset(self,*a,**kw)
        if getattr(self.task.module,'INSTRUCTION',None)==INSTRUCTION:TRACKER.reset(self._env)
        return result
    def step(self,*a,**kw):
        if getattr(self.task.module,'INSTRUCTION',None)==INSTRUCTION:TRACKER.step(self._env)
        return original_step(self,*a,**kw)
    cls.reset=reset;cls._task_step=step;cls._melon_hook=True
class Finder(importlib.abc.MetaPathFinder):
    targets={'yam_arena.scene':patch_scene,'yam_arena.inspect_embodiment':patch_embodiment}
    def find_spec(self,fullname,path,target=None):
        if fullname not in self.targets:return
        sys.meta_path.remove(self)
        try:spec=importlib.util.find_spec(fullname)
        finally:sys.meta_path.insert(0,self)
        original=spec.loader.exec_module;patch=self.targets[fullname]
        def run(mod):original(mod);patch(mod)
        spec.loader.exec_module=run;return spec
for name,patch in Finder.targets.items():
    if name in sys.modules:patch(sys.modules[name])
sys.meta_path.insert(0,Finder())
