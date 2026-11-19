"""Native YAM controller; hybrid film uses measured jaw/pose observations."""
from pathlib import Path
import argparse,os,sys,json,traceback,time,hashlib
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--mode',choices=['setup','demo','interactive'],default='setup');p.add_argument('--adapter',type=Path,required=True);p.add_argument('--device',default='cuda:0');p.add_argument('--seed',type=int,default=0);a=p.parse_args()
ROOT=Path(__file__).resolve().parent;out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
os.environ['MELON_EVIDENCE_DIR']=str(out)
from prepare_runtime import prepare
sys.path.insert(0,str(prepare(a.adapter)));sys.path.insert(0,str(ROOT))
sources=[f for f in ROOT.iterdir() if f.suffix in ['.py','.cpp'] or f.name=='contact_kernel_build.json']
for f in sources:(out/('source_'+f.name)).write_bytes(f.read_bytes())
from contact_fast import kernel
(out/'manifest.json').write_text(json.dumps({'mode':a.mode,'seed':a.seed,'control_hz':20,'physics_hz':240,'native_yam':True,'compiled_contact_kernel':kernel is not None,'sources':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}},indent=2))
def arr0(x):return (x.torch if hasattr(x,'torch') else x).detach().cpu().numpy()
def main():
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import YamArenaEmbodiment
    from types import SimpleNamespace
    from scipy.spatial.transform import Rotation,Slerp
    from PIL import Image
    from yam_arena import station
    from yam_arena.runtime import tcp_world,arm_q,link_poses
    from yam_arena.ik import solve
    INSTRUCTION="Use both arms to pull cling film from the dispenser, cover the watermelon half and tuck the film underneath, then slide the cutter across and release the wrapped fruit."
    env=YamArenaEmbodiment(task=str(ROOT/'arena_task.py'),device=a.device,control_hz=20,cam_width=160,cam_height=96,object_jitter=0)
    env.reset(Scene(id='watermelon-film-scripted',instruction=INSTRUCTION),seed=a.seed)
    base=env._env.unwrapped
    import omni.usd
    from pxr import UsdPhysics
    stage=omni.usd.get_context().get_stage()
    fruit_mesh=stage.GetPrimAtPath(base.scene.rigid_objects['watermelon'].cfg.prim_path.replace('{ENV_REGEX_NS}','/World/envs/env_0')+'/geometry/mesh')
    if not fruit_mesh.IsValid():
        fruit_mesh=next(p for p in stage.Traverse() if str(p.GetPath()).endswith('/watermelon/geometry/mesh'))
    approximation=UsdPhysics.MeshCollisionAPI(fruit_mesh).GetApproximationAttr().Get()
    (out/'collider_check.json').write_text(json.dumps({'watermelon':approximation,'expected':'convexHull'}))
    if approximation!='convexHull':raise RuntimeError('Watermelon collision shape must match its hemispherical mesh')
    robots=[base.scene.articulations[s+'_robot'] for s in ['left','right']]
    bases=[station.BASES[s] for s in ['left','right']]
    def arr(x):return (x.torch if hasattr(x,'torch') else x).detach().cpu().numpy()
    def current():return [tcp_world(r).copy() for r in robots]
    def position(n):return arr(base.scene.rigid_objects[n].data.root_pos_w)[0].copy()
    def rotation(n):return Rotation.from_quat(arr(base.scene.rigid_objects[n].data.root_quat_w)[0])
    q=[np.array(arm_q(r)).copy() for r in robots];grip=[1.,1.];steps=0
    orientations=[Rotation.from_matrix(link_poses(r)['gripper'][:3,:3]).as_quat() for r in robots]
    (out/'frames').mkdir();trace=[]
    def tick(n=1):
        nonlocal steps
        for _ in range(n):
            env.step(SimpleNamespace(data=np.r_[q[0],grip[0],q[1],grip[1]]));steps+=1
            if steps>3600:raise RuntimeError('3600-step action limit')
            if steps%4==0:
                rgb=arr(base.scene.sensors['overview_cam'].data.output['rgb'])[0,:,:,:3].astype('uint8')
                Image.fromarray(rgb).save(out/'frames'/f'{steps:05d}.png')
    def record(label):
        state={'step':steps,'label':label,'tcp':[v.tolist() for v in current()],'tcp_orientations':[Rotation.from_matrix(link_poses(r)['gripper'][:3,:3]).as_quat().tolist() for r in robots],'joint_targets':[v.tolist() for v in q],'grip':grip,'objects':{n:{'position':position(n).tolist(),'rotation':rotation(n).as_quat().tolist()} for n in ['watermelon','cutter']},'score':env.task.module.TRACKER.result}
        f=env.task.module.TRACKER.film
        state['film_pins']=[{'arm':k,'node':p.node,'local_offset':p.offset.tolist(),'world_point':f.x.reshape(-1,3)[p.node].tolist()} for k,p in f.pins.items()]
        trace.append(state);(out/'controller_trace.json').write_text(json.dumps(trace,indent=2));(out/'heartbeat.json').write_text(json.dumps(state,indent=2));print('PHASE',label,steps,state['score'],flush=True)
    def move(targets,n=50,joint=False):
        starts=current();seeds=[v.copy() for v in q];plan=[]
        rotations=[Slerp([0,1],Rotation.from_quat([Rotation.from_matrix(link_poses(r)['gripper'][:3,:3]).as_quat(),o])) for r,o in zip(robots,orientations)]
        if joint:
            goals=[]
            for k in range(2):
                goal,e,re=solve(targets[k],orientations[k],q_init=q[k],base_pos=bases[k],iters=600)
                if e>.005 or re>.08:raise RuntimeError(f'Unreachable hover arm{k}: {e}, {re}')
                goals.append(goal)
        for j in range(1,n+1):
            t=j/n;t=t*t*t*(10+t*(-15+6*t))
            if joint:plan.append([u*(1-t)+v*t for u,v in zip(q,goals)]);continue
            for k in range(2):
                seeds[k],e,re=solve(starts[k]*(1-t)+np.array(targets[k])*t,rotations[k](t).as_quat(),q_init=seeds[k],base_pos=bases[k],iters=100,tol=.0005)
                if e>.005 or re>.08:raise RuntimeError(f'Unreachable path arm{k}: {e}, {re}')
            plan.append([v.copy() for v in seeds])
        for value in plan:q[:]=value;tick()
    def go(k,pos,n=40,joint=False):
        targets=current();targets[k]=np.asarray(pos);move(targets,n,joint)
    def close(k,value,n=15):
        start=grip[k]
        for i in range(n):grip[k]=start+(value-start)*(i+1)/n;tick()
    def film_state():return env.task.module.TRACKER.film
    def set_tilt(angle):
        orientations[:]=[(Rotation.from_euler('y',angle,degrees=True)*Rotation.from_euler('x',180,degrees=True)).as_quat() for _ in range(2)]
    def grasp_edges():
        set_tilt(-25)
        corners=[film_state().x[0,-1].copy(),film_state().x[0,0].copy()]
        move([p+[0,0,.055] for p in corners],55,True)
        move(corners,40);close(0,.035);close(1,.035);record('both_leading_corners')
        if len(film_state().pins)!=2:raise RuntimeError('Both physical edge-pinches not detected')
    def demo():
        grasp_edges()
        set_tilt(-70)
        move([[.42,.20,1.25],[.42,-.20,1.25]],100);record('meter_film_upward')
        set_tilt(-25)
        move([[.49,.20,.925],[.49,-.20,.925]],100);record('draw_film')
        move([[.465,.15,.84],[.465,-.15,.84]],65);record('cover_face')
        orientations[:]=[Rotation.from_euler('x',v,degrees=True).as_quat() for v in [120,240]]
        move([[.45,.19,.91],[.45,-.19,.91]],60)
        move([[.35,.14,.80],[.35,-.14,.80]],60)
        move([[.350,.045,.775],[.350,-.045,.775]],100);record('tuck_under_rind')
        targets=current();targets[1]+=np.array([-.015,0,.015]);move(targets,25);tick(10);record('measured_right_tuck_preserving_left')
        finish_wrap()
    def finish_wrap():
        result=env.task.module.TRACKER.result
        if not all(result.get('underside_tucks',[False,False])) or result.get('coverage',0)<.95:raise RuntimeError('Coverage and both underside tucks required before cutting')
        # Keep the right corner held while the left hand operates the cutter.
        # This prevents the uncut sheet being pulled back by the dispenser.
        close(0,1.)
        # The tucked wrist is near its rotational limit. Withdraw beside the
        # rind before rotating toward the cutter; a diagonal high withdrawal
        # was rejected by the complete Cartesian-path reachability check.
        orientations[:]=[Rotation.from_matrix(link_poses(r)['gripper'][:3,:3]).as_quat() for r in robots]
        go(0,[.35,.16,.78],55)
        go(0,[.30,.20,.85],40)
        # Native slider is moved by real rigid gripper contact.
        orientations[0]=(Rotation.from_euler('z',-90,degrees=True)*Rotation.from_euler('y',-30,degrees=True)*Rotation.from_euler('x',180,degrees=True)).as_quat()
        go(0,position('cutter')+[0,0,.072],60,True)
        go(0,position('cutter')+[0,0,.012],45);close(0,.20,20)
        goal=current()[0].copy();goal[1]=-.215
        go(0,goal,130);tick(10);record('slide_guarded_cutter')
        if not film_state().cut:raise RuntimeError('Native cutter did not complete the film-width sweep')
        close(0,1.)
        orientations[0]=(Rotation.from_euler('z',-90,degrees=True)*Rotation.from_euler('y',-45,degrees=True)*Rotation.from_euler('x',180,degrees=True)).as_quat()
        go(0,[.18,-.215,.98],45,True)
        close(1,1.)
        orientations[1]=Rotation.from_matrix(link_poses(robots[1])['gripper'][:3,:3]).as_quat()
        go(1,[.33,-.16,.78],55)
        go(1,[.30,-.20,.85],40)
        set_tilt(-25)
        move([[.30,.26,1.05],[.30,-.26,1.05]],65,True)
        tick(40);record('released_wrap')
        if not env.task.module.TRACKER.result['full_task_success']:
            raise RuntimeError('Released wrap did not retain full-score geometry')
    try:
        tick(40);record('settled')
        from station_setup import save_views
        save_views(base,out)
        if a.mode=='demo':demo()
        if a.mode=='interactive':
            print('INTERACTIVE_READY',flush=True)
            while True:
                file=out/'command.json'
                if not file.exists():time.sleep(.1);continue
                request=json.loads(file.read_text());file.unlink()
                with (out/'command_history.jsonl').open('a') as f:f.write(json.dumps(request)+'\n')
                if request.get('quit'):break
                try:
                    for op in request.get('operations',[]):
                        if op.get('demo'):demo();continue
                        if op.get('finish_wrap'):finish_wrap();continue
                        if op.get('grasp_edges'):grasp_edges();continue
                        if 'grip' in op:grip[:]=op['grip']
                        if 'orientations' in op:
                            for k,value in enumerate(op['orientations']):
                                if value is not None:orientations[k]=value
                        if 'targets' in op:move(op['targets'],op.get('steps',40),op.get('joint',False))
                        if 'delta' in op:move([v+d for v,d in zip(current(),np.asarray(op['delta']))],op.get('steps',40))
                        if 'tick' in op:tick(op['tick'])
                    record(request['label']);result={'label':request['label'],'score':env.task.module.TRACKER.result}
                except Exception as error:record('failed');result={'error':str(error)}
                (out/'command_result.json').write_text(json.dumps(result,indent=2))
        (out/'result.json').write_text(json.dumps({'steps':steps,'mode':a.mode,'score':env.task.module.TRACKER.result},indent=2))
    except Exception:
        (out/'error.txt').write_text(traceback.format_exc());record('failed');raise
    finally:env.close()
try:main()
except Exception:
    (out/'startup_error.txt').write_text(traceback.format_exc());raise
