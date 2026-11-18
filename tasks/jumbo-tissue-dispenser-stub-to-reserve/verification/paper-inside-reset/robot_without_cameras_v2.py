"""Run the original Inspect embodiment callbacks with image rendering omitted.

The actual reset, ordinary action clipping, task step, mechanics callbacks,
scorer and success hold use YamArenaEmbodiment unchanged. Only the environment
is built with enable_cameras=False and observations contain no images. This
is a creator-reference/component experiment, not model/video evidence.
"""
from pathlib import Path
import argparse,ctypes,json,os,sys,time,hashlib,types

driver=ctypes.CDLL("libcuda.so.1")
if driver.cuInit(0)!=0:raise RuntimeError("CUDA failed before framework import")
parser=argparse.ArgumentParser()
parser.add_argument("--task",type=Path,required=True)
parser.add_argument("--out",type=Path,required=True)
parser.add_argument("--steps",type=int,required=True)
from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args=parser.parse_args();args.enable_cameras=False
launcher=AppLauncher(args);app=launcher.app


def main():
    import numpy as np
    import torch
    from inspect_robots import Scene
    from inspect_robots.types import Action,Observation
    from yam_arena.constants import READY,ARM_JOINTS,FINGER_JOINTS
    from yam_arena.demo import ArmPlan
    import yam_arena.inspect_embodiment as ie
    ie._APP=app
    # Task import installs its own original reset/task-step wrappers on this
    # actual class; they are exercised below through the normal public methods.
    handle=ie.YamArenaEmbodiment(task=str(args.task),object_jitter=0.,control_hz=20.)
    from yam_arena.embodiment import BimanualYamEmbodiment
    from yam_arena.scene import make_env
    args.out.mkdir(parents=True,exist_ok=False)
    source={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in args.task.parent.iterdir()if p.is_file()}
    start=time.monotonic()
    robot=BimanualYamEmbodiment(enable_cameras=False,contact_sensors=False,rig=handle.rig)
    env=make_env(robot,scene="task",task=handle.task,device=handle.device,control_hz=20.)
    handle._env=env;handle._base=env.unwrapped;handle._goal=None
    handle._torch=torch;handle._free_objects=handle.task.free_objects
    handle._ready=torch.tensor([[*READY,1.,*READY,1.]],dtype=torch.float32,device=handle._base.device)
    handle._ids={name:(handle._base.scene[name].find_joints(ARM_JOINTS,preserve_order=True)[0],
                       handle._base.scene[name].find_joints(FINGER_JOINTS,preserve_order=True)[0])
                 for name in ie.ARMS}
    handle._observation=types.MethodType(lambda self:Observation(state_time=self._t),handle)
    manifest=args.task.with_name("task.json")
    metadata=json.loads(manifest.read_text())if manifest.exists()else{}
    handle.reset(Scene(id="native-component",instruction=metadata.get("instruction","Perform the task.")),seed=0)
    plans={name:ArmPlan(list(points),handle.rig.bases[name])
           for name,points in handle.task.demo().items()}
    final=None
    with(args.out/"controls.jsonl").open("w")as stream:
        for control in range(1,args.steps+1):
            action=np.concatenate([plans[a].action7(control*.05)[0]
                if a in plans else np.r_[READY,1.]for a in("left","right")])
            final=handle.step(Action(data=action))
            stream.write(json.dumps(dict(control=control,score=final.info.get("score"),
                terminated=final.terminated,termination_reason=final.termination_reason,
                physics_dt=handle._base.physics_dt,control_dt=handle._base.step_dt,
                actual_stock_action14=action.tolist()))+"\n")
            if control%20==0:
                stream.flush();print("ORIGINAL_CALLBACK_CONTROL",control,dict(final.info),flush=True)
            if final.terminated:break
    after={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
           for p in args.task.parent.iterdir()if p.is_file()}
    result=dict(controls=control,normal_terminal=True,source_unchanged=source==after,
        source_before=source,source_after=after,physics_dt=handle._base.physics_dt,
        control_dt=handle._base.step_dt,cameras_enabled=False,
        original_embodiment_callbacks=True,wall_seconds=time.monotonic()-start,
        score=final.info.get("score"),termination_reason=final.termination_reason,
        success=final.terminated,model_evidence=False,media_evidence=False)
    (args.out/"completed.json").write_text(json.dumps(result,indent=2)+"\n")
    env.close()


try:
    main()
except BaseException as error:
    import traceback
    detail=traceback.format_exc();print(detail,flush=True)
    args.out.parent.mkdir(exist_ok=True,parents=True)
    args.out.with_name(args.out.name+"-error.json").write_text(json.dumps(
        dict(error=repr(error),traceback=detail,model_evidence=False),indent=2)+"\n")
finally:
    app.close()
