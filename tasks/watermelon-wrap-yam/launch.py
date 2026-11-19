"""Portable launcher for native scripted development or Inspect Robots Dream-1."""
import argparse,os,sys,subprocess
from pathlib import Path
from prepare_runtime import prepare

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['setup','demo','interactive','model']);p.add_argument('--adapter',required=True,type=Path);p.add_argument('--out',required=True,type=Path);p.add_argument('--device',default='cuda:0');p.add_argument('--python',default=sys.executable);a=p.parse_args()
    root=Path(__file__).resolve().parent;out=a.out.resolve()
    if out.exists():p.error('The output directory must be new')
    runtime=prepare(a.adapter);env=os.environ.copy();env['PYTHONPATH']=os.pathsep.join([str(root),str(runtime),env.get('PYTHONPATH','')]);env['YAM_ADAPTER_ROOT']=str(a.adapter.resolve())
    if a.mode=='model':
        out.mkdir(parents=True);env['MELON_EVIDENCE_DIR']=str(out/'geometry')
        command=[str(Path(a.python).parent/'inspect-robots'),'run','--embodiment','yam_arena','--policy','dream','-E','task='+str(root/'arena_task.py'),'-E','object_jitter=0','-E','control_hz=20','-E','cam_width=320','-E','cam_height=176','-E','device='+a.device,'--instruction','Use both arms to pull cling film from the dispenser, cover the watermelon half and tuck the film underneath, then slide the cutter across and release the wrapped fruit.','--scorer','task_score','--max-steps','3600','--seed','0','--epochs','1','--no-prompt','--store-frames','--log-dir',str(out/'model_logs')]
    else:command=[a.python,str(root/'run.py'),'--mode',a.mode,'--adapter',str(a.adapter.resolve()),'--device',a.device,'--out',str(out)]
    return subprocess.call(command,env=env)
if __name__=='__main__':raise SystemExit(main())
