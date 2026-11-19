"""CPU film-only mechanism smoke check. NOT a YAM feasibility demonstration."""
from pathlib import Path
import argparse,json
import numpy as np
from scipy.spatial.transform import Rotation
from film import Film,MELON,EXIT
from scoring import Evaluator,tucks
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
f=Film();e=Evaluator();r=np.array([Rotation.from_euler('x',180,degrees=True).as_matrix()]*2);tcp=np.array([f.x[0,-1],f.x[0,0]])
f.step(tcp,r,[.004,.004]);trace=[]
for label,goal,n in [('draw_up',[[.4,.2,1.19],[.4,-.2,1.19]],70),('over_face',[[.49,.2,.925],[.49,-.2,.925]],70),('drape',[[.465,.15,.84],[.465,-.15,.84]],70),('underside',[[.350,.045,.765],[.350,-.045,.765]],80)]:
 start=tcp.copy()
 for i in range(n):
  tcp=start+(np.array(goal)-start)*(i+1)/n;f.step(tcp,r,[.004,.004])
  e.update(f,MELON,np.eye(3),tcp,0)
 trace.append({'phase':label,**e.result})
 np.savez_compressed(a.out/(label+'.npz'),film=f.x,faces=f.faces)
for y in np.linspace(.215,-.215,30):f.cutter(EXIT+[0,y,.023],all(tucks(f.x,MELON,np.eye(3))))
for _ in range(30):
 tcp=np.array([[0,.3,1.2],[0,-.3,1.2]]);f.step(tcp,r,[.09,.09]);e.update(f,MELON,np.eye(3),tcp,.1)
trace.append({'phase':'released',**e.result})
result={'scope':'synthetic endpoint-driven film-only smoke test; no YAM robot, no native cutter contact','is_yam_demo':False,'trace':trace}
(a.out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
