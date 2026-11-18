"""V3 RESET-only constructed-state negative; normal original reset runs first.

Canonical OBJECTS/ref/joint starts, paper author, flat default/rest, original
node assertion and33attachments stay unchanged. Public native writes construct
five compatible rigid roots and currentpaper once after original reset and
before control1. Then only ordinary stock READY joint targets execute.
"""
from pathlib import Path
import os,sys,json,hashlib,numpy as np
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
CANONICAL=Path(os.environ.get('ROBOCURVE_CANONICAL_TASK_DIR',str(HERE.parent/'package/tasks/jumbo-tissue-dispenser-stub-to-reserve'))).resolve()
FREEZE=json.loads((HERE/'source-freeze.json').read_text())
for name,expected in FREEZE['all_file_sha256'].items():
 if hashlib.sha256((CANONICAL/name).read_bytes()).hexdigest()!=expected:raise RuntimeError('Canonical source drift: '+name)
CPU=json.loads((HERE/'CPU-RESULT.json').read_text())
if CPU['status']!='CPU_RESET_CONSTRUCTION_PASS_NATIVE_PENDING':raise RuntimeError('CPU construction not ready')
from yam_arena import task_file,ik,station
from yam_arena.constants import READY
from yam_arena.demo import ArmPlan,Waypoint
canonical=task_file.load(CANONICAL/'jumbo_tissue_dispenser_stub_to_reserve.py').module
d=canonical.d
import surface_paper as sp
OBJECTS=canonical.OBJECTS
SCENE='station';SCORE_OVER='end';SUCCESS_HOLD_S=1.
score=canonical.score;success=canonical.success
DATA=np.load(HERE/'reset-nodes.npz');POSES=json.loads((HERE/'reset-rigid-poses.json').read_text())
OUT=Path(os.environ.get('ROBOCURVE_TRACE_DIR',str(HERE.parent/(HERE.name+'-trace-native1')))).resolve()
if OUT==HERE or HERE in OUT.parents:raise RuntimeError('Runtime output must stay outside the control fixture')
RESET_ACTIVE=False;ORIGINAL_RESET_CALLS=0;RIGID_RESET_WRITES=0;PAPER_RESET_WRITES=0;CONTROL_OBSERVATIONS=0
LABEL='constructed_reset_control_v3_READY_only'
previous_action=ArmPlan.action7
def ready_action(plan,time):
 if plan.waypoints and plan.waypoints[0].label==LABEL:return np.r_[np.asarray(READY),0.],LABEL
 return previous_action(plan,time)
ArmPlan.action7=ready_action
def demo():
 return {arm:[Waypoint(20.,tuple(ik.tcp_pose(np.asarray(READY),station.BASES[arm])[0]),0.,LABEL)]for arm in ('left','right')}

def construct_once(base):
 global RIGID_RESET_WRITES,PAPER_RESET_WRITES
 if not RESET_ACTIVE or ORIGINAL_RESET_CALLS!=1 or RIGID_RESET_WRITES or PAPER_RESET_WRITES:raise RuntimeError('Construction only once after complete original RESET')
 import torch
 if len(base.scene.env_origins)!=1:raise RuntimeError('Constructed control requires exactly one environment')
 ids=torch.tensor([0],device=base.device,dtype=torch.long);origin=base.scene.env_origins[ids]
 for name in ('stub','roll','cover','latch_bolt','shutter'):
  spec=POSES['parts'][name];pose=torch.tensor([spec['position']+spec['quaternion']],device=base.device,dtype=torch.float32)
  if pose.shape!=(1,7) or not torch.isfinite(pose).all() or abs(float(torch.linalg.vector_norm(pose[0,3:]))-1.)>1e-5:raise RuntimeError('Invalid constructed rigid root')
  pose[:,:3]+=origin
  body=base.scene.rigid_objects[name]
  body.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids)
  body.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1,6),device=base.device,dtype=torch.float32),env_ids=ids)
  RIGID_RESET_WRITES+=1
 paper=base.scene.deformable_objects[d.paper.name]
 if not np.array_equal(d.paper.faces,DATA['faces']) or not np.array_equal(d.paper.root_ids,DATA['root_ids']):raise RuntimeError('Native node topology/labels changed')
 anchors_error=float(np.max(np.linalg.norm(d.paper.root_local_anchors-DATA['root_local_anchors'],axis=1)))
 if anchors_error>1e-6:raise RuntimeError('Original native root anchors changed: '+str(anchors_error))
 state=torch.zeros((1,len(DATA['nodes']),6),device=base.device,dtype=torch.float32)
 state[0,:,:3]=torch.as_tensor(DATA['nodes'],device=base.device,dtype=state.dtype)+origin[0]
 paper.write_nodal_state_to_sim_index(state,env_ids=ids)
 PAPER_RESET_WRITES+=1
 # Public forward/update refreshes native caches, without stepping physics or
 # correcting any body afterwards. All settling belongs to normal controls.
 base.sim.forward();base.scene.update(0.)
 sp._refresh(base)
 print('RESET_CONTROL_V3_CONSTRUCTED '+json.dumps(dict(original_reset_calls=ORIGINAL_RESET_CALLS,rigid_reset_writes=RIGID_RESET_WRITES,paper_reset_writes=PAPER_RESET_WRITES,control_phase_writes=0,root_anchor_max_error_m=anchors_error,original_default_assert_passed=True,default_or_rest_changes=0,added_constraints=0)),flush=True)

def patch_reset(module):
 cls=module.YamArenaEmbodiment;original=cls.reset
 def reset(self,*args,**kwargs):
  global RESET_ACTIVE,ORIGINAL_RESET_CALLS,RIGID_RESET_WRITES,PAPER_RESET_WRITES,CONTROL_OBSERVATIONS
  if RESET_ACTIVE:raise RuntimeError('Nested RESET unsupported')
  RESET_ACTIVE=True;ORIGINAL_RESET_CALLS=0;RIGID_RESET_WRITES=0;PAPER_RESET_WRITES=0;CONTROL_OBSERVATIONS=0
  try:
   original(self,*args,**kwargs)
   ORIGINAL_RESET_CALLS+=1
   construct_once(self._env.unwrapped)
  finally:RESET_ACTIVE=False
  return self._observation()
 cls.reset=reset
sp._after_mechanisms('yam_arena.inspect_embodiment',patch_reset)

def observe_constructed_control(objects):
 global CONTROL_OBSERVATIONS
 CONTROL_OBSERVATIONS+=1;n=int(d.mk._EP.get('step',0))
 if RESET_ACTIVE or ORIGINAL_RESET_CALLS!=1 or RIGID_RESET_WRITES!=5 or PAPER_RESET_WRITES!=1:raise RuntimeError('Invalid RESET/control boundary')
 if n not in (1,20,100,200,250,300,400):return
 snapshot=d.paper.snapshot
 if snapshot is None:raise RuntimeError('Missing actual native paper observation')
 OUT.mkdir(parents=True,exist_ok=True)
 np.savez_compressed(OUT/f'constructed-paper-{n:06d}.npz',nodes=snapshot.nodes,velocities=snapshot.velocities,stub_transform=snapshot.stub_transform,faces=d.paper.faces,root_ids=d.paper.root_ids,free_edge_ids=d.paper.free_edge_ids,root_local_anchors=d.paper.root_local_anchors)
 rigid={}
 for name in ('stub','roll','cover','latch_bolt','shutter'):
  p,q=d.kit.pose(objects,name);rigid[name]=dict(position=p.tolist(),quaternion=q.tolist())
 row=dict(control=n,kind='reset_construct_validity_control_not_same_initial_state_policy',original_reset_calls=ORIGINAL_RESET_CALLS,rigid_reset_writes=RIGID_RESET_WRITES,paper_reset_writes=PAPER_RESET_WRITES,control_phase_writes=0,default_or_rest_changes=0,score=float(score(objects)),actual_seats=dict(old_left=bool(d.seated(objects,'stub','p')),new_right=bool(d.seated(objects,'roll','r'))),actual_cover_locked=bool(d.cover_locked(objects)),actual_left_open=bool(d.changed_over(objects)),actual_paper_access=snapshot.access.as_dict(),actual_rigid_settled=all(objects[name].resting(max_speed=.01,max_spin=.25)for name in ('stub','roll','cover','shutter')),actual_release_clear=bool(d.release_geometry(objects)),actual_paper_hands_clear=bool(snapshot.access.hands_clear),actual_rigid_part_origins=rigid,boundary='Originalreset+assertcompletedthenoneRESETconstructionusingpublicnativeAPIs. Realgravity/contacts/scorer onlyafterwards; ifpaperbecomesaccessible oranyothergatefails,thisnegativeisinconclusive. Noexpected0.8assertion.')
 with(OUT/'constructed-control-observations.jsonl').open('a')as f:f.write(json.dumps(row,default=lambda v:v.item()if isinstance(v,np.generic)else v.tolist())+'\n')
 print('CONSTRUCTED_CONTROL '+json.dumps(row,default=lambda v:v.item()if isinstance(v,np.generic)else v.tolist()),flush=True)
d.kit.on_step(observe_constructed_control)
