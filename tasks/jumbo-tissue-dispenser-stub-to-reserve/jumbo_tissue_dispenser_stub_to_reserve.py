"""Fullordinaryservice,newphysicalPOMclosure/release,andmanualselectorchangeover."""
from pathlib import Path
import sys,json,os,numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parent))
import pom_material_scene as hardware
d=hardware.d;OBJECTS=hardware.OBJECTS
from yam_arena import ik,station
from yam_arena.constants import READY,FINGER_OPEN
from yam_arena.demo import ArmPlan,Waypoint
from surface_paper import _numpy
import raw_geometry
import material_readback
SCENE='station';SCORE_OVER='end';SUCCESS_HOLD_S=1.;score=d.score;success=d.success
ROOT=Path(__file__).resolve().parent;TARGETS=np.loadtxt(ROOT/'closure17.csv',delimiter=',');LABEL='jumbo_dispenser_final_reference'
VARIANT=os.environ.get('DEMO_VARIANT','reference') or 'reference'
if VARIANT not in('reference','held_final','left_unavailable','paper_inside'):raise ValueError('Unknown ordinary control variant')
# Counterexample trajectories are ordinary robot policy inputs. Default
# reference and Dream environments never read an external override file.
NEGATIVE_ACTIONS={};NEGATIVE_POLICY_INPUT=None
if VARIANT in('left_unavailable','paper_inside'):
 import hashlib as _policy_hash
 policy_path=Path(os.environ['ROBOCURVE_NEGATIVE_ACTION_FILE']).resolve()
 policy_sha=os.environ['ROBOCURVE_NEGATIVE_ACTION_SHA256']
 if len(policy_sha)!=64 or _policy_hash.sha256(policy_path.read_bytes()).hexdigest()!=policy_sha:raise ValueError('Negative policy input SHA mismatch')
 policy_rows=np.atleast_2d(np.loadtxt(policy_path,delimiter=','))
 if policy_rows.shape[1]!=15 or not np.isfinite(policy_rows).all():raise ValueError('Negative policy input requires finite control plus14targets')
 controls=policy_rows[:,0]
 if not(np.all(controls==np.floor(controls))and np.all(controls>=1)and np.all(controls<=6004)and np.all(np.diff(controls)>0)):raise ValueError('Negative controls must be unique increasing integers1to6004')
 for arm_start in(1,8):
  joint=policy_rows[:,arm_start:arm_start+6];finger=policy_rows[:,arm_start+6]
  if not(np.all(joint>=ik.LOWER)and np.all(joint<=ik.UPPER)and np.all(finger>=0.)and np.all(finger<=1.)):raise ValueError('Negative ordinary joint or finger target out of bounds')
 NEGATIVE_ACTIONS={int(row[0]):row[1:].copy()for row in policy_rows}
 NEGATIVE_POLICY_INPUT=dict(variant=VARIANT,path=str(policy_path),sha256=policy_sha,rows=len(policy_rows),first_control=int(controls[0]),last_control=int(controls[-1]),ordinary_robot_actions_only=True)
 print('NEGATIVE_POLICY_INPUT '+json.dumps(NEGATIVE_POLICY_INPUT),flush=True)
ANGLE=90.;BOLT=0.;LAST=-1;LEAD=.8;ANCHOR=90.;PROGRESS_SLOW=True;COMMAND_LEFT=np.r_[READY,0.];MODE='service';CLOSE_SINCE=None;RELEASE_START=None;RELEASE_Q=None;RELEASE_P=None;RELEASE_R=None;RELEASE_IK=None;PRESS_M=0.;PRESS_IK=None
PRESS_BASE_P,PRESS_BASE_R=ik.tcp_pose(TARGETS[-1],station.BASES['left'])
import hashlib
from dataclasses import asdict
SERVICE_PLAN=json.loads((ROOT/'service-plan.json').read_text());PANEL_PLAN=json.loads((ROOT/'panel-plan.json').read_text())
for plan in(SERVICE_PLAN,PANEL_PLAN):
 if hashlib.sha256((ROOT/plan['action_file']).read_bytes()).hexdigest()!=plan['sha256']:raise RuntimeError('Changedordinaryjointcommandtable')
SERVICE=np.loadtxt(ROOT/SERVICE_PLAN['action_file'],delimiter=',');PANEL=np.loadtxt(ROOT/PANEL_PLAN['action_file'],delimiter=',')
FEEDBACK_PLAN=json.loads((ROOT/SERVICE_PLAN['residual_feedback_plan_file']).read_text())
if hashlib.sha256((ROOT/SERVICE_PLAN['residual_feedback_plan_file']).read_bytes()).hexdigest()!=SERVICE_PLAN['residual_feedback_plan_sha256']:raise RuntimeError('Changedresidualordinaryfeedbacktargets')
RESIDUAL_TARGETS={r['control']:r for r in FEEDBACK_PLAN['targets']};LATEST_RESIDUAL=None;LAST_RESIDUAL_COMMAND=None;RESIDUAL_REFERENCE_REPORT=None
def residual_reference(row,control):
 global LAST_RESIDUAL_COMMAND,RESIDUAL_REFERENCE_REPORT
 if control not in RESIDUAL_TARGETS or LATEST_RESIDUAL is None:return row
 actual_q,actual_stub=LATEST_RESIDUAL;current_hand_p,current_hand_R=ik.tcp_pose(actual_q,station.BASES['right']);hand=np.eye(4);hand[:3,:3]=current_hand_R;hand[:3,3]=current_hand_p
 relative=np.linalg.inv(hand)@actual_stub;target=RESIDUAL_TARGETS[control];goal_p=np.asarray(target['stub_target_position']);goal_R=Rotation.from_quat(target['stub_target_quaternion']).as_matrix();axis=-actual_stub[:3,2];radial=float(np.linalg.norm(actual_stub[1:3,3]-goal_p[1:3]));tilt=float(np.degrees(np.arccos(np.clip(axis[0],-1,1))));entry_hold=False
 if actual_stub[0,3]>.410 and (radial>FEEDBACK_PLAN['entry_radial_tolerance_m']or tilt>FEEDBACK_PLAN['entry_axis_tolerance_deg']):
  goal_p[0]=min(goal_p[0],actual_stub[0,3]);entry_hold=True
 delta=np.clip(goal_p-actual_stub[:3,3],-FEEDBACK_PLAN['translation_correction_limit_m'],FEEDBACK_PLAN['translation_correction_limit_m']);rotation=Rotation.from_matrix(goal_R@actual_stub[:3,:3].T).as_rotvec();norm=np.linalg.norm(rotation)
 if norm>FEEDBACK_PLAN['rotation_correction_limit_rad']:rotation*=FEEDBACK_PLAN['rotation_correction_limit_rad']/norm
 corrected=np.eye(4);corrected[:3,:3]=Rotation.from_rotvec(rotation).as_matrix()@actual_stub[:3,:3];corrected[:3,3]=actual_stub[:3,3]+delta;desired_hand=corrected@np.linalg.inv(relative)
 q,pe,re=ik.solve(desired_hand[:3,3],ik.matrix_to_quat_xyzw(desired_hand[:3,:3]),actual_q,base_pos=station.BASES['right'],iters=80,tol=1e-6)
 com=(relative@np.array([0.,0.,.08636/2,1.]))[:3]
 def height(q):
  p,R=ik.tcp_pose(q,station.BASES['right']);return(p+R@com)[2]
 eps=1e-6;gradient=np.array([(height(q+np.eye(6)[i]*eps)-height(q-np.eye(6)[i]*eps))/(2*eps)for i in range(6)]);offset=np.clip(FEEDBACK_PLAN['mass_kg']*FEEDBACK_PLAN['gravity']*gradient/np.asarray(FEEDBACK_PLAN['joint_stiffness'])+FEEDBACK_PLAN['payload_bias'],-FEEDBACK_PLAN['payload_offset_limit_rad'],FEEDBACK_PLAN['payload_offset_limit_rad'])
 command=q+offset;limit=FEEDBACK_PLAN['reference_delta_limit_rad'];command=row[7:13]+np.clip(command-row[7:13],-limit,limit)
 previous=row[7:13]if LAST_RESIDUAL_COMMAND is None else LAST_RESIDUAL_COMMAND;step=FEEDBACK_PLAN['command_step_limit_rad'];command=previous+np.clip(command-previous,-step,step);command=np.clip(command,ik.LOWER+1e-7,ik.UPPER-1e-7);LAST_RESIDUAL_COMMAND=command.copy();out=row.copy();out[7:13]=command
 RESIDUAL_REFERENCE_REPORT=dict(control=control,radial_error_mm=radial*1000,axis_error_deg=tilt,entry_hold=entry_hold,position_correction_m=delta.tolist(),rotation_correction_rad=rotation.tolist(),payload_offset=offset.tolist(),IK_mm=pe*1000)
 return out
CLOSURE_START=None;PHASE_START=0;PANEL_ENTRY_LEFT=None;PANEL_ENTRY_RIGHT=None;FINAL_LEFT=None;COMMAND14=np.r_[READY,0.,READY,0.]
LATEST_RIGHT_Q=None;HELD_RELEASE_Q=None;RETREAT_P0=None;RETREAT_R0=None;RETREAT_PREVIOUS=None;BRIDGE_START=None;HELD_REFERENCE_REPORT=None
def held_release_reference(row,control):
 global HELD_RELEASE_Q,RETREAT_P0,RETREAT_R0,RETREAT_PREVIOUS,BRIDGE_START,HELD_REFERENCE_REPORT
 if not(901<=control<=1120):return row
 out=row.copy()
 if HELD_RELEASE_Q is None:HELD_RELEASE_Q=LAST_RESIDUAL_COMMAND.copy()
 if control<=960:command=HELD_RELEASE_Q.copy()
 elif control<=1060:
  if RETREAT_P0 is None:
   RETREAT_P0,RETREAT_R0=ik.tcp_pose(LATEST_RIGHT_Q,station.BASES['right']);RETREAT_PREVIOUS=HELD_RELEASE_Q.copy()
  fraction=ik.smoothstep((control-960)/100.);target=RETREAT_P0+np.array([-.070*fraction,0.,0.])
  solved,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(RETREAT_R0),LATEST_RIGHT_Q,base_pos=station.BASES['right'],iters=100,tol=1e-6)
  command=RETREAT_PREVIOUS+np.clip(solved-RETREAT_PREVIOUS,-.04,.04);RETREAT_PREVIOUS=command.copy()
  HELD_REFERENCE_REPORT=dict(control=control,phase='ordinary_actual_encoder_axial_retreat',target=target.tolist(),IK_mm=pe*1000,IK_rad=re)
 else:
  if BRIDGE_START is None:BRIDGE_START=RETREAT_PREVIOUS.copy()
  fraction=ik.smoothstep((control-1060)/60.);command=BRIDGE_START*(1-fraction)+SERVICE[1119,7:13]*fraction
 out[7:13]=np.clip(command,ik.LOWER+1e-7,ik.UPPER-1e-7);return out

original=ArmPlan.action7
def dispatch(plan,time):
 global COMMAND_LEFT,RELEASE_Q,RELEASE_IK,PRESS_IK,COMMAND14
 if not plan.waypoints or plan.waypoints[0].label!=LABEL:return original(plan,time)
 left=np.allclose(plan.base_pos,station.BASES['left']);idx=0 if left else 7
 control=max(1,round(time*20))
 if control in NEGATIVE_ACTIONS:
  COMMAND14=NEGATIVE_ACTIONS[control].copy();return COMMAND14[idx:idx+7].copy(),LABEL
 if MODE=='service':
  index=min(len(SERVICE)-1,max(0,round(time*20)-1));COMMAND14=SERVICE[index].copy()
  if not left:COMMAND14=held_release_reference(residual_reference(COMMAND14,index+1),index+1)
  return COMMAND14[idx:idx+7].copy(),LABEL
 if MODE in('panel_entry','panel_left','panel_right_entry','panel_right','final_park','final_hold'):
  elapsed=max(0.,time-PHASE_START/20.);row=np.r_[READY,0.,READY,0.]
  if MODE=='panel_entry':
   f=ik.smoothstep(min(1.,elapsed/4.));row[:6]=PANEL_ENTRY_LEFT*(1-f)+PANEL[0,:6]*f
  elif MODE=='panel_left':row=PANEL[min(319,max(0,round(elapsed*20)-1))].copy()
  elif MODE=='panel_right_entry':
   f=ik.smoothstep(min(1.,elapsed/4.));row[:6]=PANEL[319,:6];row[7:13]=PANEL_ENTRY_RIGHT*(1-f)+PANEL[320,7:13]*f
  elif MODE=='panel_right':row=PANEL[min(len(PANEL)-1,320+max(0,round(elapsed*20)-1))].copy()
  elif MODE=='final_park':
   f=ik.smoothstep(min(1.,elapsed/4.));row[:6]=FINAL_LEFT*(1-f)+np.asarray(READY)*f
  COMMAND14=row;return row[idx:idx+7].copy(),LABEL
 if not left:return np.r_[READY,0.],LABEL
 closure_time=max(0.,time-CLOSURE_START)
 if MODE in('withdraw','released_hold'):
  elapsed=max(0.,time-RELEASE_START);s=ik.smoothstep(min(1.,elapsed/2.));target=RELEASE_P+np.array([-.050*s,0.,0.])
  def residual(q):
   p,R=ik.tcp_pose(q,station.BASES['left']);return np.r_[(p-target)*100.,(R[:,2]-RELEASE_R[:,2])*.02,(q-RELEASE_Q)*.002]
  solved=least_squares(residual,np.clip(RELEASE_Q,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=100);q=solved.x;p,R=ik.tcp_pose(q,station.BASES['left']);pe=np.linalg.norm(p-target);RELEASE_Q=q;RELEASE_IK=dict(position_mm=pe*1000,actual_displacement_mm=float(np.linalg.norm(p-RELEASE_P)*1000),orientation_soft=True)
 elif MODE=='press':
  target=PRESS_BASE_P+np.array([PRESS_M,0.,0.]);previous=COMMAND_LEFT[:6].copy()
  def residual(q):
   p,R=ik.tcp_pose(q,station.BASES['left']);return np.r_[(p-target)*100.,(R[:,2]-PRESS_BASE_R[:,2])*.1,(q-previous)*.002]
  solved=least_squares(residual,np.clip(previous,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=100);q=solved.x;p,R=ik.tcp_pose(q,station.BASES['left']);PRESS_IK=dict(position_mm=float(np.linalg.norm(p-target)*1000))
 elif closure_time<4.:
  s=ik.smoothstep(max(0.,closure_time/4.));q=np.asarray(READY)*(1-s)+TARGETS[0]*s
 else:
  a=float(np.clip(ANGLE-LEAD,0.,90.));u=(90.-a)/90.*16;i=min(15,int(u));s=u-i;q=TARGETS[i]*(1-s)+TARGETS[i+1]*s
 q=COMMAND_LEFT[:6]+np.clip(q-COMMAND_LEFT[:6],-.04,.04);COMMAND_LEFT=np.r_[q,0.];return COMMAND_LEFT.copy(),LABEL
ArmPlan.action7=dispatch
def demo():return{a:[Waypoint(300.2,tuple(ik.tcp_pose(READY,station.BASES[a])[0]),FINGER_OPEN,LABEL)]for a in('left','right')}
OUT=Path(os.environ.get('ROBOCURVE_TRACE_DIR',str(ROOT.parents[2]/'observations')))
def observe(objects):
 global ANGLE,LAST,LEAD,ANCHOR,PROGRESS_SLOW,BOLT,MODE,CLOSE_SINCE,RELEASE_START,RELEASE_Q,RELEASE_P,RELEASE_R,PRESS_M,CLOSURE_START,PHASE_START,PANEL_ENTRY_LEFT,PANEL_ENTRY_RIGHT,FINAL_LEFT,LATEST_RESIDUAL,LATEST_RIGHT_Q
 n=d.mk._EP.get('step',0)
 if n==LAST:return
 LAST=n;ANGLE=float(d.kit.q_deg(objects,'cover_hinge'));BOLT=float(d.kit.q(objects,'latch_slide'));time=n/20.;base=d.mk._EP['env'].unwrapped
 if 620<=n<=1120:LATEST_RIGHT_Q=_numpy(base.scene['right_robot'].data.joint_pos)[0,:6].copy()
 if 620<=n<=900:
  actual_q=_numpy(base.scene['right_robot'].data.joint_pos)[0,:6].copy();stub_p,stub_q=d.kit.pose(objects,'stub');stub_T=np.eye(4);stub_T[:3,:3]=Rotation.from_quat(stub_q).as_matrix();stub_T[:3,3]=stub_p;LATEST_RESIDUAL=(actual_q,stub_T)
 if MODE=='service'and n>=len(SERVICE):MODE='approach';CLOSURE_START=time;ANCHOR=ANGLE
 if CLOSURE_START is not None and time>=CLOSURE_START+4. and MODE in('approach','push','press'):
  if MODE!='press':MODE='push'
  if n%20==0:progress=ANCHOR-ANGLE;ANCHOR=ANGLE;PROGRESS_SLOW=progress<.10
  if PROGRESS_SLOW and MODE=='push':LEAD=min(12.,LEAD+.02)
  if (MODE=='press' or (ANGLE<12. and LEAD>=ANGLE and PROGRESS_SLOW)):
   MODE='press';PRESS_M=min(.015,PRESS_M+.00025)
  if abs(ANGLE)<=.8 and BOLT<.003:
   if CLOSE_SINCE is None:CLOSE_SINCE=n
   if n-CLOSE_SINCE>=10 and VARIANT!='held_final':
    actual=_numpy(base.scene['left_robot'].data.joint_pos)[0];RELEASE_Q=actual[:6].copy();RELEASE_P,RELEASE_R=ik.tcp_pose(RELEASE_Q,station.BASES['left']);RELEASE_START=time;MODE='withdraw'
  else:CLOSE_SINCE=None
 if MODE=='withdraw'and time-RELEASE_START>=2.:MODE='released_hold'
 if MODE=='released_hold'and time-RELEASE_START>=3. and d.cover_locked(objects):
  MODE='final_park';PHASE_START=n;FINAL_LEFT=_numpy(base.scene['left_robot'].data.joint_pos)[0,:6].copy()
 if MODE=='panel_entry'and n-PHASE_START>=80:MODE='panel_left';PHASE_START=n
 if MODE=='panel_left'and n-PHASE_START>=320:MODE='panel_right_entry';PHASE_START=n;PANEL_ENTRY_RIGHT=_numpy(base.scene['right_robot'].data.joint_pos)[0,:6].copy()
 if MODE=='panel_right_entry'and n-PHASE_START>=80:MODE='panel_right';PHASE_START=n
 if MODE=='panel_right'and n-PHASE_START>=len(PANEL)-320:MODE='final_park';PHASE_START=n;FINAL_LEFT=_numpy(base.scene['left_robot'].data.joint_pos)[0,:6].copy()
 if MODE=='final_park'and n-PHASE_START>=80:MODE='final_hold';PHASE_START=n
 poses={}
 for name in('cover','stub','roll','latch_bolt','shutter'):
  p,q=d.kit.pose(objects,name);poses[name]=dict(position=p.tolist(),quaternion=q.tolist())
 robots={}
 if n%20==0 or n==1:
  for arm in('left','right'):
   robot=base.scene[arm+'_robot'];data=robot.data;a=dict(body_names=list(robot.body_names),joint_names=list(robot.joint_names),joint_pos=_numpy(data.joint_pos)[0].tolist(),body_link_pose_w=_numpy(data.body_link_pose_w)[0].tolist(),torque_readback={})
   for field in('applied_torque','computed_torque','joint_effort_target','joint_vel'):
    if hasattr(data,field):
     try:a['torque_readback'][field]=_numpy(getattr(data,field))[0].tolist()
     except Exception as error:a['torque_readback'][field+'_unavailable']=type(error).__name__
   robots[arm]=a
 raw_geometry.capture(base=base,device=d,output=OUT,control=n)
 material_readback.capture(device=d,output=OUT,control=n)
 paper_access=None
 if (n%20==0 or n==1)and d.paper.snapshot is not None:paper_access=asdict(d.paper.snapshot.access)
 if d.paper.snapshot is not None and (n in(1,200,250,310,340,640,900,1060,1360,1580,960,1060,1120,3900,6004)or(MODE=='final_hold'and n%100==0)):
  # Read-only complete native paper evidence, including every free-edge vertex.
  OUT.mkdir(exist_ok=True,parents=True);snapshot=d.paper.snapshot
  np.savez_compressed(OUT/f'paper-nodes-{n:06d}.npz',nodes=snapshot.nodes,velocities=snapshot.velocities,stub_transform=snapshot.stub_transform,root_ids=d.paper.root_ids,free_edge_ids=d.paper.free_edge_ids,terminal_patch_ids=d.paper.terminal_patch_ids,faces=d.paper.faces,root_local_anchors=d.paper.root_local_anchors,env_origin=np.asarray(snapshot.env_origin))
 native_seats=dict(stub_left_reserve=bool(d.seated(objects,'stub','p')),roll_right_primary=bool(d.seated(objects,'roll','r')))
 panel=d.changed_over(objects)
 observed_score=float(score(objects))
 # success() is exactly score(objects)>=1.-1e-9; reuse within this callback only.
 row=dict(control=n,phase=MODE,policy_variant=VARIANT,negative_policy_input=NEGATIVE_POLICY_INPUT,phase_start_control=PHASE_START,service_controls=len(SERVICE),panel_controls=len(PANEL),native_seats=native_seats,panel_changed_over=bool(panel),paper_access=paper_access,full_success=bool(observed_score>=1.-1e-9),cover_angle_deg=ANGLE,latch_slide_m=BOLT,target_lead_deg=LEAD,final_press_m=PRESS_M,final_press_IK=PRESS_IK,closed_hold_controls=0 if CLOSE_SINCE is None else n-CLOSE_SINCE,withdrawal_elapsed_s=None if RELEASE_START is None else time-RELEASE_START,release_IK=RELEASE_IK,original_cover_locked=bool(d.cover_locked(objects)),score=observed_score,component_poses=poses,robots=robots,most_recent_left_command7=COMMAND_LEFT.tolist(),command_readback_phase='Mostrecentdispatcher target; callbackmayprecedenextselection.',boundary='Canonicalcomplete1samephysicalflatprethreadsource20: shiftedordinaryresidual/refillservice, nativePOMlatchclosurewithrelease, ordinarytwomanualselectorpushes, actualnativepaper/root/topology/accessandsettling. Nophysicalsourcechangesfromprovedmechanism8; oldgradesnotinherited.')
 OUT.mkdir(exist_ok=True,parents=True)
 with(OUT/'closure-observations.jsonl').open('a')as f:f.write(json.dumps(row,default=lambda v:v.tolist()if hasattr(v,'tolist')else float(v))+'\n')
 if n%20==0:
  if HELD_REFERENCE_REPORT is not None and 900<=n<=1120:print('HELD_ORDINARY_REFERENCE '+json.dumps(HELD_REFERENCE_REPORT),flush=True)
  if RESIDUAL_REFERENCE_REPORT is not None and 640<=n<=920:print('RESIDUAL_ORDINARY_REFERENCE '+json.dumps(RESIDUAL_REFERENCE_REPORT),flush=True)
  print('JUMBO_FINAL '+json.dumps(row,default=lambda v:v.tolist()if hasattr(v,'tolist')else float(v)),flush=True)
d.kit.on_step(observe)
