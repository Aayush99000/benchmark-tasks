"""Unchanged F6 device on floor-supported open counter; construction transforms only."""
from dataclasses import replace
import sys,numpy as np
import closed_finger_initial
import side_hinge_hardware as hardware
import dispenser_device as d
import manual_frame_hardware as frame
import closure_tool
from yam_arena import station
from yam_arena.task_file import Box
from surface_access_geometry import Outlet
DELTA=np.array([.10,0.,-.08]);TOP=station.TABLE_CENTER[2]+station.TABLE_SIZE[2]/2
for name,part in d.kit.parts.items():
 if name!='roll':part.ref[:3,3]+=DELTA
for joint in d.kit.joints_.values():joint.anchor+=DELTA
for name in('XW','FRONT','SUPPORT_STOP_X','SUPPORT_FRONT_X','OUTLET_BACK','TAIL_X0'):setattr(d,name,getattr(d,name)+DELTA[0])
for name in('ZB','ZT','Z_AX','SEATED_Z'):setattr(d,name,getattr(d,name)+DELTA[2])
d.HINGE=tuple(float(v)for v in np.asarray(d.HINGE)+DELTA);d.STUB0=tuple(float(v)for v in np.asarray(d.STUB0)+DELTA)
frame.SLOT_X=tuple(x+DELTA[0]for x in frame.SLOT_X);frame.PANEL_REF=frame.PANEL_REF+DELTA
old=d.paper.outlet;d.paper.outlet=Outlet(old.x_min+DELTA[0],old.x_max+DELTA[0],old.y_min,old.y_max,old.z+DELTA[2])
d.paper.marker=replace(d.paper.marker,pos=tuple(float(v)for v in np.asarray(d.paper.marker.pos)+DELTA))
fixtures=[]
for item in d.FIXTURES:
 shift=np.zeros(3)if item.name=='carton'else np.array([.10,0.,0.])if item.name=='wall_board'else DELTA
 fixtures.append(replace(item,pos=tuple(float(v)for v in np.asarray(item.pos)+shift)))
for side,sgn in(('left',1),('right',-1)):
 fixtures.append(Box(side+'_platform',pos=(.36,sgn*.4275,TOP-.03),size=(1.1,.445,.06),color=d.BOARD))
 for i,(x,y)in enumerate(__import__('itertools').product((-.13,.85),(.245,.61))):fixtures.append(Box(side+'_leg_'+str(i),pos=(x,sgn*y,(TOP-.06)/2),size=(.06,.06,TOP-.06),color=d.GREY))
fixtures.append(Box('floor_service_backboard',pos=(.516,0.,TOP/2),size=(.020,.410,TOP),color=d.BOARD))
d.FIXTURES=fixtures;d.OBJECTS=[*d.kit.objects(),d.paper.marker,*fixtures];OBJECTS=[*d.OBJECTS,*closure_tool.toolkit.objects()]
def patch_open_counter(mod):
 if getattr(mod,'_f6_open_gap_counter',False):return
 mod._f6_open_gap_counter=True;original=mod.task_room
 def room(scene,top_cam):
  assets=original(scene,top_cam)
  if scene=='station':
   if sum(getattr(a,'name',None)=='table'for a in assets)!=1:raise RuntimeError('Expected one original station table')
   return[a for a in assets if getattr(a,'name',None)!='table']
  return assets
 mod.task_room=room
if 'yam_arena.scene'in sys.modules:patch_open_counter(sys.modules['yam_arena.scene'])
else:
 for finder in sys.meta_path:
  if isinstance(finder,d.mk._AfterImport)and finder.name=='yam_arena.scene':
   previous=finder.patch
   def both(mod,_first=previous):_first(mod);patch_open_counter(mod)
   finder.patch=both;break
 else:raise RuntimeError('Missing scene import hook')
