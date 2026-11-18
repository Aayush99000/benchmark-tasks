"""F2 manual changeover functional equivalent, independent diagnostic only.

Published twin-dispenser sources establish one-roll-at-a-time sliding panels.
This frame/rail/panel is an original engineering equivalent, not a measured OEM
mechanism. The six imported baseline modules remain unchanged. All geometry is
prepared before reset; callbacks only read actual poses and latch failure.
"""
import numpy as np
import dispenser_device as device
from surface_access_geometry import Outlet
mk,kit=device.mk,device.kit
ORIGINAL_PAPER_IN_PARENT=device.paper._in_parent.copy()
# Retain original cover shell/hinge/real latch, supports, roll dimensions/masses,
# paper roots/material and loose closingtool. Remove complete old auto-reset topology.
for name in ('shutter_slide','reset_pawl_slide'):
    del kit.joints_[name]
for name in ('shutter','reset_pawl','reset_cam'):
    del kit.parts[name]
device.FIXTURES=[x for x in device.FIXTURES if x.name!='rear_bottom_tray']
SLOT_X=(.296,.394)
SLOT_Y={'right':(-.225,-.015),'left':(.015,.225)}
PLATE_X=(.285,.395)
PANEL_WIDTH=.224
PANEL_Z=(.8605,.8635)
FRAME_Z=(.8645,.8675)
PANEL_REF=np.array([sum(PLATE_X)/2,-.120,sum(PANEL_Z)/2])
PANEL_DENSITY=1050.  # ABS engineering density, not an OEM material measurement.
PANEL_RAIL_FRICTION=.150  # N Coulomb engineering estimate; not tuned to a result.
PANEL_DAMPING=.30  # N*s/m; no return spring and no conditional drive switching.
frame=[]
FRAME_BOXES=[]
def add_frame(name,lo,hi):
    lo,hi=np.asarray(lo,float),np.asarray(hi,float)
    FRAME_BOXES.append((name,lo.copy(),hi.copy()))
    frame.append(mk.box(hi-lo,at=(lo+hi)/2))
# Integral fixed supportframe. New front bearing is open-sided: the standard
# prismatic rail carriage models captured lateral guidance, without a detent.
for name,lo,hi in (
    ('front_rim',(.290,-.237,.8645),(.296,.237,.8675)),
    ('rear_rim',(.394,-.237,.8645),(.400,.237,.8675)),
    ('right_rim',(.290,-.237,.8645),(.400,-.225,.8675)),
    ('left_rim',(.290,.225,.8645),(.400,.237,.8675)),
    ('center_bridge',(.290,-.015,.8645),(.400,.015,.8675)),
    ('front_lower_track',(.283,-.237,.8585),(.297,.237,.8605)),
    ('back_lower_track',(.393,-.237,.8585),(.397,.237,.8605)),
    ('back_track_wall',(.397,-.237,.8585),(.399,.237,.8665)),
    ('right_endstop',(.283,-.236,.8595),(.397,-.232,.8645)),
    ('left_endstop',(.283,.232,.8595),(.397,.236,.8645)),
):add_frame(name,lo,hi)
kit.part('manual_frame',(0,0,0),frame,color=device.WHITE,contact_offset=.001)
# The thumbtab is a plain23mm external thumbtab joined by a real2mm bridge over the front track.
# No invisibleactuator/catch/return spring or cover-cam is authored.
PANEL_BOXES=[mk.box((PLATE_X[1]-PLATE_X[0],PANEL_WIDTH,.003)),
    mk.box((.004,.020,.023),at=(.280-PANEL_REF[0],0,.854-PANEL_REF[2])),
    mk.box((.009,.020,.002),at=(.2825-PANEL_REF[0],0,.8645-PANEL_REF[2]))]
PANEL_MASS=float(sum(np.prod(b.size)for b in PANEL_BOXES)*PANEL_DENSITY)
kit.part('shutter',PANEL_REF,PANEL_BOXES,mass=PANEL_MASS,color=device.RED,contact_offset=.001)
kit.slider('shutter_slide',None,'shutter',axis=(0,1,0),lower=0.,upper=.240,start=0.,
    stiffness=0.,rest=0.,damping=PANEL_DAMPING,friction=PANEL_RAIL_FRICTION)
device.paper.outlet=Outlet(SLOT_X[0],SLOT_X[1],SLOT_Y['left'][0],SLOT_Y['left'][1],device.ZB)
assert np.array_equal(device.paper._in_parent,ORIGINAL_PAPER_IN_PARENT)

def panel_state(objects):
    pose=kit._pose_T(objects,'shutter')
    normal=pose[:3,2]
    tilt=float(np.degrees(np.arccos(np.clip(normal[2],-1.,1.))))
    # Four actual aperture corners projected onto the measured plateplane.
    result=dict(q_m=kit.q(objects,'shutter_slide'),tilt_deg=tilt,normal_valid=tilt<2.,blocked={},fully_open={})
    bounds=np.array([[-(PLATE_X[1]-PLATE_X[0])/2,-PANEL_WIDTH/2],[(PLATE_X[1]-PLATE_X[0])/2,PANEL_WIDTH/2]])
    corners={}
    for name,(a,b) in SLOT_Y.items():
        world=np.array([[x,y,pose[2,3]]for x in SLOT_X for y in(a,b)])
        local=(world-pose[:3,3])@pose[:3,:3]
        corners[name]=local[:,:2]
        result['blocked'][name]=bool(tilt<2. and np.all(local[:,:2]>=bounds[0]-1e-7)and np.all(local[:,:2]<=bounds[1]+1e-7))
        # A worldY separatingaxis is a conservative completeopening witness,
        # using actual physicalpanelvertices, including thevisiblethumbtab.
        verts=[]
        for box in kit.parts['shutter'].boxes:
            for signs in __import__('itertools').product((-.5,.5),repeat=3):
                p=np.asarray(box.at)+np.asarray(box.size)*signs;verts.append(pose[:3,:3]@p+pose[:3,3])
        verts=np.asarray(verts)
        result['fully_open'][name]=bool(tilt<2. and(verts[:,1].max()<=a+1e-7 or verts[:,1].min()>=b-1e-7))
    result['stub_first_open']=bool(result['fully_open']['left']and result['blocked']['right'])
    result['new_first_open']=bool(result['fully_open']['right']and result['blocked']['left'])
    return result

def changed_over(objects):
    return panel_state(objects)['stub_first_open']
device.changed_over=changed_over
device.OBJECTS=[*kit.objects(),device.paper.marker,*device.FIXTURES]
import closure_tool
OBJECTS=closure_tool.OBJECTS
