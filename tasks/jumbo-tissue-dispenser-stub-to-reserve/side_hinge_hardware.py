"""F6fixedcabinet with180degreeleftsidehingedfrontdoor.

Originalengineeringfunctionalequivalent, notanOEMclone. Fixedtop/sidesstart at
thefrontdoorinnerfaceX=.272m, providingarealnoninterpenetratingdoorjoint.
Originalsupport/roll/paper/root/material/scoregeometry and0.60kgdoormassremain.
No robotreset/runtimeposewriter isintroduced; documentedclosedfingerreset is
theruntimeintegrator's responsibility.
"""
import math
import itertools
import numpy as np
import fixed_case_hardware as baseline
import dispenser_device as device
import closure_tool
mk,kit=device.mk,device.kit
HINGE_F6=np.array([.270,.245,device.ZB])
joint=kit.joints_['cover_hinge']
joint.anchor=HINGE_F6
joint.axis=np.array([0.,0.,-1.])
joint.upper=math.pi
joint.start=math.pi/2  # One construction-time 90 degree open pose; all other hardware remains F6.
fixed=kit.parts['fixed_case_walls']
new=[]
for b in fixed.boxes:
    low=np.asarray(b.at)-np.asarray(b.size)/2+np.asarray(device.HINGE)
    high=np.asarray(b.at)+np.asarray(b.size)/2+np.asarray(device.HINGE)
    low[0]=.272
    new.append(mk.box(high-low,at=(low+high)/2-np.asarray(device.HINGE),color=b.color))
fixed.boxes=tuple(new)
corners=[np.asarray(b.at)+mk.quat_to_mat(b.quat)@(np.asarray(b.size)*np.asarray(s)/2)
    for b in fixed.boxes for s in itertools.product((-1,1),repeat=3)]
fixed.lo,fixed.hi=np.min(corners,axis=0),np.max(corners,axis=0)
assert kit.parts['cover'].mass==.60
device.OBJECTS=[*kit.objects(),device.paper.marker,*device.FIXTURES]
closure_tool.OBJECTS=[*device.OBJECTS,*closure_tool.toolkit.objects()]
OBJECTS=closure_tool.OBJECTS
