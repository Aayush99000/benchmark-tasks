"""One coherent F4closed shell with realfrontbottomhinge, originalmaterial.

Originalengineeringequivalent only. Fixed bottom/side seams and a complete
frontcover replace the F2openfrontwindow topology. No filter/pose/runtimeflag.
Roll/support/paper/material/scoring constants and cover mass stay unchanged.
"""
import itertools
import numpy as np
import manual_frame_hardware as baseline
import dispenser_device as device
import closure_tool
mk,kit=device.mk,device.kit
HINGE_F4=np.array([.270,0.,device.ZB])
cover=kit.parts['cover'];old=cover.boxes
front=mk.box((.004,device.CASE_W,device.CASE_H),
    at=(.270-device.HINGE[0],0.,(device.ZB+device.ZT)/2-device.HINGE[2]))
# Retain actual top/sides/bolt tongue and continuousupperlatchrib; remove all
# obsolete lowerfrontcutouts/webs, now represented by one solidfrontsheet.
cover.boxes=(front,*old[1:5],*old[11:])
corners=[np.asarray(b.at)+mk.quat_to_mat(b.quat)@(np.asarray(b.size)*np.asarray(s)/2)
    for b in cover.boxes for s in itertools.product((-1,1),repeat=3)]
cover.lo,cover.hi=np.min(corners,axis=0),np.max(corners,axis=0)
kit.joints_['cover_hinge'].anchor=HINGE_F4
assert cover.mass==.60
FIXED_BOXES=[
    ('front_bottom_bridge',(.272,-.245,.8655),(.296,.245,.8675)),
    ('right_bottom_side',(.272,-.245,.8655),(.400,-.237,.8735)),
    ('left_bottom_side',(.272,.237,.8655),(.400,.245,.8735)),
    ('right_rear_seam',(.384,-.245,.8675),(.400,-.241,device.ZT)),
    ('left_rear_seam',(.384,.241,.8675),(.400,.245,device.ZT)),
    ('back_top_seam',(.384,-.245,device.ZT-.004),(.400,.245,device.ZT)),
]
boxes=[mk.box(tuple(h-l for l,h in zip(lo,hi)),at=tuple((h+l)/2for l,h in zip(lo,hi)))
    for _,lo,hi in FIXED_BOXES]
kit.part('enclosure_frame',(0.,0.,0.),boxes,color=device.WHITE,contact_offset=.001)
device.OBJECTS=[*kit.objects(),device.paper.marker,*device.FIXTURES]
closure_tool.OBJECTS=[*device.OBJECTS,*closure_tool.toolkit.objects()]
OBJECTS=closure_tool.OBJECTS
