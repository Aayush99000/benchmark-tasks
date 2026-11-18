"""F5fixedboxcase andhingedfrontdoor; one physical engineeringequivalent.

ActualF4closedgeometry is retained. Its top and sides become fixed case walls;
only the completefrontsheet and originalreal latchtongue/rib rotate. Nothing
is removed from the closedhousing and no collision filter/runtime state writes
are introduced. Original dimensions, supports, paper/root/material and scorer
remain unchanged; the complete frontdoor mass remains0.60kg.
"""
import itertools
import numpy as np
import enclosed_hardware as baseline
import dispenser_device as device
import closure_tool
mk,kit=device.mk,device.kit
HINGE_F5=baseline.HINGE_F4.copy()
door=kit.parts['cover'];original=door.boxes
# The actualclosedreferenceframe is shared: transfer boxes withall localposes
# intact toafixedpartat the originalclosedcoverreference origin.
fixed=original[1:4]
kit.part('fixed_case_walls',tuple(device.HINGE),fixed,color=device.SMOKE,contact_offset=.001)
door.boxes=(original[0],*original[4:])
corners=[np.asarray(b.at)+mk.quat_to_mat(b.quat)@(np.asarray(b.size)*np.asarray(s)/2)
    for b in door.boxes for s in itertools.product((-1,1),repeat=3)]
door.lo,door.hi=np.min(corners,axis=0),np.max(corners,axis=0)
assert door.mass==.60 and len(door.boxes)==4
device.OBJECTS=[*kit.objects(),device.paper.marker,*device.FIXTURES]
closure_tool.OBJECTS=[*device.OBJECTS,*closure_tool.toolkit.objects()]
OBJECTS=closure_tool.OBJECTS
