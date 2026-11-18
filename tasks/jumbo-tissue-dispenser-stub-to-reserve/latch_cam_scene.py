"""Original engineeringequivalent passive sidehinge springlatch, material-matched reallead."""
import itertools,numpy as np
import clearance_scene as base
d=base.d
# Original square6x6x12mmbolt remains the loadface. Only its connectedleadprism
# changes to a real45degcam extending within the existingreceiverclearspace.
# Firstvertex lies belowthetongueleadingedge so thedoorcorner meets theincline.
part=d.kit.parts['latch_bolt'];square=part.boxes[0]
lead=d.mk.convex([(x,y,z)for z in(-.006,.006)for x,y in((-.016,-.005),(.003,-.005),(.003,.009))],
 [(0,2,1),(3,4,5),(0,1,4),(0,4,3),(1,2,5),(1,5,4),(2,0,3),(2,3,5)])
part.boxes=(square,lead)
vertices=[]
for b in part.boxes:
 v=np.asarray(b.vertices)if hasattr(b,'vertices')and len(b.vertices)else np.array([np.asarray(b.size)*np.asarray(s)/2for s in itertools.product((-1,1),repeat=3)])
 vertices.extend(v@d.mk.quat_to_mat(b.quat).T+np.asarray(b.at))
vertices=np.asarray(vertices);part.lo,part.hi=vertices.min(0),vertices.max(0)
j=d.kit.joints_['latch_slide']
assert part.mass==.020 and j.lower==0. and j.upper==.018 and j.stiffness==400. and j.damping==.8
assert d.kit.parts['cover'].mass==.60 and d.paper.thickness==.0001
d.OBJECTS=[*d.kit.objects(),d.paper.marker,*d.FIXTURES]
OBJECTS=[*d.OBJECTS,*base.base.closure_tool.toolkit.objects()]
