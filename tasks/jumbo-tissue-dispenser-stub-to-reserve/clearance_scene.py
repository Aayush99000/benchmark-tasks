"""One-millimetre physical jamb reveal for original four-millimetre overlay door."""
import itertools,numpy as np
import open_gap_scene as base
d=base.d
from yam_arena.task_file import Box
# Existing frontjambs end exactly atcloseddoorback in the older scene. Trim
# their real solids by1mm, keeping their side/top/bottom overlap andrear faces.
# No door, paper, aperture, latch, mass, material, joint or filter changes.
TRIM=[]
for name,indices in(('fixed_case_walls',range(3)),('enclosure_frame',range(3))):
 part=d.kit.parts[name];boxes=list(part.boxes)
 for i in indices:
  old=boxes[i];lo=np.asarray(old.at)-np.asarray(old.size)/2+part.ref[:3,3];hi=np.asarray(old.at)+np.asarray(old.size)/2+part.ref[:3,3]
  if abs(lo[0]-.372)>1e-8:raise RuntimeError('Unexpected untrimmed physical jamb')
  lo[0]=.373;boxes[i]=d.mk.box(hi-lo,at=(lo+hi)/2-part.ref[:3,3],color=old.color);TRIM.append((name,i))
 part.boxes=tuple(boxes);vertices=np.array([np.asarray(b.at)+np.asarray(b.size)*np.asarray(sign)/2for b in boxes for sign in itertools.product((-1,1),repeat=3)]);part.lo=vertices.min(0);part.hi=vertices.max(0)
assert len(d.kit.parts['cover'].boxes)==4 and d.kit.parts['cover'].mass==.60 and d.paper.thickness==.0001
d.OBJECTS=[*d.kit.objects(),d.paper.marker,*d.FIXTURES]
OBJECTS=[*d.OBJECTS,*base.closure_tool.toolkit.objects()]
