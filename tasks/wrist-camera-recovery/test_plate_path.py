"""Check an actual straight insertion path against non-tooth slider surfaces.

The inclined tooth is intended contact. Its connecting stem/yoke must remain
outside the plate swept volume, or it blocks insertion without retracting.
"""
import numpy as np
from quick_release import BUTTON_PARTS
from latch_geometry import SHOE,TOOTH_ENGAGED
count=0
for offset in np.linspace(0,.035,71):
 for stroke in np.linspace(0,.005,11):
  for pn,pc,ps in SHOE:
   p=np.array(pc)+[0,0,offset];h=np.array(ps)/2
   for name,c,size in BUTTON_PARTS:
    b=np.array(TOOTH_ENGAGED)+np.array(c)+[stroke,0,0];bh=np.array(size)/2
    overlap=np.minimum(p+h,b+bh)-np.maximum(p-h,b-bh)
    assert not np.all(overlap>1e-9),(offset,stroke,pn,name,overlap)
    count+=1
print('PASS:',count,'plate/slider swept-volume comparisons; no non-tooth obstruction')
