"""All dimensions in metres, camera body frame; insertion is along -Z.
Guide running clearance: 0.2 mm per side at the seated shoe.
"""
# Rear shoe: asymmetric 20 mm wide tongue, 4 mm thick, 24 mm long.
# All rails are behind the optical housing, never in front of the lens.
SHOE=[('shoe_main',(0,-.022,-.010),(.020,.004,.020)),
      ('shoe_leading',(-.0015,-.022,-.022),(.017,.004,.004)),
      ('shoe_latch_top',(.009,-.022,-.002),(.002,.004,.004)),
      ('shoe_latch_bottom',(.009,-.022,-.017),(.002,.004,.006))]
SHOE.append(('shoe_bridge',(0,-.019,-.005),(.008,.002,.014)))
# Right edge notch occupies x[8,10], z[-14,-4] mm (main core width18).
SHOE[0]=('shoe_core',(-.001,-.022,-.010),(.018,.004,.020))
MOUNT=[('channel_left',(-.013,-.022,-.012),(.004,.008,.028)),
       ('channel_right',(.013,-.022,-.012),(.004,.008,.028)),
       ('channel_back',(0,-.027,-.012),(.030,.002,.028)),
       ('channel_front_left',(-.009,-.0185,-.012),(.008,.001,.028)),
       ('channel_front_right',(.009,-.0185,-.012),(.008,.001,.028)),
       ('end_stop',(0,-.022,-.027),(.030,.012,.004))]
# Spring tooth engages the right-side notch at z=-9 mm.
# Tooth motion is contact-driven; these are geometric reference positions.
TOOTH_RETRACTED=(.014,-.022,-.009)
TOOTH_ENGAGED=(.0095,-.022,-.009)
TOOTH_SIZE=(.003,.003,.004)

# Keep housing clear of rail lips: translated rear interface, unchanged optical pose.
SHOE=[(n,(p[0],p[1]-.003,p[2]),sz) for n,p,sz in SHOE if n!='shoe_bridge']
SHOE.append(('shoe_bridge',(0,-.0205,-.005),(.008,.005,.014)))
MOUNT=[(n,(p[0],p[1]-.003,p[2]),sz) for n,p,sz in MOUNT]
TOOTH_RETRACTED=(TOOTH_RETRACTED[0],TOOTH_RETRACTED[1]-.003,TOOTH_RETRACTED[2])
TOOTH_ENGAGED=(TOOTH_ENGAGED[0],TOOTH_ENGAGED[1]-.003,TOOTH_ENGAGED[2])

# Tight running guides: 0.2 mm side/depth clearance; target optics unchanged.
MOUNT=[(n, ((-.0122,p[1],p[2]) if n=='channel_left' else
           (.0122,p[1],p[2]) if n=='channel_right' else
           (p[0],-.0282,p[2]) if n=='channel_back' else
           (p[0],-.0223,p[2]) if n.startswith('channel_front') else
           (p[0],p[1],-.0261) if n=='end_stop' else p), sz) for n,p,sz in MOUNT]

class InsertionGate:
 """Record continuous aligned insertion; physical retention is checked separately."""
 def __init__(self):
  self.entered=False;self.previous=None;self.latched=False;self.travel=0.;self.seated_samples=0
 def update(self,offset,angle_rad,speed):
  import math
  x,y,z=map(float,offset)
  corridor=abs(x)<=.0012 and abs(y)<=.0012 and angle_rad<=math.radians(3)
  if not self.latched:
   discontinuous=self.previous is not None and abs(z-self.previous[2])>.003
   if not corridor or discontinuous:
    self.entered=False;self.travel=0.;self.seated_samples=0
   elif not self.entered and .026<=z<=.050:
    self.entered=True;self.travel=0.
   elif self.entered:
    if self.previous is not None:self.travel+=max(0.,self.previous[2]-z)
    seated=abs(z)<=.0012 and speed<.03 and self.travel>=.025
    self.seated_samples=self.seated_samples+1 if seated else 0
    if self.seated_samples>=4:self.latched=True
  self.previous=(x,y,z)
  return self.latched
