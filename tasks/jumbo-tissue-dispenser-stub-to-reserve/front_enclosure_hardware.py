"""F3fixedfrontapronsandbottombridges; independentengineeringcandidate.

OriginalF2physicalparts, paper and scorer are copied unchanged. These two real
compoundfixtures close the oldfrontwindows andconnectthemto thebottomframe.
The2mmbridge underside touches, ratherthan intersects, theF2thumbtabbridge top.
No cover collider is removed and no runtime writer/filter is introduced.
"""
import manual_frame_hardware as baseline
import dispenser_device as device
import closure_tool
mk,kit=device.mk,device.kit
APRON_X=(.272,.276)
APRON_Z=(.8655,.9515)
BOTTOM_BRIDGE_X=(.272,.296)
BOTTOM_BRIDGE_Z=(.8655,.8675)
BOXES=[]
parts=[]
for side,(a,b)in baseline.SLOT_Y.items():
    for label,x,z in (('front_apron',APRON_X,APRON_Z),
            ('bottom_bridge',BOTTOM_BRIDGE_X,BOTTOM_BRIDGE_Z)):
        lo=(x[0],a,z[0]);hi=(x[1],b,z[1]);BOXES.append((side+'_'+label,lo,hi))
        parts.append(mk.box(tuple(h-l for l,h in zip(lo,hi)),
            at=tuple((h+l)/2 for l,h in zip(lo,hi))))
kit.part('front_enclosure',(0.,0.,0.),parts,color=device.WHITE,contact_offset=.001)
device.OBJECTS=[*kit.objects(),device.paper.marker,*device.FIXTURES]
closure_tool.OBJECTS=[*device.OBJECTS,*closure_tool.toolkit.objects()]
OBJECTS=closure_tool.OBJECTS
