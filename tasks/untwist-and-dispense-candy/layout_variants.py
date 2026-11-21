"""Alternative full-scale resets for native trials; all material starts outside the goal."""
import numpy as np
import geometry
from pleated_wrapper import build as formed_wrapper


def formed_candy():
    bodies,_,details=geometry.candy()
    surface=formed_wrapper()
    bottom=surface.points[:,2].min()
    dz=max(0.,.748-bottom)
    surface.points[:,2]+=dz
    core=bodies[0]
    core['center']=tuple(np.array(core['center'])+[0.,0.,dz])
    return bodies,[surface],dict(details,strip_width_m=.067,strip_axial_span_m=.080,
                                formation='eight pleats at each twisted end')


def half_fold(s,radius,width):
    """One 180-degree bend in an initially flat textile, parameterized by arc length."""
    straight=(width-np.pi*radius)/2
    theta=np.clip((s-straight)/radius,0.,np.pi)
    excess=np.maximum(s-straight-np.pi*radius,0.)
    transverse=np.minimum(s,straight)+radius*np.sin(theta)-excess
    height=radius*(1.-np.cos(theta))
    return transverse,height,theta


def single_fold_duvet():
    bodies,surfaces,details=geometry.duvet()
    insert,cover=surfaces
    def place(rest,origin_x,origin_y,base_z,radius):
        x,y,offset=rest.T
        transverse,z,angle=half_fold(y,radius,1.35)
        # Keep the panel spacing measured along the folded surface normal.
        return np.column_stack((origin_x+x,origin_y+transverse-offset*np.sin(angle),
                                base_z+z+offset*np.cos(angle)))
    insert.points=place(insert.rest_points,.245,.058,.658,.014)
    # Place the open end 45 mm over the near edge of the mattress so a jaw
    # can pass beneath it. Both textiles remain full size and fully separate.
    cover.points=place(cover.rest_points,.155,-.736,.655,.011)
    return bodies,surfaces,dict(details,reset_fold='one lengthwise half fold per article',
                                cover_mouth_overhang_m=.045)


def front_film_end():
    bodies,surfaces,details=geometry.pallet()
    film=surfaces[0]
    # Start wrapping at a different position around the same stack, ending
    # with the free vertical edge on the accessible front face. The terminal
    # 25 mm of the manufactured edge is loose, with no added tab or cutting.
    w,d=1.15,.77
    perimeter=2*(w+d)
    heights=np.unique(film.points[:,2])
    arc=np.linspace(0.,1.25*perimeter,201)
    ring=[]
    for s in arc:
        q=(s-.25*perimeter+.5*d)%perimeter
        pad=.003+(.004 if s>=perimeter else 0.)
        if q<d:
            x,y=.95-w/2-pad,-d/2+q
        elif q<d+w:
            x,y=.95-w/2+q-d,d/2+pad
        elif q<2*d+w:
            x,y=.95+w/2+pad,d/2-(q-d-w)
        else:
            x,y=.95+w/2-(q-2*d-w),-d/2-pad
        loose=max(0.,s-arc[-1]+.025)
        x-=loose*.65
        ring.append((x,y))
    film.points=np.array([[x,y,z] for x,y in ring for z in heights])
    return bodies,surfaces,dict(details,free_vertical_edge='center of near face',loose_end_m=.025)


BUILDERS={'pleated':formed_candy,'single-fold':single_fold_duvet,'front-edge':front_film_end}
TASKS={'pleated':'candy','single-fold':'duvet','front-edge':'pallet'}


def staged_stocking():
    bodies,surfaces,details=geometry.stocking()
    surfaces[0].points[:,1]-=.105
    return bodies,surfaces,dict(details,garment_shift_y_m=-.105)



BUILDERS['staged-cuff']=staged_stocking
TASKS['staged-cuff']='stocking'


def tabletop_twist():
    """Continuous pleated paper with the longitudinal overlap underneath."""
    bodies,_,details=geometry.candy()
    paper=formed_wrapper(seam_angle=-np.pi*(.5+.12))
    p=paper.points.reshape(65,73,3)
    p[:,:,0]-=.04
    p[:,:,1]*=1.6
    raised=max(0.,.7478-p[:,:,2].min())
    p[:,:,2]+=raised
    body=bodies[0]
    body['center']=(.34,0.,.754+raised)
    body['size']=(.016,.034,.010)
    details.update(candy_size_m=list(body['size']),wrapper_length_m=.128,
                   strip_width_m=.067,overlap='underneath, 12 percent of a turn')
    return bodies,[paper],details

BUILDERS['tabletop-twist']=tabletop_twist
TASKS['tabletop-twist']='candy'


def flat_strip():
    bodies,surfaces,details=tabletop_twist()
    paper=surfaces[0]
    formed=paper.points.reshape(65,73,3)
    row_length=np.linalg.norm(np.diff(formed,axis=0),axis=2).mean(axis=1)
    along=np.r_[0.,np.cumsum(row_length)]
    across=np.linspace(0.,.067,73)
    a,b=np.meshgrid(along,across,indexing='ij')
    paper.rest_points=np.column_stack((a.ravel(),b.ravel(),np.zeros(a.size)))
    return bodies,surfaces,dict(details,rest_shape='flat rectangular twist-wrap strip',strip_length_m=float(along[-1]))

BUILDERS['flat-strip']=flat_strip
TASKS['flat-strip']='candy'


def rotating_pallet():
    bodies,surfaces,details=front_film_end()
    # Ground is at -12.5 mm. Deck top is world z=0.500 m.
    rise=.5125
    for b in bodies:
        b['center']=tuple(np.array(b['center'])+[0.,0.,rise])
    surfaces[0].points[:,2]+=rise
    return bodies,surfaces,dict(details,deck_center_m=[.95,0.,.488],
        deck_radius_m=.8,deck_thickness_m=.024,grip_radius_m=.77,
        grip_count=18,load_reference='free wooden pallet',operation='hand-driven rim with passive bearing')

BUILDERS['turntable']=rotating_pallet
TASKS['turntable']='pallet'


def relaxed_seam():
    """A creased end and an unglued longitudinal seam with elastic springback."""
    bodies,surfaces,details=tabletop_twist()
    paper=surfaces[0]
    p=paper.points.reshape(65,73,3)
    rest=p.copy()
    u=(np.arange(72)+.5)/72
    for i,row in enumerate(p):
        y=abs(row[0,1])
        # Keep the measured strip width and formed end geometries. In the body,
        # let the transverse curvature relax toward an open arch.
        fraction=np.clip((.050-y)/.028,0.,1.)
        delta=np.diff(row[:,[0,2]],axis=0)
        angle=-np.pi*1.45*fraction*(u-.5)
        c,s=np.cos(angle),np.sin(angle)
        edge=np.column_stack((c*delta[:,0]-s*delta[:,1],s*delta[:,0]+c*delta[:,1]))
        section=np.vstack((np.zeros(2),np.cumsum(edge,axis=0)))
        section+=row[:,[0,2]].mean(axis=0)-section.mean(axis=0)
        rest[i,:,0]=section[:,0]
        rest[i,:,2]=section[:,1]
    paper.rest_points=rest.reshape(-1,3)
    return bodies,surfaces,dict(details,rest_shape='open arch in body, formed pleats in both tails')

BUILDERS['relaxed-seam']=relaxed_seam
TASKS['relaxed-seam']='candy'


def round_sweet():
    """A 17 mm hard sweet inside a double-ended, loosely overlapping strip."""
    bodies,surfaces,details=relaxed_seam()
    paper=surfaces[0]
    for points in (paper.points,paper.rest_points):
        points[:,2]=.757+(points[:,2]-.757)*1.55
    dz=.748-paper.points[:,2].min()
    paper.points[:,2]+=dz
    body=bodies[0]
    body.update(kind='sphere',radius=.0085,size=(.017,)*3,mass=.0034,
                center=(.34,0.,.757+dz))
    details.update(candy_shape='round hard candy',candy_size_m=[.017]*3)
    return bodies,surfaces,details

BUILDERS['round-sweet']=round_sweet
TASKS['round-sweet']='candy'


def prepared_stocking():
    """An unoccupied toe pocket and gathered shaft under a supported prone foot."""
    z0=.7475
    x,y=.35,-.09
    bodies=[geometry.box('form_base',(x,.345,z0+.01),(.19,.16,.02),(.30,.34,.38)),
            geometry.box('form_bracket',(x,.325,.875),(.065,.055,.23),(.37,.39,.40))]
    for name,center,r,h,axis in [
        ('working_calf',(x,.119,.978),.047,.335,'Y'),
        ('working_heel',(x,-.080,.978),.040,.041,'Y'),
        ('working_foot',(x,y,.910),.040,.126,'Z')]:
        bodies.append(dict(name=name,kind='capsule',center=center,radius=r,height=h,axis=axis,
                           color=(.76,.58,.46),mass=None))
    n,m=97,40
    t=np.linspace(0.,1.,n)
    theta=np.arange(m)*2*np.pi/m
    radial=np.c_[np.cos(theta),np.sin(theta)]
    rings=[];rest=[]
    for u in t:
        if u<.25:
            radius=np.interp(u,[0,.10,.25],[.008,.037,.046])
            z=z0+.0017+.018*u/.25
        else:
            # Six concentric folds stack the shaft around the empty toe pocket.
            f=(u-.25)/.75
            radius=.046+.011*(1-np.cos(12*np.pi*f))
            z=z0+.0197+.030*f
        rings.append(np.column_stack((np.array([x,y])+radial*radius,np.full(m,z))))
        # The shaft retains its separate uncompressed material dimensions.
        relaxed=np.interp(u,[0,.15,.35,.57,1.],[.008,.034,.040,.030,.043])
        heel= .012*np.sin(np.pi*(u-.25)/.25) if .25<u<.50 else 0.
        rest.append(np.column_stack((radial[:,0]*relaxed,radial[:,1]*relaxed+heel,.415*u+np.zeros(m))))
    tips=np.concatenate(rings);neutral=np.concatenate(rest)
    faces=[]
    for i in range(n-1):
        for j in range(m):
            a=i*m+j;b=i*m+(j+1)%m
            faces.extend([(a,a+m,b),(b,a+m,b+m)])
    faces.extend((n*m,(j+1)%m,j) for j in range(m))
    points=np.vstack((tips,[x,y,z0+.0017]))
    garment=geometry.Surface('stocking',points,np.array(faces),(.60,.49,.62),.0007,300.,3e4,.025,.45,.0006)
    garment.rest_points=np.vstack((neutral,[0.,0.,0.]))
    return bodies,[garment],dict(manikin='bench-mounted adult lower-leg training form, foot pointing down',
        oriented_toe_direction=[0,0,-1],mesh_shape=[n,m],heel_ring=36,stocking_length_m=.415,
        cuff_target_y_m=.27,prepared_toe_pocket=True,initial_toe_bottom_m=.807)

BUILDERS['prepared-stocking']=prepared_stocking
TASKS['prepared-stocking']='stocking'


def foam_corner():
    bodies,surfaces,details=geometry.hospital_corner()
    return bodies,surfaces,dict(details,mattress_compliance='free volumetric foam, estimated modulus 35 kPa')

BUILDERS['foam-corner']=foam_corner
TASKS['foam-corner']='hospital-corner'

def loose_film():
    from handling_setups import loose_film as make
    return make()

def dropped_sheet():
    from handling_setups import dropped_sheet as make
    return make()

BUILDERS.update({'loose-film':loose_film,'dropped-sheet':dropped_sheet})
TASKS.update({'loose-film':'pallet','dropped-sheet':'hospital-corner'})

def faceted_drop():
    from faceted_sweet import setup
    return setup()

BUILDERS['faceted-drop']=faceted_drop
TASKS['faceted-drop']='candy'


def wire_donner():
    from donning_frame import setup
    return setup()

BUILDERS['wire-donner']=wire_donner
TASKS['wire-donner']='stocking'


def open_stocking_frame():
    from donning_frame import extended_setup
    return extended_setup()

BUILDERS['open-stocking-frame']=open_stocking_frame
TASKS['open-stocking-frame']='stocking'


def low_turntable():
    from handling_setups import low_turntable as make
    return make()

BUILDERS['low-turntable']=low_turntable
TASKS['low-turntable']='pallet'


def bedside_paddle():
    from bed_paddle import setup
    return setup()

BUILDERS['bedside-paddle']=bedside_paddle
TASKS['bedside-paddle']='hospital-corner'



def printed_drop():
    bodies,surfaces,details=faceted_drop()
    return bodies,surfaces,dict(details,finish='berry print on the same sheet; rounded glazed sweet over the same hull')

BUILDERS['printed-drop']=printed_drop
TASKS['printed-drop']='candy'
