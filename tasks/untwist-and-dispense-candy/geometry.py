"""Procedural full-size research assets, in metres. No downloaded task meshes.

Bedding and pallet film use separate material rest meshes. The stocking and
wrapper retain their formed rest shapes. Material values need calibration.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class Surface:
    name: str
    points: np.ndarray
    faces: np.ndarray
    color: tuple
    thickness: float
    density: float
    young: float
    bend: float
    friction: float
    contact: float
    rest_points: np.ndarray | None = None


def grid(n, m):
    return np.asarray([(a, a+m, a+1) if k == 0 else (a+1, a+m, a+m+1)
                       for i in range(n-1) for j in range(m-1)
                       for a in [i*m+j] for k in range(2)], dtype=np.int32)


def fold(s, panel, rise):
    """Accordion placement with distinct layers. Rounded fold physics not implied."""
    k = np.floor(s/panel).astype(int)
    u = s-k*panel
    return np.where(k % 2 == 0, u, panel-u), k*rise


def box(name, center, size, color=(.65,.65,.65), mass=None):
    return dict(name=name, kind='box', center=center, size=size, color=color, mass=mass)


def bed(center, size):
    x,y,z=center; sx,sy,sz=size
    bodies=[box('mattress',center,size,(.82,.85,.87)),
            box('bed_frame',(x,y,z-sz/2-.035),(sx,sy,.07),(.26,.29,.31))]
    bottom=z-sz/2-.07
    for i,dx in enumerate([-.4*sx,.4*sx]):
        for j,dy in enumerate([-.4*sy,.4*sy]):
            bodies.append(box(f'bed_leg_{i}_{j}',(x+dx,y+dy,(bottom-.0125)/2),
                              (.06,.06,bottom+.0125),(.26,.29,.31)))
    return bodies


def duvet():
    bodies=bed((1.25,0.,.56),(2.10,1.55,.18))
    # Two 2 m textiles occupy separate halves of the mattress, folded in both
    # directions. No corner of the duvet starts inside the cover.
    n,m=81,55
    u,v=np.meshgrid(np.linspace(0,2,n),np.linspace(0,1.35,m),indexing='ij')
    xx,zx=fold(u,.5,.018); yy,zy=fold(v,.675,.012)
    p=np.column_stack((.24+xx.ravel(),.055+yy.ravel(),.67+zx.ravel()+zy.ravel()))
    insert=Surface('duvet',p,grid(n,m),(.91,.90,.82),.012,34.,1e4,1e4,.45,.004)
    insert.rest_points = np.column_stack((u.ravel(), v.ravel(), np.zeros(u.size)))
    # Sew a two-panel bag along the two sides and far end. The near end is open.
    n,m=81,55;u,v=np.meshgrid(np.linspace(0,2,n),np.linspace(0,1.35,m),indexing='ij')
    # Distinct X/Y layer spacings avoid coincident sheets at folds of equal
    # index sum. Equal spacings caused thousands of duplicate coordinates.
    xx,zx=fold(u,.5,.016);yy,zy=fold(v,.675,.006)
    lower=np.column_stack((.24+xx.ravel(),-.74+yy.ravel(),.67+zx.ravel()+zy.ravel()))
    upper=lower.copy();upper[:,2]+=.004
    # Welding boundary vertices produces genuine connected topology, not an
    # enclosure test based only on two overlapping sheets.
    seam=[i*m+j for i in range(n) for j in range(m) if j in (0,m-1) or i==n-1]
    upper[seam]=lower[seam]
    points=np.vstack((lower,upper)); remap=np.arange(len(points))
    remap[np.asarray(seam)+len(lower)]=seam
    faces=remap[np.vstack((grid(n,m),grid(n,m)[:,::-1]+len(lower)))]
    # At a doubly sewn corner, both panels can contain the same triangle.
    # Remove both coincident copies; their neighboring faces close the seam.
    _, inverse_faces, counts=np.unique(np.sort(faces,axis=1),axis=0,return_inverse=True,return_counts=True)
    faces=faces[counts[inverse_faces]==1]
    used,inverse=np.unique(faces,return_inverse=True)
    cover=Surface('cover',points[used],inverse.reshape(-1,3),(.28,.49,.67),.0005,300.,5e5,5e5,.4,.001)
    flat = np.column_stack((u.ravel(), v.ravel(), np.zeros(u.size)))
    flat_top = flat.copy()
    flat_top[:,2] += .004
    flat_top[seam] = flat[seam]
    cover.rest_points = np.vstack((flat,flat_top))[used]
    return bodies,[insert,cover],dict(duvet_flat_size_m=[2.,1.35],cover_flat_size_m=[2.,1.35],
        unresolved=['loft/compression', 'fold rest shape', 'self-contact', 'feeding and spreading the far corners'])


def pallet():
    cx,cy=.95,0.; base=.144-.0125
    bodies=[]
    for j in range(5):
        bodies.append(box(f'pallet_deck_{j}',(cx,-.34+j*.17,base-.011),
                          (1.2,.12,.022),(.49,.33,.19)))
    for j in [-.30,0.,.30]:
        bodies.append(box(f'pallet_stringer_{j}'.replace('.','_').replace('-','n'),
                          (cx,j,.05),(1.2,.07,.125),(.49,.33,.19)))
    for k in range(2):
        for i in range(3):
            for j in range(2):
                bodies.append(box(f'carton_{i}_{j}_{k}',(.57+i*.38,-.19+j*.38,base+.165+k*.33),
                                  (.376,.376,.326),(.66,.47,.28),mass=3.0))
    # Continuous 1.25-turn sheet: one overlap, no invisible sticky constraint.
    n,m=201,25; w,d=1.15,.77;perimeter=2*(w+d)
    s=np.linspace(0,1.25*perimeter,n); z=np.linspace(base+.015,base+.66,m)
    ring=[]
    for t in s:
        turn=int(t/perimeter); q=t%perimeter; pad=.003+turn*.004
        if q<d: x,y=cx-w/2-pad,-d/2+q
        elif q<d+w: x,y=cx-w/2+q-d,d/2+pad
        elif q<2*d+w: x,y=cx+w/2+pad,d/2-(q-d-w)
        else:x,y=cx+w/2-(q-2*d-w),-d/2-pad
        ring.append((x,y))
    p=np.array([[x,y,h] for x,y in ring for h in z])
    film=Surface('film',p,grid(n,m),(.38,.65,.72),.000023,920.,2e7,2e7,.35,.001)
    # A shorter material rest length creates actual stretch when installed.
    # 8% is a test setting. The frictional overlap has no adhesive law yet.
    film.rest_points = np.array([[length/1.08,h-base,0.] for length in s for h in z])
    return bodies,[film],dict(initial_longitudinal_stretch_ratio=1.08,pallet_size_m=[1.2,.8,.144],carton_count=12,
        unresolved=['film pre-stretch','interlayer cling','tearing/cutting','rear-side access'])


def stocking():
    # A bench-mounted, adult-sized below-knee training form. A proximal
    # bracket supports the calf and leaves the heel accessible on all sides.
    tabletop=.7475
    rise=tabletop-.65
    bodies=[box('form_base',(.37,.36,tabletop+.009),(.18,.16,.018),(.31,.35,.37)),
            box('form_bracket',(.37,.36,tabletop+.055),(.06,.04,.09),(.37,.39,.40))]
    for part,y,z,radius,height,axis in [
            ('calf',.155,.739,.052,.34,'Y'),('heel',-.064,.718,.045,.035,'Y'),
            ('foot',-.071,.807,.043,.142,'Z')]:
        bodies.append(dict(name='working_'+part,kind='capsule',center=(.37,y,z+rise),
                           radius=radius,height=height,axis=axis,color=(.76,.58,.46),mass=None))
    # A foot-shaped shell lying on its side. Stations describe the centre
    # line in the mattress plane, from toe through a rounded heel to the cuff.
    path=np.array([[.132,-.47],[.154,-.463],[.19,-.451],[.235,-.435],
                   [.277,-.413],[.301,-.385],[.313,-.35],[.323,-.29],
                   [.333,-.23],[.345,-.16],[.35,-.13]])
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    n,m=51,32
    distance=np.linspace(0.,arc[-1],n)
    centers=np.column_stack([np.interp(distance,arc,path[:,d]) for d in (0,1)])
    tangent=np.gradient(centers,axis=0)
    tangent/=np.linalg.norm(tangent,axis=1)[:,None]
    normal=np.column_stack((-tangent[:,1],tangent[:,0]))
    points=[]
    for i in range(n):
        # An expanded heel pocket, narrower ankle, then broader calf and cuff.
        width=np.interp(distance[i]/arc[-1],[0,.18,.38,.52,.67,1],[.015,.033,.031,.040,.026,.039])
        for a in np.arange(m)*2*np.pi/m:
            edge=centers[i]+width*np.cos(a)*normal[i]
            points.append((*edge,.668+.012*np.sin(a)))
    faces=[]
    for i in range(n-1):
        for j in range(m):
            a=i*m+j;b=i*m+(j+1)%m;c=(i+1)*m+j;d=(i+1)*m+(j+1)%m
            faces.extend([(a,c,b),(b,c,d)])
    points.append((*centers[0],.668)); tip=len(points)-1
    faces.extend((tip,(j+1)%m,j) for j in range(m))
    sock=Surface('stocking',np.array(points),np.array(faces),(.75,.71,.61),.0007,300.,3e4,3e4,.4,.001)
    sock.points[:,2]+=rise
    return bodies,[sock],dict(manikin='bench-mounted adult lower-leg training form',stocking_centerline_m=float(arc[-1]),
        unresolved=['heel fit and sizing','anisotropic knit response','compression/load calibration','cuff folding'])


def candy():
    bodies=[box('candy',(.38,0.,.754),(.016,.024,.010),(.80,.30,.08),mass=.005)]
    # A flat-bottomed rectangular sweet in a continuous overlapping sheet.
    # Its resting face and the outer pleats both clear the rigid core at spawn.
    n, m = 65, 41
    points = []
    for y in np.linspace(-.04, .04, n):
        shoulder = np.clip((abs(y)-.013)/.009, 0., 1.)
        tail = np.clip((abs(y)-.022)/.018, 0., 1.)
        half_width = .0096*(1-shoulder) + (.0028+.002*tail)*shoulder
        half_height = .0063*(1-shoulder) + (.0028+.0008*tail)*shoulder
        turn = np.sign(y)*2*np.pi*(.65*shoulder+.35*tail)
        for u in np.linspace(0., 1., m):
            angle = 2*np.pi*1.12*u
            cs, sn = np.cos(angle), np.sin(angle)
            x = np.sign(cs)*abs(cs)**.28*(half_width+.00045*u)
            z = np.sign(sn)*abs(sn)**.28*(half_height+.00045*u)
            points.append((.38+x*np.cos(turn)-z*np.sin(turn), y,
                           .7545+x*np.sin(turn)+z*np.cos(turn)))
    wrapper = Surface('wrapper', np.array(points), grid(n,m), (.85,.65,.16),
                      .000045, 1000., 2e6, .05, .38, .00035)
    return bodies, [wrapper], dict(candy_size_m=[.016,.024,.010],
        candy_shape='flat-bottomed rectangular confection', wrapper_length_m=.08,
        wrapper='continuous overlapping cellophane with two formed twists',
        unresolved=['flat material rest metric','twist retention','material response','tear/crush detection'])


BUILDERS=dict(duvet=duvet,pallet=pallet,stocking=stocking,candy=candy)


def audit(surface):
    p,f=surface.points,surface.faces
    area=.5*np.linalg.norm(np.cross(p[f[:,1]]-p[f[:,0]],p[f[:,2]]-p[f[:,0]]),axis=1)
    rest=surface.rest_points if surface.rest_points is not None else p
    rest_area=.5*np.linalg.norm(np.cross(rest[f[:,1]]-rest[f[:,0]],rest[f[:,2]]-rest[f[:,0]]),axis=1).sum()
    edges=np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1)
    _,counts=np.unique(edges,axis=0,return_counts=True)
    return dict(vertices=len(p),triangles=len(f),area_m2=float(area.sum()),
        rest_area_m2=float(rest_area), nominal_mass_kg=float(rest_area*surface.thickness*surface.density),
        min_triangle_area_m2=float(area.min()),nonmanifold_edges=int(np.sum(counts>2)),
        boundary_edges=int(np.sum(counts==1)),bounds_m=[p.min(0).tolist(),p.max(0).tolist()],
        material_parameters='unmeasured engineering starting values, not a calibration')


def hospital_corner():
    """A full flat sheet draping at the accessible foot corner of a real-size bed."""
    mattress_min=(.29,-.04)
    top=.775
    bodies=bed((1.29,.41,top-.06),(2.0,.90,.12))
    # Real slats support the mattress. Leave clearance between the outer frame
    # and the mattress edge where the free hem can be inserted.
    bodies=[b for b in bodies if b['name']!='bed_frame']
    bodies.append(box('bed_frame',(1.29,.41,top-.177),(1.82,.72,.06),(.26,.29,.31)))
    for k in range(12):
        bodies.append(box(f'bed_slat_{k}',(.375+k*.165,.41,top-.1335),(.07,.86,.027),(.55,.41,.27)))
    x,y=np.meshgrid(np.linspace(.085,2.50,106),np.linspace(-.255,1.095,71),indexing='ij')
    points=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,top+.002)))
    sheet=Surface('flat_sheet',points,grid(*x.shape),(.76,.83,.91),.00045,310.,4e5,.022,.42,.0008)
    sheet.rest_points=points-points[0]
    return bodies,[sheet],dict(mattress_near_xy_m=mattress_min,mattress_top_m=top,
        sheet_size_m=[2.415,1.35],mesh_shape=list(x.shape),entire_sheet=True,
        evaluation_region='near foot corner only',mattress_compliance='rigid approximation')

BUILDERS['hospital-corner']=hospital_corner
