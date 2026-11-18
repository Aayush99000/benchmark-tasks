"""Replenish a twin jumbo-roll dispenser using the documented 56TR service flow.

The cover is already unlocked and open. Move the remaining right roll onto the
left support, fit the new roll on the right, and close the cover until it locks.
Opening the cover resets the access door to the right, exposing the left roll.
The remaining roll's real native paper end must pass through that open outlet.

The unbranded procedural case uses published 490 x 300.5 x 140 mm dimensions.
The matching refill uses TJ0922A published dimensions and mass. Support, shoulder,
cardboard, residual-roll, latch, cam and detent details are dimensioned engineering
approximations, not copied OEM internals. All actions use physical colliders and
prebuilt joints; no task callback writes body poses, nodes or grasp attachments.
"""
import math
import os
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
import mechkit as mk
import yam_arena.demo as _ydemo
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box
from surface_paper import SurfacePaper
from surface_access_geometry import Outlet
from release_clearance import evaluate_release_clearance

SCENE='station'
TOP=TABLE_TOP[SCENE]
WHITE=(.92,.92,.90)
SMOKE=(.30,.32,.36)
GREY=(.55,.56,.58)
PAPER=(.96,.95,.90)
RED=(.70,.18,.14)
BOARD=(.70,.66,.58)

# Published external dimensions. The equivalent pivot's90mm recess/25mm rise
# follow the side-view layout approximately; neither is a published OEM dimension.
XW=.400  # outer rear face, excluding the wall mounting board
CASE_W,CASE_H,CASE_D=.490,.3005,.140
YH=CASE_W/2
ZB=TOP+.120
ZT=ZB+CASE_H
FRONT=XW-CASE_D
HINGE_RISE=.025
HINGE=(FRONT+.090,0.,ZB+HINGE_RISE)
HINGE_SHIFT=HINGE[0]-(FRONT+.012)
WALL=.004
OPEN_DEG=90.
CH=CASE_H
PRIM_Y,RES_Y=.120,-.120  # left primary, right reserve as viewed from the front
HUBS={'p':PRIM_Y,'r':RES_Y}
Z_AX=ZB+CASE_H/2

# Published TJ0922A refill: 8.80 in OD, 3.40 in width, 3.30 in core ID, 1.64 lb.
R_I,ROLL_RO,ROLL_H=.04191,.11176,.08636
FULL_MASS=.743891
# Unknown cardboard/residual dimensions and mass: explicit engineering estimates.
CORE_WALL=.002
STUB_RO,STUB_MASS=.0575,.150
SUPPORT_R,SUPPORT_LENGTH=.0385,.065
SUPPORT_STOP_X=XW-.011  # axial shoulder, deliberately distinct from the backplate
SUPPORT_FRONT_X=SUPPORT_STOP_X-SUPPORT_LENGTH
SEATED_Z=Z_AX-(R_I-SUPPORT_R)
Q_AX=tuple(mk.mat_to_quat(mk.axis_angle((0,1,0),-math.pi/2)))


def _ring(ro,h,color,n=24,ri=R_I):
    """Tangent boxes around a real hollow core; local +z points out of the case."""
    return [mk.box((ro-ri,2*ro*math.tan(math.pi/n),h),
        at=((ro+ri)/2*math.cos(a),(ro+ri)/2*math.sin(a),h/2),yaw=a,color=color)
        for a in np.arange(n)*2*math.pi/n]


kit=mk.Kit()


def _part(name,pos,boxes,**kwargs):
    return kit.part(name,pos,boxes,contact_offset=.001,**kwargs)


# Plain fixed supports: no release button, split jaws, bearing bond or roll catch.
for key,y in HUBS.items():
    _part('support_'+key,(SUPPORT_STOP_X,y,Z_AX),
        _ring(SUPPORT_R,SUPPORT_LENGTH,GREY,ri=0.),quat=Q_AX,color=GREY)
    _part('shoulder_'+key,(XW-.0045,y,Z_AX),
        _ring(.048,.0065,GREY,ri=0.),quat=Q_AX,color=GREY)

STUB0=(SUPPORT_STOP_X,RES_Y,SEATED_Z)
NEW_FRONT=.300
CARTON=(NEW_FRONT+ROLL_H/2,-.520,.060)
NEW_AX=TOP+CARTON[2]+ROLL_RO+.001
NEW0=(NEW_FRONT+ROLL_H,CARTON[1],NEW_AX)
_part('stub',STUB0,_ring(STUB_RO,ROLL_H,PAPER),mass=STUB_MASS,quat=Q_AX,color=PAPER)
_part('roll',NEW0,_ring(ROLL_RO,ROLL_H,PAPER),mass=FULL_MASS,quat=Q_AX,color=PAPER)

# The bottom-forward cover falls onto its open angle stop under gravity.
# Its top and side returns span the published depth; no invented open stay.
OUT_Y=((- .225,-.015),(.015,.225))
OUT_HEIGHT=.075
# Equivalent output chute: the sliding front/bottom door closes the forward
# 95mm; a fixed rear tray closes the remaining case depth. Unlike an invented
# full-depth rectangular door, this clears a full refill when the cover opens.
OUTLET_BACK=.355
SHUTTER_DEPTH=OUTLET_BACK-FRONT
RESET_GUIDE_Z=(.064,.084)
RESET_GUIDE_Y=(-.008,.024)
SOLID_Y=((-YH,OUT_Y[0][0]),(-.015,.015),(OUT_Y[1][1],YH))
def _cover_box(size,at=(0.,0.,0.),**kwargs):
    return mk.box(size,at=(at[0]-HINGE_SHIFT,at[1],at[2]-HINGE_RISE),**kwargs)


_cover=[_cover_box((.004,CASE_W,CH-RESET_GUIDE_Z[1]),at=(-.002,0.,(CH+RESET_GUIDE_Z[1])/2)),
        _cover_box((CASE_D-.016,CASE_W,WALL),at=((CASE_D-.040)/2,0.,CH-WALL/2)),
        _cover_box((CASE_D-.016,WALL,CH-.006),at=((CASE_D-.040)/2,YH-WALL/2,(CH+.006)/2)),
        _cover_box((CASE_D-.016,WALL,CH-.006),at=((CASE_D-.040)/2,-YH+WALL/2,(CH+.006)/2)),
        # Passive latch tongue: on approach it pushes the frame bolt backward,
        # then drops underneath its tip. Opening raises it into the bolt.
        mk.box((.012,.024,.012),at=(.303-HINGE[0],0.,CH-.028-HINGE_RISE),color=GREY)]
for a,b in SOLID_Y:
    bottom=.022 if abs(a+b)<1e-8 or a>.2 else .006  # clearance for reset cam / bottom rim
    top=RESET_GUIDE_Z[0] if abs(a+b)<1e-8 else OUT_HEIGHT
    _cover.append(_cover_box((.004,b-a,top-bottom),at=(-.002,(a+b)/2,(top+bottom)/2)))
    if abs(a+b)<1e-8:
        _cover.append(_cover_box((.004,RESET_GUIDE_Y[0]-a,OUT_HEIGHT-top),
            at=(-.002,(a+RESET_GUIDE_Y[0])/2,(top+OUT_HEIGHT)/2)))
# A physical slot lets the reset nose retreat behind the sliding door. The
# connected pair's usual collision filter is not used to hide a solid wall.
for a,b in ((-YH,RESET_GUIDE_Y[0]),(RESET_GUIDE_Y[1],YH)):
    _cover.append(_cover_box((.004,b-a,RESET_GUIDE_Z[1]-OUT_HEIGHT),
        at=(-.002,(a+b)/2,(OUT_HEIGHT+RESET_GUIDE_Z[1])/2)))
# The latch tongue is moulded into a visible continuous rib along its+Y side.
_cover.extend([
    mk.box((.033,.006,.012),at=(.2865-HINGE[0],.015,CH-.028-HINGE_RISE),color=GREY),
    mk.box((.006,.012,.012),at=(.303-HINGE[0],.009,CH-.028-HINGE_RISE),color=GREY),
])
_part('cover',HINGE,_cover,mass=.60,color=SMOKE)
kit.hinge('cover_hinge',None,'cover',anchor=HINGE,axis=(0,-1,0),lower_deg=0.,upper_deg=OPEN_DEG,
    start_deg=OPEN_DEG,damping=.60,friction=.015)

# Spring latch with real blocking contact. Initially no cover tongue is near it.
LATCH_ORIGIN=(.291,-.011,ZB+.2725)
_part('latch_bolt',LATCH_ORIGIN,[mk.box((.006,.006,.012)),
    # A continuous 19 x 8.4 x 12 mm triangular lead avoids a flat entry cap.
    # The separate square 6 mm tip remains the opening-load face.
    mk.convex([(x,y,z) for z in (-.006,.006)
        for x,y in ((-.016,-.0015),(.003,-.003),(.003,.0054))],
        [(0,2,1),(3,4,5),(0,1,4),(0,4,3),(1,2,5),(1,5,4),(2,0,3),(2,3,5)])],mass=.020,color=GREY)
kit.slider('latch_slide',None,'latch_bolt',axis=(0,-1,0),lower=0.,upper=.018,
    stiffness=400.,rest=0.,damping=.8)

# Access door starts at its used-right-roll detent. An open-cover reset lever
# physically rides onto a fixed frame cam, retracting the pawl; the door spring
# then returns it to the right. This is a disclosed equivalent reset mechanism.
SHUT_TRAVEL=.240
SHUTTER_WIDTH=.210
_part('shutter',HINGE,[
    _cover_box((.004,SHUTTER_WIDTH,.075),at=(-.010,-.120,.0375)),
    _cover_box((SHUTTER_DEPTH,SHUTTER_WIDTH,.003),
        at=((FRONT+OUTLET_BACK)/2-(HINGE[0]-HINGE_SHIFT),-.120,.002))],mass=.035,color=RED)
kit.slider('shutter_slide','cover','shutter',axis=(0,1,0),lower=0.,upper=SHUT_TRAVEL,
    start=SHUT_TRAVEL,stiffness=.60,rest=0.,damping=.30)
_part('reset_pawl',HINGE,[
    mk.box((.008,.010,.004),at=(.010,.234,0.)),
    mk.box((.004,.006,.074-HINGE_RISE),at=(.012,.234,(.074-HINGE_RISE)/2)),
    mk.box((.004,.236,.006),at=(.012,.120,.074-HINGE_RISE)),
    mk.box((.106,.006,.006),at=(-.038,.008,.074-HINGE_RISE)),
    _cover_box((.006,.012,.006),at=(-.010,.008,.074))],mass=.020,color=GREY)
kit.slider('reset_pawl_slide','cover','reset_pawl',axis=(1,0,0),lower=0.,upper=.018,
    start=.011,stiffness=25.,rest=0.,damping=.10)
_part('reset_cam',(HINGE[0],.234,HINGE[2]+.0095),[mk.box((.016,.010,.008))],color=GREY)

FIXTURES=[
    Box('wall_board',pos=(XW+.016,0.,(TOP+ZT)/2),size=(.020,.56,ZT-TOP),color=BOARD),
    Box('back_plate',pos=(XW-.003,0.,(ZB+ZT)/2),size=(.006,CASE_W,CASE_H),color=WHITE),
    Box('back_top',pos=(XW-.006,0.,ZT-WALL/2),size=(.012,CASE_W,WALL),color=WHITE),
    Box('rear_bottom_tray',pos=((OUTLET_BACK+XW-.006)/2,0.,ZB+.032),
        size=(XW-.006-OUTLET_BACK,.450,.002),color=WHITE),
    Box('carton',pos=(CARTON[0],CARTON[1],TOP+CARTON[2]/2),size=(.120,.245,CARTON[2]),color=(.62,.50,.36)),
    # Backplate-mounted receiver supports the real lateral spring bolt from its
    # -Y side, clear of the cover's tongue/rib and the bolt's18mm travel.
    Box('latch_receiver_arm',pos=(.347,-.055,ZB+.2725),size=(.094,.008,.032),color=GREY),
    Box('latch_receiver_upper',pos=(.297,-.027,ZB+.2865),size=(.018,.064,.004),color=GREY),
    Box('latch_receiver_lower',pos=(.297,-.027,ZB+.2585),size=(.018,.064,.004),color=GREY),
]
for i,(a,b) in enumerate(SOLID_Y):
    FIXTURES.append(Box('bottom_bridge_'+str(i),pos=(XW-.006,(a+b)/2,ZB+WALL/2),
        size=(.012,b-a,WALL),color=WHITE))
TAIL_W,TAIL_LENGTH=ROLL_H,.350
TAIL_X0=STUB0[0]-ROLL_H/2
TAIL_END='loose_tissue'
paper=SurfacePaper(TAIL_END,'stub',lambda:kit._spawn_poses()['stub'],
    top_world=(TAIL_X0,RES_Y+(STUB_RO+.001)*math.cos(math.pi/6),SEATED_Z+(STUB_RO+.001)*math.sin(math.pi/6)),
    outlet=Outlet(FRONT+.004,OUTLET_BACK,OUT_Y[1][0],OUT_Y[1][1],ZB),minimum_z=.004,
    width=TAIL_W,length=TAIL_LENGTH,areal_mass=.028)
OBJECTS=[*kit.objects(),paper.marker,*FIXTURES]

# A passive simulation: callbacks only sample/diagnose and latch failure history.
DT=.05
SUCCESS_HOLD_S=1.
SCORE_OVER='end'
SEAT_AXIAL,SEAT_LAT,SEAT_DEG=.006,.006,4.


def _tilt(quat):
    axis=mk.quat_to_mat(quat)[:,2]
    return math.degrees(math.acos(np.clip(-axis[0],-1.,1.)))


def seated(objects,part,hub):
    rear=kit.point(objects,part,(0,0,0))
    _,quat=kit.pose(objects,part)
    return (abs(float(rear[0])-SUPPORT_STOP_X)<=SEAT_AXIAL
        and math.hypot(float(rear[1])-HUBS[hub],float(rear[2])-SEATED_Z)<=SEAT_LAT
        and _tilt(quat)<=SEAT_DEG)


def cover_locked(objects):
    # Geometric engagement of the real spring bolt with the closed tongue.
    return abs(kit.q_deg(objects,'cover_hinge'))<2. and kit.q(objects,'latch_slide')<.003


def changed_over(objects):
    return kit.q(objects,'shutter_slide')<.010


def tail_out(objects):
    return paper.snapshot is not None and paper.snapshot.access.geometry_accessible


def release_geometry(objects):
    snapshot = paper.snapshot
    if snapshot is None:
        return False
    try:
        report = evaluate_release_clearance(robot_boxes=snapshot.robot_boxes,
            tool_boxes=snapshot.tool_boxes, rigid_states=objects, env_origin=snapshot.env_origin,
            expected_robot_names=snapshot.expected_robot_names, expected_tool_names=snapshot.expected_tool_names)
    except AttributeError:
        return False
    return report.valid and report.clear


def settled(objects):
    return (paper.snapshot is not None and paper.snapshot.access.hands_clear
        and release_geometry(objects)
        and all(objects[name].resting(max_speed=.01,max_spin=.25) for name in ('stub','roll','cover','shutter')))


def failed(objects):
    return bool(mk.MEMORY.get('floor'))


def discarded(objects):
    center=kit.point(objects,'stub',(0,0,ROLL_H/2))
    return not (FRONT<float(center[0])<XW and abs(float(center[1]))<YH and ZB<float(center[2])<ZT)


def score(objects):
    if failed(objects) or discarded(objects):
        return 0.
    if not seated(objects,'stub','p'):
        return 0.
    value=.25
    if seated(objects,'roll','r'):
        value+=.20
        if cover_locked(objects):
            value+=.20
            if changed_over(objects):
                value+=.15
                if tail_out(objects):
                    value+=.10+.10*settled(objects)
    return round(value,4)


def success(objects):
    return score(objects)>=1.-1e-9


def distance(objects):
    return sum(math.dist(kit.point(objects,part,(0,0,0)),(SUPPORT_STOP_X,HUBS[hub],SEATED_Z))
        for part,hub in (('stub','p'),('roll','r')))


def _observe(objects):
    for name in ('stub','roll'):
        if kit.point(objects,name,(0,0,ROLL_H/2))[2]<TOP-.05:
            mk.MEMORY['floor']=True


kit.on_step(_observe)


class _Ang(float):
    def __new__(cls,yaw=0.,pitch=0.,roll=0.):
        value=super().__new__(cls,yaw)
        value.pitch,value.roll=pitch,roll
        return value


_stock_grip=getattr(_ydemo.grip_rotation,'_stock',_ydemo.grip_rotation)


def _grip_rotation(yaw=0.):
    rot=_stock_grip(float(yaw))
    pitch,roll=getattr(yaw,'pitch',0.),getattr(yaw,'roll',0.)
    if pitch:rot=mk.axis_angle((0,1,0),pitch)@rot
    return rot if not roll else mk.axis_angle((1,0,0),roll)@rot


_grip_rotation._stock=_stock_grip
_ydemo.grip_rotation=_grip_rotation
PITCH=-math.pi/2
YC=math.pi/2
TIP=.0128425
TIP_INSERTION=.015
GRIP_AXIAL=TIP_INSERTION-TIP
PARK={'left':(.150,.380,.880),'right':(.150,-.380,.880)}


class _Seq:
    def __init__(self,yaw=YC):self.w,self.t,self.yaw=[],0.,yaw
    def go(self,d,pos,f,label,yaw=0.,pitch=0.,roll=0.):
        self.w.append(Waypoint(d,tuple(float(v) for v in pos),f,label,_Ang(yaw,pitch,roll)))
        self.t+=d
        return self
    def h(self,d,pos,f,label):return self.go(d,pos,f,label,self.yaw,PITCH)
    def wait(self,d,label='wait'):
        waypoint=self.w[-1]
        self.w.append(Waypoint(d,waypoint.pos,waypoint.finger,label,waypoint.yaw));self.t+=d
        return self
    def until(self,t,label='wait'):return self.wait(max(.1,t-self.t),label)


def self_test():
    """Deterministic scorer boundaries using labelled native-surface topology."""
    from surface_paper import SurfaceSnapshot,material_labels
    from surface_access_geometry import CollisionBox,evaluate_surface_access,topology_hash
    global release_geometry
    original_release_geometry = release_geometry
    old_snapshot=paper.snapshot
    xs=np.linspace(-TAIL_W/2,TAIL_W/2,17)
    distances=np.linspace(0.,TAIL_LENGTH,33)
    material=np.array([(x,TAIL_LENGTH/2-d,0.) for d in distances for x in xs])
    roots,ends,patch=material_labels(material)
    faces=[]
    for row in range(32):
        for col in range(16):
            index=row*17+col
            faces.extend(((index,index+17,index+1),(index+1,index+17,index+18)))
    faces=np.asarray(faces)
    topology=topology_hash(len(material),faces)

    def at(hub,dx=0.,dy=0.,dz=0.,tilt=0.):
        quat=mk.mat_to_quat(mk.axis_angle((0,1,0),math.radians(tilt))@mk.quat_to_mat(Q_AX))
        return ((SUPPORT_STOP_X+dx,HUBS[hub]+dy,SEATED_Z+dz),quat)

    def state(q=None,**moved):return kit.fake_states(q=q,moved=moved)

    def surface(states,mode):
        parent=kit._pose_T(states,'stub')
        transform=parent@paper._in_parent
        nodes=material@transform[:3,:3].T+transform[:3,3]
        anchors=(nodes[roots]-parent[:3,3])@parent[:3,:3]
        velocities=np.zeros_like(nodes)
        depth=np.repeat(distances,len(xs))
        # A constructed, forward-threaded positive scorer fixture. Roots stay
        # on the actual residual roll; the end travels into the front chute.
        # Native physical threading is a separate qualification requirement.
        desired_center_x=(paper.outlet.x_min+paper.outlet.x_max)/2
        forward=desired_center_x-nodes[roots,0].mean()
        nodes[:,0]+=forward*np.minimum(depth/.100,1.)
        # Construct the positive scorer fixture through the target aperture;
        # a rigid translation of this reset sheet is intentionally not usable.
        desired_center_y=(paper.outlet.y_min+paper.outlet.y_max)/2
        nodes[:,1]+=(desired_center_y-nodes[:,1])*np.minimum(depth/.100,1.)
        boxes=[]
        if mode in ('inside','short','exact20','under20'):
            end=ZB+{'inside':.015,'short':-.009,'exact20':-.020,'under20':-.0199}[mode]
            top=nodes[roots,2].mean()
            nodes[:,2]=top-depth/TAIL_LENGTH*(top-end)
        elif mode=='wrong_window':nodes[:,1]-=.240*depth/TAIL_LENGTH
        elif mode=='around_side':nodes[:,1]+=.350*np.sin(np.pi*depth/TAIL_LENGTH)
        elif mode=='detached':nodes[roots,2]-=.005
        elif mode=='no_crossing':nodes[:,2]-=.3
        elif mode=='held':boxes=[CollisionBox('real_pad',nodes[ends].mean(0),(.012,.008,.012))]
        elif mode=='moving':velocities[ends[0],0],velocities[ends[-1],0]=.04,-.04
        access=evaluate_surface_access(nodes=nodes,velocities=velocities,triangles=faces,
            root_ids=roots,free_edge_ids=ends,terminal_patch_ids=patch,stub_transform=parent,
            root_local_anchors=anchors,outlet=paper.outlet,robot_boxes=boxes,
            expected_topology_hash=topology,minimum_z=.004)
        return SurfaceSnapshot(nodes,velocities,parent,tuple(boxes),access)

    closed={'cover_hinge':0.,'shutter_slide':0.,'latch_slide':0.}
    done=state(closed,stub=at('p'),roll=at('r'))
    cases=[
        ('start',state(),{},'hanging',0.),
        ('close untouched',state(closed),{},'hanging',0.),
        ('remaining roll on left',state(stub=at('p')),{},'hanging',.25),
        ('both rolls cover open',state(stub=at('p'),roll=at('r')),{},'hanging',.45),
        ('cover nearly shut but bolt retracted',state({**closed,'latch_slide':.012},stub=at('p'),roll=at('r')),{},'hanging',.45),
        ('cover not locked',state({**closed,'cover_hinge':math.radians(4.)},stub=at('p'),roll=at('r')),{},'hanging',.45),
        ('door still left',state({**closed,'shutter_slide':SHUT_TRAVEL},stub=at('p'),roll=at('r')),{},'hanging',.65),
        ('door halfway',state({**closed,'shutter_slide':SHUT_TRAVEL/2},stub=at('p'),roll=at('r')),{},'hanging',.65),
        ('full',done,{},'hanging',1.),
        ('paper inside',done,{},'inside',.8),
        ('paper only9mm out',done,{},'short',.8),
        ('paper exactly20mm out',done,{},'exact20',1.),
        ('paper19.9mm out',done,{},'under20',.8),
        ('wrong outlet',done,{},'wrong_window',.8),
        ('around housing',done,{},'around_side',.8),
        ('root detached',done,{},'detached',.8),
        ('no outlet crossing',done,{},'no_crossing',.8),
        ('paper held',done,{},'held',.9),
        ('opposed moving nodes',done,{},'moving',1.),
        ('fresh roll12mm short',state(closed,stub=at('p'),roll=at('r',dx=-.012)),{},'hanging',.25),
        ('residual roll12mm short',state(closed,stub=at('p',dx=-.012),roll=at('r')),{},'hanging',0.),
        ('roll at rear wall instead of stop',state(closed,stub=at('p',dx=.011),roll=at('r')),{},'hanging',0.),
        ('residual roll15mm off axis',state(closed,stub=at('p',dy=.015),roll=at('r')),{},'hanging',0.),
        ('residual roll tilted7deg',state(closed,stub=at('p',tilt=7.),roll=at('r')),{},'hanging',0.),
        ('wrong roll order',state(closed,stub=at('r'),roll=at('p')),{},'hanging',0.),
        ('residual discarded',state(closed,stub=((.1,.1,TOP),Q_AX),roll=at('r')),{},'hanging',0.),
        ('floor failure latched',done,{'floor':True},'hanging',0.),
    ]
    try:
        release_geometry = lambda objects: True
        for label,states,memory,mode,want in cases:
            mk.new_episode();mk.MEMORY.update(memory)
            paper.snapshot=surface(states,mode)
            snapshot=paper.snapshot
            got=score(states)
            assert abs(got-want)<1e-9,(label,got,want,snapshot.access.reasons)
            assert score(states)==got and success(states)==(got==1.)
            assert paper.snapshot is snapshot
    finally:
        release_geometry = original_release_geometry
        paper.snapshot=old_snapshot
        mk.new_episode()
    return len(cases)

