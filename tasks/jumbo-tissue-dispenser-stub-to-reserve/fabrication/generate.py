"""Build millimetre CAD from the submitted manifest, plus explicit joint hardware.

Requires Python3.12,numpy,trimesh and manifold3d3.5.5. Does not import or mutate
the simulation. Run from any directory; outputs remain inside fabrication/.
"""
from pathlib import Path
import argparse,csv,hashlib,itertools,json,math
import numpy as np
import trimesh
import manifold3d as md

HERE=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--manifest',type=Path,default=HERE.parent/'setup_manifest.json');ap.add_argument('--out',type=Path,default=HERE);args=ap.parse_args()
OUT=args.out;OUT.mkdir(parents=True,exist_ok=True);STL=OUT/'stl';STL.mkdir(exist_ok=True)
M=json.loads(args.manifest.read_text());DATUM=np.array([368.,-245.,787.5])
def rotation(q):
 x,y,z,w=q;return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def box(lo,hi):return md.Manifold.cube((np.array(hi)-lo).tolist()).translate(lo)
def cylinder(radius,z0,z1,xy):return md.Manifold.cylinder(z1-z0,radius,circular_segments=64).translate([*xy,z0])
def bore_x(radius,x0,x1,y,z):return md.Manifold.cylinder(x1-x0,radius,circular_segments=64).rotate([0,90,0]).translate([x0,y,z])
def bore_y(radius,y0,y1,x,z):return md.Manifold.cylinder(y1-y0,radius,circular_segments=64).rotate([-90,0,0]).translate([x,y0,z])
def union(items):
 result=md.Manifold()
 for item in items:result=result+item
 # Eliminate near-coincident tangencies at the24box radial source unions.
 # Surface displacement is bounded below.001mm, tighter than all shop fits.
 return result.simplify(.0001)
def mesh_of(solid):
 mesh=solid.to_mesh64();return trimesh.Trimesh(np.array(mesh.vert_properties)[:,:3],np.array(mesh.tri_verts),process=True)
def convex(points):
 mesh=trimesh.convex.convex_hull(np.round(np.array(points),9));return md.Manifold(md.Mesh64(np.array(mesh.vertices,dtype=np.float64),np.array(mesh.faces,dtype=np.uint64)))
def primitive(b,T):
 if b.get('vertices'):v=np.array(b['vertices'])*1000
 else:v=np.array([np.array(b['size'])*np.array(s)/2 for s in itertools.product((-1,1),repeat=3)])*1000
 v=(v@rotation(b.get('quat',[0,0,0,1])).T+np.array(b['at'])*1000)@T[:3,:3].T+T[:3,3]*1000-DATUM
 return convex(v),v
def fixture(o):
 if o.get('boxes'):
  T=np.eye(4);T[:3,:3]=rotation(o.get('origin_quat',[0,0,0,1]));T[:3,3]=o.get('origin',o['pos']);return union(primitive(b,T)[0]for b in o['boxes'])
 a=o.get('yaw',0);T=np.eye(4);T[:3,:3]=rotation([0,0,math.sin(a/2),math.cos(a/2)]);T[:3,3]=o['pos'];return primitive(dict(size=o['size'],at=[0,0,0],quat=[0,0,0,1]),T)[0]
source={};stock=[]
for name,part in M['parts'].items():
 T=np.array(part['ref']);pieces=[]
 for i,b in enumerate(part['boxes']):
  piece,v=primitive(b,T);pieces.append(piece)
  stock.append(dict(id=f'{name}.{i:02d}',assembly=name,shape='convex'if b.get('vertices')else'box',
   size_mm=(np.array(b['size'])*1000).tolist(),center_mm=(np.array(b['at'])@T[:3,:3].T*1000+T[:3,3]*1000-DATUM).tolist(),
   orientation_matrix=(T[:3,:3]@rotation(b.get('quat',[0,0,0,1]))).tolist(),
   bounds_mm=[*v.min(0).tolist(),*v.max(0).tolist()],material=next((o.get('material','unspecified')for o in M['objects']if o['name']==name),'unspecified')))
 source[name]=union(pieces)
# The four ri=0 support/shoulder source unions are exactly regular24gons:
# their adjacent tangent boxes meet at the same outer corner. Construct this
# equivalent boundary once, avoiding sub-micron coincident triangle slivers
# at box-to-box tangencies after binarySTL float32 export.
radial_equivalence=[]
for name in('support_p','support_r','shoulder_p','shoulder_r'):
 part=M['parts'][name];b=part['boxes'][0];radius=b['size'][0]*1000;height=b['size'][2]*1000
 T=np.array(part['ref']);clean=md.Manifold.cylinder(height,radius/math.cos(math.pi/24),circular_segments=24).rotate([0,0,7.5]).transform(np.column_stack([T[:3,:3],T[:3,3]*1000-DATUM]))
 difference=(source[name]-clean).volume()+(clean-source[name]).volume()
 assert difference<.01,(name,difference)
 radial_equivalence.append(dict(name=name,symmetric_difference_mm3=difference,equivalent_regular_polygon_sides=24))
 source[name]=clean
for o in M['objects']:
 if o['name']in source or o['name']=='loose_tissue':continue
 source[o['name']]=fixture(o)
 if o['name']=='cover_tool':
  T=np.eye(4);T[:3,:3]=rotation(o['origin_quat']);T[:3,3]=o['origin']
  for i,b in enumerate(o['boxes']):
   _,v=primitive(b,T);stock.append(dict(id=f'cover_tool.{i:02d}',assembly='cover_tool',shape='stock_box',size_mm=(np.array(b['size'])*1000).tolist(),center_mm=(np.array(b['at'])*1000+T[:3,3]*1000-DATUM).tolist(),orientation_matrix=np.eye(3).tolist(),bounds_mm=[*v.min(0).tolist(),*v.max(0).tolist()],material='Aluminium'))
 else:stock.append(dict(id=o['name'],assembly='fixture',shape='stock_box',size_mm=(np.array(o['size'])*1000).tolist(),center_mm=(np.array(o['pos'])*1000-DATUM).tolist(),orientation_matrix=np.eye(3).tolist(),bounds_mm=list(source[o['name']].bounding_box()),material=o.get('material','see assembly specification')))

fixed_names=['fixed_case_walls','enclosure_frame','manual_frame','support_p','support_r','shoulder_p','shoulder_r','back_plate','back_top','latch_receiver_arm','latch_receiver_upper','latch_receiver_lower','bottom_bridge_0','bottom_bridge_1','bottom_bridge_2']
STATIC=union(source[n]for n in fixed_names)
CONTACT_DOOR=source['cover'];CONTACT_BOLT=source['latch_bolt'];SELECTOR=source['shutter']
# Explicit hinge replacement for the ideal vertical revolute joint. Dimensions
# use the closed assembly frame. Edge reliefs are confined to the hinge margin.
fixed_knuckles=[];moving_knuckles=[];fixed_relief=[];door_relief=[]
hinge_xy=[2.,490.]
for z in(20.,230.):
 for lo,hi in[(z,z+15),(z+30.4,z+45.4)]:
  moving_knuckles.append(cylinder(4,lo,hi,hinge_xy)-cylinder(1.6,lo-1,hi+1,hinge_xy))
  fixed_relief.append(cylinder(4.2,lo-.2,hi+.2,hinge_xy))
 lo,hi=z+15.2,z+30.2
 fixed_knuckles.append(cylinder(4,lo,hi,hinge_xy)-cylinder(1.6,lo-1,hi+1,hinge_xy))
 door_relief.append(cylinder(4.2,lo-.2,hi+.2,hinge_xy))
FAB_DOOR=(CONTACT_DOOR-union(door_relief))+union(moving_knuckles)
HINGE_FIXED=union(fixed_knuckles)
receiver_names=('latch_receiver_arm','latch_receiver_upper','latch_receiver_lower')
STATIC_SHELL=union(source[n]for n in fixed_names if n not in('support_p','support_r',*receiver_names))
FAB_STATIC=(STATIC_SHELL-union(fixed_relief))+HINGE_FIXED
# Backplate/shoulder clearance holes and blind insert seats fix the source's
# two otherwise face-mated support cylinders physically. Bores are inside the
# unused corecentre; the roll-bearing external polygon and shoulder stay exact.
axis_mounts=[]
for y in(125.,365.):
 through=bore_x(3.25,98,133,y,150.25)
 insert=bore_x(4.1,109,121.01,y,150.25)
 FAB_STATIC=FAB_STATIC-through
 axis_mounts.append(dict(center_YZ_mm=[y,150.25],back_clearance_diameter_mm=6.5,support_insert_seat_diameter_mm=8.2,insert_X_range_mm=[109,121],clearance_X_range_mm=[98,133],fastener='M6x30fromrear; M6brassinsertOD8x12'))
# Two top guide slots hold the bolt's yaw/X/Z without covering its working cam.
guide=box([13,186,280.3],[42,212,282.3])+box([13,210,280.3],[27,236,282.3])
slot_cut=[]
for x in(17.,23.):
 slot_cut.append(box([x-1.6,214,279.3],[x+1.6,232,283.3])+cylinder(1.6,279.3,283.3,[x,214])+cylinder(1.6,279.3,283.3,[x,232]))
GUIDE=guide-union(slot_cut)
pins=[];washers=[]
FAB_BOLT=CONTACT_BOLT
for x in(17.,23.):
 FAB_BOLT=FAB_BOLT-cylinder(.99,275.4,279,[x,232])
 pins.append(cylinder(1.,275.5,278.5,[x,232])+cylinder(2.5,278.5,280.1,[x,232])+cylinder(1.5,280.1,282.5,[x,232])+cylinder(.8,282.5,284.2,[x,232]))
 washers.append(cylinder(2.5,282.5,282.7,[x,232])-cylinder(.85,282.4,282.8,[x,232]))
SPRING_SEAT=box([12,204,266.5],[26,206,278.5])+box([12,204,276],[14,210,282.3])
SPRING_SEAT=SPRING_SEAT-bore_y(3.8,205.6,206.1,18,272.5)
FAB_BOLT=FAB_BOLT-bore_y(3.8,228.9,229.4,18,272.5)
# Nominal ground-end spring: OD7.5,wire.5,meanD7,active4.5,total6.5,
# freeL23.8. Pocket bottoms atY205.6/229.4 carry the spring ends.
spring_rate=79000*.5**4/(8*7.**3*4.5)
# Side keepers replace otherwise missing transverse prismatic constraint.
FRONT_KEEPER=box([15,8,-7],[16.8,482,-4.2])
REAR_KEEPER=box([127.2,8,-7],[129,482,-4.2])
# Hard stops replace ideal0/180degree hinge limits, outside the working cam.
CLOSED_STOP=box([4,0,80],[6,4,100])
OPEN_STOP=box([5,486,95],[15,510,105])+box([4,500,95],[5,510,105])
FAB_STATIC=FAB_STATIC+FRONT_KEEPER+REAR_KEEPER+CLOSED_STOP+OPEN_STOP
# A buildable passive Couette rotary damper, above the cabinet. Its physical
# measured coefficient must be set on the bench; idealjoint damping is a target.
damper_bottom=303.7;housing=cylinder(31,damper_bottom,308.7,hinge_xy)-cylinder(25.5,305.7,309,hinge_xy)
housing=housing-cylinder(1.6,302.7,309,hinge_xy)
lid=cylinder(31,308.7,310.7,hinge_xy)-cylinder(1.6,307.7,311.7,hinge_xy)
rotor=cylinder(25,306.2,308.2,hinge_xy)-cylinder(1.5,305.2,309.2,hinge_xy)
for deg in(0,90,180,270):
 xy=np.array(hinge_xy)+28*np.array([math.cos(math.radians(deg)),math.sin(math.radians(deg))])
 housing=housing-cylinder(1.6,303,309,xy)
 lid=lid-cylinder(1.1,308,312,xy)
# 3IDx1csO-rings in .8mm axial glands, ID3.2/OD5.6. They seal the shaft;
# their real drag is part of the measured Coulomb target, not a fixed claim.
housing=housing-(cylinder(2.8,303.7,304.5,hinge_xy)-cylinder(1.6,303,305,hinge_xy))
lid=lid-(cylinder(2.8,309.9,310.7,hinge_xy)-cylinder(1.6,309,312,hinge_xy))
# The upper O-ring gland needs a real axial retainer. The mounting plate
# captures the lower gland. Blind lidtaps do not break into the fluid cavity.
seal_cap=cylinder(8,310.7,311.7,hinge_xy)-cylinder(1.6,310,313,hinge_xy)
for x in(-4.,8.):
 seal_cap=seal_cap-cylinder(.85,310,313,[x,490])
 lid=lid-cylinder(.625,309,311,[x,490])
mount=box([-30,455,300.7],[36,525,303.7])-cylinder(1.6,300,305,hinge_xy)
for x,y in[(30,460),(30,468)]:mount=mount-cylinder(1.7,300,305,[x,y])
# Four-mm-long M3 inserts fit the actual4mm ABS top; finish to supplier fit.
for x,y in[(30,460),(30,468)]:FAB_STATIC=FAB_STATIC-cylinder(2.25,296.49,300.51,[x,y])
mount_fastener_envelopes=union(cylinder(3.5,303.7,307.7,[x,y])for x,y in[(30,460),(30,468)])
mount_head_interference=(mount_fastener_envelopes^(housing+lid)).volume()
assert mount_head_interference<.001,'Damper mount washer/head intersects cup'
shaft=cylinder(1.5,15,312,hinge_xy)
source_cad={'cabinet_contact_envelope':STATIC,'cover_contact_envelope':CONTACT_DOOR,'bolt_POM_contact_envelope':CONTACT_BOLT,'selector_contact_envelope':SELECTOR,'manual_frame_contact_envelope':source['manual_frame'],'receiver_contact_envelope':union(source[n]for n in['latch_receiver_arm','latch_receiver_upper','latch_receiver_lower']),'support_pair_contact_envelope':union(source[n]for n in['support_p','support_r','shoulder_p','shoulder_r']),'closing_tool_contact_envelope':source['cover_tool']}
fabricated={'cabinet_with_joint_hardware':FAB_STATIC,'cover_with_hinge_knuckles':FAB_DOOR,'bolt_with_mounting_bores':FAB_BOLT,'bolt_guide_plate':GUIDE,'bolt_guide_pins_pair':union(pins),'bolt_guide_washers_pair':union(washers),'spring_reaction_seat':SPRING_SEAT,'selector_front_keeper':FRONT_KEEPER,'selector_rear_keeper':REAR_KEEPER,'hinge_fixed_knuckles':HINGE_FIXED,'hinge_shaft_D3_L297':shaft,'hinge_damper_cup':housing,'hinge_damper_lid':lid,'hinge_damper_rotor':rotor,'hinge_damper_mount':mount,'hinge_closed_stop':CLOSED_STOP,'hinge_open_stop':OPEN_STOP}
fabricated['hinge_damper_shaft_seal_cap']=seal_cap
fabricated['hinge_moving_knuckles']=union(moving_knuckles)
for name,y in(('support_p',365.),('support_r',125.)):
 fabricated[name+'_with_insert_bore']=source[name]-bore_x(3.25,98,133,y,150.25)-bore_x(4.1,109,121.01,y,150.25)
for name,y in(('shoulder_p',365.),('shoulder_r',125.)):
 fabricated[name+'_with_M6_clearance']=source[name]-bore_x(3.25,120,133,y,150.25)
# The source closingtool neck/bridge meet along a line, not a physical bonding
# face. An explicit2mm weld diamond gives positive-volume overlap into both.
tool_obj=next(o for o in M['objects']if o['name']=='cover_tool')
tool_origin=np.array(tool_obj['origin'])*1000-DATUM
weld=convex([[x,y,z]for x in(-4,4)for y,z in[(54,4),(56,2),(58,4),(56,6)]]).translate(tool_origin)
fabricated['closing_tool_neck_bridge_weld']=weld
fabricated['closing_tool_with_weld']=source['cover_tool']+weld
receiver_upper=source['latch_receiver_upper']
# Machine a steel body with a stepped seat for the original removable upper.
# Its two small flush screws are outside the working latch/cam zone.
receiver_body=(source['latch_receiver_arm']+source['latch_receiver_lower']+GUIDE+SPRING_SEAT)-receiver_upper
for y in(188.,192.):
 receiver_body=receiver_body-cylinder(.8,277.5,284.51,[35,y])
 receiver_upper=receiver_upper-cylinder(1.1,284,289,[35,y])
 # M2 countersunk head: D3.8 flush at Z288.5, D2.2 neck at Z287.7.
 receiver_upper=receiver_upper-md.Manifold.cylinder(.8,1.1,1.9,32).translate([35,y,287.7])
for z in(264.,281.):
 # Steel receiver: tap M3 in D2.5x8 blind pilots; ABS backplate has D3.2 clearance.
 receiver_body=receiver_body-bore_x(1.25,118,126.01,190,z)
 FAB_STATIC=FAB_STATIC-bore_x(1.6,125.99,133,190,z)
receiver=receiver_body+receiver_upper
fabricated['receiver_steel_body']=receiver_body
fabricated['receiver_upper_removable']=receiver_upper
fabricated['receiver_guide_spring_subassembly']=receiver
fabricated['cabinet_with_joint_hardware']=FAB_STATIC
checks=[]
def export(name,solid,category):
 assert solid.status()==md.Error.NoError,(name,solid.status())
 mesh=mesh_of(solid);assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume>0,name
 path=STL/(name+'.stl');mesh.export(path)
 # Read the actual binarySTL back; checkmm bounds and volume after roundtrip.
 back=trimesh.load(path,process=True);assert back.is_watertight and back.is_winding_consistent
 error=float(abs(back.bounds-mesh.bounds).max());assert error<.001
 checks.append(dict(name=name,category=category,watertight=True,consistent_winding=True,volume_mm3=float(back.volume),bounds_mm=back.bounds.tolist(),components=len(back.split()),triangles=len(back.faces),roundtrip_bound_error_mm=error,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
for name,solid in source_cad.items():export('source_'+name,solid,'source_exact_contact_geometry')
for name,solid in fabricated.items():export(name,solid,'fabrication_hardware_or_mount_relief')
# One assembly transform table. The manufacturingSTLs are in the same closed
# datum, so a part needs no guessed rotation/offset at assembly.
closed_scene=trimesh.Scene()
display={'cabinet':FAB_STATIC,'steel_receiver_guide_spring_unit':receiver,'door':FAB_DOOR,'selector':SELECTOR,'bolt':FAB_BOLT,
 'pins':union(pins),'washers':union(washers),'axle_left':fabricated['support_p_with_insert_bore'],
 'axle_right':fabricated['support_r_with_insert_bore'],'damper_cup':housing,
 'damper_lid':lid,'damper_rotor':rotor,'hinge_shaft':shaft,'damper_mount':mount,'shaft_seal_cap':seal_cap,
 'closing_tool':fabricated['closing_tool_with_weld']}
for name,solid in display.items():closed_scene.add_geometry(mesh_of(solid),node_name=name)
closed_scene.apply_scale(.001) # glTF specifies metres; STL/drawings remain millimetres.
closed_scene.export(OUT/'closed_assembly.glb')

# Finite motion/interference checks for actual extra hardware. The hinge pin
# sweeps are measured without the intentionaljournal/bushing fitpairs.
sweep=[]
extras=GUIDE+SPRING_SEAT+FRONT_KEEPER+REAR_KEEPER+CLOSED_STOP+OPEN_STOP+HINGE_FIXED+mount+housing+lid
for deg in np.linspace(0,180,181):
 a=-math.radians(float(deg));R=rotation([0,0,math.sin(a/2),math.cos(a/2)]);c=np.array([2.,490.,0.]);door=FAB_DOOR.transform(np.column_stack([R,c-R@c]))
 overlap=(door^extras).volume()
 if overlap>.001:sweep.append(dict(angle_deg=float(deg),interference_mm3=overlap))
bolt_sweep=[]
for q in np.linspace(0,18,73):
 moving=(FAB_BOLT+union(pins)+union(washers)).translate([0,-float(q),0])
 # Capwashers rest.2mm above theplate; pinlowercollars rest.2mmbelow.
 vol=(moving^(GUIDE+SPRING_SEAT)).volume()
 if vol>.001:bolt_sweep.append(dict(travel_mm=float(q),interference_mm3=vol))
selector_sweep=[]
for q in np.linspace(0,240,241):
 vol=(SELECTOR.translate([0,float(q),0])^(FRONT_KEEPER+REAR_KEEPER)).volume()
 if vol>.001:selector_sweep.append(dict(travel_mm=float(q),interference_mm3=vol))
status='CAD_GEOMETRY_PASS_PHYSICAL_BUILD_NOT_YET_PERFORMED'if not(sweep or bolt_sweep or selector_sweep)else'HOLD_ADDED_HARDWARE_INTERFERENCE'
out=dict(status=status,units='millimetres',datum_world_mm=DATUM.tolist(),manifest_sha256=hashlib.sha256(args.manifest.read_bytes()).hexdigest(),radial_source_equivalence=radial_equivalence,CAD_parts=checks,source_cut_list=stock,hinge_extra_hardware_door_sweep_181poses=sweep,bolt_guide_fullstroke_73poses=bolt_sweep,selector_keepers_fullstroke_241poses=selector_sweep,spring=dict(wire_mm=.5,mean_coil_diameter_mm=7.,outer_diameter_mm=7.5,active_turns=4.5,total_turns=6.5,free_length_mm=23.8,reaction_pocket_bottoms_Y_mm=[205.6,229.4],solid_height_mm=3.25,maximum_deflection_mm=18.,compressed_length_mm=5.8,computed_rate_N_per_mm=spring_rate,target_N_per_mm=.4),damper=dict(rotor_radius_mm=25.,shaft_radius_mm=1.5,two_axial_gaps_mm=.5,target_Nm_s_per_rad=.6,dynamic_viscosity_Pa_s_for_target=.6*.0005/(math.pi*(.025**4-.0015**4)),coefficient_formula='c=pi*mu*(Ro^4-Ri^4)/h, two faces; radial rim/shear and seal drag require bench adjustment.'),boundary='Allsourcecontactsfromsubmittedmanifest. Additionalmanufacturinghardwareisexplicitandnotpresentinthefrozenphysicalsimulation. Hingeedgereliefs/boltboresarelistednonworkmountfeatures. WatertightCADandfinitemotionchecksarenotaphysicalbuild,forcecalibrationorOEMclaim.')
out['damper_mount_fastener_clearance']=dict(centres_XY_mm=[[30,460],[30,468]],conservative_head_washer_envelope_radius_mm=3.5,envelope_Z_mm=[303.7,307.7],cup_interference_mm3=mount_head_interference,minimum_radial_plan_clearance_mm=min(math.hypot(x-2,y-490)-31-3.5 for x,y in[(30,460),(30,468)]))
out['fastener_stacks']=dict(lid_M2x4=dict(lid_thickness_mm=2,thread_engagement_mm=2,tip_Z_mm=306.7,mount_clearance_mm=3),mount_M3x8=dict(washer_mm=1,mount_mm=3,standoff_mm=.2,insert_length_mm=4,thread_engagement_mm=3.8,tip_Z_mm=296.7,insert_bottom_margin_mm=.2),seal_cap_M1_6x3=dict(washer_mm=.5,cap_mm=1,thread_engagement_mm=1.5,pilot_depth_mm=1.7,bottom_margin_mm=.2),receiver_upper_M2x10=dict(upper_mm=4,thread_engagement_mm=6,pilot_depth_mm=7,bottom_margin_mm=1),receiver_rear_M3x12=dict(washer_mm=1,ABS_backplate_mm=6,thread_engagement_mm=5,pilot_depth_mm=8,bottom_margin_mm=3),wallboard_M5x40=dict(countersunk_overall_length_mm=40,backplate_mm=6,spacer_mm=6,board_mm=20,remaining_for_washer_nut_protrusion_mm=8))
board_bounds=np.array(source['wall_board'].bounding_box()).reshape(2,3)
wall_holes=[[y,z]for y in(60.,430.)for z in(35.,265.)]
board_edge_margin=min(min(y-board_bounds[0,1],board_bounds[1,1]-y,z-board_bounds[0,2],board_bounds[1,2]-z)-2.75 for y,z in wall_holes)
assert board_edge_margin>0,'M5 hole extends beyond actual wall_board'
out['wallboard_holes']=dict(board_name='wall_board',board_bounds_mm=board_bounds.tolist(),hole_axes_X_centres_YZ_mm=wall_holes,hole_diameter_mm=5.5,minimum_hole_edge_margin_mm=board_edge_margin,note='floor_service_backboard is a separate below-cabinet board; it is not the four mounting holes target.')
(OUT/'verification.json').write_text(json.dumps(out,indent=2)+'\n')
(OUT/'parts-mm.json').write_text(json.dumps(dict(datum_world_mm=DATUM.tolist(),source_cut_list=stock,CAD_parts=checks),indent=2)+'\n')
with(OUT/'cut-list.csv').open('w',newline='')as stream:
 writer=csv.writer(stream);writer.writerow(['ID','assembly','shape','sizeXYZ_mm','centerXYZ_mm','rotation_matrix','boundsXYZminXYZmax_mm','material'])
 for p in stock:writer.writerow([p['id'],p['assembly'],p['shape'],json.dumps(p['size_mm']),json.dumps(p['center_mm']),json.dumps(p['orientation_matrix']),json.dumps(p['bounds_mm']),p['material']])
# ParametricOpenSCADsourcewith exactcontactpolyhedra and explicitfabricationSTLs.
scad=['// Millimetres; closed datum: world[368,-245,787.5]mm.','part="assembly"; // or any module name below','// Controlled contact solids are generated from the submitted manifest.']
for name,solid in source_cad.items():
 mesh=mesh_of(solid);scad.append('module '+name+'(){polyhedron(points='+json.dumps(np.round(mesh.vertices,7).tolist())+',faces='+json.dumps(mesh.faces.tolist())+',convexity=20);}')
for name in fabricated:scad.append('module '+name+'(){import("stl/'+name+'.stl");}')
for name in source_cad:scad.append('if(part=="'+name+'")'+name+'();')
for name in fabricated:scad.append('if(part=="'+name+'")'+name+'();')
scad.append('if(part=="assembly"){color("lightgrey")cabinet_with_joint_hardware();color("steelblue")receiver_guide_spring_subassembly();color("silver")support_p_with_insert_bore();color("silver")support_r_with_insert_bore();color([.3,.32,.36,.7])cover_with_hinge_knuckles();color("red")selector_contact_envelope();color("ivory")bolt_with_mounting_bores();}')
(OUT/'assembly.scad').write_text('\n'.join(scad)+'\n')
print(json.dumps(dict(status=status,STL_files=len(checks),all_watertight=True,door_interference_poses=len(sweep),bolt_interference_poses=len(bolt_sweep),selector_interference_poses=len(selector_sweep),spring_rate_N_per_mm=spring_rate)))
