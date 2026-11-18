"""Dimension sheets in SVG; values inmm, not screenshots of a renderer."""
from pathlib import Path
import argparse,html,math,shutil,subprocess,tempfile
ap=argparse.ArgumentParser();ap.add_argument('--pdf',action='store_true',help='Also print the six A3 SVG sheets to dimensions.pdf using local Chrome/Chromium');args=ap.parse_args()
P=Path(__file__).resolve().parent/'drawings';P.mkdir(exist_ok=True)
class Sheet:
 def __init__(self,name,title):
  self.name=name;self.s=['<svg xmlns="http://www.w3.org/2000/svg" width="420mm" height="297mm" viewBox="0 0 1680 1188">',
   '<defs><marker id="arr" markerWidth="8" markerHeight="8" refX="4" refY="4" orient="auto-start-reverse"><path d="M0 0L8 4L0 8" fill="none" stroke="#222"/></marker></defs>',
   '<rect x="16" y="16" width="1648" height="1156" fill="white" stroke="#222"/>'];self.text(45,57,title,26);self.text(45,86,'P0 prototype fabrication | dimensions in mm | not to scale | datum O = world (368,-245,787.5) mm',16)
 def text(self,x,y,s,size=16):self.s.append(f'<text x="{x}" y="{y}" font-family="sans-serif" font-size="{size}" fill="#111">{html.escape(str(s))}</text>')
 def line(self,x,y,X,Y,color='#222',width=1):self.s.append(f'<line x1="{x}" y1="{y}" x2="{X}" y2="{Y}" stroke="{color}" stroke-width="{width}"/>')
 def rect(self,x,y,w,h,color='#ddd',stroke='#222'):self.s.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{color}" fill-opacity=".7" stroke="{stroke}"/>')
 def circle(self,x,y,r,fill='none'):self.s.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="#222"/>')
 def poly(self,points,fill='#ddd'):self.s.append('<polygon points="'+' '.join(f'{x},{y}'for x,y in points)+f'" fill="{fill}" fill-opacity=".6" stroke="#111" stroke-width="2"/>')
 def dim(self,x,y,X,Y,label):
  self.s.append(f'<path d="M{x} {y}L{X} {Y}" stroke="#222" fill="none" marker-start="url(#arr)" marker-end="url(#arr)"/>');self.text((x+X)/2+4,(y+Y)/2-7,label,15)
 def notes(self,x,y,lines):
  for i,line in enumerate(lines):self.text(x,y+i*25,line,16)
 def save(self):
  self.text(45,1120,'General cut +/-0.10; cam, guide pins/slots and spring faces +/-0.05 unless stated. Ream fit holes after printing.',16)
  self.text(45,1148,'STLs share closed assembly coordinates. cut-list.csv and parts-mm.json are the complete coordinate/stock schedules.',16)
  (P/(self.name+'.svg')).write_text('\n'.join(self.s+['</svg>'])+'\n')

s=Sheet('01-cabinet','01 | Cabinet and installation datums')
x,y,k=85,170,1.9
s.rect(x,y,490*k,300.5*k,'#dce4ed');s.rect(x,y,4*k,300.5*k,'#ccc');s.rect(x+486*k,y,4*k,300.5*k,'#ccc')
for Y in(125,365):s.circle(x+Y*k,y+(300.5-150.25)*k,38.5*k);s.circle(x+Y*k,y+(300.5-150.25)*k,48*k)
s.dim(x,y-30,x+490*k,y-30,'490 overall Y');s.dim(x-28,y,x-28,y+300.5*k,'300.5 Z');s.dim(x+125*k,y+330*k,x+365*k,y+330*k,'240 axes');s.text(x,y+300.5*k+75,'FRONT: Y left-to-right on this sheet; Z up. Door surface X=0..4; case starts X=5.',17)
s.notes(1080,190,['Closed contact envelope:','X 0..132; Y 0..490; Z 0..300.5','Rear plate X126..132, thickness6','Sides Y0..4 and486..490','Top Z296.5..300.5','Hinge axis: (X2,Y490), vertical','Supports: Y125 and365; Z150.25','Axle body X56..121; L65','Shoulder X121..127.5; L6.5','Support section: regular24gon','77 across flats; 77.664 across corners','Shoulder96 across flats','M6 bolt holes: X-axis at both centres','Support bore6.5 blind to X98','Insert seat D8.2,X109..121','Back/shoulder clearance D6.5','Roll seating shoulder: X121','Expected resting roll axis Z146.84'])
s.notes(85,910,['WORLD INSTALLATION: add [368,-245,787.5] to these coordinates. Cabinet bottom worldZ787.5; axisZ937.75.',
'Two1100x445x60 platforms: centres world[360,+/-427.5,717.5]; top747.5; inner edgesY+/-205.',
'Eight60x60x687.5 legs; worldcentresX=-130/850,Y=+/-245/+/-610,Z343.75. Backboard20x410x747.5.',
'Rear mounting board20x560x420.5; centre world[516,0,957.75]. See fixture rows for every remaining cut/position.',
'Fixed shell CAD is one assembly, not an instruction to print490mm across on a small printer. Cut sheet and bond subparts.'])
s.save()

s=Sheet('02-latch','02 | Machined POM cam bolt, real guide and compression spring')
ox,oy,k=130,220,16
xy=lambda X,Y:(ox+(X-7)*k,oy+(243-Y)*k)
s.poly([xy(7,229),xy(26,229),xy(26,243)],'#e5dcb5')
sq=[xy(20,231),xy(26,231),xy(26,237),xy(20,237)];s.poly(sq,'none');s.text(140,185,'TOP VIEW of original contact profile; extrude Z266.5..278.5 (12)',18)
s.dim(*xy(7,226),*xy(26,226),'19');s.dim(*xy(29,229),*xy(29,243),'14');s.text(120,540,'Vertices (X,Y): (7,229),(26,229),(26,243). Slope atan(14/19)=36.384 degrees.',17)
for X in(17,23):s.circle(*xy(X,232),.99*k,'white');s.text(xy(X,232)[0]-35,xy(X,232)[1]-28,'D1.98',13)
s.notes(760,170,['BOLT: CNC machine one POM-C solid; do not FDM-substitute cam material.','The6x6 square load-zone X20..26,Y231..237 is inside the triangle.','No separate square is glued on; original volume1596mm3; mass2.25036g.','Two top mounting bores: D1.98(+/-0.005),3.1 deep at(X17,Y232),(X23,Y232).','Both are non-working top-face mounting holes; floorZ275.4,0.10belowpeg.','Rear spring seat: D7.60,0.40 deep, axisY atX18,Z272.5,startsY229.','No hole, chamfer or rounding may change the inclined cam or side load face.','Remove burrs; cam flatness0.05; do not oil the cam/door contact.'])
gx,gy,k2=100,665,5
pt=lambda X,Y:(gx+(X-13)*k2,gy+(236-Y)*k2)
s.rect(*pt(13,236),14*k2,26*k2,'#a7bdcd');s.rect(*pt(13,212),29*k2,26*k2,'#a7bdcd')
for X in(17,23):
 px,py=pt(X,232);s.s.append(f'<rect x="{px-1.6*k2}" y="{py-1.6*k2}" width="{3.2*k2}" height="{21.2*k2}" rx="{1.6*k2}" fill="white" stroke="#222"/>')
 if X==23:s.dim(px+45,py,px+45,py+18*k2,'18 centres')
s.notes(760,570,['STEEL GUIDE:2mm plate Z280.3..282.3; bridge X13..42,Y186..212.','Slotted region X13..27,Y210..236. Two slots D3.20 end caps,','centres(X17,Y214..232),(X23,Y214..232); overall length21.20.','Guidepin peg D2.00(+/-0.005),Z275.5..278.5; flangeD5,Z278.5..280.1;','bearing D3.00,Z280.1..282.5; M1.6 stud Z282.5..284.2.','D5x0.2 washers atZ282.5..282.7; M1.6 thin nuts<=1.3 high.','Removable steel upper beginsZ284.5; fit it after guide nuts.','Slots permit18.20 free travel; install0.10 stop shims at both ends,','then gauge actual0..18.00 stroke. Side clearance0.10 each; Z gap0.20.'])
s.notes(100,1000,['SPRING: wire0.50,mean coilD7.00,OD7.50,4.5 active/6.5 total turns,ground ends,freeL23.80,solidL3.25.',
'Actual pocket bottomsY205.60/229.40 atq0. Fullq18 gives L5.80,2.55mm above nominal solid; no preload.',
'k=G*d^4/(8*D^3*Na)=0.39986N/mm forG=79GPa. Target0.400N/mm(+/-5%); compression at18mm=7.20N.'])
s.save()

s=Sheet('03-selector','03 | Captured selector and lower outlet frame')
x,y,k=95,185,1.9
def panel(X,Y,w,h,c):s.rect(x+(Y-8)*k,y+(132-X)*k,h*k,w*k,c)
panel(22,8,6,474,'#aaa');panel(126,8,6,474,'#aaa');panel(22,8,110,12,'#aaa');panel(22,470,110,12,'#aaa');panel(22,230,110,30,'#aaa');panel(17,13,110,224,'#df7068')
s.dim(x,y-40,x+474*k,y-40,'474 fixed rails Y8..482');s.dim(x+5*k,y+270,x+229*k,y+270,'224 plate width Y');s.text(100,495,'BOTTOM VIEW: red selector startsY13..237; at+240 it spans253..477.',18)
s.notes(1050,175,['Panel X17..127 (110),Z-7..-4 (3)','Thumb tab X10..14,Y115..135,Z-25..-2','Bridge X10..19,Y115..135,Z-4..-2','Two clear openings:','X28..126; Y20..230 and260..470','Physical stops atY13 and477; travel240','Initial q=0 covers the Y<245 aperture.','Do not add automatic spring or pawl.'])
s.text(90,595,'SECTION AT Y125 (initial covered side), looking along +Y',19)
xx,yy,kk=95,860,7
rectxz=lambda X,Z,w,h,c:s.rect(xx+(X-8)*kk,yy-(Z+h+25)*kk,w*kk,h*kk,c)
rectxz(15,-9,14,2,'#999');rectxz(22,-3,6,3,'#999');rectxz(125,-9,4,2,'#999');rectxz(126,-3,6,3,'#999');rectxz(129,-9,2,8,'#999');rectxz(17,-7,110,3,'#df7068');rectxz(15,-7,1.8,2.8,'#70a8be');rectxz(127.2,-7,1.8,2.8,'#70a8be')
s.notes(100,925,['NEW BLUE KEEPERS: front X15..16.8; rear X127.2..129; Y8..482; Z-7..-4.2.',
'They sit inside the original track envelope and below the thumbtab bridge; lateral gaps0.20+0.20.',
'The original lower tracks support Z=-7; upper rim begins Z=-3. The3mm plate has1.0mm vertical play.',
'Bond keepers after fitting the plate. Bond one4mmY endstop last, so the plate can be inserted from the end.',
'Original10frame piece sizes/positions are every manual_frame.* row of cut-list.csv; no dimensions are inferred from this drawing.'])
s.save()

s=Sheet('04-hinge-damper','04 | Real vertical pin hinge, edge reliefs and passive rotary damper')
x,y,k=170,170,2.8
s.line(x,y,x,y+300.5*k,'#c33',2)
for z in(20,230):
 for lo,hi in[(z,z+15),(z+30.4,z+45.4)]:s.rect(x-12,y+(300.5-hi)*k,24,(hi-lo)*k,'#82a4b7')
 s.rect(x-12,y+(300.5-(z+30.2))*k,24,15*k,'#ddd')
s.text(70,145,'Elevation: hinge axis X2,Y490; stainless D3 shaft Z15..312 (L297)',18)
s.notes(330,185,['Two hinges, module starts Z20 and230:','moving knuckles Zs..s+15 ands+30.4..s+45.4;','fixed middle knuckle Zs+15.2..s+30.2.','All OD8.00,reamed boreD3.20; shaftD3.00.','PTFE thrustwashers D7.8/ID3.2/t0.20,','atZ50.2..50.4 and260.2..260.4 support door.','Bond shaft only into moving knuckles and rotor;','fixed middle bores remain lubricated journals.','Relief radius4.20 around axis: remove door-edge','material only at fixed knuckle heights(+/-0.20);','remove case-edge material only at moving heights.','This hinge margin is explicitly omitted by simulation.','Stop0: X4..6,Y0..4,Z80..100.','Stop180: X4..15,Y486..510,Z95..105.','Set with angle gauge before bonding stops.'])
s.text(900,155,'Damper section through shaft; dimensions in Z',19)
dx,dy,ks=1250,500,6
rz=lambda X,Z,w,h,c:s.rect(dx+X*ks,dy-(Z-300)*ks,w*ks,h*ks,c)
rz(-31,303.7,62,2,'#bbb');rz(-31,305.7,5.5,3,'#bbb');rz(25.5,305.7,5.5,3,'#bbb');rz(-25,306.2,50,2,'#79a8bc');rz(-31,308.7,62,2,'#bbb');rz(-1.5,300,3,12,'#666')
s.notes(870,620,['Cup OD62,innerD51,base2.0,totalheight5.0.','Cup Z303.7..308.7; lidt2,Z308.7..310.7.','Rotor D50,t2,Z306.2..308.2,shaft boreD3.0.','Actual fluid gaps:0.50 below,0.50 above,0.50 radial.','Four M2x4 lidbolts onR28,at0/90/180/270deg.','Cup insertboresD3.2,lidclearanceD2.2.','Two O-rings3IDx1CS; glandsID3.2/OD5.6,','depth0.80 atcupbottom/lidtop; shaft boreD3.2.','Mount66x70x3; M3x8+OD7x1washers/4mminserts,','Z300.7..303.7; M3 holes(X30,Y460),(X30,Y468).','Nominal silicone dynamic viscosity244.5Pa.s','at22C gives c~0.600Nms/rad from both faces.','Measure combined rim/seal drag; adjustfluid blend.','Upper O-ring: capOD16/ID3.2/t1,Z310.7..311.7;','M1.6x3+0.5washers:1.5engagement,.2bottommargin.','Two screwsX-4/8,Y490; blind taps1.7; passive shear.'])
s.save()

s=Sheet('05-assembly','05 | Assembly order and non-working mounting details')
s.notes(65,150,['1. Cut/machine the fixed ABS sheet pieces using cut-list.csv. Fixture at the closed datum; solvent-bond only fixed-to-fixed overlaps.',
'   Machine separate steelreceiverbody114x64x32; its original steelupper18x64x4 remains off until the pins and nuts are assembled.',
'2. Fit M6 brass inserts OD8x12 into both65mm axles atX109..121. Through-bore shoulders/backplateD6.5.',
'   Insert M6x30 bolts from rear before wallboard mounting. Tighten to1.5Nm maximum into inserts; check support axes240mm apart.',
'3. Assemble lower rails with blue keepers. Insert3mm selector from one Y end; bond the last endstop. Check240.0+/-0.2 stroke.',
'4. Machine single POM-C triangle/contained square; finish the two top mounting bores and rear shallow spring pocket.',
'   Install guidepins in bolt; bring bolt/pins up from below the D3.2slots; fit0.2washers/M1.6thin nuts before receiver upper final closure.',
'5. Steelguide/seat are continuous with steelbody. Fix body from rear with2xM3x12+1mmwashers intoM3tapped8mm pilots.',
'   Fit0.10mm end shims in slots and gauge18.0mm stroke; fit0.400N/mm springL23.80 between pocket bottomsY205.60/229.40.',
'   Fit steelupper last:2xM2x10 flushheadsD3.8 atX35,Y188/192; D2.2clearance,bodyD1.6x7tap;6mmengagement.',
'6. Machine the hinge reliefs in the narrow edge margin. Bond fixed and moving knuckles with a temporary D3 alignment shaft.',
'   Mask fixed journals against adhesive; add PTFE thrustwashers; bond real steel shaft to moving knuckles after the door turns freely.',
'7. Install the above-cabinet passive damper. Add two shaftO-rings,rotor keyed/bonded to shaft,0.5mm fluid gaps and a flush lidgasket.',
'   Fit four M2x4 lid screws into brass inserts. Mount with2xM3x8+1mmwashers into4mm inserts;0.2mm stand-offs.',
'8. Fill with silicone fluid; set damping and friction by the force/angle procedures in ASSEMBLY.md. Oil may not reach cam or tissue.',
'9. Set mechanical0/180stops with an angle fixture. Apply force only at existing door broadface; no spring may self-close the door.',
'10. Mount with4xM5x40csk,6mmspacers and1mmwashers/nuts:6mmplate+6mmgap+20mmboard. Prepare rolls/tail last.',
'11. Open to90degrees; selector at q0/right covered; old roll right,new roll oncarton. Thread real tail through the open left outlet.',
'    Root planeX34.64..121,Y175.6625,Z176.09; freeedgeY350.6625,Z-127.0189 at declared initial construction (gravity changes it).',
'12. Benchcheck all constraints and record measurements before a physical trial. The STL/drawing package has not been physically built.'])
s.text(65,670,'Fasteners and joints that simulation omits',23)
s.notes(65,710,['M6axle bores/fasteners lie inside the unused core centre; support outercontact polygon/shoulder face remain the provided profile.',
'Hinge reliefR4.2 is confined to the Y~490 edge, away from robot closing contact and latch. Knuckles have explicit real pin/bearing fits.',
'Bolt topbores/pins and rear springseat are outside the working cam; guide upperhardware stays below receiverZ284.5.',
'All external source contact CAD is supplied separately for comparison. Added parts/masses/friction are manufacturing approximations,',
'not secretly present in the native evaluation. No exact force/inertia equivalence or completed physical demonstration is claimed.'])
s.text(65,900,'Manufacturing hand-off',23)
s.notes(65,940,['Mill POM bolt and ream bearingfits; ABS shell may be routed sheet plus bonded printedbrackets or CNC/large-format printed.',
'Do not use an STL viewer preview as a tolerance check. Inspect dimensions in mm and the verification.json roundtrip results.',
'36closedSTLs,assembly.scad,closed_assembly.glb,parts-mm.json,cut-list.csv,BOM.csv and ASSEMBLY.md form this package.'])
s.save()

s=Sheet('06-closing-tool','06 | Aluminium closing tool and explicit neck/bridge weld')
s.text(80,145,'SIDE VIEW in tool-local Y/Z. Prefer one-piece CNC from60x102x85 aluminium blank; do not scale this view.',18)
ox,oy,k=100,750,6
rect=lambda Y,Z,w,h,c:s.rect(ox+(Y+32)*k,oy-(Z+20+h)*k,w*k,h*k,c)
for Y,Z,w,h,c in[(-32,-15,40,30,'#ccc'),(8,-4,48,8,'#aab7c4'),(56,4,8,8,'#ccc'),(54,6.5,12,15,'#ccc'),(50,15,20,50,'#ccc'),(-10,-20,80,5,'#ccc')]:rect(Y,Z,w,h,c)
pp=lambda Y,Z:(ox+(Y+32)*k,oy-(Z+20)*k)
s.poly([pp(54,4),pp(56,2),pp(58,4),pp(56,6)],'#e3a04c')
s.dim(ox,oy+45,ox+102*k,oy+45,'102 overall Y');s.dim(ox+650,oy,ox+650,oy-85*k,'85 overall Z')
s.notes(910,175,['Tool-local origin = CAD(-208,745,-20).','Foot:60X x40Y x30Z at(0,-12,0).','Neck:8X x48Y x8Z at(0,32,0).','Bridge:28X x8Y x8Z at(10,60,8).','Riser:8X x12Y x15Z at(20,60,14).','Handle:18X x20Y x50Z at(20,60,40).','Base:50X x80Y x5Z at(0,30,-17.5).','Source constituents of CNC union.'])
s.text(910,460,'ENLARGED ZERO-AREA JOIN',20)
ox2,oy2,k2=1000,690,30
pt=lambda Y,Z:(ox2+(Y-54)*k2,oy2-(Z-2)*k2)
s.poly([pt(54,4),pt(56,2),pt(58,4),pt(56,6)],'#e3a04c');s.circle(*pt(56,4),4,'#c33')
s.notes(910,745,['Original neck endsY56/topZ4; bridge startsY56/bottomZ4.','Source contact is only an8mm X-edge, with no mating area.','New web: X-4..4, YZdiamond(54,4),(56,2),(58,4),(56,6).','2mm web integral to CNC part; weld if using stock route.','Envelope64mm3; netnew union32mm3; unmeasured mass.','Fabricated combined tool must have1connectedwatertight body.','Leave foot/handle exterior faces unchanged; proof-load20N.'])
s.notes(80,990,['Optional stockroute: mill riser notches before welding (tool-local coordinates):',
'X16..24,Y56..64,Z6.5..12 removes352mm3 for bridge; X16..24,Y54..66,Z15..21.5 removes624mm3 for handle.',
'Source toolSTL retains its2shells as evidence. Use closing_tool_with_weld.stl for one-piece machining,',
'not slicer auto-repair. The small added web and machining/fastening details are omitted from simulation.'])
s.save()
if args.pdf:
 chrome=shutil.which('google-chrome')or shutil.which('chromium')or shutil.which('chromium-browser')
 if not chrome:ap.error('--pdf requires local Google Chrome or Chromium')
 with tempfile.TemporaryDirectory(prefix='dispenser-drawings-')as folder:
  temp=Path(folder);page=temp/'dimension-sheets.html'
  sheets=sorted(P.glob('[0-9][0-9]-*.svg'))
  page.write_text('<!doctype html><meta charset="utf-8"><style>@page{size:A3 landscape;margin:0}body{margin:0}img{display:block;width:420mm;height:297mm;page-break-after:always}img:last-child{page-break-after:auto}</style>'+''.join('<img src="'+s.as_uri()+'">'for s in sheets))
  subprocess.run([chrome,'--headless=new','--no-sandbox','--disable-gpu','--allow-file-access-from-files','--user-data-dir='+str(temp/'chrome-profile'),'--no-pdf-header-footer','--print-to-pdf='+str(P/'dimensions.pdf'),page.as_uri()],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
print(P)
