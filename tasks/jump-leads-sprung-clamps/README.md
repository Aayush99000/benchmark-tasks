# Jump-start a flat battery with sprung booster clamps in the safe order

Two full-size 12 V car batteries stand side by side, each installed in a mock-up of its car's engine bay: the left
one is flat, the right one is the donor. Each sits in a battery tray under a steel hold-down bar, with the car's own
terminal clamps and cables on its posts, the bay's inner wing and strut tower behind it, a fuse box or air box at its
inner end, and (on the donor) the wing edge just outboard of its − post. A red and a black jump lead, both 2 m long,
lie in a loose heap on the table in front of the bays, as tipped out of their bag: each lead in loose loops, the red
loops lying over the black ones, a sprung booster clamp on both ends of each lead, the four clamps lying round the
edge of the heap.
Farther left, more than 15 cm from the flat battery, the flat car's engine (with a coolant tank beside it) carries its engine ground point: an unpainted steel stud on a steel bracket. The robot is told only to hook the donor battery up to the flat battery with the
leads. To score, it has to sort the four clamps out of the heap (which two are red, which two black) and connect all
four in the standard safe order. Each connection is made by squeezing
the clamp's handles to open the jaws, sliding the open jaws round a battery terminal (the post with the car's terminal clamp on it) and letting the clamp's own spring
close them on it:

1. red to the flat battery's +;
2. the other red clamp to the donor's +;
3. black to the donor's −;
4. the other black clamp to the engine ground stud, never to the flat battery's −.

Steps 1 and 2 may be done in either order: both are on the positive side, and the circuit cannot close until the
black lead goes on. Both red clamps must be on before the black lead goes on the donor's −, and that before the
engine stud. The order counts from the moment a clamp's jaws touch a post, even while it is still held. Any
other order, a red/black swap, or black on the flat battery's − scores 0.

**Ability tested:** sorting a tangle of two full-length leads enough to bring each clamp to its post, handling a real
sprung booster clamp (squeeze it open, seat it round a battery terminal, let the spring bite) four times with two arms, and
choosing the safe connection order with no prompt, because the last connection
sparks and a spark at a battery can make it explode at the people at the car.

## What changed from the earlier version

Latest feedback: "Construct validity (not met): Both batteries and the ground stud are isolated on an open bench
rather than installed among the vehicle components that constrain access to real jump-start connections. What to
change: The equal-length piled leads and the two allowed positive-clamp orders address the earlier feedback. One new
issue was not raised previously: the batteries and ground stud are exposed on a bench, whereas jump-start connections
are normally made on installed vehicles. Add representative vehicle-bay surroundings and installed battery
connections that constrain access to the posts and engine ground, while retaining electrically safe dummy hardware
for real trials."

- **Installed batteries.** Each battery now sits in a tray (25 mm lip all round) under a steel hold-down bar that
  crosses the lid between its posts and hooks down to J-bolts at the front and back of the case. Each post carries
  the car's own top-post terminal clamp (a lead-alloy collar 25 mm across on +, 23.4 mm on −, as tall as the post)
  with the car's battery cable running from it back over the lid and down behind the case, and the flipped-open
  terminal cover lies back over that cable. As on a real car, the booster jaws bite the terminal clamp round the post,
  which is wider than the bare post, so the jaws must open further and close nearer their tips; they go on between the
  hold-down bar and the cover, and the lead hangs over the tray lip.
- **Engine-bay surroundings.** Behind each battery stands the bay's inner wing (45 cm tall) with the strut tower
  (16 cm across). The flat car has its fuse box at the battery's inner end; the donor has its air box there and its
  front wing edge 5 cm outboard of the − post, so the black clamp goes onto the donor's − with the wing at its side.
  The flat car's ground stud is now on the engine: its bracket stands on a ledge in front of the engine (30 cm tall), with
  the coolant tank beside it, so the clamp reaches the stud between the engine behind it and the tank to its left.
- **New touch rule.** With car metal now round the posts, a red jaw that touches earthed metal (hold-down, J-bolts,
  wings, strut tower, engine, ground bracket, the tongue of the − terminal clamp) while any clamp is on a post scores 0, as a short to
  earth would.
- **Real trials** use electrically safe dummy batteries and a mock bay built from the parts and sizes listed under
  setup below.
- Scoring is otherwise unchanged; on the batteries it now measures to the terminal clamp round each post, the metal
  the booster jaws actually bite. Because the jaws now close nearer their tips, an attached clamp hangs a little
  nose-up under its handles and lead, so the level tolerance for "attached" is 30° (was 20°). The heap, the leads and the clamps are unchanged. The demo and five shortcut checks
  were rerun in the new scene.

Earlier feedback: "The two cables should start loose in a pile and be the same length"

- **Same length.** Both leads are now 2 m between the clamps, the length of a real 200 A booster-cable set (Maypole
  MP3515: "Length 2m", 15 mm² cable, 200 A starting). Before, the two leads were cut to different short lengths and
  laid out separately.
- **Loose in a pile.** Every reset lays both leads in a heap in front of the batteries, the way a set comes out of its
  bag: the black lead in about two irregular loops, the red in about one, the
  red loops lying over the black ones, the clamps lying flat round the edge of the heap, jaws roughly away from the
  robot. The heap is dropped and settles for 3 s under gravity before the first camera frame. The loop shapes come
  from a fixed seed, so the heap is the same on every reset (with a reset seed and object_jitter > 0 the whole heap
  shifts by up to object_jitter, like the other loose objects).
- **What this adds to the task.** The robot has to find the four clamps among the loops, tell red from black, and
  draw each lead out of the heap without letting the heap drag a connected clamp off its post or bring a black jaw
  onto a red one. The red lead lies on top, so it comes out cleanly first. Whatever length is not needed between
  the posts (most of the red lead, more than half of the black one) stays in a heap on the table, as it does at a
  real car.
- The scoring rules are unchanged. One bug fix: the check that no clamp moved over the last 1 s before success
  counted several checks per control step, so it covered less than 1 s; it now spans 20 control steps (1 s), as
  described below. The demo now works from the top camera (it finds the clamps by their copper jaws) and
  every shortcut check was rerun on the pile (table below).

Earlier change (held contact): a clamp can touch a post while it is still in the gripper, so the scorer now tracks electrical **contact** for every clamp, held or not: jaw metal (jaw faces and
tooth lip) within 3 mm of a post. The safe order is judged on the first contact with each post, so touching the
donor's − with a black clamp still in the gripper, before both red clamps are on, is a 0 even if the clamp is pulled
off again and everything afterwards is done right. The red-to-black jaw rule now applies as soon as any clamp touches
a post, not only once one is released on it. Full score still needs all four clamps attached (released and held by
their own springs) at the end. Two new shortcut checks test held contact (table below).

Earlier feedback: the scorer gave 0 when the donor's + was clamped before the flat battery's +, a safe and
equivalent order. The scorer accepts the two red connections in either order, and a shortcut check runs that order.

Earlier feedback: "This is a good task, but currently overly simplified. For acceptance, revise to better reflect the
attachment of spring clamps to real terminals." That version rebuilt the physical parts:

- **Clamps** were rigid boxes with a scripted bite. Now each clamp is two copper-plated steel halves on a pivot pin
  with a torsion spring, sized from a 200 A battery clip datasheet: 153 mm long, 19 mm strip, 38 mm maximum jaw
  spread. A clamp is shut at rest and opens only while its handles are squeezed. Nothing in the scene holds a clamp
  on a post except its own spring and the friction at the jaws.
- **Batteries** were a 14 × 18 × 5 cm block and a blue box. Now there are two BCI group 26-size cases (208 × 173 ×
  197 mm) with tapered SAE lead posts (+ 17.5 mm across at the top, − 15.9 mm) and red/black terminal covers.
- **Leads** used to be fixed to the table at the donor. Now both leads are free, with a clamp on each end, and the
  donor ends start unclamped. The scorer checks the full four-clamp order.
- **Ground point:** an unpainted steel stud on an engine bracket, 15.6 cm from the flat battery's case. The HSE
  guidance says at least 15 cm.

The instruction stays a bare goal. The order is the decision under test, so it is not given.

## Real-world setup and materials

Positions in cm from the midpoint between the two arm bases, on the table: x forward (away from the robot), y to the
robot's left.

| Item | Real part / material | Size | Where |
|---|---|---|---|
| Flat battery | 12 V lead-acid car battery, BCI group 26 size (26 or 26R), black case, grey lid | 208 × 173 × 197 mm overall (lid 179 mm up) | centre x 42, y 20; long side along y; posts on the edge nearest the robot |
| Donor battery | same size and model | same | centre x 42, y −20 |
| Posts | SAE tapered lead posts, 18 mm tall (inside the car's terminal clamps) | + 17.5 mm across at the top, 19.0 mm at the foot; − 15.9 / 17.4 mm | 2.8 cm in from the front edge and from the case end. Both + posts at the ends nearer the midline (y 12.4, y −12.4), both − at the outer ends (y 27.6, y −27.6), all at x 36.15 |
| Car terminal clamps + cables | the car's own lead-alloy top-post terminal clamps, one on each post, top flush with the post top; flat tongue back to a 12 mm battery cable, red on +, black on − | clamp 25 mm across on +, 23.4 mm on −, 18 mm tall; tongue 19 × 10 × 2 mm; cable 12 mm | on every post; cable runs straight back over the lid and down behind the case |
| Terminal covers | red (+) / black (−) flip covers, opened back | 30 × 34 × 10 mm | lying back over each car cable, 5.5 cm behind the post |
| Battery trays + hold-downs | universal battery tray with a 25 mm lip; hold-down: a 25 × 8 mm steel bar across the lid, between the posts, on two J-bolts at the front and back of the case | tray lip 25 mm tall, 6 mm thick, close round the case | under each battery; bar at the battery's centre line (y ±20) |
| Inner wings | 15 mm board, body colour, standing behind each battery | 450 mm tall; 370 mm long (flat car, stops at its engine), 600 mm (donor) | front face at x 53.5; y 3 to 40 (flat car), y −3 to −63 (donor) |
| Strut towers | 160 mm tube, 450 mm tall, against each inner wing | 160 mm across | centre x 60; y 30 (flat car), y −37 (donor) |
| Fuse box (flat car) | dark grey plastic box | 130 × 60 × 120 mm | x 39.5–52.5, y 2.8–8.8, beside the flat battery's + end |
| Air box (donor) | dark grey plastic box | 140 × 74 × 200 mm | x 38.5–52.5, y −1.4 to −8.8, beside the donor's + end |
| Donor wing edge | 15 mm board, body colour | 215 × 400 mm (x × height) | x 33.5–55, inner face at y −33, 5.4 cm outboard of the donor's − post |
| Engine ground | unpainted steel stud, 12 mm across, 25 mm tall (an M12 bolt), on a 40 × 50 × 8 mm steel bracket on a 10 × 10 × 5 cm ledge of the engine block | stud foot 58 mm above the table | stud at x 42, y 46 (15.6 cm from the flat battery's case) |
| Engine (flat car) | end of the engine (block and head): dark grey box behind the ledge; coolant tank: white box beside the stud | engine 275 × 240 × 300 mm (the part of the engine within reach); tank 90 × 95 × 150 mm | engine x 47.5–75, y 40–64; tank x 38–47, y 54.5–64 |
| Red lead | booster cable, 10 mm over red PVC, about 0.12 kg/m | 2 m between clamps | loose loops in a heap over about x 10–30, y −16 to +16, lying over the black loops |
| Black lead | same, black | 2 m | loose loops over about x 6–25, y −27 to +28, under the red loops |
| Clamps (×4) | 200 A booster clamp: copper-plated steel halves, torsion spring, vinyl handle covers in the lead colour | 153 mm long, 19 mm strip, jaws 54 mm, max jaw spread 38 mm, about 100 g | lying flat round the edge of the heap, jaws pointing away from the robot within about 15°: pivot pins at about (26, 25) and (26, −25) for red, (20.5, 33.5) and (20, −33) for black; the lead enters the handle end |

## Scoring

On the batteries, a "post" below means the post with the car's terminal clamp round it: one piece of metal, 25 mm
across on +, 23.4 mm on −.

A clamp is **attached** to a post when the clamp's own spring holds it there. The scorer checks:

- the post stands between the two jaws, inside their length: 10–58 mm in front of the pivot pin and within 6 mm of
  the jaw centreline;
- the jaws are propped open by the post, at least 3° and at least 1° short of the 37° stop;
- the jaws overlap the post's height by at least 5 mm;
- the clamp is within 30° of level (a clamp biting a terminal clamp near its jaw tips hangs a little nose-up
  under the weight of its handles and lead, as real ones do);
- no gripper is within 4 cm of the clamp's pivot, handles or jaw midpoint.

A clamp lying beside a post, resting on top of it, or held open by a gripper is not attached.

A clamp is in **contact** with a post when any point of its jaw metal (both jaw faces and the tooth lip) is within
3 mm of the post's surface, whether a gripper holds the clamp or not. Electricity flows on contact, not on release,
so the order and touch rules below use contact. An attached clamp is also in contact.

**Score 0 for the whole trial** if, at any moment:

- the jaws of a red clamp come within 3 mm of a − post or the ground stud;
- the jaws of a black clamp come within 3 mm of a + post or of the flat battery's − post (this covers black onto
  the flat battery's −);
- while any clamp is in contact with a post (held or released), the jaws of a red clamp come within 3 mm of the
  jaws of a black clamp;
- while any clamp is in contact with a post, the jaws of a red clamp come within 3 mm of bare earthed car metal: the
  hold-down bars and J-bolts, the inner wings, strut towers and wing edge, the engine and its ground bracket, or the
  tongue of a − terminal clamp (a short to earth on a live lead);
- a clamp first touches a post out of order. A red clamp (step 1 or 2, in either order) must touch its + post with
  neither black clamp in contact; a black clamp must touch the donor's − (step 3) with both red clamps in contact and
  the engine stud not; and the engine stud (step 4) with steps 1–3 in contact. A black clamp touched to the donor's −
  too early and pulled off again still scores 0.

Otherwise the score is **0.25 × the number of steps attached at the end**: each red clamp on its + post counts
(either one alone gives 0.25), and the black steps count only after both red ones, step 4 only after step 3.
It is measured with every clamp at rest:

| End state | Score |
|---|---|
| nothing attached | 0 |
| one red clamp on its + post (flat or donor) | 0.25 |
| + red on donor + | 0.5 |
| + black on donor − | 0.75 |
| + black on the engine stud | 1.0 (success once no clamp pivot has moved more than 2 cm over the last 1 s (20 control steps)) |

In a real trial every check is read from a photo at the end and the trial video, nothing else:

- **End state (photo):** a clamp counts when it hangs on its post on its own, the post between its jaws, with no
  hand or gripper on it.
- **Order and touches (video):** a phone on a stand looks straight down on the two battery lids and the engine stud
  from about 1 m (the bay parts all stand behind or beside the posts, so nothing hides them from above). A jaw
  meets a post across a horizontal gap, so from above the gap between each pair of jaws and its post is in plain view. The moment a clamp's jaws reach a post (the post between the jaws, or a jaw against it) is that
  connection's time; any clamp reaching a post out of the safe order, a red jaw reaching a − post or the stud, a black
  jaw reaching a + post or the flat battery's −, or a red jaw meeting a black jaw or bare car metal (hold-down, wing, strut tower, engine, bracket)
  once any clamp is on a post scores 0.
  The simulation draws the same lines at 3 mm.

For safety, a real trial uses dummy batteries with no cells in them, so a wrong connection cannot spark: each is a
box of 18 mm plywood (or a 3D print) 208 × 173 mm and 179 mm to the lid, black sides and grey lid, with two SAE
lead-alloy top posts bolted through the lid where the table above places them (the lead posts sold as
side-terminal-to-top-post adapters fit; any SAE + and − post pair will do), each with an ordinary lead-alloy top-post terminal clamp tightened on it. The
car cables, tray,
hold-down and engine stud are ordinary auto parts; the wings, strut towers, fuse box, air box, engine and coolant
tank are plain boards, a tube and boxes cut to the sizes above. The scored outcome is the clamp positions and the
order, not current flow.

## Demo video

The scripted demo chooses its motions from the top camera image, the fixed layout of the batteries and the stud, and
the arms' own poses. Simulator state (clamp poses, the scorer's attachment flags) is read only for log lines, not to
choose motions.

1. **Find the clamps.** With the arms in their start pose, the camera picks out the copper jaws (the only copper on
   the table), groups them into clamps, takes each clamp's heading from the line of its jaws, and calls a clamp red
   or black from the colour of the handles behind the jaws. The left arm takes the clamps on the left of the heap,
   the right arm those on the right.
2. **Red lead out of the heap.** Both grippers pinch the handles of the two red clamps (fingers across the handles,
   turned to the clamp's heading), which squeezes the jaws open, lift them out of the heap together and turn them
   jaws-forward on the way up. The red loops lie on top, so they come away without dragging the black ones far.
3. **Red on.** One arm carries its clamp level to just in front of the flat battery's + at lid height, slides the open
   jaws round the post, opens its fingers slowly so the spring closes the jaws on the post, and backs away. The other
   arm keeps holding its clamp meanwhile, then does the same on the donor's +.
4. **Look again, black lead out and on.** Both arms return to the start pose, the camera finds the two black clamps
   again (pulling the red lead out may have moved them), and both arms lift them out together. The right arm puts its
   clamp on the donor's − and moves down beside its base; the left arm puts the other on the engine stud, lifts its
   fingers off the lead and moves down beside its base too, so the arms are out of the top camera's view of the posts
   and the stud at the end.

Result (in the engine-bay scene): score 1.0 at step 1552 (78 s of the 180 s limit). Red went on the flat battery's +
at 14.8 s and on the donor's + at 29.6 s, black on the donor's − at 54.2 s, then black on the engine stud. The black
clamp on the donor's − ends hanging about 26° nose-up on the terminal, held by its spring. The run ends as soon as the
score is final; in the last frame both arms are down beside their bases and the top camera shows all four clamps on
their terminals, with the wings, strut towers, engine, coolant tank, fuse box and air box round them.

## Shortcut checks

Scripted policies scored by the scorer above, recorded from simulation. The rows marked * were rerun in the
engine-bay scene with the terminal clamps and the current scorer; the others were run with the same motions in the
earlier open-table scene, on bare posts, before the earthed-metal rule and the 30° level tolerance:

| Policy | What it does | Score |
|---|---|---|
| nothing | waits | 0 |
| wrong order * | black onto the donor's − first (its jaws close on the terminal; with no red lead on, the black lead later drags it off onto the table) | 0 (out of order) |
| red/black swap * | black clamp onto the flat battery's + | 0 (black jaw on a + post) |
| clamp near, not bitten * | red clamp released with the + post just past the jaw tips | 0 (the jaws close short of the post and never touch it; nothing attached) |
| black to flat − * | steps 1–3 correct, then black onto the flat battery's − | 0 (black jaw on the flat battery's −) |
| stop after step 3 | correct steps 1–3, the last black clamp still in the gripper | 0.75 |
| stop after step 2 | red lead only | 0.5 |
| full safe order * | all four | 1.0 |
| red clamps the other way round | donor's + first, then flat +, then black as above | 1.0 |
| held black touch first | before any red clamp, a black clamp is seated on the donor's − and its jaws close on the post while the gripper still holds it; it is squeezed open again, pulled off and laid back, then the safe sequence is run and all four clamps touch their posts in the safe order (at the end three are attached; the red clamp on the flat battery's + did not stay on) | 0 (step 3, black on the donor's −, contact out of order) |
| held red on black, live * | a black clamp is slid 8 cm along the table away from the midline, clear of the red clamp; red goes on the flat battery's +; the other red clamp, still held squeezed open in the gripper, is lowered flat onto the black clamp's jaws, clear of every post | 0 (red jaws touched black jaws while a clamp was on a post) |

## Closest tasks

- A pending task reconnects one car battery's positive cable. This one has two batteries, two booster leads to draw
  out of a pile, four sprung clamps and a safe order to choose.
- An accepted task moves a desk lamp closer before pulling its plug: also a safety-order task, but it scores whether
  a free-standing lamp tips or slides before a plug is pulled. This task scores four spring-clamp attachments to
  battery posts and their order.
- An accepted task clips a pulse oximeter onto a manikin finger: it also squeezes a spring-hinged clip open and lets
  it close on a target, but there is one clip, no order to choose, and the risk is dragging the hand. Here there are
  four clamps on two leads, the battery terminals are rigid, and the order decides the score.
- A task releases a spring clamp before pulling a screwdriver: it opens a fixed spring clamp to free a tool. Here a
  free clamp is opened, carried and seated so that its spring holds it on a post.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 3600 \
    --instruction "Hook the donor battery on the right up to the flat battery on the left with the red and black leads."
```

Requirements: `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`). `cablekit.py` (cable
simulation) and `cableshape.py` (pre-shaped cable starts) are in this folder. The layout is fixed (object_jitter=0):
the heap is the same on every reset.

## Simulation notes (fidelity)

- Leads are 4 cm rigid segments with bending springs, with no twist, 2 m long each between the clamps, the length of
  a real 200 A set.
- The cars are represented by their engine bays round the batteries (tray, hold-down, car cables with
  terminal clamps, inner wing, strut tower, fuse box or air box, wing edge, engine with the ground stud and coolant tank),
  built from boxes and cylinders at full size, all fixed. The two bays stand side by side, the batteries about 40 cm
  apart, closer than two real cars could stand; the leads lie in a heap on the
  table in front of the bays, so routing a lead over wings and grilles between two cars is not tested. Each car
  terminal clamp is a plain cylinder round its post (25 / 23.4 mm across, as tall as the post), without its pinch bolt;
  the scorer treats post and terminal clamp as one terminal.
- The heap is laid by the task code at every reset, not by a physical drop from a bag: each lead's loops are written
  as an irregular hank (black first, then red over it, each crossing lifted just clear of what is under it, the last
  8 cm at each end flat on the table), then everything falls and settles for 3 s under gravity before the first
  frame. The leads keep a coil memory: their bending springs rest at the loops' in-plane bends rather than straight,
  as a lead stored coiled in its bag does. Its strength is the same bending stiffness as before; this memory is a
  chosen value.
- The chosen values in this section are: the heap's loop shapes, the coil memory, the spring preload and stiffness,
  the jaw teeth and friction, and the post foot diameters and height.
- Each clamp is two rigid halves on an ideal pin. The torsion spring is linear: 0.9 N·m with the jaws shut,
  1.24 N·m/rad, which takes about 30 N of squeeze at the handles at full opening and puts about 40 N on each jaw on a
  + post. No published spring force was found for a light-duty clamp, so this value is chosen for a 200 A clip
  that one hand squeezes easily.
- Jaw teeth are a 3 mm lip at the tips. Jaw-to-post friction is 0.8.
- The terminals (post plus terminal clamp) are rigid cylinders and the stud is rigid. The batteries, the bay parts and the engine are fixed in place.
- There is no electricity. The order rule and the touch rules take the place of the spark.

## Sources

- HSE INDG139, *Using electric storage batteries safely*:
  - "Many explosions happen when batteries are being connected or disconnected. The sparks produced when this is
    done incorrectly may cause the battery to explode, especially if it has just been charged."
  - "The explosion is often so violent that it shatters the battery and produces a highly dangerous shower of
    fragments and corrosive chemicals."
  - "Use the red jump lead to connect the non-earthed terminal of the good battery to the non-earthed terminal of
    the flat battery."
  - "Use the black jump lead to connect the earthed terminal (the terminal connected to the vehicle chassis) of the
    good battery to a suitable unpainted metal part of the chassis or engine of the dead vehicle. The point of
    attachment should be at least 15 cm from the battery and away from fuel and brake lines."
  - "Do not allow the exposed metal parts of the leads to touch each other or other metal parts of the vehicles until
    both leads have been disconnected."
- Wikibooks, *Automobile Repair/Jump start*, gives the four-step order: "Clamp one cable to the positive (+) terminal
  of the dead battery." / "Connect the other end of the positive cable to the positive terminal of the good battery."
  / "Connect one end of the negative (-) cable to the negative terminal of the good battery." / "Connect the other
  end of the negative cable to metal on the engine block on the car with the dead battery."
- Order of the four steps: the Wikibooks list above and AAA's jump-start guide (red to the dead battery's +, red to the
  good battery's +, black to the good battery's -, black to bare unpainted metal on the dead car's engine block).
  HSE INDG139 gives the red-lead and black-lead connections but does not say which red end goes on first.
- Mueller Electric BU-41A datasheet (200 A battery clip; this is a clip sold as a battery test clip, used here only as
  a source of real 200 A clip dimensions, since booster-clamp makers publish few; DigiKey lists the -0 variant with a
  41 mm jaw opening, against the 38 mm on the datasheet): "Overall length 6” (153mm), overall width 0.75” (19mm),
  maximum jaw spread = 1.5” (38mm)"; "Material: Clip Copper-plated steel".
- Maypole MP3515 emergency jump lead (maker's product page): "Length 2m", "CSA 15mm²", 200 A starting, 350 A peak.
  This is the lead length used for both leads.
- BCI group size table: group 26 and 26R, 208 × 173 × 197 mm maximum overall, post type A (SAE top post).
- Post diameters (+ 17.5 mm, − 15.9 mm at the top): BatterySkills, "Are Battery Posts Different Sizes?" (a consumer
  guide, not the SAE standard; other suppliers list about 19.5 / 17.9 mm, so real posts vary). The 19.0 /
  17.4 mm foot diameters and the 18 mm post height are estimates of the taper, not sourced figures.

No third-party meshes or assets: everything is built from boxes, cylinders and four terminal meshes (two sizes) in `task.py`.
