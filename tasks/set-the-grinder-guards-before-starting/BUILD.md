# Building the grinder mock's adjustable parts

How to build the tool rest's slide, its quarter-turn cam clamp and the tongue guard's friction mount, so that
another lab can reproduce the real trial. Sizes are the simulation's (`guards.py`); millimetres throughout.

- Dimensioned drawings: `drawings/rest_slide_and_cam_clamp.png` and `drawings/tongue_guard_guide.png`
  (regenerate with `python make_drawings.py`, which reads the sizes from `guards.py`).
- 3D-printable parts: `cad/face_cam_lower.stl` and `cad/cam_lever.stl` (regenerate with
  `python make_parts_stl.py`). Print in PETG or nylon, 100% infill, cam faces up, no supports needed for the
  lower cam; print the lever with the T-handle down.
- Everything else is stock metal, standard fasteners and adhesive felt. Parts cost about US$30.

The rest of the mock (plywood motor housing, weighted aluminium disc, gear motor, sheet-steel fixed guard, START
box) is as listed in the task's setup.

## Parts list

| # | Part | Size | Material | Qty | Notes |
|---|---|---|---|---|---|
| 1 | U-channel | 25 wide x 15 high x 2 wall, 100 long (21 inside) | steel | 1 | slot 5.5 x 45 in its floor (see step 1) |
| 2 | Spacer strips | 12 x 6, 25 long | plywood or aluminium | 2 | under the channel's ends, so the bolt head clears the base |
| 3 | Felt strip | 15 x 90, 3 thick, self-adhesive | wool felt | 1 | on one inside wall of the channel: sliding friction |
| 4 | Post | 20 x 20 square bar, 50 long | steel | 1 | 5.8 hole lengthwise, 2.5 off centre (step 2) |
| 5 | Platform (the rest) | 40 x 10 flat bar, 70 long | steel | 1 | counterbore for the lower cam (step 3) |
| 6 | Clamp bolt | M5 x 90 socket head | steel | 1 | |
| 7 | Disc springs (Belleville) | M5, 10 OD | spring steel | 2 | under the bolt head, stacked in series |
| 8 | Nyloc nut and washer | M5 | steel | 1 each | on top of the T-handle |
| 9 | Lower face cam | 18 dia., 5.8 bore, 3 base + 2 ramp | printed PETG/nylon | 1 | `cad/face_cam_lower.stl`; two 2 x 2 key pins |
| 10 | Cam lever | stem 14 x 14 x 13 on a mating cam; T-bar 45 x 12 x 12 | printed PETG/nylon | 1 | `cad/cam_lever.stl`; 30 tall in all |
| 11 | Tongue | 35 x 50 x 6 plate | steel | 1 | |
| 12 | Tongue tab | 20 x 50 x 2 strip | steel | 1 | welded, or two M3 screws, down from the tongue's back edge |
| 13 | Thumbscrew | M6 x 20, knurled head | steel | 1 | into a tapped hole (or captive nut) in the tab |
| 14 | Fibre washer | M6 | fibre | 1 | between the back plate and the thumbscrew's head: friction |
| 15 | Wood screws | M4 x 25 | steel | 4 | channel and spacers to the grinder's base board |
| 16 | Countersunk screws | M4 x 12 | steel | 2 | platform onto the post |

## Making and assembling the tool rest and its clamp

1. **Channel.** Cut the U-channel to 100. In its floor, on the centre line, cut a 5.5 wide slot 45 long whose
   centre is 5 from the channel's middle, toward the wheel (drill a row of 5.5 holes and file it smooth). Stick the
   felt strip along one inside wall.
2. **Post.** Cut the square bar to 50. Drill a 5.8 hole lengthwise through it, 2.5 off its centre toward the end of
   the platform away from the wheel. Drill and tap two M4 holes in its top face, 12 apart, either side of that hole.
3. **Platform.** Cut the flat bar to 70. At 27.5 from its centre toward its outer end (the end away from the wheel)
   drill a 5.8 hole through it, counterbore it 18.5 diameter and 5 deep from the top, and drill two 2.1 holes 2.5 deep
   in the counterbore's floor, 6 either side of the centre along the platform's width, for the cam's key pins.
   Countersink two M4 holes to match the post's.
4. **Fit.** Screw the platform onto the post (M4 x 12) so the post sits under its outer end, flush with that end.
   Press the lower face cam into the counterbore, key pins in their holes, cam face up.
5. **Mount.** Set the channel on its two spacer strips, its long axis pointing at the wheel's rim, its middle under
   the post when the platform's inner edge is 25 from the rim, so the floor slot lies under the bolt's whole travel
   (from that start position to the rest touching the wheel). Screw it to the grinder's base board (four M4 x 25)
   so that the platform's top face is level with the wheel's axle (68 above the table) when the post stands on the
   channel floor. Stand the post in the channel against the felt.
6. **Clamp.** Put the two disc springs on the M5 x 90 bolt (in series: back to back), pass it up through the
   floor slot, the post, the platform and the lower cam, put the cam lever on (its cam down, meshing with the lower
   cam, T-handle along the platform), then the washer and nyloc nut.

**How it holds and locks.** Loose (T-handle along the platform), the post slides along the channel on its floor;
the felt makes it stay where it is pushed (2 to 4 N to slide it). Turning the T-handle a quarter turn clockwise
(seen from above) rides the two cams' ramps up 2 mm until the lever reaches the cams' stop walls at 90 degrees; the
last 10 degrees are flat, so the lock holds by itself. The rise compresses the disc springs, which pull the bolt
head up under the channel floor and clamp post and channel together: locked, the rest does not move under a 30 N
push. **Setting:** with the lever loose, tighten the nyloc nut until the cams just touch with no play, then back
it off an eighth of a turn; check that a quarter turn locks the rest and that turning back frees it.

## Making and assembling the tongue guard's mount

7. **Tongue.** Fix the 20 x 50 x 2 tab to the middle of the tongue's back edge (the 50 edge toward the fixed
   guard), pointing straight down, flush with that edge, so the tongue and tab form an L. Drill and tap M6 (or fix a
   captive nut) in the tab on its centre line, 20 below the tongue's lower face.
8. **Slot.** In the fixed guard's back plate cut a vertical slot 6.5 wide and 35 long, centred 27 left of the
   wheel's centre (to the robot's left), running from 23 below to 12 above the top of the wheel's rim.
9. **Fit.** Hang the tongue over the wheel with its tab in the 2.5 gap between the tongue's back edge and the back
   plate, and put the thumbscrew in from behind the back plate, through the fibre washer and the slot, into the tab.

**How it holds.** The thumbscrew presses the tab against the back plate through the fibre washer. **Setting:**
tighten it until the tongue needs 3 to 5 N to slide up or down (a luggage scale or spring gauge on the tongue's front
edge); it then stays where it is left and does not drop under its own weight (about 0.1 kg). This is how bench
grinders mount their tongue guards: a slotted tab held by a bolt.

## Before each trial

- Set the rest's inner edge 25 from the wheel's rim and the tongue's lower face 25 above the rim (a 25 spacer block),
  with the cam lever loose (along the platform).
- After a trial, measure the two gaps with feeler gauges (OSHA 1910.215: rest at most 3, tongue at most 6), and
  check the lever's position (locked: turned at least 80 degrees clockwise from loose).

## Forces for the arms

All well inside the YAM's roughly 40 N per finger: sliding the rest 2 to 4 N; turning the cam lever about 0.3 N m,
about 12 N at the T-handle's end; sliding the tongue 3 to 5 N; pressing START 3 to 5 N.

## Cost (US$, approximate)

| Item | Cost |
|---|---|
| Steel U-channel, square bar and flat bar offcuts | 12 |
| Fasteners, disc springs, thumbscrew, fibre washer | 8 |
| Felt strip | 3 |
| Printed cams and lever (about 15 g PETG) | 1 |
| Steel plate and strip for the tongue | 5 |
| **Total** | **about 30** |
