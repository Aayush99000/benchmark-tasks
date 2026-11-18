# Change the refill in a wall soap dispenser

A GOJO ADX-7 push-style soap dispenser hangs on a wall panel standing on the table, its front cover open. In its pump
house is an empty refill, locked in. On the robot's right the new refill stands in its case, and beyond it is a
recycling box. The instruction asks for the refill to be changed the way GOJO shows it:
1. Hold the empty refill while pushing the release latch on the front of the pump house.
2. Take the empty refill out and drop it in the recycling box.
3. Take the new refill out of its case and push its pump back into the pump house until it clicks.
4. Swing the cover up until it latches.

One hand holds the refill while the other pushes the latch.

## The real step
Janitors and building cleaners keep restroom dispensers stocked as part of their routine:
- **Jobs:** 2,432,600 jobs (BLS Occupational Outlook Handbook, [Janitors and Building
  Cleaners](https://www.bls.gov/ooh/building-and-grounds-cleaning/janitors-and-building-cleaners.htm), Number of Jobs,
  2025).
- **The duty:** the same handbook lists "Clean restrooms and stock them with supplies".
- **O*NET:** [37-2011.00](https://www.onetonline.org/link/details/37-2011.00) lists "Service, clean, or supply
  restrooms" as a core task.

GOJO's own refill video for this dispenser
([GOJO ADX-7 Manual Soap Dispenser: Refill](https://www.webstaurantstore.com/video-3389/gojo-adx-7-manual-soap-dispenser-refill.html))
says to "push the release lever to release the old refill", then "push the refill into the pump house until its
secure. Close the dispenser cover securely." In the video:
- one hand holds the bottle while the thumb of the other pushes the latch under it;
- the new refill goes in pump down, pushed back into the pump house from the front;
- the cover, hinged along its bottom edge, is swung up shut.

Done by: janitors, custodians and housekeepers.

What goes wrong in real life:
- A refill not pushed home doesn't dispense, so the restroom has no soap until someone notices.
- A cover not latched swings open.

What we simplified:
- **The dispenser:** it is fixed, 10 cm above the table (GOJO asks for at least 7.6 cm above a counter). It is
  102.5 mm deep, against GOJO's 94 mm, so a seated refill has 3 mm behind it (below).
- **The cover:** it starts open, so the dual side latches that open it (at its top) aren't used. Its hinge stops it
  70 degrees open, where a real one hangs straight down: 10 cm above the table, a hanging cover would rest on it, and
  no hand could get to its edge to swing it shut.
- **The mechanism:** the pump house, its latch and the cover's latch are a model (below). The click takes a light push
  (up to 3 N); a real one takes a firmer push.
- **The new refill:** 0.30 kg, where a full 700 mL refill is about 0.75 kg (below).
- **The refills:** they are rigid boxes with rounded upright edges on a collar. The pump's nozzle inside the pump house
  isn't modelled, and the caps are off.

## The real parts, for a real-arm trial
- **The dispenser:** GOJO ADX-7 Dispenser, white, SKU 8780-06.
- **The refills:** GOJO 700 mL ADX-7 refills, for example 8711-04 (sold four to a case).
- **Soap:** nothing in the task presses the push bar, so no soap is dispensed. A drip tray under the dispenser keeps
  the rig clean if a refill's valve weeps when it is pushed in.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=soap_dispenser_refill.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Change the soap dispenser's refill. Hold the empty refill while you push the release latch on the front of the pump house, take the empty refill out and drop it in the recycling box. Take the new refill out of its case, push its pump into the pump house until it clicks, then swing the cover up until it latches." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/soap-dispenser-refill-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=soap_dispenser_refill.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Change the soap dispenser's refill. Hold the empty refill while you push the release latch on the front of the pump house, take the empty refill out and drop it in the recycling box. Take the new refill out of its case, push its pump into the pump house until it clicks, then swing the cover up until it latches." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/soap-dispenser-refill-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/soap-dispenser-refill-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the finished state has held for 1 s; otherwise it
scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| wall panel | 40 x 40 cm, tiled, standing on the table, fixed | its face at x 43.3, centred on y 0 |
| dispenser | GOJO ADX-7 push-style, white, fixed to the panel: 10 cm wide, 24.9 cm tall, 10.25 cm deep with its cover shut; its bottom 10 cm above the table | its pump house's front at x 33.9, y 0 |
| cover | the dispenser's front cover (25 x 10.6 cm), hinged along its bottom edge 1 cm under the dispenser, open on its stop 70 degrees from shut | on the dispenser |
| pump house | at the dispenser's bottom (5 cm tall): a groove open at the front, 55 mm wide, for a refill's 50 mm collar | its groove's floor 14 cm above the table |
| release latch | blue, 2 cm wide and 2.4 cm tall, standing 4.5 mm out of a 6 x 3.4 cm well in the pump house's front; pushed 2.5 mm in (about 2.5 N) it lets the refill go; 5 mm of travel | its middle 11.95 cm above the table |
| empty refill | GOJO 700 mL ADX-7 refill, empty (60 g): a clear bottle 8.8 x 8.5 x 16.5 cm, pump down on a collar 5 cm across and 2 cm tall; locked in the pump house | in the dispenser |
| refill case | an open carton 6 cm deep (3 mm longer than a refill front to back, 2 cm wider either side) standing on a closed case 5 cm tall, fixed | (34, -13.5) |
| new refill | the same refill, full (0.30 kg), pump down | in its case |
| recycling box | open, blue, 15 x 15 cm inside and 15 cm tall, fixed | (22, -28.5) |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the pump house's model and the parts' poses.

| Score | When, at the end |
|---|---|
| 1.0 | the new refill clicked into the pump house, the empty one in the recycling box and at rest, the cover latched, and both grippers more than 2 cm from both refills and the cover, held 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.6 | the new refill clicked in and the empty one out of the dispenser, the cover not latched or the empty one not in the box |
| 0.35 | the new refill's collar at least 25 mm into the pump house (on the groove's line and floor, upright) but not clicked, the empty one out |
| 0.15 | the empty refill out of the pump house |
| 0 | anything else |

- **In the groove:** a collar is in the groove when it is within 4 mm of the groove's middle line and floor and within
  8 degrees of upright.
- **The detent and the click:** the last 3 mm before the seat take a push (a stop of up to 3 N). Within 2 mm of the
  seat, the collar clicks and the refill is locked.
- **The release:** a locked refill comes out only while the latch is pushed 2.5 mm in. If it is still at the back of
  the groove when the latch comes out, it clicks again.
- **The empty refill out:** not locked, and its collar in front of the pump house, more than 6 cm off the groove's
  line, or lifted more than 5 cm.
- **In the box:** its centre of mass over the box's inside and less than 5 cm above its rim, moving under 3 cm/s.
- **The cover:** it latches when swung within 1.5 degrees of shut, and is then held shut. Short of that it falls open
  again under its own weight.

## The pump house model
`PumpHouse` in `soap_dispenser_refill.py` runs before every physics substep (240 Hz).

**What it reads:** each refill's pose, the latch's travel and the cover's angle.

**The groove's mouth:** a collar in the mouth and within 12 mm of the line gets a sideways push towards the line, as
a real pump house's chamfer would give (up to 6 N).

**The click and the lock:**
- In the last 3 mm before the seat it pushes the collar back (3,000 N/m, up to 3 N).
- Within 2 mm of the seat it notes the click and holds the refill on its seat until the latch is pushed in. The hold is
  a spring and damper on the refill's place (capped at 40 N) and on its turn.
- Each spring and damper is capped by that refill's own mass or inertia, so the hold stays stable at the 240 Hz
  substep.

**The cover's latch:** it holds the cover shut once it latches.

**The physics:**
- The latch is a rigid body on a sprung PhysX prismatic joint in the dispenser (`fittingkit.Slider` with a drive:
  1,000 N/m).
- The cover is a rigid body on a PhysX revolute joint (`fittingkit.Hinge`, limited to its 70 degrees).
- The groove's floor, sides and back, the back plate and the top cap are the dispenser's own collision boxes.

## The scripted demo
Open loop, both arms, on one shared timeline, every pose fixed in the plan (no correction from the simulator's poses).
The right hand handles both refills; the left pushes the latch and shuts the cover.
1. **The empty refill out:**
   1. The right hand, pointing at the dispenser with its fingers across the refill, comes round the empty refill and
      holds it.
   2. The left hand comes in from its own side, pointing 70 degrees across and 20 degrees down, and pushes the release
      latch in with its closed fingertips.
   3. The right draws the refill off its seat, and the left lets the latch go and moves out to its side.
2. **Out and in:**
   1. The right draws the empty refill 9.5 cm out, lifts it clear, carries it past the front of the case and drops it
      in the recycling box.
   2. It goes up over the gap to the case, comes round the new refill and lifts it out.
   3. It brings the new refill in line 3 cm in front of the pump house and 1.5 cm high, lowers its pump to the groove
      and pushes the collar back along the groove, aimed 12 mm past the seat.
   4. It lets the refill go and draws its fingers off.
3. **The cover shut:**
   1. The left hand turns to point across, comes beside the cover and slips its closed fingers under the cover's edge,
      21 cm from the hinge.
   2. It swings the cover up through 50, 30 and 12 degrees to shut, and pushes it 8 mm past shut.
4. **Clear:** both hands move clear and wait.

**Offline check:** the poses were checked against the YAM's gripper and wrist collision hulls (URDF forward
kinematics, `plan_check.py`), including each carried refill against the dispenser, the cover, the case and the box.
- **Reach:** both arms reach every pose within 0.3 mm and 0.1 degree, and no step exceeds 83% of the per-step joint
  limit.
- **Contacts:** nothing is hit except while the left hand pushes the latch. There, the leading corner of the hand,
  turned 70 degrees, meets the pump house's face round the latch's well (up to 1.6 mm). It also meets the bottom of the
  empty refill drawn out over it (2.3 mm). The latch's stop is 0.5 mm behind that face, so a finger pushing it home
  meets the face. The check reports these on their own and the tests hold them under 2.5 mm.
- **The hands:** they stay at least 19.8 mm apart.
- **The full refill:** it is placed where the plan means it to be, with the plan's aim against the arm's give (11 mm up,
  rolled 5.7 degrees) taken off. In the simulator, pushed along the groove, its collar ran within 0.2 mm of the
  groove's line (runs soap27, soap28).

**Where the numbers come from:**
- **The hand's shape:** the YAM's closed fingertips are 6 x 14 mm at the end and flare to 46 x 67 mm 6 cm back, with
  the gripper 100 mm across behind them. That is why the latch is pushed with the hand coming in from its side and
  pointing down.
- **Gripping the refill 4.5 mm off centre:** only one YAM finger closes and the other stays at its open stop. So a held
  refill is pushed against that finger and the hand holds it 4.5 mm off its middle.
- **13 cm up:** the hand holds each refill 13 cm above its collar's bottom, above the full one's centre of mass so it
  hangs plumb. It carries the full one 11 mm higher for the arm's sag `[estimate]`, and rolled 5.7 degrees against the
  hand's own roll under its weight.

**Why the model is set where it is (each from an official run):**
- **The 0.30 kg refill:** carried at 0.75 kg, the full refill swung the YAM's soft wrist (10 N m/rad in the simulator)
  6 cm and 25 degrees off its pose. Aimed against that give, the joints reached their force limits and stopped 9 to
  11 cm short.
- **The 3 N click, 2 mm short of the seat:** holding the refill out in front, the arm pushed it no further than 42 mm
  into the groove against about 3 N, however far past the seat it was aimed (10, 12 or 22 mm).
- **The 3 mm behind a seated refill:** with the refill's back on the back plate, a degree of tilt jammed it there.
- **The capped hold:** at its first gains the hold's damping overshot each substep about the empty bottle's upright
  axis. Set turning by the closing finger, the bottle spun up and flew off.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with one addition noted in its docstring: `Slider`
takes an optional spring drive (`stiffness`, `damping`, `target`), used here for the latch. `plan_check.py` is the
offline check of the demo described above.

## Runtime and seeds
- **Official runs:** measured 2026-11-18 on an L40S with the commands above, one fixed layout.
  - **The scripted demo:** 1.0 (run soap28). The empty refill is let go at 10.61 s and the new one clicks in at
    36.27 s. The cover latches at 50.67 s, and the trial ends at 54.45 s (1,089 steps).
  - **The same demo before the right hand's last move was set** (run soap27): also 1.0, with the same event times.
  - **Dream-1:** 0.0 over 1,200 steps (run soapdream1). Its arms move about the table, but it never takes hold of
    either refill or touches the latch.
  - **Development runs:** soap1 to soap26 failed in the ways listed under "Why the model is set where it is" and in
    the code's comments.
- **The plan:** the scripted demo's plan is 56.2 s of simulated time.
- **Offline tests:** these run without the simulator: `python -m pytest -q test_soap_dispenser_refill.py`. They cover:
  - the pump house model (groove, detent, click, lock, release) and the hold's stability;
  - the cover's latch;
  - the scoring and the sizes;
  - the reach and step-guard checks;
  - the demo against the real collision hulls.
- **Seeds:** one fixed layout, `-E object_jitter=0`, so the seed doesn't change the start.
