# Scan a patient's wristband with its barcode turned under the wrist

A training forearm lies palm down on an arm board. Its identification band has turned on the wrist, as bands do, so
the label with the barcode is on the palm side, face down on the board, where no scanner can see it. The forearm
turns about its long axis like a patient's and is limp: let go, it rolls back palm down. A cordless handheld scanner
lies on the table.

To read the band the robot has to do what a nurse does at the bedside: take the patient's hand in one hand, roll the
forearm over until the label faces out, and keep holding it there while the other hand picks up the scanner (the
grip pulls its trigger), brings its window to reading distance, square enough to the label, and holds it steady
until the scanner reads. Then the scanner goes back on the table and the arm is laid back down, not dropped.

The scanner is not a proximity switch. It reads by its manufacturer's published figures: reading range, field of
view, tilt tolerance, motion tolerance, a clear line of sight, and a decode session that times out 9.9 s after the
trigger is pulled.

## Files

| File | What it is |
|---|---|
| `task.py` | Layout, objects, the scanner's reading rule, the trial's record and the score |
| `bandkit.py` | The training arm on its sprung pivot, the wristband and its symbol, the scanner with its trigger; geometry helpers |
| `demo.py` | Scripted demo and control runs (its own runner: the grips are not straight-down picks) |
| `test_scoring.py` | 16 offline tests of the reading rule and the score (no simulator) |
| `task.json` | The task's details for the board |

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (on the program's launchable: `source /data/IsaacLab-Arena/yam.env`).
No other packages. No third-party assets: every part is built from boxes, cylinders and round-ended rods in
`bandkit.py`. The band's Data Matrix was generated once with `ppf.datamatrix` (MIT licence) and is stored in
`bandkit.py` as 22 rows of bits; no font, image or mesh file is used.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Scan the barcode on the training arm's wristband with the handheld scanner. Then put the scanner down on the table and lay the arm back palm down." \
  --scorer task_score --max-steps 1200 --log-dir ~/yam-eval/band-dream --store-frames
```

The second layout: put `BAND_LOT=B` in front of the command (the arm 1.5 cm further out and 1.5 cm to the right,
the scanner 1.5 cm nearer and 1.5 cm to the left). Layouts are fixed; `object_jitter=0`.

## Scripted demo

```bash
python demo.py task.py out_dir                 # full demo; writes one MP4 per camera, setup.png and result.json
BAND_LOT=B python demo.py task.py out_dir_b
python demo.py task.py out_dir --mode nohold   # controls: nohold, letgo, turn, keep, drop, still
```

Open loop: every pose comes from the layout constants in `task.py`; object states are read only for the report.

1. The left hand comes in level from the left, fingers one above the other, and takes the training arm's hand by
   its fingers. A quarter turn of its wrist rolls the forearm over (at up to 53 deg/s) until the thumb is up and the
   label faces the arms. It keeps holding.
2. The right hand comes straight down on the scanner's handle and closes on it, one finger on the trigger and one
   on the back of the handle: the trigger is pulled and the decode session starts. It lifts the scanner to the
   wrist's height and brings its window in line with the label, 9 cm away, and holds still. The scanner reads
   (step 223, 11.2 s; 3.6 s after the trigger pull) and its good-decode light turns green.
3. The right hand sets the scanner back down and lets go.
4. The left hand rolls the forearm back palm down, lets go and backs away. Success at step 468 (23.4 s).

## The scanner's reading rule

The scanner is modelled on a cordless healthcare imager, the Zebra DS2278-HC (17.5 x 6.6 x 9.0 cm, 214 g). No light
or image is simulated; a read is decided each control step from the poses of the scanner, the forearm and the
grippers, using the figures below. All five points of the symbol (its centre and four corners, which follow the
band's curve) must pass every geometric test.

| Condition | Value | Source |
|---|---|---|
| Trigger | A pull (3 mm of the 5 mm travel) opens a decode session; it ends at a decode, at release, or after 9.9 s. A timed-out session reads nothing until the trigger is released and pulled again | Product reference guide: Trigger Mode "Standard (Level)"; Decode Session Timeout, default 9.9 s |
| Reading range | 0.8 to 15.7 cm from the scan window to the symbol | Specification sheet: Data Matrix, 10 mil |
| Field of view | Within 16.4 deg of the reading direction across the imager and 12.4 deg up and down | "32.8 deg H x 24.8 deg V nominal" |
| Tilt | The band's surface at the point no more than 65 deg from facing the window | Skew and pitch tolerance +/- 65 deg |
| Roll | Any | Roll tolerance 0 to 360 deg (the gripper holds the scanner on its side) |
| Motion | The symbol moving under 13 cm/s across the scanner's view | Motion tolerance, handheld |
| Line of sight | The straight line from the window to the point passes through neither the arm board, the training hand nor either gripper | Geometry |
| Steady view | All of the above without a break for 0.25 s | Our figure: a decode is not instant |

Then the scanner has read: its good-decode light goes green for 2 s (the real one beeps and flashes green).

What this makes of the scene, checked in `test_scoring.py` over 1,632 scanner poses all round the symbol (every
15 deg round it, every 10 deg of height, 2 to 14 cm away, each pointed straight at it): with the forearm palm down,
or turned 10 deg, no pose reads the label; turned a quarter turn, 304 poses do. The forearm has to be turned about
20 deg before any pose reads at all.

## Scoring

Every control step (50 ms) the task records the forearm's angle and speed, the trigger, the scanner's pose and the
grippers' poses. At the end of the run:

- **1.0**: the scanner read the band; the forearm was never turned faster than 200 deg/s; and at the end the
  scanner lies on the table and the forearm is back within 10 deg of palm down, both let go and at rest.
- **0.75**: read, and never turned too fast, but not everything is back (something still in hand, the scanner not
  on the table, or the forearm still turned).
- **0.5**: read, but the forearm was turned faster than 200 deg/s at some time (dropped or jerked).
- **0.25**: no read, but the forearm was turned at least 60 deg and kept there for 1 s, never too fast.
- **0** otherwise.

Definitions. *Too fast*: the forearm's turning speed above 200 deg/s at any control step; let go from a quarter
turn it rolls back by itself at up to 380 deg/s, and the demo never exceeds 53 deg/s. *Lies on the table*: the
scanner's lowest point within 5 mm of the table top, moving under 0.02 m/s and 0.2 rad/s. *Let go*: each gripper's
grasp point at least 30 mm from the thing. *Back*: the forearm within 10 deg of palm down and turning under
10 deg/s. Success is a score of 1.0 held for 1 s.

In a real trial the scanner itself decides the read: its beep and green flash, and the record it sends its host.
The forearm's speed is judged from video filmed end-on against a protractor dial: too fast is a change of more than
10 degrees between two frames 50 ms apart (see "Measuring 'too fast' from video" below). Its final angle is read
with a protractor or a level laid on the back of the hand, and the release from a photo.

## Verification

| Run | Score | What happened |
|---|---|---|
| Scripted demo, layout A | 1.0 | Read at step 223 (window 9.1 cm from the label, 3 deg off the reading direction, tilt 8.5 deg); success at step 468 |
| Scripted demo, layout B | 1.0 | Same script; read at step 214, success at step 468 |
| Control `nohold`: scanner aimed at the band, forearm not turned | 0.0 | Trigger pulled, no read: the label is face down on the board |
| Control `letgo`: forearm turned, let go, then scanned | 0.0 | The forearm rolled back palm down (at 381 deg/s) before the scanner arrived; trigger pulled, no read |
| Control `turn`: forearm turned, held 2 s, laid back; no scan | 0.25 | Label shown, nothing read |
| Control `keep`: read, then everything kept in hand | 0.75 | Read; nothing put back |
| Control `drop`: read, then the turned forearm let go | 0.5 | Read; the forearm rolled back at 382 deg/s |
| Control `still`: both arms still | 0.0 | |
| Dream-1, 1,200 steps, layout A | 0.0 | The left arm stayed folded near its base. The right gripper hovered over the scanner, then over the training arm's wrist and hand, without picking the scanner up or turning the forearm. Nothing was read |

`python -m pytest test_scoring.py`: 16 tests pass (range, field of view, tilt, line of sight, trigger and session
timeout, steady view, motion, every score tier, the speed limit, and the put-back conditions).

## Runtime and seeds

About 90 s of wall time per demo trial on one L40S (25 s to start, then 23 s of simulated time), and about 3
minutes per 1,200-step Dream-1 run. The layouts are
fixed and nothing is random, so seeds do not matter; run with `-E object_jitter=0`.

## Where this comes up, and what it measures

Registered nurses and licensed practical nurses scan the patient's wristband before every medication (bar-code
medication administration), and phlebotomists before every blood draw; it is how the two-identifier rule of the
Joint Commission's patient-identification goal (NPSG.01.01.01) is met at the bedside. Bands turn on the wrist, and
the code is as often underneath as on top. The task measures three things together: handling a limp limb gently
with one hand (take it, turn it, hold it, lay it back), using a triggered instrument inside its working envelope
with the other, and doing both at once, in the right order, inside the scanner's 9.9 s session.

Nearby tasks, and how this differs:

- RoboTwin 2.0 `scan_object`: one arm picks up a scanner and the other picks up a box, and the two are brought
  together. Here the code is on a person: it cannot be picked up, carried to the scanner or let go, and the read
  follows the scanner's optical limits.
- TK-UJVHAJ (turn each carton's label toward a fixed scanner on a moving line) and TK-Z23MD6 (space parcels before
  a scanner arch): the scanner is fixed and the cartons are free objects.
- TK-G8L4KF (pilot, "Scan, then pack"): items are scanned and placed in a carton.
- TK-SZTAYG (clip a pulse oximeter onto a finger while steadying the hand) and TK-9DAZLJ (ear thermometer): a
  device is put on or into the manikin. Here nothing touches the patient but the holding hand; the instrument has
  to be aimed from a distance while the limb is held turned against its tendency to roll back.

## Fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control.
- The training arm is rigid: no soft skin, no joints at the wrist or fingers. A limp forearm's roll-back is a
  torsion spring at the elbow pivot (0.035 N m/rad: 0.055 N m to hold a quarter turn, which is 1.4 N at the edge
  of the hand); a real limp forearm is returned by gravity and soft tissue and more slowly.
- The band is fixed to the wrist (a band fastened snug); a loose band could also be slid round the wrist.
- The scanner is three boxes and a sprung trigger blade (500 N/m: 2.5 N at full travel). The reading rule is task
  code from the published figures above; the 0.25 s of steady view is our own figure. Print contrast, ambient
  light and glare are not modelled.
- The symbol is a real 22 x 22 ECC 200 Data Matrix (10 mil modules, 5.6 mm square) drawn module by module. It is
  too small to see in the 320 x 176 cameras: a policy finds the label by the band and its white panel.

## Real-world build

- The training arm on its sprung pivot: see **Building the arm fixture** below and the drawing `fixture.svg`.
- An arm-positioning block (vinyl-covered foam, 90 x 261 x 75 mm) under the forearm, 3 mm clear of it.
- A 1 in adult thermal wristband printed with one 10 mil Data Matrix, fastened snug with the symbol under the
  wrist.
- A Zebra DS2278-HC (or any cordless pistol-grip imager) in handheld trigger mode, paired to a host that logs
  reads, lying on its left side on tape marks. Its aimer and illumination are eye-safe LEDs.
- Reset: lay the scanner back on its marks; the forearm returns by itself; check the band has not slipped.

## Building the arm fixture

![The fixture: side view, end view of the stop, parts and the spring](fixture.svg)

The fixture is a shaft in two bearings with a torsion spring and a stop, built from catalogue parts, a block of
wood and a plywood disc. No machining is needed beyond drilling, a file and a saw. **It is a design: it has not been
built.** The spring's rate is calculated, so the check at the end is part of the build, not an option.

### Parts

| No. | Part | Size | Notes |
|---|---|---|---|
| 1 | Pillow-block bearing, two | 8 mm bore (the "KP08" size: fixing holes 42 mm apart) | Shielded, not rubber-sealed, or with the seals taken out: seal drag would keep the arm from coming all the way back |
| 2 | Steel shaft | 8 mm diameter, 70 mm long | File a small flat where each set screw bears |
| 3 | Rigid flange coupling | 8 mm bore | Joins the shaft to the arm. One of its set screws is replaced by an M4 x 16 screw that stands 12 mm proud: the stop pin |
| 4 | Shaft collar | 8 mm bore | Its set screw is replaced by an M4 x 16 screw: the pin the spring's moving leg bears on |
| 5 | Torsion spring | 1.0 mm round music wire, 12 mm mean coil diameter (11 inside, 13 outside), 7.75 turns, two straight 25 mm legs | 35 N mm/rad, calculated below. A stock spring of 0.55 to 0.65 N mm/deg that fits over an 8 mm shaft and allows 180 degrees does as well |
| 6 | Stop post | M5 x 40 screw with a rubber sleeve (fuel hose or heat-shrink) | Stands in the riser's top beside the coupling |
| 7 | Adapter disc | 9 mm plywood, 56 mm across | Bolted to the coupling's flange, screwed to the arm |
| 8 | Riser | Hardwood or aluminium block, 70 x 65 mm, height 108 mm minus the bearing's centre height (93 mm for a 15 mm centre height) | Bolted to the table, or to a base plate clamped to it (then take the plate's thickness off the riser) |
| 9 | Cover (optional) | Thin sheet or printed box, 70 x 76 x 148 mm, open toward the hand | Makes the fixture look like the simulated block and keeps fingers out |
| | Training arm | An adult injection or IV training arm cut or jointed at the elbow: forearm and hand, about 455 mm and 1.2 kg | |

Measure your bearing's centre height before cutting the riser: the one dimension that has to come out right is the
shaft's axis, 108 mm above the table. The simulated bearing block is 70 x 56 x 148 mm; this fixture under its cover is
20 mm longer at the back, away from the arm, where nothing in the task reaches.

### How it goes together

1. **Bearings on the riser.** Screw the two pillow blocks to the riser's top, in line, 21 mm apart (47 mm over
   both), their bores along the riser's 65 mm length. Slide the shaft through both. It must turn freely.
2. **Collar and spring between the bearings.** On the shaft between the bearings go the collar (4), toward the back,
   and the torsion spring, toward the arm. One spring leg bears on the collar's long screw; the other bears on an M4
   screw driven into the riser's top beside the spring. Leave the collar loose for now.
3. **Arm on the shaft.** Find the centre of the arm's elbow end (the middle of its cut face) and screw the adapter
   disc to it, centred: four 4 x 30 mm wood screws into a rigid core, or, for a foam-filled arm, glue the disc on with
   construction adhesive and add two screws. Bolt the coupling's flange to the disc, centred, marking the holes from
   the flange itself. Push the coupling onto the shaft's front end and tighten its set screws, with the long screw
   (the stop pin) leaning about 45 degrees toward the robot's side when the arm is palm down (drawing, view B).
4. **Balance.** With the spring's leg lifted off its pin, the arm should stay where it is put at palm down, at a
   quarter turn and halfway. If it rolls to one side, the arm's centre of mass is off the shaft's axis: screw a
   stack of washers to the adapter disc on the light side until it stays.
5. **Stop.** Hold the arm palm down (a small level on the back of the hand) and drive the stop post into the
   riser's top so that its rubber sleeve just touches the stop pin from the robot's side. The arm can now turn
   toward thumb up but not the other way. Check that at 170 degrees the pin still clears the riser.
6. **Zero the spring.** With the arm palm down against the stop, turn the collar on the shaft until both spring
   legs just touch their screws, then tighten it. Up to 5 degrees of wind-up at palm down is acceptable (it holds
   the arm against the stop and changes the quarter-turn force by under 6%).
7. **Arm board.** Stand the board under the forearm; pack it until a 3 mm feeler passes between it and the forearm.

### The return torque, and how to check it

The task needs 0.035 N m/rad: 0.055 N m to hold the forearm a quarter turn over. For a round-wire torsion spring the
rate per radian is `E d^4 / (64 D n)` (E the wire's modulus, d the wire, D the mean coil diameter, n the turns):
with music wire (E about 206,000 N/mm^2), d = 1.0 mm, D = 12 mm and n = 7.75 it is 34.6 N mm/rad. The rate goes with
the fourth power of the wire's diameter, so a wire 0.02 mm off changes it by 8%. That is why the fixture is accepted
on a measurement, not on the spring's label:

- **Holding force.** Hook a spring scale (or a luggage scale) to the edge of the hand, 40 mm from the shaft's axis,
  and hold the forearm a quarter turn over, pulling square to the palm. It should read 1.4 N (143 g); 1.2 to 1.6 N
  passes. Too high: use a spring with more turns or thinner wire; too low: fewer turns.
- **Return.** Let the forearm go from a quarter turn, five times. Each time it must come back to within 5 degrees
  of palm down (level on the back of the hand) and stay there. If it stops short, the bearings drag: take their
  seals out, or allow the 5 degrees of wind-up in step 6.
- **Bearing drag.** With the spring's leg lifted off, a 10 g weight hung at the hand's edge (4 mN m) should start the
  arm turning.

The simulated pivot also has a little viscous damping (0.003 N m s/rad); the bearings' grease stands in for it, and
it is not adjusted. A lighter or slimmer arm than the simulated one rolls back faster when let go; that does not
change the task, whose limit is on how fast the forearm is turned, measured as below.

### Measuring "too fast" from video

The limit is on the forearm's turning speed at any moment, not on the average over a quarter turn. Film the hand
end-on (camera on the forearm's axis, beyond the fingertips) at 60 frames a second or more, with a printed
protractor dial of at least 15 cm fixed behind the hand, centred on the axis, and a straight strip of tape along the
back of the hand as the pointer. Read the pointer's angle on frames 50 ms apart (every third frame at 60 frames a
second, every sixth at 120). **Too fast: the angle changes by more than 10 degrees between two such frames at any
time in the trial.** That is 200 deg/s averaged over 50 ms, which is what the simulator checks at each 50 ms control
step. For a forearm let go from a quarter turn the 50 ms average is within 1% of the peak speed.
