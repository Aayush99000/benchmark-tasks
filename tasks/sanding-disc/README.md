# Fit a sanding disc to a random orbital sander, its dust holes lined up

A 5-inch random orbital sander lies upside down on the bench, its hook-and-loop backing pad facing up, and a new
8-hole sanding disc stands on edge in a slotted disc stand beside it. The instruction asks for the disc to be fitted
the way the sander's maker describes: line the disc's dust holes up with the holes in the pad, put it on, and press it
on firmly. The pad sits turned 20 degrees, so a disc put on square misses every hole. One hand carries the disc over
and touches its far edge on, lined up; the other holds that edge down while the first lets go, then presses the disc
on.

## The real step
Carpenters, cabinetmakers, floor finishers, painters and auto body repairers sand every day, and change the disc on a
random orbital sander for each grit and each time one wears out. Ryobi,
[RS290 random orbit sander operator's manual, Attaching Hook-and-Loop Sanding Discs](https://www.manualslib.com/manual/143271/Ryobi-Rs290.html?page=10):
"Align the holes in the hook-and-loop sanding disc with the holes in the backing pad", "Line up the holes in the
sanding disc with the holes in the backing pad in order for the dustless feature to function properly", "Press the
fuzzy side of the sanding disc against the backing pad as firmly as possible." Done by: carpenters, cabinetmakers,
floor finishers, painters and auto body repairers. The BLS counts about 163,200 automotive body and related repairer
jobs in 2025, and lists among their duties: "Sand, buff, and prime refurbished and repaired surfaces"
([Occupational Outlook Handbook](https://www.bls.gov/ooh/installation-maintenance-and-repair/automotive-body-and-glass-repairers.htm)).

What goes wrong in real life: a disc put on turned or off centre covers the pad's dust holes, so the sander throws the
dust into the air (and into the finish) instead of the bag or extractor; a disc not pressed on firmly flies off.

What we simplified: the sander is fixed (as if held pad up on the bench) and unplugged (the maker's first step); the
old disc is already off; the pad's hook face is a model (below), and the pad doesn't turn on its bearing; the disc is a
rigid plate, 2 mm thick and 40 g (a real one is about 1 mm and 6 g), so the YAM's fingers can hold it; the disc stand
is a plain slotted block; the hole pattern's sizes are our estimate (holes 10 mm across on a circle 78 mm across: the
makers don't publish them).

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sanding_disc.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the new sanding disc to the sander's pad, grit side out. Line the disc's dust holes up with the holes in the pad, put it on, press it on firmly, then let go of it." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/sanding-disc-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sanding_disc.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the new sanding disc to the sander's pad, grit side out. Line the disc's dust holes up with the holes in the pad, put it on, press it on firmly, then let go of it." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/sanding-disc-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/sanding-disc-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the finished state has held for 1 s; otherwise it
scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| sander | a 5-inch random orbital sander, unplugged, lying upside down, fixed; its 125 mm hook-and-loop backing pad faces up, 12 cm above the table, its 8 dust holes turned 20 degrees (anticlockwise, seen from above) from square to the table; its dust port points away from the arms | the pad's middle at (28, 0) |
| disc stand | a wooden block, 5 x 9 cm and 2.5 cm tall, with a slot 4 mm wide and 15 mm deep across it | (20, -14) |
| disc | a new 5-inch (125 mm), 8-hole hook-and-loop sanding disc, standing on edge in the stand's slot, its grit side facing away from the arms and its fuzzy side towards them, one of its holes at the top | in the stand |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the hook-and-loop model and the disc's pose.

| Score | When, at the end |
|---|---|
| 1.0 | the disc caught on the pad with every one of its holes within 2.5 mm of one of the pad's, pressed on, and both grippers more than 2 cm from it, held 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | the disc caught on the pad, but its holes out of line, or not pressed on |
| 0.25 | the disc lying on the pad but not caught: tilted, or fuzzy side up |
| 0 | anything else |

- Caught: the disc comes down flat on the pad (its lowest point within 1 mm of the pad's face, tilted under 4
  degrees, its middle within 20 mm of the pad's), fuzzy side down. From then on it stays where it landed, turned as it
  landed: hook-and-loop grabs on contact, so the holes have to be lined up before it goes on.
- Lined up: every disc hole's centre within 2.5 mm of a pad hole's centre, which leaves each 10 mm hole at least
  two-thirds open (68.5%). With the disc centred, that is within 3.7 degrees of a hole's angle.
- Pressed on: a fingertip held on the caught disc, over the pad, for 0.3 s in all.
- On the pad (not caught): its middle within 4 cm of the pad's axis and under 3 cm above its face, still.

## The hook-and-loop model
`HookAndLoop` in `sanding_disc.py` runs before every physics substep (240 Hz). Until the disc is caught it is a free
rigid body that collides with the pad like any other. When it comes down flat on the pad's face, fuzzy side down, the
model notes where its middle is and how it is turned, and from then on writes that pose back every substep: on the
pad's face, where it landed. Every 0.1 s it checks the grippers' collision hulls for a fingertip on the caught disc.
The model doesn't let a caught disc be peeled off again (in real life it can be, and refitted).

## The scripted demo
Open loop, both arms, on one shared timeline, with fixed poses planned from the layout:
1. The right hand comes down over the disc's top edge, pointing down, its fingers either side of the disc, and pinches
   it 16 mm in from the edge.
2. It lifts it out of the stand and tips it forward to level in two halves: the hand ends pointing away from the arms,
   the disc held flat at its near edge, grit side up.
3. It turns it 20 degrees to the pad's holes, brings it over the pad, tips it 10 degrees down at its far side and
   touches that edge on the pad's far edge.
4. The left hand, fingers closed and pointing down, holds the far edge on while the right hand opens and backs away;
   the disc drops flat and is caught.
5. The left hand, still pressing, keeps its fingertips on the disc as it drops and is caught (that presses it on),
   then moves to the disc's middle and presses it there for 2 s, and both hands move clear.

The demo is open loop: fixed poses planned from the layout, with two small offsets measured in the first sim run
(sand1) and written into the plan as constants: held level, the disc's middle is 0.7 mm further out from the TCP than
planned, and the disc turns 0.8 degrees less than the hand. It reads nothing from the simulator while it runs.

The poses were checked offline against the YAM's gripper and wrist collision hulls (URDF forward kinematics,
`plan_check.py`, with the disc moving with the right hand while it is held): neither hand touches the sander or the
stand (other than the left fingertips pressed past the disc's face, as planned), and the hands stay at least 41.7 mm
apart. Both arms reach every pose within 0.3 mm and 0.1 degree, and no step exceeds 77% of the per-step joint limit.
Picking the disc up flat by its edge off a tin beside the sander was planned first and dropped: a level hand that low
and that far to the right is out of the right arm's reach (20 to 50 mm off), so the disc stands on edge and is picked
from above.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with one addition noted in its docstring (a spring
drive for `Slider`, not used here). `plan_check.py` is the offline check of the demo described above.

## Runtime and seeds
- Official runs (2026-11-18, the Robocurve Brev launchable, 1,200 steps, `object_jitter=0`): the scripted demo (run
  sand2) scores 1.0: the disc is caught at 19.13 s with its middle 0.8 mm (x) and 0.7 mm (y) off the pad's and its holes 1.47 mm
  off the pad's (each about 80% open), pressed on at 19.40 s, and the trial ends at 27.3 s (546 steps) once both hands
  are clear. The first run of the demo (sand1, before the two measured offsets) also scored 1.0, its holes 1.59 mm off.
  A probe of the same demo with the disc put on square (not turned to the pad's holes) scores 0.5: caught and
  pressed on at 19.21 s with its middle within 1 mm of the pad's, but its holes 15.2 mm off, nearly half way between
  the pad's. Dream-1 scores 0.0 over the full 1,200 steps: both arms draw back towards their bases; the right hand
  passes near the disc stand but never takes the disc out, and the pad stays bare.
- The scripted demo's plan is 28.3 s of simulated time.
- The hook-and-loop model, the hole alignment, the scoring, the sizes, reach and step-guard checks, and the demo
  against the real collision hulls run without the simulator: `python -m pytest -q test_sanding_disc.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
