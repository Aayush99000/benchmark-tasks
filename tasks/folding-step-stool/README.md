# Unfold a child's folding step stool

A child's low folding step stool lies folded flat on the table, its two legs folded in under its top. The instruction
asks for it to be set up the way its maker says: each leg opened fully, until its own latch clicks, then the stool stood
on its feet and let go of. One hand lifts one end of the stool by the grip there, so the leg under that end hangs from
its hinge, while the other hand pushes the leg out past straight down until it latches; then the same for the other end,
with the hands' jobs swapped.

## Changes in this revision
For the review of TK-2RU7AY (2026-11-19):
- **The stool can now be built.** `hardware/` has every printed part as an STL file, the script that makes them
  (`build_parts.py`, from the numbers in `stool_dimensions.py`), and a dimensioned drawing (`drawing.png`).
  "Building it for real trials" below gives the parts list: the latch springs are a catalogue part (Lee Spring
  LC 026C 05 M); the hinges are M5 rod through printed knuckles; the rest is screws, a pine strip and under 1 kg of
  filament [estimate].
- **The lock is a spring latch with a release.** Each leg's latch is a printed bolt in a channel in the top, pushed out
  by the spring. As the leg opens, a wedge on its knuckle pushes the bolt back, and the bolt pushes the leg back. Past
  98 degrees the bolt drops 3.5 mm behind the wedge, and the leg can't fold. Pulling the knob on the stool's far side
  4.5 mm releases it.
- **The sim follows that hardware.** It has the build's sizes, its hinge ears and its latch bosses and knobs. The
  push-back torque now comes from the spring's catalogue rate and the wedge's rise: 0.10 N m at 80 degrees to 0.165 N m
  just short of 98. Before, it was an estimate, 0 to 0.16 N m. A test holds the sim's numbers to the build's. The CAD
  check turns each printed leg through its travel against the printed top. The bolt's travel it finds matches the sim's
  model within 0.01 mm, and the leg stops at 100 degrees.
- **"Scoring a real trial" says how a real trial is scored.** It covers how each latch and the 40 N standing test are
  checked.
- The task, the layout, the instruction and the scoring tiers are unchanged.

## The real step
Parents set up a folding step stool at the bathroom or kitchen sink for a child, and workers use them to reach shelves.
Hailo's instructions for its folding step stools (K70 StandardLine): "When setting up the step stool pay attention to the
entire secured opening of both step stool parts". OSHA 1926.1053(a)(8) asks the same of every stepladder: a "metal spreader
or locking device", "to hold the front and back sections in an open position when the ladder is being used". Done by:
parents and carers; stock clerks.

What goes wrong in real life: a leg that looks open but isn't locked folds when someone steps on it, and the stool
collapses under them.

How this differs from folding a camp stool (robocurve#1143, not accepted): there, the job was folding, and nothing
showed whether it was done right. Here each leg has its own latch that catches only past straight down, so the stool can
look open with a leg unlatched. The score tells the two apart by loading the stool once it stands on its own.

What we simplified: the stool is our own buildable design of a low children's folding stool. It's lighter and a little
smaller than most, so the arms can reach under it. In the sim the latch is a model on each leg's hinge, not simulated
contacts (see "The latch model"). A 40 N push down on the top stands in for a child stepping on it.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=folding_step_stool.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unfold the step stool. Open each of its two legs fully, until its lock clicks, then stand the stool on its feet and let go of it." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/folding-step-stool-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=folding_step_stool.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unfold the step stool. Open each of its two legs fully, until its lock clicks, then stand the stool on its feet and let go of it." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/folding-step-stool-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/folding-step-stool-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the finished state has held for 1 s; otherwise it
scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left. The sizes are those of the buildable stool (`hardware/stool_dimensions.py`).

| Object | Description | Position |
|---|---|---|
| stool's top | blue, 24.6 cm long (along y), 20 cm deep, 1.8 cm thick, with a raised grey grip rib (7 cm long, 1.4 cm thick, 2 cm high) across each end; under it, at each end, the two ears its leg's hinge rod runs through; on its far side (+x), at each end, the latch's boss (1.95 cm out) and its black release knob (1.4 cm across, 0.7 cm out); 0.55 kg | folded, its middle at (31, 0), 3.4 cm up |
| legs | two, white, one at each end: each on a hinge under the top 10.5 cm from its middle, 15 cm wide and 10 cm long, folded in flat under the top; each ends in a grey foot rail 34 cm long that sticks out 7 cm past the top's front and back edges; 0.10 kg each | under the top |

Each leg opens to 100 degrees from folded (10 past straight down, its foot outwards) and latches at 98. Open and standing,
the stool is 12.6 cm high. Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the stool's latch model and its pose.

| Score | When, at the end |
|---|---|
| 1.0 | both legs latched, and the stool standing on its feet (its top level within 5 degrees, at its standing height within 1 cm) with both grippers more than 2 cm from it, under the 40 N push, held 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | both legs latched, the stool not standing on its own |
| 0.25 | one leg latched |
| 0 | anything else: a leg that only looks open earns nothing |

- A leg latches once it is opened 98 degrees from folded; once latched it stays latched.
- Short of its latch, from 80 degrees on, the latch's bolt rides the leg's wedge and pushes the leg back: 0.10 N m at 80
  degrees, rising to 0.165 N m (1.6 N at the foot) just short of 98. Let go of, an unlatched leg comes back to about 80
  degrees, where it looks nearly open.
- The 40 N push down on the top comes on whenever the stool stands level at its height with both grippers more than
  2 cm from it. A leg that isn't latched then gives way (the model folds it, 6 N m), and the stool comes down on that
  end; a latched leg holds.

## The latch model
`Locks` in `folding_step_stool.py` runs before every physics substep (240 Hz):
- **Angle.** It reads each leg's angle on its hinge from the legs' and the top's poses. The legs are rigid bodies on
  PhysX revolute joints under the top (`fittingkit.Hinge`, limited to 0 to 100 degrees, as the printed pocket's end
  wall stops the real leg).
- **Push-back.** Short of the latch it applies the push-back torque, `stool_dimensions.fold_back_torque_nm`. That is
  the spring's force (9.59 N preload plus 2.189 N/mm, from its catalogue numbers) times the wedge's rise per radian
  (3 mm over 18 degrees), friction left out.
- **Latch.** It notes the latch at 98 degrees, and from then on holds the leg open against its stop (2 N m, against
  PhysX's joint limit).
- **Load.** Every 0.1 s it checks the grippers' collision hulls against the stool's parts. It applies the 40 N push
  while the stool stands on its own; once the push is on, a leg that isn't latched gives way.

That last part is a rule, not left to the contacts. With both feet held by friction, the stool and a leg sprung back to
81 degrees make a rigid triangle, and in a probe run the stool stood level under the push without it. The bolt, the
wedge and the top's pocket aren't simulated as contacts or drawn (they're inside the top). The ears, the bosses and the
knobs are collision shapes of the top.

The model is the CAD's. `hardware/build_parts.py` turns the printed leg from 0 to 101 degrees against the printed top
and, at each half degree, finds how far the bolt has to be pushed back to clear the wedge. It finds:
- the leg clears the top up to 100 degrees and meets the pocket's end wall there;
- the wedge reaches the bolt's bevel at about 78.5 degrees and has it 0.50 mm back at 80, rising evenly to 3.42 mm at
  97.5 (the model: 0.50 and 3.42);
- the bolt drops behind the wedge at 98.0.

`test_the_cad_parts_move_as_the_simulator_says` repeats that check.

## Building it for real trials
Everything is in `hardware/`:
- `drawing.png`, the dimensioned drawing;
- `stl/`, the printed parts;
- `build_parts.py`, which regenerates both and repeats the check (`pip install manifold3d trimesh matplotlib numpy`,
  then `python build_parts.py`);
- `stool_dimensions.py`, every number, in mm.

Parts (about US$45 in all [estimate: the springs' listed price, plus typical prices for the rest]):

| Part | Qty | Notes |
|---|---|---|
| `top.stl` | 1 | blue PLA; print upside down (its upper face on the bed), 4 walls, 15% infill; needs a bed of at least 250 x 220 mm |
| `leg_plus_y.stl`, `leg_minus_y.stl` | 1 each | white PLA; panel face down, supports under the knuckle and wedge; 3 walls, 30% infill |
| `bolt.stl` | 2 | PETG, 100% infill, lying on its 5 mm side so the knife edge is drawn in one layer; one is fitted turned over (the -y end's bevel faces -y) |
| `rib.stl` | 2 | grey PLA |
| `knob.stl` | 2 | black PLA; pressed over the latch screw's head |
| compression spring, Lee Spring LC 026C 05 M | 2 | music wire, 6.1 mm outside, 15.88 mm free, 2.189 N/mm, solid at 5.16 mm (leespring.com, US$8.04 each for 1 to 19, read 2026-11-19) |
| M5 threaded rod, 195 mm | 2 | the hinge pins; with 4 M5 nylon-insert nuts and 8 M5 washers (1 mm: one in each knuckle-to-ear gap, one under each nut) |
| M3 x 18 socket cap screw, M3 washer (0.5 mm) | 2 each | the latch: the screw goes through the bore's end wall and the spring into the bolt (tap the bolt's 2.5 mm hole M3, with a drop of threadlocker) |
| M3 x 20 socket cap screw, M3 nut | 4 each | the ribs: screwed up through the top into nuts in the ribs' bases |
| pine strip, 16 x 16 mm, 680 mm | 1 | cut into two 340 mm foot rails, painted grey; each fixed with 3 countersunk wood screws 3.0 x 30 mm into the leg's pilot holes |
| steel washers up to 30 mm across | as needed | ballast, glued into the two recesses under the top, to bring the top to 0.55 kg |

Assembly:
1. Tap each bolt's hole M3. Put a bolt in its channel, bevel towards the end of the stool its leg's wedge comes from
   (+y at the +y end), then a washer and a spring in the bore. Screw the M3 x 18 through the end wall into the bolt.
2. Turn the screw until the bolt's knife edge stands 81.5 mm from the top's centre line in x (`drawing.png`, section
   B-B), 4.5 mm into the pocket from its outer wall (x 86). Lock the screw with threadlocker and press the knob on its head.
3. Fit each leg between its ears with a washer in each gap. Pass the rod through and fit a washer and nut at each end,
   snug but free to turn.
4. Screw the foot rails on, and the ribs on through the top.
5. Weigh the top (with its ribs, latches and knobs) and add ballast to 0.55 ± 0.03 kg. Weigh each leg with its rail,
   0.10 kg in the sim, and note the weights.

Before using it, check on the bench, for each leg:
- it clicks at 98 ± 1 degrees from folded, which is 8 degrees past square to the top's underside (a phone inclinometer
  on the leg);
- it stops at 100;
- let go of anywhere between 80 and 98, it springs back towards 80;
- once clicked, it holds a 5 N push inwards at the foot.

Note any leg that sticks short of 80 by friction.

What isn't verified yet: none of the parts has been printed or assembled. The springs' numbers are the maker's
catalogue values, and the latch's travel is checked in CAD only. The first build should repeat the bench check above
and record the angles and forces it measures.

## Scoring a real trial
1. **Start.** The stool lies folded at (31, 0), both latches released: pull each knob out while folding its leg flat.
2. **Run.** Run the policy with the cameras recording.
3. **Standing test (40 N).** At the end of the trial, if both grippers are more than 2 cm from the stool (from the
   video), an operator sets a 4.08 kg weight gently on the middle of the top and takes their hand away. 4.08 kg is
   40 N: for example a 4 kg dumbbell plate and 80 g, or a bag of sand weighed out. After 1 s, the stool is standing if:
   - both foot rails are on the table;
   - its top is level within 5 degrees (a phone inclinometer on the top beside the weight);
   - its upper face is 12.6 ± 1 cm above the table.

   The operator then lifts the weight off.
4. **Latch check, each leg.** Without touching the knob, push the leg's foot rail inwards at its middle with 5 N (a
   push-pull gauge, or a kitchen scale held against the rail and pressed to 510 g). A latched leg doesn't move: its
   bolt is behind the wedge, which shows in the gap under the top's far corner. An unlatched leg folds.
5. **Score.** The sim's tiers apply:
   - 1.0: both legs latched, and the stool stood under the weight with the grippers clear;
   - 0.5: both latched, but it didn't stand;
   - 0.25: one latched;
   - 0 otherwise.

   In the sim the push comes on by itself as soon as the stool stands with the hands off, and the trial ends 1 s
   later. In a real trial the operator applies it at the end. A leg that isn't latched scores nothing either way.

## The scripted demo
Open loop, both arms, on one shared timeline:
1. The right hand pinches the grip rib at the -y end by the rib's two ends and lifts that end. The stool turns on its
   other end, and between the pads, so the hand needn't turn, until the top is tilted about 33 degrees and the -y leg
   hangs straight down.
2. The left hand, fingers closed and pointing down, comes down beside the -y leg's foot rail where it sticks out at the
   front, 6 cm outside the top's edge. It pushes the rail outwards round past straight down until it latches: the push
   is aimed 20 degrees past the stop, and the fingers' side meets the rail, the tips below it.
3. The right hand lets the end down onto that leg, lets go and rises; the left hand goes up and over to the +y end.
4. The left hand pinches the grip rib there and lifts that end, the stool standing on its latched -y leg, until the +y
   leg hangs clear (the latch's spring holds it at about 80 degrees). The right hand pushes that leg's foot rail out
   until it latches. The left lets the end down, the stool level on both legs, and lets go.
5. Both hands move clear.

As the -y end is lifted, the stool slides about 25 mm along y on its other end and turns about 5 degrees about the
vertical (every run did, by much the same amount). The first push is aimed at where the stool really is once lifted,
measured in earlier runs (`LIFTED_1`); aimed at the planned pose it missed the rail or rode over it.

The poses were checked offline against the YAM's gripper and wrist collision hulls (URDF forward kinematics,
`plan_check.py`), with the stool where the plan expects it and where the sim puts it during the first push:
- no part of either hand touches the stool, except the lifting hand on the rib it pinches and the pushing hand on the
  rail it pushes;
- the latch bosses, knobs and ears stay clear of both hands by at least 25 mm;
- the two hands stay at least 26.9 mm apart;
- both arms reach every pose within 0.5 mm and 3 degrees, and no step exceeds 75% of the per-step joint limit.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with one addition noted in its docstring: `Slider`
takes an optional spring drive (`stiffness`, `damping`, `target`), unused here. `plan_check.py` is the offline check of
the demo described above. `hardware/` is the buildable stool.

## Runtime and seeds
- **This revision** (2026-11-19, the Robocurve Brev launchable, 1,200 steps, `object_jitter=0`):
  - The scripted demo scores 1.0: the trial ends at 42.0 s (839 steps), as before the revision.
  - Dream-1 scores 0.0 over the full 1,200 steps. Around 40 s both arms reach in beside the stool's ends and draw
    back again; the stool stays folded.
- Before this revision (2026-11-18, the same launchable, the earlier spring estimate):
  - The scripted demo scored 1.0: the -y leg locked at 14.7 s, the +y leg at 33.5 s, the stool took the push standing
    level at 41.0 s, and the trial ended at 42.0 s (840 steps).
  - Dream-1 scored 0.0 over the full 1,200 steps: both arms drew back towards their bases and never touched the stool,
    which stayed folded.
  - A probe of the demo with the +y leg pushed only halfway (to 97 degrees, one short of its lock) scored 0.25. The leg
    sprang back to 81 degrees and the stool, set down, looked open. When the push came on, that leg gave way and the
    stool came down on that end.
- **Offline checks** run without the simulator: `python -m pytest -q test_folding_step_stool.py`. They cover the latch
  model, the leg angle, the load and giving way, the scoring, the sizes against the build's, the spring, the CAD's
  latch travel, the reach and step-guard checks, and the demo against the real collision hulls. The CAD test needs
  `manifold3d` and is skipped without it.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
