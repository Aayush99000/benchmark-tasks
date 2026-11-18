# Unfold a child's folding step stool

A child's low plastic folding step stool lies folded flat on the table, its two legs folded in under its top. The
instruction asks for it to be set up the way its maker says: each leg opened fully, until its own lock clicks, then the
stool stood on its feet and let go of. One hand lifts one end of the stool by the grip there, so the leg under that end
hangs from its hinge, while the other hand pushes the leg out past straight down until it locks; then the same for the
other end, with the hands' jobs swapped.

## The real step
Parents set up a folding step stool at the bathroom or kitchen sink for a child, and workers use them to reach shelves.
Hailo's instructions for its folding step stools (K70 StandardLine): "When setting up the step stool pay attention to the
entire secured opening of both step stool parts". OSHA 1926.1053(a)(8) asks the same of every stepladder: a "metal spreader
or locking device", "to hold the front and back sections in an open position when the ladder is being used". Done by:
parents and carers; stock clerks.

What goes wrong in real life: a leg that looks open but isn't locked folds when someone steps on it, and the stool
collapses under them.

What's different from folding a camp stool (robocurve#1143, not accepted): there, the job was folding, and nothing showed
whether it was done right. Here each leg has its own lock that catches only past straight down, so the stool can look open
with a leg unlocked, and the score tells the two apart by loading the stool once it stands on its own.

What we simplified: the stool is our model of a low children's folding stool, lighter and a little smaller than most so
the arms can reach under it; the locks are a model on the legs' hinges, with no release; a 40 N push down on the top stands
in for a child stepping on it.

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
robot's left.

| Object | Description | Position |
|---|---|---|
| stool's top | blue plastic, 24.6 cm long (along y), 20 cm deep, 1.8 cm thick, with a raised grey grip rib (7 cm long, 1.4 cm thick, 2 cm high) across each end; 0.55 kg | folded, its middle at (31, 0), 3.4 cm up |
| legs | two, white, one at each end: each on a hinge under the top 10.5 cm from its middle, 15 cm wide and 10 cm long, folded in flat under the top; each ends in a grey foot rail 34 cm long that sticks out 7 cm past the top's front and back edges; 0.10 kg each | under the top |

Each leg opens to 100 degrees from folded (10 past straight down, its foot outwards) and locks at 98. Open and standing,
the stool is 12.6 cm high. Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the stool's lock model and its pose.

| Score | When, at the end |
|---|---|
| 1.0 | both legs locked, and the stool standing on its feet (its top level within 5 degrees, at its standing height within 1 cm) with both grippers more than 2 cm from it, under the 40 N push, held 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | both legs locked, the stool not standing on its own |
| 0.25 | one leg locked |
| 0 | anything else: a leg that only looks open earns nothing |

- A leg locks once it is opened 98 degrees from folded; once locked it stays locked.
- Short of its lock, from 80 degrees on, a spring folds it back (about 0.15 N m, 1.6 N at the foot just short of the
  lock), so let go of, an unlocked leg comes back to about 80 degrees: it looks nearly open.
- The 40 N push down on the top comes on whenever the stool stands level at its height with both grippers more than
  2 cm from it. A leg that isn't locked then gives way (the model folds it, 6 N m), and the stool comes down on that
  end; a locked leg holds.

## The lock model
`Locks` in `folding_step_stool.py` runs before every physics substep (240 Hz). It reads each leg's angle on its hinge
from the legs' and the top's poses; short of the lock it applies the folding spring, it notes the lock at 98 degrees, and
from then on holds the leg open against its stop (2 N m, against PhysX's joint limit). The legs are rigid bodies on PhysX
revolute joints under the top (`fittingkit.Hinge`, limited to 0 to 100 degrees). Every 0.1 s it checks the grippers'
collision hulls against the stool's parts, and applies the 40 N push while the stool stands on its own; once the push is
on, a leg that isn't locked gives way. That last part is a rule, not left to the contacts: with both feet held by
friction, the stool and a leg sprung back to 81 degrees make a rigid triangle, and in a probe run the stool stood level
under the push without it.

## The scripted demo
Open loop, both arms, on one shared timeline:
1. The right hand pinches the grip rib at the -y end by the rib's two ends and lifts that end, the stool turning on its
   other end (and between the pads, so the hand needn't turn), until the top is tilted about 33 degrees and the -y leg
   hangs straight down.
2. The left hand, fingers closed and pointing down, comes down beside the -y leg's foot rail where it sticks out at the
   front, 6 cm outside the top's edge, and pushes it outwards round past straight down until it locks (the push is
   aimed 20 degrees past the stop; the fingers' side meets the rail, the tips below it).
3. The right hand lets the end down onto that leg, lets go and rises; the left hand goes up and over to the +y end.
4. The left hand pinches the grip rib there and lifts that end, the stool standing on its locked -y leg, until the +y
   leg hangs clear (its spring holds it at about 80 degrees); the right hand pushes that leg's foot rail out until it
   locks; the left lets the end down, the stool level on both legs, and lets go.
5. Both hands move clear.

As the -y end is lifted, the stool slides about 25 mm along y on its other end and turns about 5 degrees about the
vertical (every run did, by much the same amount). The first push is aimed at where the stool really is once lifted,
measured in earlier runs (`LIFTED_1`); aimed at the planned pose it missed the rail or rode over it.

The poses were checked offline against the YAM's gripper and wrist collision hulls (URDF forward kinematics,
`plan_check.py`, with the stool where the plan expects it, and where the sim puts it during the first push): no part of
either hand touches the stool except the lifting hand on the rib it pinches and the pushing hand on the rail it pushes,
and the two hands stay at least 26.9 mm apart. Both arms reach every pose within 0.5 mm and 3 degrees, and no step
exceeds 75% of the per-step joint limit.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with one addition noted in its docstring: `Slider`
takes an optional spring drive (`stiffness`, `damping`, `target`), unused here. `plan_check.py` is the offline check of
the demo described above.

## Runtime and seeds
- Official runs (2026-11-18, the Robocurve Brev launchable, 1,200 steps, `object_jitter=0`): the scripted demo scores
  1.0: the -y leg locks at 14.7 s, the +y leg at 33.5 s, the stool takes the push standing level at 41.0 s, and the trial
  ends at 42.0 s (840 steps). An earlier run of the same demo gave the same times. Dream-1 scores 0.0 over the full
  1,200 steps: both arms draw back towards their bases and never touch the stool, which stays folded.
- A probe of the demo with the +y leg pushed only halfway (to 97 degrees, one short of its lock) scores 0.25: the leg
  springs back to 81 degrees and the stool, set down, looks open; when the push comes on, that leg gives way and the
  stool comes down on that end.
- The lock model, the leg angle, the load and giving way, the scoring, the sizes, reach and step-guard checks, and the
  demo against the real collision hulls run without the simulator: `python -m pytest -q test_folding_step_stool.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
