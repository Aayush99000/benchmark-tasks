# Crush a peeled garlic clove in a two-handle garlic press

A garlic press lies open on the table: its lower handle flat, its chamber next to the hinge, and its top handle swung
back past upright, resting on its stop. A peeled clove lies beside it. The robot has to put the clove in the chamber
the right way round (it fits only lengthways), swing the top handle back over the hinge so the plunger comes down on
the clove, and press the handle down hard enough to squeeze the clove through the chamber's perforated floor.

## The real step
America's Test Kitchen, [How to use a garlic press](https://www.americastestkitchen.com/articles/6126-how-to-use-a-garlic-press):
"You just put the clove in and press". Done by: home cooks and line cooks, many times a day in a busy kitchen.

What goes wrong in real life: a clove dropped in crosswise sits on the rim and the plunger smears it over the press
instead of through it; a handle not pressed home leaves the clove half squashed in the chamber.

What we simplified:
- The clove is a rigid block (2.6 x 1.6 x 1.4 cm) and weighs 30 g in the sim; a real clove that size is about 7 g.
  With 7 g, or 15 g, the simulated gripper's fingers closed through it (logged fingertip positions, sim runs on
  2026-11-17); at 30 g they hold it.
- Garlic doesn't squash in the simulator, so the crush is modeled. Once the plunger comes down on a clove lying in the
  chamber, the press's model pushes back on the handle as the clove would: stiff at first, then yielding at 10 N
  pressed at the grip, 13 cm from the hinge (`F_CRUSH`, an estimate, kept within what the arm can press). When the
  plunger's face is within 6 mm of the floor the clove is logged crushed and removed, and the squeezed garlic is drawn
  under the floor. The plunger itself has no collision shape; the model is what stops it.
- The press's sizes follow a common cast press (15.5 cm long, the length of a Zyliss Susi 3); the chamber's inside
  (3.4 x 2.4 x 2.4 cm) and the masses (130 g lower half, 70 g top handle) are estimates.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=garlic_press.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Crush the garlic clove in the garlic press. Put the clove in the press's chamber lengthways, swing the top handle back over and press it down until the garlic is squeezed through." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/garlic-press-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=garlic_press.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Crush the garlic clove in the garlic press. Put the clove in the press's chamber lengthways, swing the top handle back over and press it down until the garlic is squeezed through." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/garlic-press-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/garlic-press-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the clove is crushed and the press still lies on
the table, held 1 s; otherwise it scores the partial credit below at the last step. Set `GARLIC_LOG=<file>` to have
the press's model write each event (plunger on the clove, crushed, mashed on the rim) with its time.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| garlic press, lower half | silver cast aluminium, black rubber grip; loose, lying flat; 15.5 cm long from the hinge to the grip's end | along y at x 26, from y -6.3 (hinge) to 9.5 (grip end) |
| chamber | part of the lower half, next to the hinge: inside 3.4 cm long (along y), 2.4 cm wide, 2.4 cm deep; perforated floor 1 cm above the table, on two feet | centre (26, -3.7) |
| top handle | 15.4 cm, silver with a black grip, 2.0 x 2.6 cm plunger on its underside; hinged 3.9 cm above the table across the press; resting open on its stop, 25 degrees past upright | hinge at (26, -6); its end at (26, -12.6), 18 cm up |
| garlic clove | peeled, modeled as a yellow block 2.6 x 1.6 x 1.4 cm (30 g in the sim; a deeper yellow than a real clove, so it shows from the top camera) lying flat, its length along x | (32, -21) |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the press's crush log and the clove's position. The clove is in the chamber when its centre is inside the
chamber's walls and below its rim.

| Score | When |
|---|---|
| 1.0 | crushed: the clove in the chamber and the top handle pushed down until the plunger's face is within 6 mm of the floor (10 N at the grip, 13 cm from the hinge, once the plunger is on the clove), with the press still lying on the table (tilted under 20 degrees), held 1 s, which ends the trial |
| 0.5 | at the end, the clove in the chamber with the plunger brought down onto it, not crushed |
| 0.25 | at the end, the clove in the chamber |
| 0 | anything else: the clove elsewhere, an empty press shut, the clove mashed on the rim (the handle shut past 30 degrees with the clove on the chamber's rim), or the press knocked over |

On a real table: the clove is in the chamber if it lies inside it below the rim (a photo from above); crushed if the
handles are pressed together until the plunger is within 6 mm of the floor and garlic has come through the holes.

## The press model
`GarlicPress` in `garlic_press.py` runs before every physics substep (240 Hz). It reads the top handle's opening
angle from the two halves' poses and where the clove lies in the press's frame. With the clove in the chamber, once
the plunger's face reaches the clove's top it applies a torque about the hinge that opens the handle: 40 N m/rad of
squash, capped at 1.3 N m (10 N at the grip), and the clove keeps whatever squash it was pushed to (it doesn't spring
back past its elastic part). The two halves are rigid bodies joined by a PhysX revolute joint (`fittingkit.Hinge`),
limited between shut and the open stop.

## The scripted demo
Open loop, both arms, on one shared timeline (`demo()`):
1. The right arm picks the clove up across its width, carries it round the open handle, turns it a quarter turn over
   the chamber and drops it in from 4 mm above the rim.
2. With its fingers closed it goes round the open handle's end (sideways: the handle is 2.2 cm wide, and a
   straight-down grip can't reach over its 18 cm top), comes in just beyond it and pushes it back past upright; the
   handle falls onto the clove.
3. The left arm then presses the handle's grip down, aimed 3 cm below the shut handle's top: the arm's compliance
   turns that aim into force.

The poses were checked offline against the YAM's gripper collision hulls (URDF forward kinematics, `plan_check.py`):
both arms reach every pose within 0.1 mm, no step exceeds the per-step joint limits, and outside the intended
contacts neither hand comes within 6 mm of the press.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit
de817b9), under the MIT licence its authors state in that task's README, with one addition noted in its docstring
(`Slider` takes an optional spring drive; this task doesn't use it). It provides the compound bodies, the hinge and
the per-substep model hook.

## Runtime and seeds
- The scripted demo (official settings, 2026-11-17): 1.0, ending after 508 steps (25.4 s of simulated time; the
  plunger met the clove at 19.3 s and crushed it at 24.4 s). The whole run took 83 s on an L40S, after Isaac Sim's
  first start on the machine.
- Dream-1 (official settings, 2026-11-17): 0.0 over the full 1,200 steps. Its right arm reached around the press
  and the clove again and again but never picked the clove up; the handle stayed open and the left arm hardly moved.
- With the pressing hand aimed only 5 mm below the shut handle instead of 30 mm, the plunger comes down on the clove
  but stops 5 degrees open, short of the crush (2.4 degrees): the trial ends at 0.5.
- The crush model, the scoring, the layout, reach and step-guard checks, and the demo against the real gripper hulls
  run without the simulator: `python -m pytest -q test_garlic_press.py` (about 15 s).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
